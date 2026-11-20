//
//  S0ixStandby.cpp
//  S0ixStandby — Modern Standby (S0ix) support for macOS "Deep Idle" sleep.
//
//  How it fits together (see docs/RESEARCH-2A.md):
//    * An SSDT (acpi/SSDT-DEEPIDLE) makes AppleACPIPlatform publish
//      IOPMDeepIdleSupported; macOS then sleeps with IOPMSystemSleepType 7
//      ("Deep Idle", XNU acpi_idle_kernel()) instead of ACPI S3.
//    * Apple never calls the LPS0 _DSM that PC firmware expects for Modern
//      Standby. This kext calls it from IOPlatformSleepAction (after all
//      drivers slept, before the CPUs halt, thread context, interrupts on) and
//      IOPlatformWakeAction, but only for Deep Idle sleeps.
//    * It also reads the SLP_S0 residency counter described by ACPI LPIT, so
//      every sleep reports how long the platform really spent in S0ix.
//
//  _DSM GUIDs and function numbers follow the Intel/Microsoft/AMD LPS0
//  specifications (Linux drivers/acpi/x86/s2idle.c used only as a cross-check).
//
//  Boot-args:  -s0ixoff    do not load
//              -s0ixnodsm  meter only, never call _DSM
//              -s0ixdbg    verbose logging
//              -s0ixd3     keep driverless PCI endpoints in D3hot (from 60 s after
//                          boot, re-applied before each Deep Idle)
//              -s0ixpmcdump snapshot the PMC register page (with the SLP_S0
//                          counter) at boot and at each Deep Idle halt
//              (default)   Intel Gen11/12 iGPU: full GT reset at a Deep Idle wake,
//                          before drivers, when the GPU kept its state (S0ixGPU.cpp)
//              -s0ixnogpureset  never reset the GPU
//              -s0ixwifipm let IOPCIFamily put the Wi-Fi PCI function in D3 for
//                          sleep (enablePCIPowerManagement), with _PS3 and config
//                          restore on wake (experimental)
//              -s0ixgpuprobe  also probe an idle GPU (canary) and keep the last
//                          result in NVRAM (s0ix-gpu-last); for debugging
//              s0ixfn=N    bitmask of _DSM functions allowed to run, e.g.
//                          s0ixfn=0x18 = display off/on only (3,4),
//                          s0ixfn=0x60 = entry/exit only (5,6). Default: all.
//
//  SPDX-License-Identifier: BSD-3-Clause
//

#include "S0ixStandby.hpp"

#include <IOKit/IOLib.h>
#include <IOKit/acpi/IOACPIPlatformExpert.h>
#include <IOKit/pwr_mgt/RootDomain.h>
#include <libkern/OSByteOrder.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
#include <architecture/i386/pio.h>
#include <stdarg.h>
#include <libkern/libkern.h>

OSDefineMetaClassAndStructors(S0ixStandby, IOService)

#define LOG(fmt, ...)  s0ixLog(fmt, ##__VA_ARGS__)
#define DBG(fmt, ...)  do { if (gDebug) s0ixLog(fmt, ##__VA_ARGS__); } while (0)

static bool gDebug = false;

// ---------------------------------------------------------------------------
// Log ring: IOLog output from third-party kexts doesn't reliably reach dmesg
// or the unified log on recent macOS, so the last lines are also kept here and
// published as "Recent log". Safe from the sleep/wake actions (no allocation,
// no registry access); publishLog() copies it to the registry later.
// ---------------------------------------------------------------------------
static const unsigned kLogLines = 40;
static const unsigned kLogLen = 128;
static char gLogBuf[kLogLines][kLogLen];
static uint64_t gLogTime[kLogLines];
static uint32_t gLogNext;
static volatile bool gLogDirty;
static IOSimpleLock *gLogLock;

void s0ixLog(const char *fmt, ...)
{
	char line[kLogLen];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(line, sizeof(line), fmt, ap);
	va_end(ap);
	IOLog("S0ixStandby: %s\n", line);

	uint64_t now = mach_absolute_time();
	IOInterruptState st = 0;
	if (gLogLock) st = IOSimpleLockLockDisableInterrupt(gLogLock);
	uint32_t i = gLogNext % kLogLines;
	memcpy(gLogBuf[i], line, sizeof(line));
	gLogTime[i] = now;
	gLogNext++;
	gLogDirty = true;
	if (gLogLock) IOSimpleLockUnlockEnableInterrupt(gLogLock, st);
}

void S0ixStandby::publishLog()
{
	if (!gLogDirty) return;
	IOInterruptState st = 0;
	if (gLogLock) st = IOSimpleLockLockDisableInterrupt(gLogLock);
	uint32_t next = gLogNext;
	gLogDirty = false;
	if (gLogLock) IOSimpleLockUnlockEnableInterrupt(gLogLock, st);

	uint32_t count = next < kLogLines ? next : kLogLines;
	OSArray *arr = OSArray::withCapacity(count ? count : 1);
	if (!arr) return;
	for (uint32_t k = 0; k < count; k++) {
		uint32_t i = (next - count + k) % kLogLines;
		char text[kLogLen];
		uint64_t t;
		// One line at a time under the lock (small stack, no allocation held).
		if (gLogLock) st = IOSimpleLockLockDisableInterrupt(gLogLock);
		memcpy(text, gLogBuf[i], sizeof(text));
		t = gLogTime[i];
		if (gLogLock) IOSimpleLockUnlockEnableInterrupt(gLogLock, st);
		text[kLogLen - 1] = 0;
		uint64_t ns = 0;
		absolutetime_to_nanoseconds(t, &ns);
		char line[kLogLen + 24];
		snprintf(line, sizeof(line), "%llu.%03llu %s",
		         ns / 1000000000ULL, (ns / 1000000ULL) % 1000ULL, text);
		if (OSString *s = OSString::withCString(line)) {
			arr->setObject(s);
			s->release();
		}
	}
	setProperty("Recent log", arr);
	arr->release();
}

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

// GUIDs in the textual form used by ACPI ToUUID().
static const char *const kDsmGuid[] = {
	"c4eb40a0-6cd2-11e2-bcfd-0800200c9a66",   // Intel LPS0
	"11e00d56-ce64-47ce-837b-1f898f9aa461",   // Microsoft Modern Standby
	"e3f32452-febc-43ce-9039-932122d37721",   // AMD LPS0
};
static const char *const kFamilyName[] = { "Intel", "Microsoft", "AMD" };

// Function numbers
enum : uint32_t {
	kFnQuery = 0,
	// Intel and Microsoft
	kFnDisplayOff = 3, kFnDisplayOn = 4, kFnEntry = 5, kFnExit = 6,
	// Microsoft only
	kFnMsSleepEntry = 7, kFnMsSleepExit = 8,
	// AMD
	kFnAmdEntry = 2, kFnAmdExit = 3, kFnAmdDisplayOff = 4, kFnAmdDisplayOn = 5,
};

// Sleep/wake action priority: high = late in the sleep queue and early in the
// (reversed) wake queue, so _DSM entry runs after other sleep actions and _DSM
// exit before other wake actions.
static const uint32_t kActionPriority = 0x8000;

static const uint32_t kSleepTypeDeepIdle = 7;   // kIOPMSleepTypeDeepIdle
static const uint16_t kPm1RtcEn = 1u << 10;      // PM1_EN.RTC_EN (ACPI fixed event)
static const uint16_t kPm1PwrBtn = 1u << 8;       // PM1_STS.PWRBTN_STS / PM1_EN.PWRBTN_EN
static uint8_t kbcArm(uint8_t &before, uint8_t &after);
// ACPI EC (ECDT: data 0x62, command/status 0x66). Status: bit0 OBF, bit1 IBF, bit5 SCI_EVT.
static bool ecWait(uint8_t mask, uint8_t want)
{
	for (int i = 0; i < 2000; i++) {                      // up to ~20 ms
		if ((inb(0x66) & mask) == want) return true;
		IODelay(10);
	}
	return false;
}
// PCH GPE0 registers: use aligned 32-bit I/O (byte writes to GPE0_EN did not stick on Alder/Raptor Lake PCH).
static uint32_t gpeEnSet(uint16_t enBase, uint32_t gpe)
{
	uint16_t p = (uint16_t)(enBase + (gpe / 32) * 4);
	uint32_t m = 1u << (gpe % 32), v = inl(p);
	if (!(v & m)) outl(p, v | m);
	return inl(p);
}
static const uint8_t  kCmosAie  = 1u << 5;       // RTC register B: alarm interrupt enable
static inline uint8_t cmosRead(uint8_t reg) { outb(0x70, reg); return inb(0x71); }
static inline void cmosWrite(uint8_t reg, uint8_t v) { outb(0x70, reg); outb(0x71, v); }

// ACPI ToUUID() byte order: first three fields little-endian, rest as written.
static bool guidToBytes(const char *s, uint8_t out[16])
{
	uint8_t raw[16];
	int n = 0;
	for (const char *p = s; *p && n < 32; p++) {
		if (*p == '-') continue;
		int v;
		if (*p >= '0' && *p <= '9') v = *p - '0';
		else if (*p >= 'a' && *p <= 'f') v = *p - 'a' + 10;
		else if (*p >= 'A' && *p <= 'F') v = *p - 'A' + 10;
		else return false;
		if (n & 1) raw[n / 2] |= (uint8_t)v; else raw[n / 2] = (uint8_t)(v << 4);
		n++;
	}
	if (n != 32) return false;
	static const uint8_t order[16] = { 3, 2, 1, 0, 5, 4, 7, 6, 8, 9, 10, 11, 12, 13, 14, 15 };
	for (int i = 0; i < 16; i++) out[i] = raw[order[i]];
	return true;
}

// --- CPU helpers -----------------------------------------------------------

static inline void cpuid(uint32_t leaf, uint32_t &a, uint32_t &b, uint32_t &c, uint32_t &d)
{
	asm volatile("cpuid" : "=a"(a), "=b"(b), "=c"(c), "=d"(d) : "a"(leaf), "c"(0));
}

static inline uint64_t rdmsr(uint32_t msr)
{
	uint32_t lo, hi;
	asm volatile("rdmsr" : "=a"(lo), "=d"(hi) : "c"(msr));
	return ((uint64_t)hi << 32) | lo;
}

static inline uint64_t rdtsc()
{
	uint32_t lo, hi;
	asm volatile("rdtsc" : "=a"(lo), "=d"(hi));
	return ((uint64_t)hi << 32) | lo;
}

// Per-CPU core residency, read on demand on every CPU (sysctl debug.s0ix.cores, read-only, no root):
// per logical CPU 48 bytes {u32 valid, u32 core type (CPUID 0x1A eax[31:24]: 0x20 Atom, 0x40 Core), u64 TSC,
// u64 MSR 0x3FC core C3, 0x3FD core C6, 0x3FE core C7, 0x664 Atom module C6} (~0 = read failed). Read it just before
// and right after a Deep Idle sleep: the deltas show which CPUs stayed deep through the halt (offline CPUs that
// take an interrupt in _xcpm_cpu_halt fall back to cli;hlt = C1 for the rest of the sleep).
extern "C" int rdmsr_carefully(uint32_t msr, uint32_t *lo, uint32_t *hi);
extern "C" void mp_rendezvous_no_intrs(void (*action_func)(void *), void *arg);
extern "C" int cpu_number(void);
#include <sys/sysctl.h>
#include <sys/errno.h>
static constexpr int kCoreSnapMax = 32;
struct CoreSnap { uint32_t valid, type; uint64_t tsc, c3, c6, c7, mc6; };
static CoreSnap gCoreSnap[kCoreSnapMax];
static void coreSnapOne(void *)
{
	int cpu = cpu_number();
	if (cpu < 0 || cpu >= kCoreSnapMax) return;
	auto rd = [](uint32_t msr) -> uint64_t {
		uint32_t lo = 0, hi = 0;
		return rdmsr_carefully(msr, &lo, &hi) == 0 ? ((uint64_t)hi << 32 | lo) : ~0ULL;
	};
	uint32_t lo, hi, a, b, c, d;
	asm volatile("rdtsc" : "=a"(lo), "=d"(hi));
	asm volatile("cpuid" : "=a"(a), "=b"(b), "=c"(c), "=d"(d) : "a"(0x1A), "c"(0));
	gCoreSnap[cpu] = {1, a >> 24, (uint64_t)hi << 32 | lo, rd(0x3FC), rd(0x3FD), rd(0x3FE), rd(0x664)};
}
static IOLock *gCoreSnapLock;
static int coreSnapHandler(struct sysctl_oid *, void *, int, struct sysctl_req *req)
{
	if (req->newptr) return EPERM;
	if (!gCoreSnapLock) return ENOENT;
	IOLockLock(gCoreSnapLock);
	bzero(gCoreSnap, sizeof(gCoreSnap));
	mp_rendezvous_no_intrs(coreSnapOne, nullptr);
	int err = SYSCTL_OUT(req, gCoreSnap, sizeof(gCoreSnap));
	IOLockUnlock(gCoreSnapLock);
	return err;
}
// Live copy of the PMC MMIO page that holds the LPIT residency counter (0xFE001000 here = PMC offsets 0x1000-0x1FFF:
// LTR_IGNORE 0x1B0C, LTR 0x1B50-0x1C0C, LPM status 0x1C34+, PPFEAR 0x1D90) - sysctl debug.s0ix.pmc, read-only.
// Same layout as the "PMC page at halt" / "after halt" properties of -s0ixpmcdump, so awake vs halt compare 1:1.
static volatile uint32_t *gPmcPage;
static uint32_t gPmcCopy[1024];
static int pmcPageHandler(struct sysctl_oid *, void *, int, struct sysctl_req *req)
{
	if (req->newptr) return EPERM;
	if (!gCoreSnapLock || !gPmcPage) return ENOENT;
	IOLockLock(gCoreSnapLock);
	for (uint32_t i = 0; i < 1024; i++) gPmcCopy[i] = gPmcPage[i];
	int err = SYSCTL_OUT(req, gPmcCopy, sizeof(gPmcCopy));
	IOLockUnlock(gCoreSnapLock);
	return err;
}
// Display engine registers (same ranges as macos/pmcdump PMCDump.cpp / gpu-wm.py: DBUF, WM_LINETIME, DC_STATE,
// transcoder A/B timings, pipes A-D cursor/planes/watermarks/DDB, DMC DC5/DC6 counts). debug.s0ix.disp = live,
// debug.s0ix.disphalt = read at the last Deep Idle quiesce action (just before the halt), as u32 values in range order.
struct DispRange { uint32_t from, to; };
static constexpr DispRange kDispRanges[] = {
	{0x44FE0, 0x45040}, {0x45270, 0x45280}, {0x45500, 0x45530}, {0x60000, 0x60080}, {0x61000, 0x61080},
	{0x70000, 0x70600}, {0x71000, 0x71600}, {0x72000, 0x72600}, {0x73000, 0x73600}, {0x101080, 0x101090},
};
static constexpr uint32_t dispRegCount() { uint32_t n = 0; for (auto &r : kDispRanges) n += (r.to - r.from) / 4; return n; }
static volatile uint8_t *gDispMMIO;      // set while the GPU BAR is mapped and in D0 with memory decode (checked at sleep)
static bool gDispOk;                     // onSleep: GPU in D0 with memory decode -> the quiesce read is safe
static uint32_t gDispHalt[dispRegCount()], gDispLive[dispRegCount()];
static bool gDispHaltValid;
static void dispRead(uint32_t *out)
{
	uint32_t n = 0;
	for (auto &r : kDispRanges)
		for (uint32_t o = r.from; o < r.to; o += 4) out[n++] = *reinterpret_cast<volatile uint32_t *>(gDispMMIO + o);
}
static int dispHandler(struct sysctl_oid *, void *arg1, int, struct sysctl_req *req)
{
	if (req->newptr) return EPERM;
	if (!gCoreSnapLock) return ENOENT;
	if (arg1) {   // halt copy
		if (!gDispHaltValid) return ENOENT;
		return SYSCTL_OUT(req, gDispHalt, sizeof(gDispHalt));
	}
	if (!gDispMMIO) return ENOENT;
	IOLockLock(gCoreSnapLock);
	dispRead(gDispLive);
	int err = SYSCTL_OUT(req, gDispLive, sizeof(gDispLive));
	IOLockUnlock(gCoreSnapLock);
	return err;
}
SYSCTL_DECL(_debug);
SYSCTL_NODE(_debug, OID_AUTO, s0ix, CTLFLAG_RW | CTLFLAG_LOCKED, nullptr, "S0ixStandby");
SYSCTL_PROC(_debug_s0ix, OID_AUTO, cores, CTLTYPE_OPAQUE | CTLFLAG_RD | CTLFLAG_LOCKED, nullptr, 0,
            coreSnapHandler, "S", "per-CPU TSC + core C3/C6/C7 + Atom module C6 residency");
SYSCTL_PROC(_debug_s0ix, OID_AUTO, pmc, CTLTYPE_OPAQUE | CTLFLAG_RD | CTLFLAG_LOCKED, nullptr, 0,
            pmcPageHandler, "S", "live PMC page (the LPIT counter's 4 KB MMIO page)");
SYSCTL_PROC(_debug_s0ix, OID_AUTO, disp, CTLTYPE_OPAQUE | CTLFLAG_RD | CTLFLAG_LOCKED, nullptr, 0,
            dispHandler, "S", "live display registers");
SYSCTL_PROC(_debug_s0ix, OID_AUTO, disphalt, CTLTYPE_OPAQUE | CTLFLAG_RD | CTLFLAG_LOCKED, (void *)1, 0,
            dispHandler, "S", "display registers at the last Deep Idle quiesce");

// C-state residency MSRs (Intel SDM vol. 4). All count at TSC rate.
struct CState { const char *name; uint32_t msr; };
static const CState kCStates[] = {
	{ "PC2", 0x60D }, { "PC3", 0x3F8 }, { "PC6", 0x3F9 }, { "PC7", 0x3FA },
	{ "PC8", 0x630 }, { "PC9", 0x631 }, { "PC10", 0x632 },
	// Core C-state MSRs are per core: only read in the quiesce/active actions,
	// which XNU runs on the boot CPU around the Deep Idle halt.
};
static const uint32_t kMsrCoreC6 = 0x3FD, kMsrCoreC7 = 0x3FE;
enum : uint32_t {
	kPC2 = 1 << 0, kPC3 = 1 << 1, kPC6 = 1 << 2, kPC7 = 1 << 3,
	kPC8 = 1 << 4, kPC9 = 1 << 5, kPC10 = 1 << 6,
};

// Reading an MSR the CPU lacks raises #GP (a kernel panic), so only CPUs
// known to have them are measured. Family 6 client models with PC8-PC10
// (Haswell-ULT onwards), per the Intel SDM.
struct CpuModel { uint8_t model; uint32_t mask; };
static const uint32_t kAllUpToTGL = kPC2 | kPC3 | kPC6 | kPC7 | kPC8 | kPC9 | kPC10;
static const uint32_t kNewer      = kPC2 | kPC6 | kPC8 | kPC9 | kPC10;
static const CpuModel kModels[] = {
	{ 0x45, kAllUpToTGL },  // Haswell-ULT
	{ 0x3D, kAllUpToTGL },  // Broadwell-U
	{ 0x4E, kAllUpToTGL }, { 0x5E, kAllUpToTGL },   // Skylake
	{ 0x8E, kAllUpToTGL }, { 0x9E, kAllUpToTGL },   // Kaby/Coffee/Whiskey/Amber Lake
	{ 0x66, kAllUpToTGL },  // Cannon Lake
	{ 0x7D, kAllUpToTGL }, { 0x7E, kAllUpToTGL },   // Ice Lake
	{ 0xA5, kAllUpToTGL }, { 0xA6, kAllUpToTGL },   // Comet Lake
	{ 0x8C, kAllUpToTGL }, { 0x8D, kAllUpToTGL },   // Tiger Lake
	{ 0x97, kNewer }, { 0x9A, kNewer }, { 0xBE, kNewer },   // Alder Lake
	{ 0xB7, kNewer }, { 0xBA, kNewer }, { 0xBF, kNewer },   // Raptor Lake
	{ 0xAA, kNewer }, { 0xAC, kNewer },             // Meteor Lake
};

// MSR_PKG_CST_CONFIG_CONTROL (0xE2), Skylake+ client encoding of bits 3:0.
static const uint32_t kMsrPkgCstConfig = 0xE2;
static const char *pkgLimitName(uint32_t v)
{
	static const char *const names[] = { "C0/C1", "C2", "C3", "C6", "C7", "C7S", "C8", "C9", "C10" };
	return v < 9 ? names[v] : (v == 0xF ? "no limit" : "reserved");
}

// Returns the readable C-state mask for this CPU, 0 if unknown/unsupported.
static uint32_t cstateSupport(uint32_t &modelOut)
{
	uint32_t a, b, c, d;
	cpuid(0, a, b, c, d);
	if (!(b == 0x756e6547 && d == 0x49656e69 && c == 0x6c65746e))   // "GenuineIntel"
		return 0;
	cpuid(1, a, b, c, d);
	uint32_t family = (a >> 8) & 0xF;
	uint32_t model = ((a >> 4) & 0xF) | (((a >> 16) & 0xF) << 4);
	modelOut = model;
	if (family != 6) return 0;
	for (const auto &m : kModels)
		if (m.model == model) return m.mask;
	return 0;
}

// AMD (and Hygon): XNU's Deep Idle halt, pmCPUHalt(PM_HALT_SLEEP), only parks
// the CPU in a wakeable state through a registered CPU power-management
// handler (pmDispatch: Intel XCPM / AppleIntelCPUPowerManagement). Without
// one it stops the CPU with interrupts off (osfmk/i386/pmCPU.c), so the first
// Deep Idle sleep never wakes. Reported on a Ryzen 5 5500U, with and without
// the _DSM calls.
static bool cpuIsAMD()
{
	uint32_t a, b, c, d;
	cpuid(0, a, b, c, d);
	bool amd   = b == 0x68747541 && d == 0x69746e65 && c == 0x444d4163;   // "AuthenticAMD"
	bool hygon = b == 0x6f677948 && d == 0x6e65476e && c == 0x656e6975;   // "HygonGenuine"
	return amd || hygon;
}

static bool bootArg(const char *name)
{
	int dummy = 0;
	return PE_parse_boot_argn(name, &dummy, sizeof(dummy));
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

bool S0ixStandby::start(IOService *provider)
{
	if (bootArg("-s0ixoff")) {
		LOG("disabled by -s0ixoff");
		return false;
	}
	gDebug = bootArg("-s0ixdbg");
	pmcDump = bootArg("-s0ixpmcdump");
	d3Unbound = bootArg("-s0ixd3");
	dsmEnabled = !bootArg("-s0ixnodsm");
	// GPU reset at Deep Idle wake: on by default (Gen11/Gen12 iGPU only,
	// decided in setupGPU); -s0ixnogpureset turns it off. -s0ixgpuprobe adds
	// the full probe (canary, NVRAM record); -s0ixgpureset is accepted for
	// compatibility (same as the default).
	gpuProbe = bootArg("-s0ixgpuprobe");
	gpuReset = !bootArg("-s0ixnogpureset");
	gpuResetForced = bootArg("-s0ixgpureset");
	wifiPM = bootArg("-s0ixwifipm");
	refclkWA = bootArg("-s0ixrefclk");
	acNoRtc = bootArg("-s0ixacnortc");
	ecWake = bootArg("-s0ixecwake");
	pbWake = bootArg("-s0ixpwrbtn");
	lwWake = bootArg("-s0ixlidwake");
	kbWake = bootArg("-s0ixkbdwake");
	edWake = bootArg("-s0ixecdrain");
	iomNoDpof = bootArg("-s0ixnodpof");
	tcssD3 = bootArg("-s0ixtcssd3");
	uint32_t fn = 0;
	if (PE_parse_boot_argn("s0ixfn", &fn, sizeof(fn)))
		fnAllow = fn;

	if (!gLogLock) gLogLock = IOSimpleLockAlloc();
	if (!IOService::start(provider))
		return false;

	if (cpuIsAMD()) {
		// Stay out of the sleep path: no platform actions, no _DSM calls.
		setProperty("Deep Idle", "unsupported on AMD: XNU has no CPU power-management halt handler there, "
		            "the CPU cannot wake from Deep Idle. Remove SSDT-DEEPIDLE.");
		LOG("AMD CPU: Deep Idle cannot wake on this platform (no XNU halt handler); "
		    "not registering sleep/wake actions. Remove SSDT-DEEPIDLE.");
		publishLog();
		registerService();
		return true;
	}

	lps0 = OSDynamicCast(IOACPIPlatformDevice, provider);
	if (!lps0) {
		LOG("provider is not an ACPI device");
		return false;
	}
	lps0->retain();

	if (!probeDsm() && dsmEnabled)
		LOG("no supported LPS0 _DSM found on %s; meter only", lps0->getName());

	bool meter = setupMeter();
	if (!meter)
		LOG("no usable LPIT SLP_S0 counter; residency will not be reported");
	if (pmcDump && counterPage) {
		for (uint32_t i = 0; i < 1024; i++) pmcAtBoot[i] = counterPage[i];
		if (OSData *d = OSData::withBytes(pmcAtBoot, sizeof(pmcAtBoot))) {
			setProperty("PMC page at boot", d);
			d->release();
		}
		char hex[24];
		snprintf(hex, sizeof(hex), "0x%llx", (unsigned long long)(counterAddr & ~0xFFFULL));
		setProperty("PMC page address", hex);
	}

	uint32_t model = 0;
	cstateMask = cstateSupport(model);
	if (bootArg("-s0ixnocst")) cstateMask = 0;
	cpuModel = model;
	setProperty("CPU model", (unsigned long long)model, 32);
	setupBlockerReport(model);
	setProperty("C-state meter", cstateMask != 0);
	if (cstateMask) {
		// Package C-state limit: a value below C10 caps how deep the package
		// can go in Deep Idle, whatever the devices do.
		uint64_t e2 = rdmsr(kMsrPkgCstConfig);
		char buf[96];
		snprintf(buf, sizeof(buf), "0x%llx: limit %s, CFG lock %s, pkg demotion %s/%s",
		         (unsigned long long)e2, pkgLimitName((uint32_t)(e2 & 0xF)),
		         (e2 & (1ULL << 15)) ? "on" : "off",
		         (e2 & (1ULL << 27)) ? "on" : "off", (e2 & (1ULL << 28)) ? "undemote on" : "undemote off");
		setProperty("PKG_CST_CONFIG_CONTROL", buf);
		LOG("MSR 0xE2 %s", buf);
	}

	workLoop = IOWorkLoop::workLoop();
	if (workLoop) {
		publishTimer = IOTimerEventSource::timerEventSource(this, &S0ixStandby::publishTimerFired);
		if (publishTimer && workLoop->addEventSource(publishTimer) != kIOReturnSuccess)
			OSSafeReleaseNULL(publishTimer);
		if (d3Unbound) {
			d3Lock = IOLockAlloc();
			scanTimer = IOTimerEventSource::timerEventSource(this, &S0ixStandby::scanTimerFired);
			if (scanTimer && workLoop->addEventSource(scanTimer) != kIOReturnSuccess)
				OSSafeReleaseNULL(scanTimer);
			// Drivers keep loading after us: scan once boot has settled.
			if (scanTimer) scanTimer->setTimeoutMS(60000);
		}
	}
	setProperty("D3 unbound devices", d3Unbound);

	if (wifiPM && workLoop) {
		// The Wi-Fi driver must be attached first: set up 30 s after boot.
		wifiTimer = IOTimerEventSource::timerEventSource(this, &S0ixStandby::wifiTimerFired);
		if (wifiTimer && workLoop->addEventSource(wifiTimer) != kIOReturnSuccess)
			OSSafeReleaseNULL(wifiTimer);
		if (wifiTimer) wifiTimer->setTimeoutMS(30000);
		setProperty("Wi-Fi PCI PM", "pending (set up 30 s after boot)");
	}

	if (acNoRtc) {
		if (setupRtcBlock() && workLoop) {
			acTimer = IOTimerEventSource::timerEventSource(this, &S0ixStandby::acTimerFired);
			if (acTimer && workLoop->addEventSource(acTimer) != kIOReturnSuccess)
				OSSafeReleaseNULL(acTimer);
			if (acTimer) acTimer->setTimeoutMS(5000);
		}
	} else {
		setProperty("AC RTC wake block", "off (boot-arg -s0ixacnortc)");
	}

	setupWakeInfo();
	setupLidWake();
	setupIom();

	if (workLoop) {
		battTimer = IOTimerEventSource::timerEventSource(this, &S0ixStandby::battTimerFired);
		if (battTimer && workLoop->addEventSource(battTimer) != kIOReturnSuccess)
			OSSafeReleaseNULL(battTimer);
		if (battTimer) battTimer->setTimeoutMS(30000);
		setProperty("Battery full state", "pending (first check 30 s after boot)");
	}

	if ((gpuProbe || gpuReset) && !setupGPU() && gpuTries < 100 && workLoop) {
		// The iGPU nub may not be published yet this early: retry.
		gpuTimer = IOTimerEventSource::timerEventSource(this, &S0ixStandby::gpuTimerFired);
		if (gpuTimer && workLoop->addEventSource(gpuTimer) != kIOReturnSuccess)
			OSSafeReleaseNULL(gpuTimer);
		if (gpuTimer) gpuTimer->setTimeoutMS(10000);
	}

	setProperty("DSM enabled", dsmEnabled);
	{
		char f[16];
		snprintf(f, sizeof(f), "0x%x", fnAllow);
		setProperty("DSM function filter", f);
	}
	setProperty("Sleep count", (unsigned long long)0, 32);

	// Register for the sleep/wake platform action queues. XNU installs these
	// after start() succeeds (IOService.cpp, IOInstallServiceSleepPlatformActions).
	setProperty("IOPlatformSleepAction", (unsigned long long)kActionPriority, 32);
	setProperty("IOPlatformWakeAction", (unsigned long long)kActionPriority, 32);
	if (cstateMask || (pmcDump && counterPage) || gatingPtr || pm1EnPort || gpe0StsPort) {
		// Boot-CPU snapshot right around the Deep Idle halt (MSR/MMIO reads only).
		setProperty("IOPlatformQuiesceAction", (unsigned long long)kActionPriority, 32);
		setProperty("IOPlatformActiveAction", (unsigned long long)kActionPriority, 32);
	}

	if (workLoop && gatingPtr) {
		liveTimer = IOTimerEventSource::timerEventSource(this, &S0ixStandby::liveTimerFired);
		if (liveTimer && workLoop->addEventSource(liveTimer) != kIOReturnSuccess)
			OSSafeReleaseNULL(liveTimer);
		if (liveTimer) liveTimer->setTimeoutMS(10000);
	}

	registerService();
	LOG("started on %s (Intel 0x%x, Microsoft 0x%x, AMD 0x%x, dsm %s)",
	    lps0->getName(), dsmMask[kIntel], dsmMask[kMicrosoft], dsmMask[kAMD],
	    dsmEnabled ? "on" : "off");
	if (!gCoreSnapLock) gCoreSnapLock = IOLockAlloc();
	if (gCoreSnapLock) {
		sysctl_register_oid(&sysctl__debug_s0ix);
		sysctl_register_oid(&sysctl__debug_s0ix_cores);
		sysctl_register_oid(&sysctl__debug_s0ix_pmc);
		sysctl_register_oid(&sysctl__debug_s0ix_disp);
		sysctl_register_oid(&sysctl__debug_s0ix_disphalt);
		gPmcPage = counterPage;
	}
	publishLog();
	return true;
}

void S0ixStandby::stop(IOService *provider)
{
	if (gCoreSnapLock) {
		gPmcPage = nullptr;
		gDispMMIO = nullptr;
		sysctl_unregister_oid(&sysctl__debug_s0ix_disphalt);
		sysctl_unregister_oid(&sysctl__debug_s0ix_disp);
		sysctl_unregister_oid(&sysctl__debug_s0ix_pmc);
		sysctl_unregister_oid(&sysctl__debug_s0ix_cores);
		sysctl_unregister_oid(&sysctl__debug_s0ix);
	}
	if (publishTimer) {
		publishTimer->cancelTimeout();
		if (workLoop) workLoop->removeEventSource(publishTimer);
	}
	if (scanTimer) {
		scanTimer->cancelTimeout();
		if (workLoop) workLoop->removeEventSource(scanTimer);
	}
	if (liveTimer) {
		liveTimer->cancelTimeout();
		if (workLoop) workLoop->removeEventSource(liveTimer);
	}
	if (gpuTimer) {
		gpuTimer->cancelTimeout();
		if (workLoop) workLoop->removeEventSource(gpuTimer);
	}
	if (wifiTimer) {
		wifiTimer->cancelTimeout();
		if (workLoop) workLoop->removeEventSource(wifiTimer);
	}
	if (acTimer) {
		acTimer->cancelTimeout();
		if (workLoop) workLoop->removeEventSource(acTimer);
	}
	if (battTimer) {
		battTimer->cancelTimeout();
		if (workLoop) workLoop->removeEventSource(battTimer);
	}
	IOService::stop(provider);
}

void S0ixStandby::free()
{
	if (IOSimpleLock *l = gLogLock) {
		gLogLock = nullptr;
		IOSimpleLockFree(l);
	}
	OSSafeReleaseNULL(publishTimer);
	OSSafeReleaseNULL(scanTimer);
	OSSafeReleaseNULL(liveTimer);
	OSSafeReleaseNULL(gpuTimer);
	OSSafeReleaseNULL(wifiTimer);
	OSSafeReleaseNULL(wifiDev);
	OSSafeReleaseNULL(acTimer);
	OSSafeReleaseNULL(battTimer);
	OSSafeReleaseNULL(workLoop);
	gpuMMIO = nullptr;
	OSSafeReleaseNULL(gpuMap);
	OSSafeReleaseNULL(gpuDev);
	for (uint32_t i = 0; i < d3Count; i++) OSSafeReleaseNULL(d3Dev[i]);
	d3Count = 0;
	if (d3Lock) { IOLockFree(d3Lock); d3Lock = nullptr; }
	counter = nullptr;
	counterPage = nullptr;
	OSSafeReleaseNULL(counterMap);
	OSSafeReleaseNULL(lps0);
	OSSafeReleaseNULL(ipNames);
	OSSafeReleaseNULL(ipIgnore);
	gatingPtr = nullptr;
	IOService::free();
}

// ---------------------------------------------------------------------------
// LPS0 _DSM
// ---------------------------------------------------------------------------

IOReturn S0ixStandby::callDsm(Family family, uint32_t function, uint64_t *intResult)
{
	uint8_t guid[16];
	if (!lps0 || !guidToBytes(kDsmGuid[family], guid))
		return kIOReturnBadArgument;

	// _DSM(UUID, Revision, Function, Arguments). Revision 0 for all three
	// families [verify against each spec; HP's _DSM ignores Arg1].
	OSData   *uuid = OSData::withBytes(guid, sizeof(guid));
	OSNumber *rev  = OSNumber::withNumber((unsigned long long)0, 32);
	OSNumber *fn   = OSNumber::withNumber((unsigned long long)function, 32);
	OSArray  *args = OSArray::withCapacity(1);
	if (!uuid || !rev || !fn || !args) {
		OSSafeReleaseNULL(uuid); OSSafeReleaseNULL(rev); OSSafeReleaseNULL(fn); OSSafeReleaseNULL(args);
		return kIOReturnNoMemory;
	}

	OSObject *params[4] = { uuid, rev, fn, args };
	OSObject *result = nullptr;
	IOReturn ret = lps0->evaluateObject("_DSM", &result, params, 4);

	if (ret == kIOReturnSuccess && intResult) {
		*intResult = 0;
		if (OSData *d = OSDynamicCast(OSData, result)) {
			uint64_t v = 0;
			unsigned len = d->getLength() < 8 ? d->getLength() : 8;
			const uint8_t *b = static_cast<const uint8_t *>(d->getBytesNoCopy());
			for (unsigned i = 0; b && i < len; i++) v |= (uint64_t)b[i] << (8 * i);
			*intResult = v;
		} else if (OSNumber *n = OSDynamicCast(OSNumber, result)) {
			*intResult = n->unsigned64BitValue();
		}
	}

	OSSafeReleaseNULL(result);
	uuid->release(); rev->release(); fn->release(); args->release();
	DBG("_DSM %s fn %u -> 0x%x", kFamilyName[family], function, ret);
	return ret;
}

bool S0ixStandby::probeDsm()
{
	bool any = false;
	for (uint32_t f = 0; f < kFamilyCount; f++) {
		uint64_t mask = 0;
		if (callDsm(static_cast<Family>(f), kFnQuery, &mask) == kIOReturnSuccess && (mask & 1)) {
			dsmMask[f] = (uint32_t)mask;
			any = true;
		}
		char key[48];
		snprintf(key, sizeof(key), "DSM %s mask", kFamilyName[f]);
		setProperty(key, (unsigned long long)dsmMask[f], 32);
	}
	return any;
}

void S0ixStandby::runIf(Family family, uint32_t function)
{
	if (!(dsmMask[family] & (1U << function)))
		return;
	if (!(fnAllow & (1U << function))) {
		DBG("_DSM %s fn %u skipped (s0ixfn)", kFamilyName[family], function);
		return;
	}
	DBG("_DSM %s fn %u ...", kFamilyName[family], function);
	IOReturn r = callDsm(family, function);
	if (r != kIOReturnSuccess)
		LOG("_DSM %s fn %u failed: 0x%x", kFamilyName[family], function, r);
}

// Order follows the specs (and Linux s2idle): display off, then entry.
void S0ixStandby::dsmEntry()
{
	if (dsmMask[kAMD]) runIf(kAMD, kFnAmdDisplayOff);
	else               runIf(kIntel, kFnDisplayOff);
	runIf(kMicrosoft, kFnDisplayOff);

	if (dsmMask[kAMD]) runIf(kAMD, kFnAmdEntry);
	else               runIf(kIntel, kFnEntry);
	runIf(kMicrosoft, kFnMsSleepEntry);
	runIf(kMicrosoft, kFnEntry);
}

// Exact reverse: exit, then display on.
void S0ixStandby::dsmExit()
{
	runIf(kMicrosoft, kFnExit);
	runIf(kMicrosoft, kFnMsSleepExit);
	if (dsmMask[kAMD]) runIf(kAMD, kFnAmdExit);
	else               runIf(kIntel, kFnExit);

	runIf(kMicrosoft, kFnDisplayOn);
	if (dsmMask[kAMD]) runIf(kAMD, kFnAmdDisplayOn);
	else               runIf(kIntel, kFnDisplayOn);
}

// ---------------------------------------------------------------------------
// LPIT SLP_S0 residency meter
// ---------------------------------------------------------------------------

// IOACPIPlatformExpert::getACPITableData() is protected. A derived type may
// form a pointer to it; calling through that pointer dispatches to the real
// (AppleACPIPlatformExpert) implementation. Never instantiated.
struct ACPITableAccess : public IOACPIPlatformExpert {
	static const OSData *get(IOACPIPlatformExpert *pe, const char *name, UInt32 instance) {
		auto fn = &ACPITableAccess::getACPITableData;
		return (pe->*fn)(name, instance);
	}
};

bool S0ixStandby::setupMeter()
{
	// Preferred: ask the ACPI platform expert for the table. Fallback: the
	// "ACPI Tables" registry property some macOS versions publish.
	const OSData *lpit = nullptr;
	const char *source = "none";
	IOService *pe = getPlatform();
	if (IOACPIPlatformExpert *acpi = OSDynamicCast(IOACPIPlatformExpert, pe)) {
		lpit = ACPITableAccess::get(acpi, "LPIT", 0);
		if (lpit) source = "getACPITableData";
	}
	if (!lpit && pe) {
		if (OSDictionary *tables = OSDynamicCast(OSDictionary, pe->getProperty("ACPI Tables"))) {
			lpit = OSDynamicCast(OSData, tables->getObject("LPIT"));
			if (lpit) source = "ACPI Tables property";
		}
	}
	setProperty("LPIT source", source);
	if (!lpit || lpit->getLength() < 36) {
		setProperty("LPIT status", lpit ? "too short" : "not found");
		return false;
	}

	const uint8_t *t = static_cast<const uint8_t *>(lpit->getBytesNoCopy());
	uint32_t tableLen = OSReadLittleInt32(t, 4);
	if (tableLen > lpit->getLength()) tableLen = lpit->getLength();

	// LPI descriptors follow the 36-byte header. Native C-state descriptor:
	//   +0 type(4) +4 length(4) +8 uid(2) +12 flags(4) +16 entry trigger GAS(12)
	//   +28 residency(4) +32 latency(4) +36 residency counter GAS(12) +48 frequency(8)
	for (uint32_t off = 36; off + 56 <= tableLen; ) {
		const uint8_t *d = t + off;
		uint32_t type = OSReadLittleInt32(d, 0), len = OSReadLittleInt32(d, 4);
		if (len < 8 || off + len > tableLen) break;
		uint32_t flags = OSReadLittleInt32(d, 12);
		uint8_t space = d[36], bits = d[37];
		uint64_t addr = OSReadLittleInt64(d, 40), freq = OSReadLittleInt64(d, 48);
		bool disabled = flags & 1, noCounter = flags & 2;

		DBG("LPIT desc type %u flags 0x%x space %u bits %u addr 0x%llx freq %llu",
		    type, flags, space, bits, addr, freq);

		// Prefer the last enabled descriptor with a SystemMemory counter: on
		// Intel client platforms that is the SLP_S0 (S0ix) residency counter.
		if (type == 0 && !disabled && !noCounter && space == 0 && addr && (bits == 32 || bits == 64)) {
			// Map the whole 4 KB page (same MMIO page; lets -s0ixpmcdump read
			// its neighbours too). The counter must not straddle the page.
			uint64_t page = addr & ~0xFFFULL, pageOff = addr & 0xFFF;
			IOMemoryDescriptor *md = (pageOff + bits / 8 <= 0x1000)
				? IOMemoryDescriptor::withPhysicalAddress(page, 0x1000, kIODirectionIn) : nullptr;
			if (md) {
				IOMemoryMap *map = md->map(kIOMapInhibitCache);
				md->release();
				if (map) {
					OSSafeReleaseNULL(counterMap);
					counterMap = map;
					counterPage = reinterpret_cast<volatile uint32_t *>(map->getVirtualAddress());
					counter = reinterpret_cast<volatile uint32_t *>(map->getVirtualAddress() + pageOff);
					counterBits = bits;
					counterFreq = freq;
					counterAddr = addr;
				}
			}
		}
		off += len;
	}

	if (!counter) {
		setProperty("LPIT status", "no usable SystemMemory residency counter");
		return false;
	}
	setProperty("LPIT status", "ok");

	// The LPIT physical address (the map's getPhysicalAddress() would return
	// an IOMMU-translated address when VT-d is active).
	char hex[24];
	snprintf(hex, sizeof(hex), "0x%llx", (unsigned long long)counterAddr);
	setProperty("Residency counter address", hex);
	setProperty("Residency counter frequency", (unsigned long long)counterFreq, 64);
	uint64_t v = 0;
	readCounter(v);
	setProperty("Residency counter at start", (unsigned long long)v, 64);
	LOG("LPIT residency counter at 0x%llx, %u-bit, %llu Hz, now %llu",
	    (unsigned long long)counterAddr, counterBits, counterFreq, v);
	return true;
}

bool S0ixStandby::readCounter(uint64_t &value)
{
	if (!counter) return false;
	if (counterBits == 64) {
		uint32_t lo, hi, hi2;
		do { hi = counter[1]; lo = counter[0]; hi2 = counter[1]; } while (hi != hi2);
		value = ((uint64_t)hi << 32) | lo;
	} else {
		value = counter[0];
	}
	return true;
}

// ---------------------------------------------------------------------------
// SLP_S0 blocker report
// ---------------------------------------------------------------------------

// Fabric/infrastructure IPs that are normally still powered at the moment the
// CPU halts; everything else still powered there is a likely S0ix blocker.
static bool isInfrastructureIP(const char *n)
{
	static const char *const prefixes[] = { "PMC", "OPI", "SBR", "PSF", "CSME", "NPK_AON", "RES_" };
	for (const char *p : prefixes) {
		size_t l = strlen(p);
		if (!strncmp(n, p, l)) return true;
	}
	return false;
}

void S0ixStandby::setupBlockerReport(uint32_t cpuModel)
{
	setProperty("SLP_S0 blocker report", "unavailable");
	if (!counterPage || !counterAddr) return;
	OSDictionary *all = OSDynamicCast(OSDictionary, getProperty("S0ixPlatforms"));
	if (!all) return;
	char key[16];
	snprintf(key, sizeof(key), "cpu-%x", cpuModel);
	OSDictionary *pf = OSDynamicCast(OSDictionary, all->getObject(key));
	if (!pf) return;
	OSNumber *off = OSDynamicCast(OSNumber, pf->getObject("Gating offset from SLP_S0 counter"));
	OSNumber *len = OSDynamicCast(OSNumber, pf->getObject("Gating bytes"));
	OSArray *names = OSDynamicCast(OSArray, pf->getObject("IP names"));
	if (!off || !len || !names) return;

	uint64_t reg = counterAddr + off->unsigned64BitValue();
	uint32_t bytes = len->unsigned32BitValue();
	uint64_t page = counterAddr & ~0xFFFULL;
	// Must be dword aligned, fit our buffer and lie in the page already mapped.
	if ((reg & 3) || bytes == 0 || bytes > sizeof(gatingAtHalt) || reg < page || reg + bytes > page + 0x1000)
		return;
	gatingPtr = counterPage + (reg - page) / 4;
	gatingBytes = bytes;
	ipNames = names;
	ipNames->retain();
	if (OSArray *ign = OSDynamicCast(OSArray, pf->getObject("Non-blocking IPs"))) {
		ipIgnore = ign;
		ipIgnore->retain();
	}
	if (OSString *n = OSDynamicCast(OSString, pf->getObject("Name")))
		setProperty("SLP_S0 blocker report", n);
}

OSArray *S0ixStandby::blockerNames(const uint8_t *b, char *line, size_t lineSize)
{
	OSArray *blockers = OSArray::withCapacity(8);
	size_t n = 0;
	if (lineSize) line[0] = 0;
	for (uint32_t i = 0; i < gatingBytes * 8 && i < ipNames->getCount(); i++) {
		bool gated = (b[i / 8] >> (i % 8)) & 1;
		OSString *name = OSDynamicCast(OSString, ipNames->getObject(i));
		if (gated || !name || isInfrastructureIP(name->getCStringNoCopy()))
			continue;
		bool ignored = false;
		for (unsigned j = 0; ipIgnore && j < ipIgnore->getCount(); j++)
			if (OSString *ig = OSDynamicCast(OSString, ipIgnore->getObject(j)))
				if (ig->isEqualTo(name)) { ignored = true; break; }
		if (ignored)
			continue;
		if (blockers) blockers->setObject(name);
		if (n < lineSize)
			n += snprintf(line + n, lineSize - n, " %s", name->getCStringNoCopy());
	}
	return blockers;
}

void S0ixStandby::liveTimerFired(OSObject *owner, IOTimerEventSource *sender)
{
	if (S0ixStandby *self = OSDynamicCast(S0ixStandby, owner)) {
		self->publishLiveBlockers();
		sender->setTimeoutMS(10000);
	}
}

// Same report while awake (every 10 s). Many IPs are legitimately on while
// the system runs; useful to see whether a given IP ever powers down.
void S0ixStandby::publishLiveBlockers()
{
	if (!gatingPtr || !ipNames) return;
	uint32_t now[8] {};
	for (uint32_t i = 0; i < (gatingBytes + 3) / 4; i++) now[i] = gatingPtr[i];
	char line[256];
	if (OSArray *a = blockerNames(reinterpret_cast<const uint8_t *>(now), line, sizeof(line))) {
		setProperty("Live SLP_S0 blockers", a);
		a->release();
	}
}

void S0ixStandby::publishBlockers()
{
	if (!snap.gatingValid || !ipNames) return;
	snap.gatingValid = false;
	const uint8_t *b = reinterpret_cast<const uint8_t *>(snap.gatingAtHalt);
	char line[256];
	size_t n = 0;
	if (OSArray *blockers = blockerNames(b, line, sizeof(line))) {
		n = blockers->getCount();
		setProperty("Last SLP_S0 blockers at halt", blockers);
		blockers->release();
	}
	char raw[80]; size_t r = 0; raw[0] = 0;
	for (uint32_t i = 0; i < gatingBytes && r < sizeof(raw); i++)
		r += snprintf(raw + r, sizeof(raw) - r, "%02x", b[i]);
	setProperty("Last gating status at halt", raw);
	LOG("IPs still powered at the Deep Idle halt (likely S0ix blockers):%s", n ? line : " none");
}

// ---------------------------------------------------------------------------
// Driverless PCI endpoints -> D3hot during Deep Idle (-s0ixd3)
// ---------------------------------------------------------------------------

void S0ixStandby::scanTimerFired(OSObject *owner, IOTimerEventSource *)
{
	if (S0ixStandby *self = OSDynamicCast(S0ixStandby, owner))
		self->scanUnbound();
}

void S0ixStandby::scanUnbound()
{
	if (!d3Unbound || !d3Lock) return;
	OSDictionary *match = serviceMatching("IOPCIDevice");
	if (!match) return;
	OSIterator *it = getMatchingServices(match);
	match->release();
	if (!it) return;

	IOPCIDevice *found[kMaxD3] {};
	UInt8 cap[kMaxD3] {};
	uint32_t n = 0;
	OSArray *names = OSArray::withCapacity(kMaxD3);

	while (OSObject *o = it->getNextObject()) {
		IOPCIDevice *dev = OSDynamicCast(IOPCIDevice, o);
		if (!dev)
			continue;
		uint32_t cls = dev->configRead32(kIOPCIConfigClassCode) >> 8;   // class/subclass/prog-if
		uint8_t base = (cls >> 16) & 0xFF;
		if (dev->getClient())                         // has a driver (or a nub)
			continue;
		if (base == 0x06 || base == 0x03 || base == 0x05)   // bridges, display, memory: never
			continue;
		UInt8 pm = 0;
		if (!dev->findPCICapability(kIOPCIPowerManagementCapability, &pm) || !pm)
			continue;
		if (n >= kMaxD3) break;
		dev->retain();
		found[n] = dev;
		cap[n] = pm;
		n++;
		if (names) {
			char buf[48];
			uint32_t id = dev->configRead32(kIOPCIConfigVendorID);
			snprintf(buf, sizeof(buf), "%02x:%02x.%x %04x:%04x class %06x",
			         dev->getBusNumber(), dev->getDeviceNumber(), dev->getFunctionNumber(),
			         id & 0xFFFF, id >> 16, cls);
			if (OSString *str = OSString::withCString(buf)) { names->setObject(str); str->release(); }
		}
	}
	it->release();

	IOLockLock(d3Lock);
	for (uint32_t i = 0; i < d3Count; i++) OSSafeReleaseNULL(d3Dev[i]);
	for (uint32_t i = 0; i < n; i++) { d3Dev[i] = found[i]; d3Cap[i] = cap[i]; }
	d3Count = n;
	IOLockUnlock(d3Lock);

	if (names) { setProperty("D3 devices", names); names->release(); }
	LOG("%u driverless PCI endpoint(s) put in D3hot", n);

	// They have no driver, so nothing uses them: keep them in D3hot while
	// awake too. Gives firmware-managed blocks (e.g. ISH) time to power down
	// before the next sleep.
	IOLockLock(d3Lock);
	for (uint32_t i = 0; i < d3Count; i++) {
		UInt16 pmcsr = d3Dev[i]->configRead16(d3Cap[i] + 4);
		if ((pmcsr & 3) == 0)
			d3Dev[i]->configWrite16(d3Cap[i] + 4, (UInt16)((pmcsr & ~3) | 3));
	}
	IOLockUnlock(d3Lock);
}

void S0ixStandby::wifiTimerFired(OSObject *owner, IOTimerEventSource *)
{
	if (S0ixStandby *self = OSDynamicCast(S0ixStandby, owner))
		if (!self->setupWifiPM() && ++self->wifiTries < 6 && self->wifiTimer)
			self->wifiTimer->setTimeoutMS(10000);
}

// Wi-Fi = PCI class 0x0280 (network, other) with a driver attached.
bool S0ixStandby::setupWifiPM()
{
	OSDictionary *match = serviceMatching("IOPCIDevice");
	if (!match) return false;
	OSIterator *it = getMatchingServices(match);
	match->release();
	if (!it) return false;
	IOPCIDevice *found = nullptr;
	while (OSObject *o = it->getNextObject()) {
		IOPCIDevice *dev = OSDynamicCast(IOPCIDevice, o);
		if (!dev || (dev->configRead32(kIOPCIConfigClassCode) >> 16) != 0x0280 || !dev->getClient())
			continue;
		found = dev;
		found->retain();
		break;
	}
	it->release();
	if (!found) { setProperty("Wi-Fi PCI PM", "no Wi-Fi function with a driver yet"); return false; }

	UInt8 pm = 0;
	found->findPCICapability(kIOPCIPowerManagementCapability, &pm);
	IOReturn ret = found->enablePCIPowerManagement(kPCIPMCSPowerStateD3);
	char buf[96];
	uint32_t id = found->configRead32(kIOPCIConfigVendorID);
	snprintf(buf, sizeof(buf), "%02x:%02x.%x %04x:%04x: D3 for sleep %s (0x%x)",
	         found->getBusNumber(), found->getDeviceNumber(), found->getFunctionNumber(),
	         id & 0xFFFF, id >> 16, ret == kIOReturnSuccess ? "enabled" : "refused", ret);
	setProperty("Wi-Fi PCI PM", buf);
	LOG("Wi-Fi %s", buf);
	if (ret != kIOReturnSuccess || !pm) { found->release(); return true; }   // done, nothing to sample
	wifiDev = found;
	wifiPmCap = pm;
	return true;
}

// PMCSR is at capability + 4; bits 1:0 = power state (0 = D0, 3 = D3hot).
// Re-asserts D3hot before each Deep Idle (in case firmware brought a device
// back to D0). Devices stay in D3hot after wake: they have no driver.
void S0ixStandby::d3Enter()
{
	if (!d3Unbound || !d3Lock) return;
	IOLockLock(d3Lock);
	for (uint32_t i = 0; i < d3Count; i++) {
		UInt16 pmcsr = d3Dev[i]->configRead16(d3Cap[i] + 4);
		if ((pmcsr & 3) == 0)
			d3Dev[i]->configWrite16(d3Cap[i] + 4, (UInt16)((pmcsr & ~3) | 3));
	}
	IOLockUnlock(d3Lock);
}


// ---------------------------------------------------------------------------
// Sleep / wake
// ---------------------------------------------------------------------------

void S0ixStandby::readHalt(uint64_t *out)
{
	out[0] = rdmsr(kMsrCoreC6);
	out[1] = rdmsr(kMsrCoreC7);
	readCStates(out + 2);
}

void S0ixStandby::readCStates(uint64_t *out)
{
	for (uint32_t i = 0; i < kCStateCount; i++)
		out[i] = (cstateMask & (1U << i)) ? rdmsr(kCStates[i].msr) : 0;
}

IOReturn S0ixStandby::callPlatformFunction(const OSSymbol *functionName, bool waitForFunction,
                                           void *param1, void *param2, void *param3, void *param4)
{
	if (functionName) {
		if (functionName->isEqualTo("IOPlatformSleepAction")) { onSleep(); return kIOReturnSuccess; }
		if (functionName->isEqualTo("IOPlatformWakeAction"))  { onWake();  return kIOReturnSuccess; }
		// Interrupts are disabled in these two: MSR reads only, no locks/logging.
		if (functionName->isEqualTo("IOPlatformQuiesceAction")) {
			if (rtcBlocked) outw(pm1EnPort, (uint16_t)(inw(pm1EnPort) & ~kPm1RtcEn));
			if (kbArm) kbArmResult = kbcArm(kbCfgBefore, kbCfgSet);
			kbRteUnmasked = false;
			if (kbArm && kbArmResult == 1 && ioapic) {
				ioapic[0] = 0x12; kbRteLow = ioapic[4];            // redirection entry for GSI 1 (IRQ1), low dword
				ioapic[0] = 0x13; kbRteHigh = ioapic[4];
				if (kbRteLow & 0x10000) {                          // masked by the PS/2 driver for sleep
					ioapic[0] = 0x12; ioapic[4] = kbRteLow & ~0x10000u;
					kbRteUnmasked = true;
				}
			}
			if (lwArm && lwGpe != 0xFFFFFFFF) lwEnReadback = gpeEnSet(gpe0EnPort, lwGpe);
			lwGpioArmed = false;
			if (lwArm && lwGpeEn) {
				lwGpeEnSaved = *lwGpeEn;
				*lwGpeSts = lwGpioBit;                             // clear a stale status (write 1)
				*lwGpeEn = lwGpeEnSaved | lwGpioBit;
				lwEnReadback = *lwGpeEn;
				lwGpioArmed = true;
			}
			if (pbArm) {
				// The laptop reports its power button through the EC (FADT control-method button), so the OS
				// never enables the PCH's fixed PWRBTN event; the EC is in standby and does not deliver the
				// press. The PCH still latches PWRBTN_STS: enable it for the halt only.
				outw(pm1StsPort, kPm1PwrBtn);                               // drop a stale press (write 1 to clear)
				outw(pbEnPort, (uint16_t)(inw(pbEnPort) | kPm1PwrBtn));
				pbArmed = true;
			}
			if (edWake && lastSleepType == kSleepTypeDeepIdle) {
				ecStsAtHalt = inb(0x66);
				edGpeStsAtHalt = (gpe0StsPort && ecGpe != 0xFFFFFFFF) ? ((inl((uint16_t)(gpe0StsPort + (ecGpe / 32) * 4)) >> (ecGpe % 32)) & 1) : false;
				edN = 0;
				for (int i = 0; i < 8 && (inb(0x66) & 0x20); i++) {   // SCI_EVT: a query is pending
					if (!ecWait(0x02, 0)) break;
					outb(0x66, 0x84);                                      // QR_EC
					if (!ecWait(0x01, 0x01)) break;
					uint8_t q = inb(0x62);
					if (q && edN < sizeof(edQ)) edQ[edN++] = q;
				}
				if (gpe0StsPort && ecGpe != 0xFFFFFFFF)                // clear the EC GPE status so a new SCI edge can wake us
					outl((uint16_t)(gpe0StsPort + (ecGpe / 32) * 4), 1u << (ecGpe % 32));
				pm1CntAtHalt = pm1CntPort ? inw(pm1CntPort) : 0;
				sciUnmasked = false;
				if (ioapic) {                                          // SCI = GSI 9 (MADT override: level, active high)
					ioapic[0] = 0x22; sciRteLow = ioapic[4];
					ioapic[0] = 0x23; sciRteHigh = ioapic[4];
					if (sciRteLow & 0x10000) { ioapic[0] = 0x22; ioapic[4] = sciRteLow & ~0x10000u; sciUnmasked = true; }
				}
			}
			if (ecArm) {
				// Last thing before the halt: macOS has set up its wake GPEs.
				uint16_t p = (uint16_t)(gpe0EnPort + (ecGpe / 32) * 4);
				uint32_t v = inl(p);
				ecEnAtHalt = (v >> (ecGpe % 32)) & 1;
				ecEnReadback = gpeEnSet(gpe0EnPort, ecGpe);
				if (!ecEnAtHalt) ecArmed = true;
			} else if (gpe0EnPort && ecGpe != 0xFFFFFFFF) {
				ecEnAtHalt = (inb((uint16_t)(gpe0EnPort + ecGpe / 8)) >> (ecGpe % 8)) & 1;
			}
			if (gpe0EnPort)
				for (uint32_t i = 0; i + 4 <= gpe0Half && i + 4 <= sizeof(gpe0EnAtHalt); i += 4) {
					uint32_t d = inl((uint16_t)(gpe0EnPort + i));
					for (uint32_t b = 0; b < 4; b++) gpe0EnAtHalt[i + b] = (uint8_t)(d >> (8 * b));
				}
			if (gDispOk && gDispMMIO) { dispRead(gDispHalt); gDispHaltValid = true; }
			if (cstateMask) { readHalt(haltAtQuiesce); tscAtQuiesce = rdtsc(); haltValid = false; }
			if (pmcDump && counterPage) {
				for (uint32_t i = 0; i < 1024; i++) pmcAtHalt[i] = counterPage[i];
				pmcHaltValid = true;
			}
			if (gatingPtr) {
				for (uint32_t i = 0; i < (gatingBytes + 3) / 4; i++) gatingAtHalt[i] = gatingPtr[i];
				gatingValid = true;
			}
			return kIOReturnSuccess;
		}
		if (functionName->isEqualTo("IOPlatformActiveAction")) {
			// Right after the halt: the PMC's latched LPM status (latched at
			// package C10 entry by default) shows the state during the sleep.
			if (cstateMask && tscAtQuiesce) { readHalt(haltAtActive); tscAtActive = rdtsc(); haltValid = true; }
			if (gpe0StsPort) {
				// What woke the CPU: status bits are still set here (cleared later by ACPI).
				pm1StsAtWake = pm1StsPort ? inw(pm1StsPort) : 0;
				if (edWake) {
					ecStsAtWakeP = inb(0x66);
					ecEnAtWake = (gpe0EnPort && ecGpe != 0xFFFFFFFF) ? inl((uint16_t)(gpe0EnPort + (ecGpe / 32) * 4)) : 0;
					if (sciUnmasked && ioapic) { ioapic[0] = 0x22; ioapic[4] = sciRteLow; sciUnmasked = false; }
				}
				if (pbArmed) {
					outw(pbEnPort, (uint16_t)(inw(pbEnPort) & ~kPm1PwrBtn));   // never leave it on: nobody handles it awake
					if (pm1StsAtWake & kPm1PwrBtn) { outw(pm1StsPort, kPm1PwrBtn); pbWoke = true; }
					pbArmed = false;
				}
				for (uint32_t i = 0; i < gpe0Half && i < sizeof(gpe0StsAtWake); i++)
					gpe0StsAtWake[i] = inb((uint16_t)(gpe0StsPort + i));
				wakeInfoValid = true;
				if (lwArm && lwGpe != 0xFFFFFFFF && (gpe0StsAtWake[lwGpe / 8] >> (lwGpe % 8) & 1)) lwWoke = true;
				if (lwGpioArmed) {
					lwGpeStsAtWake = *lwGpeSts;
					if (lwGpeStsAtWake & lwGpioBit) lwWoke = true;
					*lwGpeEn = lwGpeEnSaved;                          // restore; ACPI clears the status when it handles _L17
					lwGpioArmed = false;
				}
				if (kbArm) {
					kbStsAtWake = inb(0x64);
					if (kbRteUnmasked && ioapic) { ioapic[0] = 0x12; ioapic[4] = kbRteLow; kbRteUnmasked = false; }
					if (kbArmResult == 1 && (kbStsAtWake & 0x21) == 0x01) kbWoke = true;   // keyboard byte waiting
				}
			}
			if (pmcDump && counterPage && pmcHaltValid) {
				for (uint32_t i = 0; i < 1024; i++) pmcAtActive[i] = counterPage[i];
				pmcActiveValid = true;
			}
			return kIOReturnSuccess;
		}
	}
	return IOService::callPlatformFunction(functionName, waitForFunction, param1, param2, param3, param4);
}

// ---------------------------------------------------------------------------
// -s0ixacnortc: no RTC (alarm) wakes from Deep Idle while on AC
// ---------------------------------------------------------------------------
// On AC macOS schedules a maintenance dark wake every ~5 min (battery
// management); on some machines one of those re-sleeps eventually hangs
// (FINDINGS §12). RTC alarms reach the CPU only through the ACPI fixed event
// RTC_EN (PM1_EN bit 10). Clearing it at the Deep Idle sleep action, and
// restoring it first thing at wake, lets the alarm be programmed but never
// wake the machine; power button, lid and other GPEs are unaffected.

bool S0ixStandby::setupRtcBlock()
{
	const OSData *fadt = nullptr;
	if (IOACPIPlatformExpert *acpi = OSDynamicCast(IOACPIPlatformExpert, getPlatform()))
		fadt = ACPITableAccess::get(acpi, "FACP", 0);
	if (!fadt && getPlatform())
		if (OSDictionary *tables = OSDynamicCast(OSDictionary, getPlatform()->getProperty("ACPI Tables")))
			fadt = OSDynamicCast(OSData, tables->getObject("FACP"));
	if (!fadt || fadt->getLength() < 89) {
		setProperty("AC RTC wake block", "unavailable: no FADT");
		return false;
	}
	const uint8_t *f = static_cast<const uint8_t *>(fadt->getBytesNoCopy());
	uint32_t len = fadt->getLength();
	uint64_t base = OSReadLittleInt32(f, 56);                 // PM1a_EVT_BLK
	uint8_t evtLen = f[88];                                     // PM1_EVT_LEN
	if (len >= 160 && f[148] == 1 && OSReadLittleInt64(f, 152))  // X_PM1a_EVT_BLK, SystemIO
		base = OSReadLittleInt64(f, 152);
	uint64_t port = base + evtLen / 2;
	char buf[96];
	if (!base || evtLen < 4 || port > 0xFFFE) {
		snprintf(buf, sizeof(buf), "unavailable: PM1a event block 0x%llx, length %u",
		         (unsigned long long)base, evtLen);
		setProperty("AC RTC wake block", buf);
		return false;
	}
	pm1EnPort = (uint16_t)port;
	snprintf(buf, sizeof(buf), "armed: PM1_EN at I/O 0x%x (RTC_EN now %s)", pm1EnPort,
	         (inw(pm1EnPort) & kPm1RtcEn) ? "set" : "clear");
	setProperty("AC RTC wake block", buf);
	LOG("AC RTC wake block %s", buf);
	return true;
}

// FADT event blocks and the EC's GPE: lets every sleep report what woke it
// (read-only), and -s0ixecwake keep the EC GPE armed during Deep Idle.
void S0ixStandby::setupWakeInfo()
{
	const OSData *fadt = nullptr;
	if (IOACPIPlatformExpert *acpi = OSDynamicCast(IOACPIPlatformExpert, getPlatform()))
		fadt = ACPITableAccess::get(acpi, "FACP", 0);
	if (!fadt && getPlatform())
		if (OSDictionary *tables = OSDynamicCast(OSDictionary, getPlatform()->getProperty("ACPI Tables")))
			fadt = OSDynamicCast(OSData, tables->getObject("FACP"));
	if (!fadt || fadt->getLength() < 93) {
		setProperty("EC wake", "unavailable: no FADT");
		return;
	}
	const uint8_t *f = static_cast<const uint8_t *>(fadt->getBytesNoCopy());
	uint32_t len = fadt->getLength();
	uint64_t pm1 = OSReadLittleInt32(f, 56);                     // PM1a_EVT_BLK
	if (len >= 160 && f[148] == 1 && OSReadLittleInt64(f, 152))   // X_PM1a_EVT_BLK, SystemIO
		pm1 = OSReadLittleInt64(f, 152);
	uint64_t gpe = OSReadLittleInt32(f, 80);                     // GPE0_BLK
	uint8_t gpeLen = f[92];                                       // GPE0_BLK_LEN
	if (len >= 232 && f[220] == 1 && OSReadLittleInt64(f, 224))   // X_GPE0_BLK, SystemIO
		gpe = OSReadLittleInt64(f, 224);
	if (pm1 && pm1 < 0xFFFF) pm1StsPort = (uint16_t)pm1;
	{ uint32_t cnt = OSReadLittleInt32(f, 64); if (cnt && cnt < 0xFFFF) pm1CntPort = (uint16_t)cnt; }   // PM1a_CNT_BLK
	{
		uint8_t evtLen = f[88];                                   // PM1_EVT_LEN
		if (pm1StsPort && evtLen >= 4 && pm1 + evtLen / 2 < 0xFFFF) pbEnPort = (uint16_t)(pm1 + evtLen / 2);
		char pb[96];
		if (!pbWake) snprintf(pb, sizeof(pb), "off (boot-arg -s0ixpwrbtn)");
		else if (pbEnPort) snprintf(pb, sizeof(pb), "armed during Deep Idle (-s0ixpwrbtn): PM1_EN at I/O 0x%x bit 8", pbEnPort);
		else snprintf(pb, sizeof(pb), "unavailable: no PM1 event block");
		setProperty("Power button wake", pb);
		LOG("Power button wake %s", pb);
		if (pbWake && pbEnPort && !pbCall) pbCall = thread_call_allocate(pbUserActive, this);
		if (edWake && !edCall) edCall = thread_call_allocate(edReplay, this);
		setProperty("EC drain", edWake ? "on (-s0ixecdrain)" : "off (boot-arg -s0ixecdrain)");
	}
	if (gpe && gpe + gpeLen <= 0xFFFF && gpeLen >= 2 && !(gpeLen & 1)) {
		gpe0StsPort = (uint16_t)gpe;
		gpe0Half = gpeLen / 2;
		gpe0EnPort = (uint16_t)(gpe + gpe0Half);
	}

	findEcGpe();

	publishEcWake();
}

// The EC's GPE number: s0ixecgpe=N overrides; otherwise _GPE of the PNP0C09
// device, searched in the ACPI plane (the EC nub isn't always registered yet,
// or at all). An integer is a GPE0 bit; a package means a GPE block device.
// i8042 at the quiesce action (interrupts off, the PS/2 driver has already put the controller to sleep with the
// keyboard clock and IRQ disabled): turn the keyboard interface and its interrupt back on so a key press raises
// IRQ1 and ends the halt. The PS/2 driver re-initialises the controller on wake.
static bool kbcWait(uint8_t mask, uint8_t want)
{
	for (int i = 0; i < 2000; i++) {                      // up to ~20 ms
		if ((inb(0x64) & mask) == want) return true;
		IODelay(10);
	}
	return false;
}
static uint8_t kbcArm(uint8_t &before, uint8_t &after)
{
	if (inb(0x64) == 0xFF) return 2;                      // no controller
	if (!kbcWait(0x02, 0)) return 3;
	outb(0x64, 0xAE);                                     // enable the keyboard interface
	for (int i = 0; i < 16 && (inb(0x64) & 0x01); i++) { (void)inb(0x60); IODelay(50); }   // drop stale bytes
	if (!kbcWait(0x02, 0)) return 3;
	outb(0x64, 0x20);                                     // read the command byte
	if (!kbcWait(0x01, 0x01)) return 4;
	before = inb(0x60);
	after = (uint8_t)((before | 0x01) & ~0x10);           // keyboard IRQ on, keyboard clock not disabled
	if (!kbcWait(0x02, 0)) return 3;
	outb(0x64, 0x60);
	if (!kbcWait(0x02, 0)) return 3;
	outb(0x60, after);
	return 1;
}

// Lid (PNP0C0D) and sleep button (PNP0C0E): on ThinkPads their _PSW sets the EC's wake-on-lid-open / wake-on-Fn bits
// and _PRW names the EC's wake GPE. macOS does not do this for Deep Idle, so the lid never wakes the machine.
void S0ixStandby::publishTypeC()
{
	static const uint8_t devs[][2] = { {0x0d, 0}, {0x0d, 2}, {0x0d, 3}, {0x07, 0}, {0x07, 1}, {0x07, 2}, {0x07, 3} };
	char pci[200]; size_t n = 0;
	char ports[400]; size_t pn = 0;
	ports[0] = 0;
	OSDictionary *match = serviceMatching("IOPCIDevice");
	OSIterator *it = match ? getMatchingServices(match) : nullptr;
	OSSafeReleaseNULL(match);
	if (!it) return;
	while (OSObject *o = it->getNextObject()) {
		IOPCIDevice *d = OSDynamicCast(IOPCIDevice, o);
		if (!d || d->getBusNumber() != 0) continue;
		for (auto &w : devs) {
			if (d->getDeviceNumber() != w[0] || d->getFunctionNumber() != w[1]) continue;
			uint8_t pm = 0;
			uint32_t ps = 0xFF;
			if (d->findPCICapability(kIOPCIPowerManagementCapability, &pm) && pm) ps = d->configRead16(pm + 4) & 3;
			n += snprintf(pci + n, sizeof(pci) - n, "%s%x.%x D%u", n ? ", " : "", w[0], w[1], ps);
			if (w[0] == 0x0d && w[1] == 0) {
				if (!txhcDev) { d->retain(); txhcDev = d; txhcPm = pm; }
				if (!txhcMap) txhcMap = d->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0, kIOMapInhibitCache);
				if (txhcMap && ps == 0) {
					volatile uint8_t *b = reinterpret_cast<volatile uint8_t *>(txhcMap->getVirtualAddress());
					uint32_t capLen = b[0], maxPorts = (*reinterpret_cast<volatile uint32_t *>(b + 4)) >> 24;
					for (uint32_t p = 0; p < maxPorts && p < 16; p++) {
						uint32_t sc = *reinterpret_cast<volatile uint32_t *>(b + capLen + 0x400 + 0x10 * p);
						// CCS b0, PED b1, PR b4, PLS b8:5, PP b9, speed b13:10
						pn += snprintf(ports + pn, sizeof(ports) - pn, "%sP%u %08x(ccs%u ped%u pls%u pp%u spd%u)", pn ? " " : "",
						               p + 1, sc, sc & 1, (sc >> 1) & 1, (sc >> 5) & 0xF, (sc >> 9) & 1, (sc >> 10) & 0xF);
					}
				} else if (ps != 0) {
					snprintf(ports, sizeof(ports), "controller not in D0");
				}
			}
		}
	}
	it->release();
	setProperty("Type-C PCI", pci);
	setProperty("TXHC PORTSC", ports);
	// MSR_SMI_COUNT (0x34): SMIs since reset - hidden firmware (SMM) time shows up as core power at OS idle.
	uint64_t smi = rdmsr(0x34) & 0xffffffffULL, tsc = rdtsc();
	char sb[96];
	if (smiTscLast && tsc > smiTscLast) {
		uint64_t dsmi = smi - smiLast;
		snprintf(sb, sizeof(sb), "%llu total, +%llu since last sample (%llu M TSC ticks)", (unsigned long long)smi,
		         (unsigned long long)dsmi, (unsigned long long)((tsc - smiTscLast) / 1000000ULL));
	} else {
		snprintf(sb, sizeof(sb), "%llu total", (unsigned long long)smi);
	}
	smiLast = smi; smiTscLast = tsc;
	setProperty("SMI count", sb);
}

void S0ixStandby::setupIom()
{
	char buf[128] = "not found";
	const IORegistryPlane *plane = IORegistryEntry::getPlane("IOACPIPlane");
	OSString *nm = OSString::withCString("PC00");
	IOACPIPlatformDevice *pc = nullptr;
	if (plane && nm)
		if (IORegistryIterator *it = IORegistryIterator::iterateOver(plane, kIORegistryIterateRecursively)) {
			while (IORegistryEntry *e = it->getNextObject()) {
				IOACPIPlatformDevice *d = OSDynamicCast(IOACPIPlatformDevice, e);
				if (d && nm->isEqualTo(d->getName())) { pc = d; break; }
			}
			it->release();
		}
	OSSafeReleaseNULL(nm);
	OSObject *r = nullptr;
	uint64_t base = 0;
	if (pc && !pc00) { pc->retain(); pc00 = pc; }
	if (pc && pc->evaluateObject("IOMA", &r) == kIOReturnSuccess && r)
		if (OSNumber *n = OSDynamicCast(OSNumber, r)) base = n->unsigned64BitValue();
	OSSafeReleaseNULL(r);
	if (base) {
		uint64_t a = base + 0xC10000 + 0x40;
		if (IOMemoryDescriptor *md = IOMemoryDescriptor::withPhysicalAddress(a & ~0xFFFULL, 0x1000, kIODirectionInOut)) {
			iomMap = md->map(kIOMapInhibitCache);
			md->release();
			if (iomMap) iomReg = reinterpret_cast<volatile uint32_t *>(iomMap->getVirtualAddress() + (a & 0xFFF));
		}
		snprintf(buf, sizeof(buf), "IOM+0x40 at 0x%llx = 0x%08x (TD3C b15, TACK b16, DPOF b17)%s", (unsigned long long)a,
		         iomReg ? *iomReg : 0xFFFFFFFF, iomNoDpof ? "; -s0ixnodpof: DPOF cleared after the entry _DSM" : "");
	}
	setProperty("Type-C IOM", buf);
	LOG("Type-C IOM %s", buf);
	if (iomReg && !iomCall) iomCall = thread_call_allocate(iomLater, this);
}

void S0ixStandby::iomLater(thread_call_param_t self, thread_call_param_t)
{
	S0ixStandby *me = static_cast<S0ixStandby *>(self);
	if (!me->iomReg) return;
	me->iomV[4] = *me->iomReg;
	char b[160];
	snprintf(b, sizeof(b), "before entry 0x%08x, after entry 0x%08x, wake before exit 0x%08x, after exit 0x%08x, +2 s 0x%08x",
	         me->iomV[0], me->iomV[1], me->iomV[2], me->iomV[3], me->iomV[4]);
	me->setProperty("Type-C IOM at last sleep", b);
	if (me->tcssD3) {
		char t[260];
		snprintf(t, sizeof(t), "%u sleeps; xHCI 0d.0 PMCSR at sleep 0x%04x (restored to 0x%04x); D3C._OFF 0x%x: IOM 0x%08x -> 0x%08x "
		         "(TACK after %u ms); wake: IOM 0x%08x, D3C._ON 0x%x -> 0x%08x", me->tcssCount, me->txhcPmcsrSleep,
		         me->txhcPmcsrSaved, me->tcssRet[0], me->tcssIom[0], me->tcssIom[1], me->tcssAckMs, me->tcssIom[2],
		         me->tcssRet[1], me->tcssIom[3]);
		me->setProperty("Type-C D3cold at last sleep", t);
	}
	me->publishTypeC();
}

// TCSS D3cold (-s0ixtcssd3). The DSDT's TCSS xHCI (TXHC) has _PR0/_PR3 = \_SB.PC00.D3C; its _OFF (TCOF) sets the IOM's
// TD3C bit when no TCSS device asks to skip D3cold, _ON (TCON) clears it and waits for TACK. macOS never does ACPI
// runtime D3 for these devices, so TD3C stayed 0 in every Deep Idle and the package never left PC2 once the IOM was
// told the display is off (DPOF or DC9). Only with nothing attached to USB-C (never sleep docked).
void S0ixStandby::tcssSleep()
{
	tcssOff = false;
	if (!pc00 || !txhcDev || !txhcPm || !iomReg) return;
	txhcPmcsrSleep = txhcDev->configRead16(txhcPm + 4);
	txhcPmcsrSaved = txhcPmcsrSleep;
	if ((txhcPmcsrSleep & 3) != 3) {
		txhcDev->configWrite16(txhcPm + 4, (uint16_t)((txhcPmcsrSleep & ~3) | 3));
		IODelay(10000);                                   // D0 -> D3hot: 10 ms
	}
	tcssIom[0] = *iomReg;
	tcssRet[0] = pc00->evaluateObject("D3C._OFF");
	tcssAckMs = 0;
	while (tcssAckMs < 100 && !(*iomReg & (1u << 16))) { IODelay(1000); tcssAckMs++; }
	tcssIom[1] = *iomReg;
	tcssOff = tcssRet[0] == kIOReturnSuccess;
	if (!tcssOff && (txhcPmcsrSaved & 3) != 3) txhcDev->configWrite16(txhcPm + 4, txhcPmcsrSaved);
}

void S0ixStandby::tcssWake()
{
	if (!tcssOff) return;
	tcssOff = false;
	tcssIom[2] = iomReg ? *iomReg : 0;
	tcssRet[1] = pc00->evaluateObject("D3C._ON");
	tcssIom[3] = iomReg ? *iomReg : 0;
	if ((txhcPmcsrSaved & 3) != 3) {
		uint16_t v = txhcDev->configRead16(txhcPm + 4);
		txhcDev->configWrite16(txhcPm + 4, (uint16_t)((v & ~3) | (txhcPmcsrSaved & 3)));
		IODelay(10000);
	}
	tcssCount++;
}

void S0ixStandby::setupLidWake()
{
	setProperty("Keyboard wake", kbWake ? "armed during Deep Idle when the lid is open (-s0ixkbdwake): i8042 keyboard port + IRQ1"
	                                    : "off (boot-arg -s0ixkbdwake)");
	if ((kbWake || edWake) && !ioapic) {
		// MADT on this machine: one I/O APIC at 0xFEC00000, no source override for IRQ1 (GSI 1).
		if (IOMemoryDescriptor *md = IOMemoryDescriptor::withPhysicalAddress(0xFEC00000ULL, 0x1000, kIODirectionInOut)) {
			ioapicMap = md->map(kIOMapInhibitCache);
			md->release();
			if (ioapicMap) {
				ioapic = reinterpret_cast<volatile uint32_t *>(ioapicMap->getVirtualAddress());
				ioapic[0] = 0x01;                               // IOAPICVER: sanity check
				uint32_t ver = ioapic[4];
				if ((ver & 0xFF) < 0x11 || ((ver >> 16) & 0xFF) < 1) { ioapic = nullptr; OSSafeReleaseNULL(ioapicMap); }
			}
		}
	}
	if (!lwWake && !kbWake) { setProperty("Lid wake", "off (boot-arg -s0ixlidwake)"); return; }
	static const char *hids[2] = { "PNP0C0D", "PNP0C0E" };
	const IORegistryPlane *plane = IORegistryEntry::getPlane("IOACPIPlane");
	if (!plane) return;
	for (int i = 0; i < 2; i++) {
		OSString *hid = OSString::withCString(hids[i]);
		if (!hid) continue;
		if (IORegistryIterator *it = IORegistryIterator::iterateOver(plane, kIORegistryIterateRecursively)) {
			while (IORegistryEntry *e = it->getNextObject()) {
				IOACPIPlatformDevice *d = OSDynamicCast(IOACPIPlatformDevice, e);
				if (!d || !d->compareName(hid, nullptr)) continue;
				d->retain();
				lwDev[i] = d;
				OSObject *prw = nullptr;
				if (lwGpe == 0xFFFFFFFF && d->evaluateObject("_PRW", &prw) == kIOReturnSuccess && prw) {
					if (OSArray *a = OSDynamicCast(OSArray, prw))
						if (OSNumber *n = OSDynamicCast(OSNumber, a->getObject(0)))
							lwGpe = n->unsigned32BitValue();
				}
				OSSafeReleaseNULL(prw);
				break;
			}
			it->release();
		}
		hid->release();
	}
	if (lwGpe != 0xFFFFFFFF && (!gpe0EnPort || lwGpe / 8 >= gpe0Half)) lwGpe = 0xFFFFFFFF;
	// GPIO-routed GPE (DW0..DW2 = GPEs 0x00-0x5F): find its group/pad from the BIOS GNVS (GEI0-2 group, GED0-2 dword)
	// and the GPIO community's GPI_GPE_STS / GPI_GPE_EN through the DSDT's own helper \_SB.GADR(group, 5).
	if (lwGpe != 0xFFFFFFFF && lwGpe < 0x60 && lwDev[0]) {
		// Apple's evaluateObject only handles methods/names (not GNVS field units) and is device-relative: from the
		// \_SB node, find the pad whose GGPE() is our GPE by walking the DSDT's GPIO groups (GMXG, GINF(g,1) pads).
		UInt32 grp = 0, dw = 0, padFound = 0xFFFFFFFF;
		char gb[160] = "";
		IOACPIPlatformDevice *sb = nullptr;
		if (const IORegistryPlane *ap = IORegistryEntry::getPlane("IOACPIPlane"))
			sb = OSDynamicCast(IOACPIPlatformDevice, lwDev[0]->getParentEntry(ap));
		UInt32 ngrp = 0;
		const char *form = nullptr;
		if (sb && sb->evaluateInteger("GMXG", &ngrp) == kIOReturnSuccess && ngrp && ngrp < 64) {
			for (UInt32 g = 0; g < ngrp && padFound == 0xFFFFFFFF; g++) {
				UInt32 npads = 0;
				OSObject *ga[2] = { OSNumber::withNumber(g, 32), OSNumber::withNumber(1, 32) };
				OSObject *gr = nullptr;
				if (ga[0] && ga[1] && sb->evaluateObject("GINF", &gr, ga, 2) == kIOReturnSuccess && gr)
					if (OSNumber *n = OSDynamicCast(OSNumber, gr)) npads = n->unsigned32BitValue();
				OSSafeReleaseNULL(gr); OSSafeReleaseNULL(ga[0]); OSSafeReleaseNULL(ga[1]);
				for (UInt32 p = 0; p < npads && p < 256; p++) {
					OSObject *pa0 = OSNumber::withNumber(((uint64_t)g << 16) | p, 32);
					OSObject *r = nullptr;
					UInt32 gpe = 0xFFFFFFFF;
					if (pa0 && sb->evaluateObject("GGPE", &r, &pa0, 1) == kIOReturnSuccess && r)
						if (OSNumber *n = OSDynamicCast(OSNumber, r)) gpe = n->unsigned32BitValue();
					OSSafeReleaseNULL(r); OSSafeReleaseNULL(pa0);
					if (gpe == lwGpe) { grp = g; padFound = p; dw = p / 32; form = "GGPE search"; break; }
				}
			}
		}
		if (form) {
			OSObject *args[2] = { OSNumber::withNumber(grp, 32), OSNumber::withNumber(5, 32) };
			OSObject *res = nullptr;
			uint64_t sts = 0;
			if (args[0] && args[1] && sb->evaluateObject("GADR", &res, args, 2) == kIOReturnSuccess && res)
				if (OSNumber *n = OSDynamicCast(OSNumber, res)) sts = n->unsigned64BitValue() + (uint64_t)dw * 4;
			OSSafeReleaseNULL(res);
			OSSafeReleaseNULL(args[0]); OSSafeReleaseNULL(args[1]);
			uint32_t pad = padFound;
			uint32_t padcfg = 0xFFFFFFFF, own = 0xFFFFFFFF;
			OSObject *pa = OSNumber::withNumber(((uint64_t)grp << 16) | pad, 32);
			if (pa) {
				OSObject *r2 = nullptr;
				if (sb->evaluateObject("GPC0", &r2, &pa, 1) == kIOReturnSuccess && r2)
					if (OSNumber *n = OSDynamicCast(OSNumber, r2)) padcfg = n->unsigned32BitValue();
				OSSafeReleaseNULL(r2);
				if (sb->evaluateObject("GHPO", &r2, &pa, 1) == kIOReturnSuccess && r2)
					if (OSNumber *n = OSDynamicCast(OSNumber, r2)) own = n->unsigned32BitValue();
				OSSafeReleaseNULL(r2);
				pa->release();
			}
			if (sts && (sts & 3) == 0 && ((sts + 0x20) & ~0xFFFULL) == (sts & ~0xFFFULL)) {
				if (IOMemoryDescriptor *md = IOMemoryDescriptor::withPhysicalAddress(sts & ~0xFFFULL, 0x1000, kIODirectionInOut)) {
					lwGpioMap = md->map(kIOMapInhibitCache);
					md->release();
					if (lwGpioMap) {
						uint8_t *base = reinterpret_cast<uint8_t *>(lwGpioMap->getVirtualAddress()) + (sts & 0xFFF);
						lwGpeSts = reinterpret_cast<volatile uint32_t *>(base);
						lwGpeEn = reinterpret_cast<volatile uint32_t *>(base + 0x20);
						lwGpioBit = 1u << (lwGpe % 32);
					}
				}
			}
			snprintf(gb, sizeof(gb), "GPE 0x%x = GPIO group %u pad %u, GPI_GPE_STS 0x%llx (EN +0x20)%s, PADCFG0 0x%08x, HOSTSW_OWN %u [%s]",
			         lwGpe, (unsigned)grp, pad, (unsigned long long)sts, lwGpeEn ? "" : " NOT MAPPED", padcfg, own, form);
		} else {
			snprintf(gb, sizeof(gb), "GPE 0x%x: no pad found (\\_SB %s, GMXG %u)", lwGpe, sb ? sb->getName() : "not found", (unsigned)ngrp);
		}
		setProperty("Lid wake GPIO", gb);
		LOG("Lid wake GPIO %s", gb);
	}
	char buf[128];
	snprintf(buf, sizeof(buf), "%s: lid %s, sleep button %s, wake GPE 0x%x",
	         lwWake ? "armed during Deep Idle (-s0ixlidwake)" : "off (boot-arg -s0ixlidwake)",
	         lwDev[0] ? lwDev[0]->getName() : "not found", lwDev[1] ? lwDev[1]->getName() : "not found", lwGpe);
	setProperty("Lid wake", buf);
	LOG("Lid wake %s", buf);
	if (!pbCall) pbCall = thread_call_allocate(pbUserActive, this);
}

void S0ixStandby::lidPsw(bool on)
{
	for (int i = 0; i < 2; i++) {
		if (!lwDev[i]) continue;
		OSObject *arg = OSNumber::withNumber(on ? 1 : 0, 32);
		if (!arg) continue;
		IOReturn r = lwDev[i]->evaluateObject("_PSW", nullptr, &arg, 1);
		DBG("%s._PSW(%d) -> 0x%x", lwDev[i]->getName(), on ? 1 : 0, r);
		arg->release();
	}
}

void S0ixStandby::findEcGpe()
{
	uint32_t forced = 0;
	if (PE_parse_boot_argn("s0ixecgpe", &forced, sizeof(forced)) && forced < 0x100) {
		ecGpe = forced;
		ecGpeFrom = "s0ixecgpe";
		return;
	}
	OSString *hid = OSString::withCString("PNP0C09");
	if (!hid) return;
	const IORegistryPlane *plane = IORegistryEntry::getPlane("IOACPIPlane");
	if (!plane) { hid->release(); return; }
	if (IORegistryIterator *it = IORegistryIterator::iterateOver(plane, kIORegistryIterateRecursively)) {
		while (IORegistryEntry *e = it->getNextObject()) {
			IOACPIPlatformDevice *ec = OSDynamicCast(IOACPIPlatformDevice, e);
			if (!ec || !ec->compareName(hid, nullptr)) continue;
			UInt32 v = 0;
			if (ec->evaluateInteger("_GPE", &v) == kIOReturnSuccess) {
				ecGpe = v;
				ecGpeFrom = ec->getName();
				if (!ecDev) { ecDev = ec; ecDev->retain(); }
				break;
			}
		}
		it->release();
	}
	hid->release();
}

void S0ixStandby::publishEcWake()
{
	char buf[160];
	if (!gpe0StsPort || ecGpe == 0xFFFFFFFF || ecGpe / 8 >= gpe0Half) {
		snprintf(buf, sizeof(buf), "unavailable: GPE0 block 0x%x length %u, EC _GPE %s",
		         gpe0StsPort, gpe0Half * 2u,
		         ecGpe == 0xFFFFFFFF ? "not found (set s0ixecgpe=0xNN)" : "outside GPE0");
		if (ecGpe != 0xFFFFFFFF && ecGpe / 8 >= gpe0Half) ecGpe = 0xFFFFFFFF;
	} else {
		snprintf(buf, sizeof(buf), "%s: EC GPE 0x%x (%s), enable at I/O 0x%x bit %u",
		         ecWake ? "armed during Deep Idle (-s0ixecwake)" : "off (boot-arg -s0ixecwake)",
		         ecGpe, ecGpeFrom ? ecGpeFrom : "?", gpe0EnPort + ecGpe / 8, ecGpe % 8);
	}
	setProperty("EC wake", buf);
	LOG("EC wake %s", buf);
}

void S0ixStandby::publishWakeInfo()
{
	if (!gpe0StsPort) return;
	if (!wakeInfoValid) {
		setProperty("Last wake source", "not recorded (not a Deep Idle halt)");
		return;
	}
	char buf[192];
	int n = snprintf(buf, sizeof(buf), "%s", "");
	if (pm1StsAtWake & (1u << 8))  n += snprintf(buf + n, sizeof(buf) - (size_t)n, "%sPWRBTN", n ? ", " : "");
	if (pm1StsAtWake & (1u << 10)) n += snprintf(buf + n, sizeof(buf) - (size_t)n, "%sRTC", n ? ", " : "");
	for (uint32_t i = 0; i < gpe0Half && i < sizeof(gpe0StsAtWake) && n < (int)sizeof(buf) - 24; i++)
		for (uint32_t b = 0; b < 8; b++)
			if (gpe0StsAtWake[i] & (1u << b)) {
				uint32_t g = i * 8 + b;
				n += snprintf(buf + n, sizeof(buf) - (size_t)n, "%sGPE 0x%x%s", n ? ", " : "", g,
				              g == ecGpe ? " (EC)" : "");
			}
	if (!n) snprintf(buf, sizeof(buf), "none latched (PM1_STS 0x%04x)", pm1StsAtWake);
	setProperty("Last wake source", buf);
	{
		char en[2 * sizeof(gpe0EnAtHalt) + 1]; size_t k = 0;
		for (uint32_t i = 0; i < gpe0Half && i < sizeof(gpe0EnAtHalt); i++)
			k += snprintf(en + k, sizeof(en) - k, "%02x", gpe0EnAtHalt[i]);
		setProperty("GPE0_EN at last halt (byte 0 first)", en);
		char rb[140];
		snprintf(rb, sizeof(rb), "lid GPE enable 0x%08x (GPIO GPI_GPE_EN when mapped), status at wake 0x%08x, EC GPE dword 0x%08x",
		         lwEnReadback, lwGpeStsAtWake, ecEnReadback);
		setProperty("GPE0_EN readback at last halt", rb);
	}
	if (ecGpe != 0xFFFFFFFF) {
		snprintf(buf, sizeof(buf), "%s by macOS%s", ecEnAtHalt ? "enabled" : "disabled",
		         ecWake && !ecEnAtHalt ? ", armed by -s0ixecwake" : "");
		setProperty("EC GPE at last halt", buf);
		setProperty("EC wakes", (unsigned long long)ecWakeCount, 32);
	}
}

// Power source state for the sleep action, which must not touch the registry.
void S0ixStandby::updateAC()
{
	bool ac = false;
	if (OSDictionary *match = serviceMatching("IOPMPowerSource")) {
		if (OSIterator *it = getMatchingServices(match)) {
			while (OSObject *o = it->getNextObject())
				if (IOService *ps = OSDynamicCast(IOService, o))
					if (OSBoolean *b = OSDynamicCast(OSBoolean, ps->getProperty("ExternalConnected")))
						if (b->isTrue()) ac = true;
			it->release();
		}
		match->release();
	}
	onAC = ac;
}

void S0ixStandby::acTimerFired(OSObject *owner, IOTimerEventSource *sender)
{
	if (S0ixStandby *self = OSDynamicCast(S0ixStandby, owner)) {
		self->updateAC();
		sender->setTimeoutMS(10000);
	}
}

static bool psBool(IOService *ps, const char *key, bool &out)
{
	OSBoolean *b = OSDynamicCast(OSBoolean, ps->getProperty(key));
	if (!b) return false;
	out = b->isTrue();
	return true;
}

static bool psNum(IOService *ps, const char *key, uint64_t &out)
{
	OSNumber *n = OSDynamicCast(OSNumber, ps->getProperty(key));
	if (!n) return false;
	out = n->unsigned64BitValue();
	return true;
}

// Detects an AC-connected battery that the EC stopped charging just short of
// its last full charge: macOS then sees "not charging, not full, no reason"
// and powerd/PowerUIAgent schedule dark wakes every few minutes
// (docs/BATTERY-FULL.md). Read-only.
void S0ixStandby::battCheck()
{
	enum { kNone, kBattery, kCharging, kFull, kNotFull, kStuck };
	uint8_t state = kNone;
	uint64_t cur = 0, max = 0;
	if (OSDictionary *match = serviceMatching("IOPMPowerSource")) {
		if (OSIterator *it = getMatchingServices(match)) {
			while (OSObject *o = it->getNextObject()) {
				IOService *ps = OSDynamicCast(IOService, o);
				bool ext = false, chg = false, full = false;
				if (!ps || !psBool(ps, "ExternalConnected", ext)) continue;
				if (!psNum(ps, "CurrentCapacity", cur) || !psNum(ps, "MaxCapacity", max) || !max) continue;
				psBool(ps, "IsCharging", chg);
				psBool(ps, "FullyCharged", full);
				if (!ext) state = kBattery;
				else if (chg) state = kCharging;
				else if (full) state = kFull;
				else if (cur < max && cur * 100 >= max * 97) state = kStuck;
				else state = kNotFull;
				break;
			}
			it->release();
		}
		match->release();
	}

	char buf[192];
	switch (state) {
	case kNone:     snprintf(buf, sizeof(buf), "no battery"); break;
	case kBattery:  snprintf(buf, sizeof(buf), "ok: on battery"); break;
	case kCharging: snprintf(buf, sizeof(buf), "ok: on AC, charging"); break;
	case kFull:     snprintf(buf, sizeof(buf), "ok: on AC, fully charged"); break;
	case kNotFull:  snprintf(buf, sizeof(buf), "on AC, not charging (%llu/%llu)", cur, max); break;
	default:
		snprintf(buf, sizeof(buf), "stuck: on AC, not charging, %llu/%llu, FullyCharged No: "
		         "macOS dark-wakes every few minutes (see docs/BATTERY-FULL.md)", cur, max);
		break;
	}
	setProperty("Battery full state", buf);
	if (state != battState && (state == kStuck || battState == kStuck))
		LOG("battery full state: %s", buf);
	battState = state;
}

void S0ixStandby::battTimerFired(OSObject *owner, IOTimerEventSource *sender)
{
	if (S0ixStandby *self = OSDynamicCast(S0ixStandby, owner)) {
		self->battCheck();
		if (self->gpe0EnPort && self->ecGpe != 0xFFFFFFFF) {
			bool on = (inb((uint16_t)(self->gpe0EnPort + self->ecGpe / 8)) >> (self->ecGpe % 8)) & 1;
			self->setProperty("EC GPE now", on ? "enabled" : "DISABLED (EC events not handled)");
			if (!on && self->ecGpeWasOn) LOG("EC GPE 0x%x is disabled while awake", self->ecGpe);
			self->ecGpeWasOn = on;
		}
		if (self->gpe0StsPort && self->ecGpe == 0xFFFFFFFF && self->ecTries < 10) {
			self->ecTries++;
			self->findEcGpe();
			if (self->ecGpe != 0xFFFFFFFF) self->publishEcWake();
		}
		if (self->gpuMMIO) gDispMMIO = self->gpuMMIO;
		self->publishTypeC();
		self->publishLog();
		sender->setTimeoutMS(60000);
	}
}

void S0ixStandby::onSleep()
{
	lastSleepType = 0;
	if (IOPMrootDomain *root = getPMRootDomain())
		if (OSNumber *n = OSDynamicCast(OSNumber, root->getProperty("IOPMSystemSleepType")))
			lastSleepType = n->unsigned32BitValue();

	counterAtSleep = 0;
	readCounter(counterAtSleep);
	timeAtSleep = mach_absolute_time();
	if (cstateMask) { readCStates(csAtSleep); tscAtSleep = rdtsc(); e2AtSleep = rdmsr(kMsrPkgCstConfig); }

	if (lastSleepType == kSleepTypeDeepIdle)
		d3Enter();
	if (gpuMMIO) {
		gpuSleep();
		dispCapture(dispAtSleep, nullptr);
		gDispMMIO = gpuMMIO;
		gDispOk = dispAtSleep.valid;
		if (refclkWA) refclkSleep();
	}
	// Drivers (and IOPCIFamily's D3 transition) are done by now.
	wifiPmcsrSleep = wifiDev ? wifiDev->configRead16(wifiPmCap + 4) : 0xFFFF;

	if (lwWake && lastSleepType == kSleepTypeDeepIdle) lidPsw(true);   // while the EC still answers normally
	if (tcssD3 && lastSleepType == kSleepTypeDeepIdle) tcssSleep();   // before the LPS0 entry _DSM, like Linux
	entered = false;
	if (dsmEnabled && lastSleepType == kSleepTypeDeepIdle &&
	    (dsmMask[kIntel] || dsmMask[kMicrosoft] || dsmMask[kAMD])) {
		if (iomReg) iomV[0] = *iomReg;
		dsmEntry();
		if (iomReg) {
			if (iomNoDpof && (*iomReg & (1u << 17))) *iomReg = *iomReg & ~(1u << 17);
			iomV[1] = *iomReg;
		}
		entered = true;
	}
	// Last step: nothing after us may touch PM1_EN before the halt.
	rtcBlocked = false;
	if (acNoRtc && pm1EnPort && onAC && lastSleepType == kSleepTypeDeepIdle) {
		// Two layers: the RTC's own alarm interrupt enable (CMOS register B
		// AIE), so no alarm event is raised at all, and the ACPI fixed event
		// enable RTC_EN, cleared again at the quiesce action in case the
		// platform re-enabled it.
		cmosBSaved = cmosRead(0x0B);
		if (cmosBSaved & kCmosAie) cmosWrite(0x0B, (uint8_t)(cmosBSaved & ~kCmosAie));
		pm1EnSaved = inw(pm1EnPort);
		if (pm1EnSaved & kPm1RtcEn) outw(pm1EnPort, (uint16_t)(pm1EnSaved & ~kPm1RtcEn));
		rtcBlocked = true;
	}
	ecArm = ecWake && gpe0EnPort && ecGpe != 0xFFFFFFFF && lastSleepType == kSleepTypeDeepIdle;
	pbArm = pbWake && pbEnPort && pm1StsPort && gpe0StsPort && lastSleepType == kSleepTypeDeepIdle;
	lwArm = lwWake && (lwDev[0] || lwDev[1]) && lastSleepType == kSleepTypeDeepIdle;
	lwWoke = false;
	kbArm = false; kbWoke = false; kbArmResult = 0; kbStsAtWake = 0;
	if (kbWake && lastSleepType == kSleepTypeDeepIdle) {
		UInt32 open = 1;                                   // no lid device: treat as open
		if (lwDev[0] && lwDev[0]->evaluateInteger("_LID", &open) != kIOReturnSuccess) open = 1;
		kbArm = open != 0;
	}
	pbArmed = false; pbWoke = false;
	ecArmed = false;
	wakeInfoValid = false;
	DBG("sleep: type %u, dsm %s, counter %llu, rtc wake %s", lastSleepType, entered ? "entry" : "skipped",
	    counterAtSleep, rtcBlocked ? "blocked (AC)" : "allowed");
}

void S0ixStandby::edReplay(thread_call_param_t self, thread_call_param_t)
{
	// Run the EC query methods we drained at the halt, so no EC event (battery, AC, lid, keys) is lost.
	S0ixStandby *me = static_cast<S0ixStandby *>(self);
	uint8_t n = me->edN;
	me->edN = 0;
	for (uint8_t i = 0; i < n && me->ecDev; i++) {
		char name[8]; snprintf(name, sizeof(name), "_Q%02X", me->edQ[i]);
		IOReturn r = me->ecDev->evaluateObject(name);
		LOG("EC drain replay %s -> 0x%x", name, r);
	}
}

void S0ixStandby::pbUserActive(thread_call_param_t self, thread_call_param_t)
{
	// Same call the HID system makes on user input: in a dark wake the root domain turns it into a full wake
	// ("Requesting full wake due to dark wake activity tickle").
	S0ixStandby *me = static_cast<S0ixStandby *>(self);
	if (OSDictionary *m = IOService::serviceMatching("IODisplayWrangler")) {
		if (IOService *w = IOService::copyMatchingService(m)) {
			w->activityTickle(0, 0);
			w->release();
		}
		m->release();
	}
	// Once more a little later, in case the first one came before the wrangler was ready.
	if (me->pbTickles++ < 1 && me->pbCall) {
		uint64_t dl; clock_interval_to_deadline(1200, kMillisecondScale, &dl);
		thread_call_enter_delayed(me->pbCall, dl);
	}
}

void S0ixStandby::onWake()
{
	// First: give RTC wakes back before anything else runs.
	if (rtcBlocked) {
		(void)cmosRead(0x0C);                    // drop an alarm flag raised while blocked
		if (cmosBSaved & kCmosAie) cmosWrite(0x0B, (uint8_t)(cmosRead(0x0B) | kCmosAie));
		outw(pm1EnPort, (uint16_t)(inw(pm1EnPort) | (pm1EnSaved & kPm1RtcEn)));
		rtcBlocked = false;
		rtcBlockCount++;
	}
	// Leave the EC GPE enable set: it is the EC's runtime GPE, which ACPI keeps
	// enabled while awake anyway. Clearing it here (0.5.4 before this fix) could
	// switch it off for the whole session when ACPI had already re-enabled it,
	// and then no EC event (battery, AC, lid, keys, thermal) was handled.
	ecArmed = false;
	if (wakeInfoValid && ecGpe != 0xFFFFFFFF && (gpe0StsAtWake[ecGpe / 8] >> (ecGpe % 8) & 1))
		ecWakeCount++;
	if (edWake && lastSleepType == kSleepTypeDeepIdle) {
		char ed[300]; int n = snprintf(ed, sizeof(ed), "EC status at halt 0x%02x (GPE sts %d), drained %u:", ecStsAtHalt,
		                               (int)edGpeStsAtHalt, edN);
		for (uint8_t i = 0; i < edN && n < (int)sizeof(ed) - 6; i++) n += snprintf(ed + n, sizeof(ed) - (size_t)n, " _Q%02X", edQ[i]);
		snprintf(ed + n, sizeof(ed) - (size_t)n, "; EC status at wake 0x%02x, EC GPE dword at wake 0x%08x; PM1_CNT 0x%04x; SCI RTE 0x%08x:%08x%s",
		         ecStsAtWakeP, ecEnAtWake, pm1CntAtHalt, sciRteHigh, sciRteLow, ioapic ? "" : " (no IOAPIC map)");
		setProperty("EC drain at last sleep", ed);
		if (edN && ecDev && edCall) {
			uint64_t dl; clock_interval_to_deadline(500, kMillisecondScale, &dl);
			thread_call_enter_delayed(edCall, dl);
		}
	}
	if (kbWake) {
		char kb[160];
		snprintf(kb, sizeof(kb), "%s, arm result %u, command byte 0x%02x -> 0x%02x, status at wake 0x%02x, IRQ1 RTE 0x%08x:%08x%s",
		         kbArm ? "armed" : "not armed (lid closed)", kbArmResult, kbCfgBefore, kbCfgSet, kbStsAtWake,
		         kbRteHigh, kbRteLow, ioapic ? "" : " (no IOAPIC map)");
		setProperty("Keyboard at last sleep", kb);
	}
	if (kbWoke) {
		kbWoke = false;
		kbWakeCount++;
		setProperty("Keyboard wakes", (unsigned long long)kbWakeCount, 32);
		pbTickles = 0;
		if (pbCall) {
			uint64_t dl; clock_interval_to_deadline(300, kMillisecondScale, &dl);
			thread_call_enter_delayed(pbCall, dl);
		}
	}
	if (lwWoke) {
		lwWoke = false;
		lwWakeCount++;
		setProperty("Lid wakes", (unsigned long long)lwWakeCount, 32);
		pbTickles = 0;
		if (pbCall) {
			uint64_t dl; clock_interval_to_deadline(300, kMillisecondScale, &dl);
			thread_call_enter_delayed(pbCall, dl);
		}
	}
	if (pbWoke) {
		// macOS sees no wake reason and would stay in dark wake and sleep again: ask for a full (user) wake
		// once the wake transition is under way.
		pbWoke = false;
		pbWakeCount++;
		setProperty("Power button wakes", (unsigned long long)pbWakeCount, 32);
		pbTickles = 0;
		if (pbCall) {
			uint64_t dl; clock_interval_to_deadline(300, kMillisecondScale, &dl);
			thread_call_enter_delayed(pbCall, dl);
		}
	}
	if (entered) {
		if (iomReg) iomV[2] = *iomReg;
		dsmExit();
		if (iomReg) {
			iomV[3] = *iomReg;
			if (iomCall) { uint64_t dl; clock_interval_to_deadline(2000, kMillisecondScale, &dl); thread_call_enter_delayed(iomCall, dl); }
		}
		entered = false;
	}
	tcssWake();
	if (lwWake && lastSleepType == kSleepTypeDeepIdle) lidPsw(false);
	// Before any driver wakes: Apple's accelerator restarts the engines later.
	if (gpuMMIO) {
		dispCapture(dispAtWake, &dispAtSleep);
		if (refclkWA) refclkWake();
		gpuWake();
	}
	wifiPmcsrWake = wifiDev ? wifiDev->configRead16(wifiPmCap + 4) : 0xFFFF;
	counterAtWake = 0;
	readCounter(counterAtWake);
	timeAtWake = mach_absolute_time();
	if (cstateMask) { readCStates(csAtWake); tscAtWake = rdtsc(); }
	sleepCount++;

	// Snapshot this cycle for publish() (plain copies, no registry access).
	snap.sleepCount = sleepCount;
	snap.sleepType = lastSleepType;
	snap.counterAtSleep = counterAtSleep; snap.counterAtWake = counterAtWake;
	snap.timeAtSleep = timeAtSleep; snap.timeAtWake = timeAtWake;
	for (uint32_t i = 0; i < kCStateCount; i++) { snap.csAtSleep[i] = csAtSleep[i]; snap.csAtWake[i] = csAtWake[i]; }
	snap.tscAtSleep = tscAtSleep; snap.tscAtWake = tscAtWake; snap.e2AtSleep = e2AtSleep;
	snap.haltValid = haltValid;
	for (uint32_t i = 0; i < kHaltCount; i++) { snap.haltAtQuiesce[i] = haltAtQuiesce[i]; snap.haltAtActive[i] = haltAtActive[i]; }
	snap.tscAtQuiesce = tscAtQuiesce; snap.tscAtActive = tscAtActive;
	snap.gatingValid = gatingValid;
	for (uint32_t i = 0; i < 8; i++) snap.gatingAtHalt[i] = gatingAtHalt[i];
	snap.pmcHaltValid = pmcHaltValid;
	snap.pmcActiveValid = pmcActiveValid;
	if (pmcActiveValid)
		for (uint32_t i = 0; i < 1024; i++) pmcSnapActive[i] = pmcAtActive[i];
	snap.gpuAtSleep = gpuAtSleep; snap.gpuAtWake = gpuAtWake;
	snap.gpuVerdict = gpuVerdict; snap.gpuResetResult = gpuResetResult;
	snap.gpuResetUs = gpuResetUs; snap.gpuEnginesReady = gpuEnginesReady;
	snap.wifiPmcsrSleep = wifiPmcsrSleep; snap.wifiPmcsrWake = wifiPmcsrWake;
	snap.dispAtSleep = dispAtSleep; snap.dispAtWake = dispAtWake;
	if (pmcHaltValid)
		for (uint32_t i = 0; i < 1024; i++) pmcSnap[i] = pmcAtHalt[i];
	haltValid = gatingValid = pmcHaltValid = pmcActiveValid = false;   // consumed by this cycle
	if (counter && counterFreq) {
		uint64_t mask = counterBits == 64 ? ~0ULL : 0xFFFFFFFFULL;
		totalResidencyUs += ((counterAtWake - counterAtSleep) & mask) * 1000000ULL / counterFreq;
	}
	pendingPublish = true;

	// Do not touch the registry from the platform-action context.
	if (publishTimer)
		publishTimer->setTimeoutMS(500);
}

void S0ixStandby::publishTimerFired(OSObject *owner, IOTimerEventSource *sender)
{
	S0ixStandby *self = OSDynamicCast(S0ixStandby, owner);
	if (!self) return;
	if (self->pendingPublish) {
		self->publish();
		// The framebuffer restores the display a few seconds after the wake
		// action: take a second look once it should be done.
		if (self->gpuMMIO && sender) {
			self->dispLate = true;
			sender->setTimeoutMS(15000);
		}
	} else if (self->dispLate) {
		self->dispLate = false;
		self->publishDisplay();
		self->publishLog();
		if (OSObject *o = self->getProperty("Display power now"))
			self->setProperty("Display power 15 s after wake", o);
	}
}

void S0ixStandby::publish()
{
	if (!pendingPublish) return;
	pendingPublish = false;
	if (d3Unbound) scanUnbound();

	uint64_t ticks = 0, residencyUs = 0;
	if (counter) {
		uint64_t mask = counterBits == 64 ? ~0ULL : 0xFFFFFFFFULL;
		ticks = (snap.counterAtWake - snap.counterAtSleep) & mask;   // handles one wrap
		if (counterFreq)
			residencyUs = ticks * 1000000ULL / counterFreq;
	}

	// Note: XNU rebases mach_absolute_time over Deep Idle (deep_idle_rebase),
	// so this is NOT the wall-clock sleep length; it is reported for reference.
	uint64_t ns = 0;
	absolutetime_to_nanoseconds(snap.timeAtWake - snap.timeAtSleep, &ns);

	setProperty("Sleep count", (unsigned long long)snap.sleepCount, 32);
	if (acNoRtc && pm1EnPort) {
		setProperty("AC RTC wake blocks", (unsigned long long)rtcBlockCount, 32);
		setProperty("On AC (last check)", onAC);
	}
	publishWakeInfo();
	setProperty("Last sleep type", (unsigned long long)snap.sleepType, 32);
	setProperty("Last DSM entry/exit", snap.sleepType == kSleepTypeDeepIdle && dsmEnabled);
	setProperty("Last residency ticks", (unsigned long long)ticks, 64);
	setProperty("Last residency us", (unsigned long long)residencyUs, 64);
	setProperty("Total residency us", (unsigned long long)totalResidencyUs, 64);
	setProperty("Last action span ns", (unsigned long long)ns, 64);
	setProperty("Residency counter at wake", (unsigned long long)snap.counterAtWake, 64);

	// C-state shares of the sleep: all counters run at TSC rate, and the TSC
	// keeps counting through Deep Idle (unlike rebased mach_absolute_time).
	if (cstateMask && snap.tscAtWake > snap.tscAtSleep) {
		uint64_t tsc = snap.tscAtWake - snap.tscAtSleep;
		setProperty("Last TSC ticks", (unsigned long long)tsc, 64);
		char e2[48];
		snprintf(e2, sizeof(e2), "0x%llx (limit %s)", (unsigned long long)snap.e2AtSleep,
		         pkgLimitName((uint32_t)(snap.e2AtSleep & 0xF)));
		setProperty("Last 0xE2 at sleep", e2);
		if (OSDictionary *d = OSDictionary::withCapacity(kCStateCount)) {
			char line[160]; size_t n = 0; line[0] = 0;
			for (uint32_t i = 0; i < kCStateCount; i++) {
				if (!(cstateMask & (1U << i))) continue;
				uint64_t pct = (snap.csAtWake[i] - snap.csAtSleep[i]) * 100 / tsc;
				if (OSNumber *num = OSNumber::withNumber((unsigned long long)pct, 32)) {
					d->setObject(kCStates[i].name, num);
					num->release();
				}
				if (n < sizeof(line))
					n += snprintf(line + n, sizeof(line) - n, " %s=%llu%%", kCStates[i].name, (unsigned long long)pct);
			}
			setProperty("Last C-state percent", d);
			d->release();
			LOG("wake #%u: C-states over %llu TSC ticks:%s", snap.sleepCount, (unsigned long long)tsc, line);
		}
	}

	if (snap.haltValid && snap.tscAtActive > snap.tscAtQuiesce) {
		uint64_t tsc = snap.tscAtActive - snap.tscAtQuiesce;
		setProperty("Last halt TSC ticks", (unsigned long long)tsc, 64);
		if (OSDictionary *d = OSDictionary::withCapacity(kHaltCount)) {
			const char *names[kHaltCount];
			names[0] = "CC6 (boot core)"; names[1] = "CC7 (boot core)";
			for (uint32_t i = 0; i < kCStateCount; i++) names[2 + i] = kCStates[i].name;
			for (uint32_t i = 0; i < kHaltCount; i++) {
				if (i >= 2 && !(cstateMask & (1U << (i - 2)))) continue;
				uint64_t pct = (snap.haltAtActive[i] - snap.haltAtQuiesce[i]) * 100 / tsc;
				if (OSNumber *num = OSNumber::withNumber((unsigned long long)pct, 32)) {
					d->setObject(names[i], num);
					num->release();
				}
			}
			setProperty("Last halt C-state percent", d);
			d->release();
		}
	}
	publishBlockers();
	publishGPU();
	publishDisplay();
	publishLog();
	if (wifiDev) {
		char w[96];
		snprintf(w, sizeof(w), "at sleep action D%u (0x%04x), at wake action D%u (0x%04x)",
		         snap.wifiPmcsrSleep & 3, snap.wifiPmcsrSleep, snap.wifiPmcsrWake & 3, snap.wifiPmcsrWake);
		setProperty("Wi-Fi PCI power state", w);
	}

	if (snap.pmcHaltValid) {
		if (OSData *d = OSData::withBytes(pmcSnap, sizeof(pmcSnap))) {
			setProperty("PMC page at halt", d);
			d->release();
		}
		snap.pmcHaltValid = false;
	}
	if (snap.pmcActiveValid) {
		if (OSData *d = OSData::withBytes(pmcSnapActive, sizeof(pmcSnapActive))) {
			setProperty("PMC page after halt", d);
			d->release();
		}
		snap.pmcActiveValid = false;
	}

	LOG("wake #%u: sleep type %u, SLP_S0 residency %llu us (%llu ticks)",
	    snap.sleepCount, snap.sleepType, residencyUs, ticks);
}
