// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// Portions Copyright The Phorepoc Authors
// SPDX-License-Identifier: EPL-2.0
//
// a64_variant.cpp -- the minimal static variant of the AArch64 full kernel, linked into the image (the bootstrap's
// fabricated ROM records point at the entry points below). Modelled on Phorepoc's DX64Variant (adaptation-x86-64
// ekern/xk_entry.cpp): an Asic over the shared platform layer (GICv3, generic timer), the variant interface block
// NKern::Init0 consumes, and one kernel extension, which for now hosts a kernel-thread self-test.
#include <kernel/kern_priv.h>
#include <nk_priv.h>
#include <assp.h>
#include <mmboot.h>
#include "board.h"
#include "a64_dbgmon.h"
#include "a64_blk.h"
#include "a64_device.h"
#include "a64_platform.h"

const TInt KTickUs = 1000;			// system tick period: 1 ms, as every EKA2 port

// cutils.cpp is built with __EARLY_DEBUG__: Kern::Printf reaches the board console from the first line of the kernel,
// before the variant exists
void A64Log_Putc(char aChar);								// a64_dbgmon.cpp: the log ring the debug console replays
void A64Log_Report();										// a64_dbgmon.cpp: says whether a previous boot's log was kept
void DebugOutputChar(TUint aChar)
	{
	A64Log_Putc((char)aChar);
	A64_DebugPutc((char)aChar);
	}

class DA64Variant : public Asic
	{
public:
	virtual TMachineStartupType StartupReason()				{ return EStartupColdReset; }
	virtual void Init1()									{ A64_InitGic(); A64_TimerInit(KTickUs); }
	virtual void Init2AP()									{ A64_InitThisCpuGic(); A64_StartSystemTimer(); }
	virtual void Init3()									{ A64_StartSystemTimer(); }
	virtual void DebugOutput(TUint aChar)					{ DebugOutputChar(aChar); }
	virtual void Idle()										{ __asm__ __volatile__("dsb sy\n\twfi" ::: "memory"); }
	virtual TInt MsTickPeriod()								{ return KTickUs; }
	// the PMIC RTC can't be set: the kernel boots on the ROM header's time, then neoui sets the real one from the RTC
	// and its saved offset (ENeoHalRtcRead, clock.dat)
	virtual TInt SystemTimeInSecondsFrom2000(TInt&)			{ return KErrNotSupported; }
	virtual TInt SetSystemTimeInSecondsFrom2000(TInt)		{ return KErrNone; }
	// K::DoNanoWait counts the generic timer (a64e_nkern.cpp), so the calibration is never used
	virtual TUint32 NanoWaitCalibration()					{ return 1; }
	virtual TInt VariantHal(TInt aFunction, TAny* a1, TAny* a2);
	virtual TPtr8 MachineConfiguration()					{ return TPtr8((TUint8*)KMachineConfigLinAddr, 0x400, 0x400); }
	};

// NeoSymbian's own EHalGroupVariant functions, for the dev tools on the phone (docs/rust-integration.md "Dev tools"):
//   ENeoHalLogRead     a1 = TUint32* cursor (in/out; 0 = the oldest kept), a2 = TDes8* -> this boot's kernel log
//   ENeoHalLastLogRead a1 = TUint32* cursor (in/out; 0 = the start),      a2 = TDes8* -> the previous boot's (lastlog)
//   ENeoHalConsoleSet  a1 = 1 on / 0 off -> the kernel log on screen (dev mode); ENeoHalConsoleGet a1 = TInt* out
//   ENeoHalBootOk      no arguments -> mark the booted A/B slot "successful" for ABL (FS5b; neoui calls it once the
//                      launcher is drawn). KErrNone if done or already set; KErrNotSupported without A/B slots
// Reading needs ReadDeviceData, switching the console and ENeoHalBootOk WriteDeviceData. Each read fills at most
// 1024 bytes.
//   ENeoHalUsbDebugSet a1 = 1 on / 0 off -> USB debugging: fshell's ACM port joins (or leaves) the USB device and the
//                      host re-enumerates (Settings -> Developer, docs debug-access.md; WriteDeviceData).
//                      KErrNotSupported without it (QEMU: its second serial port is always there)
//   ENeoHalUsbDebugGet a1 = TInt* out -> 1 on, 0 off
//   ENeoHalSlotInfo    a1 = TDes8* -> one line: the booted A/B slot and both slots' state (ReadDeviceData)
//   ENeoHalBuildInfo   a1 = TDes8* -> this kernel's build: "kernel <commit>[+dirty] <UTC time> <board>" (no capability)
enum { ENeoHalLogRead = 0x4e00, ENeoHalLastLogRead, ENeoHalConsoleSet, ENeoHalConsoleGet, ENeoHalUsbDebugSet,
	ENeoHalUsbDebugGet, ENeoHalBootOk = 0x4e06,
	ENeoHalSlotInfo = 0x4e09, ENeoHalBuildInfo = 0x4e0a, ENeoHalPower = 0x4e0b, ENeoHalRtcRead = 0x4e0c, ENeoHalDeviceList = 0x4e19, ENeoHalBatteryRead = 0x4e1a };
//   ENeoHalPower       a1 = 0 restart, 1 restart to fastboot, 2 power off (PowerMgmt). Returns only on a failure; the
//                      caller finalises the drives first (neoui's Power menu)
//   ENeoHalRtcRead     a1 = TUint32* -> the PMIC RTC's raw seconds; KErrNotSupported without one (QEMU)
//   0x4e0d..0x4e18     the modem and the ADSP (telephony C4-C6): boot, state, QRTR send/receive, EFS partition reads,
//                      the rmtfs carve-out, IPA, ADSP channels, audio; CommDD. qcom_telhal.cpp on the Pixel,
//                      KErrNotSupported elsewhere
//   ENeoHalDeviceList  a1 = TUint32* cursor (in/out; 0 = the first), a2 = TDes8* (1024 bytes) -> the device registry
//                      as lines "tag\tstate\tname\tdriver\tdetail\n", none past the end (a64_devices.cpp; ReadDeviceData)
__attribute__((weak)) TInt A64Battery_Hal(TAny*) { return KErrNotSupported; }
void A64_RebootNormal();									// a64_reboot.cpp (weak QEMU defaults: a64_dbgmon.cpp)
void A64_RebootToFastboot();
TInt A64_PowerOff();
TInt A64_RtcSeconds(TUint32& aSecs);						// qcom_rtc.cpp
__attribute__((weak)) TInt A64Tel_Hal(TInt, TAny*, TAny*) { return KErrNotSupported; }	// qcom_telhal.cpp
__attribute__((weak)) TInt A64UsbDebugSet(TInt) { return KErrNotSupported; }		// qcom_dwc3.cpp
__attribute__((weak)) TInt A64UsbDebugGet() { return 0; }
#ifndef NEO_BUILD_KERNEL
#define NEO_BUILD_KERNEL "kernel unknown"				// build-ekern.sh passes the real one
#endif

static TInt LogToUser(TBool aLast, TAny* aCursor, TAny* aDes)
	{
	if (!Kern::CurrentThreadHasCapability(ECapabilityReadDeviceData, __PLATSEC_DIAGNOSTIC_STRING("NeoSymbian log read")))
		return KErrPermissionDenied;
	TUint32 pos;
	kumemget32(&pos, aCursor, sizeof(pos));
	TBuf8<1024> b;
	b.SetLength(aLast ? A64Log_LastRead(pos, (TUint8*)b.Ptr(), b.MaxLength())
		: A64Log_Read(pos, (TUint8*)b.Ptr(), b.MaxLength()));
	kumemput32(aCursor, &pos, sizeof(pos));
	return Kern::ThreadDesWrite(&Kern::CurrentThread(), aDes, b, 0, KChunkShiftBy0, NULL);
	}

// qcom_cpufreq.cpp on the Pixel (smooth-ui S3c); boards without one keep these
__attribute__((weak)) TUint A64CpuMaxKHz() { return 0; }		// the fastest core's top clock; 0 = unknown
__attribute__((weak)) void A64CpuBoost() {}					// every input event (Kern::AddEvent)
__attribute__((weak)) void A64CpufreqDump() { Kern::Printf("[cpufreq] none on this board"); }	// monitor `cpufreq`

// EHalGroupVariant. EVariantHalVariantInfo is the one UserHal::MachineInfo() requires (us_exec.cpp); Phorepoc's shape
TInt DA64Variant::VariantHal(TInt aFunction, TAny* a1, TAny* a2)
	{
	switch (aFunction)
		{
	case ENeoHalLogRead:
	case ENeoHalLastLogRead:
		return LogToUser(aFunction == ENeoHalLastLogRead, a1, a2);
	case ENeoHalConsoleSet:
		if (!Kern::CurrentThreadHasCapability(ECapabilityWriteDeviceData, __PLATSEC_DIAGNOSTIC_STRING("NeoSymbian console")))
			return KErrPermissionDenied;
		A64_ConsoleShow((TInt)a1);
		return KErrNone;
	case ENeoHalUsbDebugSet:
		if (!Kern::CurrentThreadHasCapability(ECapabilityWriteDeviceData, __PLATSEC_DIAGNOSTIC_STRING("NeoSymbian USB debugging")))
			return KErrPermissionDenied;
		Kern::Printf("[usb] USB debugging requested %d", (TInt)a1);
		return A64UsbDebugSet((TInt)a1);
	case ENeoHalUsbDebugGet:
		{
		TInt on = A64UsbDebugGet();
		kumemput32(a1, &on, sizeof(on));
		return KErrNone;
		}
	case ENeoHalBootOk:
		{
		if (!Kern::CurrentThreadHasCapability(ECapabilityWriteDeviceData, __PLATSEC_DIAGNOSTIC_STRING("NeoSymbian boot OK")))
			return KErrPermissionDenied;
		TBuf8<800> b;
		return A64Slot_MarkOk(EFalse, b);
		}
	case ENeoHalSlotInfo:
		{
		if (!Kern::CurrentThreadHasCapability(ECapabilityReadDeviceData, __PLATSEC_DIAGNOSTIC_STRING("NeoSymbian slot info")))
			return KErrPermissionDenied;
		TBuf8<400> b;
		A64Slot_Show(b);
		return Kern::ThreadDesWrite(&Kern::CurrentThread(), a1, b, 0, KChunkShiftBy0, NULL);
		}
	case ENeoHalBuildInfo:
		{
		TPtrC8 b((const TUint8*)NEO_BUILD_KERNEL);
		return Kern::ThreadDesWrite(&Kern::CurrentThread(), a1, b, 0, KChunkShiftBy0, NULL);
		}
	case ENeoHalPower:
		if (!Kern::CurrentThreadHasCapability(ECapabilityPowerMgmt, __PLATSEC_DIAGNOSTIC_STRING("NeoSymbian power")))
			return KErrPermissionDenied;
		Kern::Printf("[power] request %d", (TInt)a1);
		switch ((TInt)a1)
			{
		case 0:
			A64_RebootNormal();
			break;
		case 1:
			A64_RebootToFastboot();
			break;
		case 2:
			return A64_PowerOff();
		default:
			return KErrArgument;
			}
		return KErrGeneral;									// a reset that came back
	case ENeoHalRtcRead:
		{
		TUint32 secs = 0;
		TInt r = A64_RtcSeconds(secs);
		if (r == KErrNone)
			kumemput32(a1, &secs, sizeof(secs));
		return r;
		}
	case 0x4e0d: case 0x4e0e: case 0x4e0f: case 0x4e10: case 0x4e11: case 0x4e12: case 0x4e13:
	case 0x4e14: case 0x4e15: case 0x4e16: case 0x4e17: case 0x4e18:
		return A64Tel_Hal(aFunction, a1, a2);
	case ENeoHalBatteryRead:
		if (!Kern::CurrentThreadHasCapability(ECapabilityReadDeviceData, __PLATSEC_DIAGNOSTIC_STRING("NeoSymbian battery")))
			return KErrPermissionDenied;
		return A64Battery_Hal(a1);
	case ENeoHalDeviceList:
		return A64Device_Hal(a1, a2);
	case ENeoHalConsoleGet:
		{
		TInt on = A64_ConsoleShown();
		kumemput32(a1, &on, sizeof(on));
		return KErrNone;
		}
	case EVariantHalVariantInfo:
		{
		TVariantInfoV01Buf infoBuf;
		TVariantInfoV01& info = infoBuf();
		TUint clockKHz = A64CpuMaxKHz();				// the fastest core's top step (qcom_cpufreq.cpp)
		if (!clockKHz)
			clockKHz = 1800000;							// shortcut: nominal, on boards without a cpufreq driver
		info.iRomVersion = Epoc::RomHeader().iVersion;
		info.iMachineUniqueId = TInt64(0x10005152) << 32;
		info.iLedCapabilities = 0;
		info.iProcessorClockInKHz = clockKHz;
		info.iSpeedFactor = clockKHz / 25;
		Kern::InfoCopy(*(TDes8*)a1, infoBuf);
		return KErrNone;
		}
	case EVariantHalDebugPortSet:
		{
		TUint32 port = (TUint32)a1;					// the board console only
		if (port != 0 && port != (TUint32)KNullDebugPort)
			return KErrNotSupported;
		Kern::SuperPage().iDebugPort = port;
		return KErrNone;
		}
	case EVariantHalDebugPortGet:
		{
		TUint32 port = Kern::SuperPage().iDebugPort;
		kumemput32(a1, &port, sizeof(port));
		return KErrNone;
		}
	default:
		return KErrNotSupported;
		}
	}

static DA64Variant TheVariant;		// constructed by _E32Startup's static-constructor walk, before KernelMain
static SVariantInterfaceBlock TheVib;
// per-CPU power bookkeeping NKern::Init0 points each TSubScheduler at (ARM's default is the AP boot page, which a
// PSCI boot doesn't have); cores are coherent and never powered down by this port, so ordinary memory serves
static UPerCpuUncached TheUncached[KMaxCpus] __attribute__((aligned(64)));

// the system tick: CPU 0 drives the nanokernel timer queue (NTimerQ::Init1 ran in A::Init1, before Init3 starts it)
extern "C" void A64_TickHook(TSubScheduler& aSS, TUint64)
	{
	if (aSS.iCpuNum == 0)
		TheTimerQ.Tick();
	}

// The variant image's entry point (KModuleEntryReasonVariantInit0): bit 0 = also call VariantInitialise(1)
extern "C" TInt A64VariantEntry(TInt)
	{
	return 1;
	}

// The variant's export 1 (A::CreateVariant): 0 = the Asic, 1 = the interface block for NKern::Init0
extern "C" TAny* A64VariantInitialise(TInt aMode)
	{
	if (aMode == 0)
		return (TAny*)(Asic*)&TheVariant;
	if (aMode != 1)
		return NULL;
	TUint64 cntfrq;
	__asm__ __volatile__("mrs %0, cntfrq_el0" : "=r"(cntfrq));
	TheVib.iVer = 0;
	TheVib.iSize = sizeof(SVariantInterfaceBlock);
	TheVib.iMaxCpuClock = UI64LIT(1800000000);		// shortcut: nominal, see VariantHal
	TheVib.iMaxTimerClock = (TUint32)cntfrq;
	TheVib.iGicDistAddr = A64_Io(A64_GicdBase);
	TheVib.iGicRedistAddr = A64_Io(A64_GicrBase);
	TheVib.iPsciConduit = A64_PsciSmc ? 0 : 1;
	for (TInt i = 0; i < KMaxCpus; ++i)
		TheVib.iUncached[i] = &TheUncached[i];
	return &TheVib;
	}

// Self-test, as Phorepoc's ProbeThread: on the real kernel, a supervisor thread proves DThread creation, a fault
// under an exception trap (data abort -> Exc::Dispatch -> TExcTrap::Exception longjmps back with KErrBadDescriptor),
// blocking on the nanokernel timer queue (NKern::Sleep -> tick -> timer DFC -> wake) and thread exit. Then one
// sleeper thread pinned to each CPU: every core's GIC, IPIs and timer path, and the SMP scheduler.
static TInt StartThread(const TDesC& aName, TThreadFunction aFunction, TAny* aPtr, TInt aCpu)
	{
	SThreadCreateInfo info;
	info.iType = EThreadSupervisor;
	info.iFunction = aFunction;
	info.iPtr = aPtr;
	info.iSupervisorStack = NULL;
	info.iSupervisorStackSize = 0;		// default
	info.iInitialThreadPriority = 24;
	info.iName.Set(aName);
	info.iTotalSize = sizeof(info);
	TInt r = Kern::ThreadCreate(info);
	if (r != KErrNone)
		return r;
	DThread* t = (DThread*)info.iHandle;
	if (aCpu >= 0)
		NKern::ThreadSetCpuAffinity(&t->iNThread, aCpu);
	Kern::ThreadResume(*t);
	return KErrNone;
	}

static volatile TUint32 SleepersDone;
static TInt SleeperThread(TAny* aCpu)
	{
	TInt cpu = (TInt)aCpu;
	TInt t0 = NKern::TickCount();
	TBool onCpu = ETrue;
	for (TInt i = 0; i < 4; ++i)
		{
		NKern::Sleep(100);
		onCpu = onCpu && NKern::CurrentCpu() == cpu;
		}
	Kern::Printf("[probe] sleeper on cpu %d: 4 x Sleep(100) took %d ticks%s", cpu, NKern::TickCount() - t0,
		onCpu ? "" : " (ran on another CPU!)");
	if (onCpu)
		__e32_atomic_add_ord32(&SleepersDone, 1);
	return KErrNone;
	}

_LIT(KProbeName, "A64Probe");
_LIT(KSleeperName, "A64Sleeper");
static TInt ProbeThread(TAny*)
	{
	TInt ncpus = NKern::NumberOfCpus();
	Kern::Printf("[probe] thread running, tick %d, %d CPU(s) up of %d present", NKern::TickCount(), ncpus, A64_CpuCount());
	TInt r = 12345;
	XTRAP(r, XT_DEFAULT, { volatile TInt v = *(volatile TInt*)0x0BAD0000u; (void)v; });	// unmapped local address
	Kern::Printf("[probe] XTRAP around a read of 0x0BAD0000 -> %d (KErrBadDescriptor = %d)", r, KErrBadDescriptor);
	TInt t0 = NKern::TickCount();
	for (TInt i = 1; i <= 4; ++i)
		{
		NKern::Sleep(100);
		Kern::Printf("[probe] woke #%d at tick %d (slept 100)", i, NKern::TickCount());
		}
	TInt dt = NKern::TickCount() - t0;
	for (TInt c = 0; c < ncpus; ++c)
		{
		TBuf<16> name(KSleeperName);					// thread names are unique within a process
		name.AppendNum(c);
		TInt e = StartThread(name, SleeperThread, (TAny*)c, c);
		if (e != KErrNone)
			Kern::Printf("[probe] sleeper for cpu %d: create -> %d", c, e);
		}
	for (TInt w = 0; w < 40 && __e32_atomic_load_acq32(&SleepersDone) < (TUint32)ncpus; ++w)
		NKern::Sleep(50);
	TInt done = (TInt)__e32_atomic_load_acq32(&SleepersDone);
	TBool ok = r == KErrBadDescriptor && dt >= 400 && dt < 600 && ncpus == A64_CpuCount() && done == ncpus;
	Kern::Printf("[probe] %s: XTRAP %d, 4 x Sleep(100) took %d ticks, %d/%d CPUs, %d sleepers back",
		ok ? "PASS" : "FAIL", r, dt, ncpus, A64_CpuCount(), done);
	return KErrNone;
	}

void A64StartMedia();									// a64_media.cpp
void A64StartDisplay();									// a64_display.cpp
// Board/ASSP buses, before any peripheral starts using them (SDM670: SPMI).
__attribute__((weak)) void A64StartBuses() {}
// a64_serialkbd.cpp on QEMU virt; boards without a serial keyboard keep this
__attribute__((weak)) void A64StartSerialKeyboard() {}
// a64_ecomm.cpp: the serial LDD + the board's debug serial PDD (QEMU virt's second PL011)
__attribute__((weak)) void A64StartDebugSerial() {}
// a64_keys.cpp on the Pixel: power and volume keys
__attribute__((weak)) void A64StartKeys() {}
// a64_touch.cpp on the Pixel: the touchscreen
__attribute__((weak)) void A64StartTouch() {}
// a64_reboot.cpp on the Pixel: the AUTOFASTBOOT timer of debug builds
__attribute__((weak)) void A64StartReboot() {}
// qcom_dwc3.cpp + a64_usb_acm_monitor.cpp on the Pixel: the USB debug console (tools/phone.py)
__attribute__((weak)) void A64StartUsbConsole() {}
// qcom_cpufreq.cpp on the Pixel: CPU and L3 clocks with touch boost (smooth-ui S3c)
__attribute__((weak)) void A64StartCpufreq() {}

// The one kernel extension the ROM must carry (P::StartExtensions sizes an RArray by the count: zero panics).
extern "C" TInt A64ExtensionEntry(TInt aReason)
	{
	if (aReason != KModuleEntryReasonExtensionInit1)
		return KErrNone;
	A64Log_Report();									// the first point where Kern::Printf reaches the log ring
	Kern::Printf("[neo] %s", NEO_BUILD_KERNEL);			// which kernel this is (HAL ENeoHalBuildInfo too)
	A64StartBuses();
	A64StartCpufreq();									// before the input drivers: their events boost the clocks
	A64StartDisplay();									// the screen's HAL, for the window server
	A64StartSerialKeyboard();
	A64StartDebugSerial();
	A64StartKeys();
	A64StartTouch();
	A64StartReboot();
	A64StartUsbConsole();
	A64StartMedia();									// ELOCD + the RAM drive, before exstart starts F32
	TInt r = StartThread(KProbeName, ProbeThread, NULL, -1);
	Kern::Printf("[probe] Kern::ThreadCreate -> %d", r);
	return KErrNone;
	}

// The real exstart (kernel/exstart.cpp, KernelModuleEntry renamed by build-ekern.sh) starts the ROM's secondary
// process. Linked statically, its global `TDfc StartSecondaryDfc(.., Kern::SvMsgQue(), ..)` was constructed by
// _E32Startup's constructor walk, before the supervisor message queue existed, so it holds a NULL queue: bind it at
// the moment a real extension's constructors would have run (Phorepoc's X64ExstartShim).
TInt A64ExstartEntry(TInt aReason);
extern TDfc StartSecondaryDfc;
extern "C" TInt A64ExstartShim(TInt aReason)
	{
	if (aReason == KModuleEntryReasonExtensionInit1)
		StartSecondaryDfc.SetDfcQ(Kern::SvMsgQue());
	return A64ExstartEntry(aReason);
	}
