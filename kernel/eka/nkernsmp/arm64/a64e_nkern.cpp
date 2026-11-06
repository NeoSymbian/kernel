// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// SPDX-License-Identifier: EPL-1.0
//
// a64e_nkern.cpp -- the nanokernel API tail only the FULL kernel (ekern) references, plus small runtime pieces, for
// the AArch64 port. The nkernsa image never needed these. Sources: ARM nkernsmp/arm/{ncutils.cpp,ncutils.cia,
// ncutilf.cia,nccpu.cpp,ncthrd.cpp} and kernel/arm/cutils.cia semantics; shape after Phorepoc's x86-64
// port/x64ekern/xe_nkern.cpp.
#define __INCLUDE_NTHREADBASE_DEFINES__
#include <kernel/kern_priv.h>
#include <nk_priv.h>
#include <nk_irq.h>
#include <arm.h>
#include "../board/board.h"
#include "../platform/a64_platform.h"

TSubScheduler& SubScheduler();
extern "C" void send_resched_ipi(TInt aCpu);				// a64s_ncsched.cpp

/******************************************************************************
 * Boot-time nanokernel init (ARM ncutils.cpp NKern::Init0)
 ******************************************************************************/
static void DetachComplete() {}						// cores are never powered down by this port

void NKern::Init0(TAny* a)
	{
	SVariantInterfaceBlock* v = (SVariantInterfaceBlock*)a;
	TheScheduler.iVIB = v;
	__NK_ASSERT_ALWAYS(v && v->iVer==0 && v->iSize==sizeof(SVariantInterfaceBlock));
	TScheduler& s = TheScheduler;
	s.iSX.iGicDistAddr = (TAny*)v->iGicDistAddr;
	s.iSX.iGicRedistBase = (TAny*)v->iGicRedistAddr;
	// timeslice ticks are generic-timer counts (a64s_ncsched.cpp UpdateThreadTimes), so the "timer max" is CNTFRQ
	s.iSX.iTimerMax = v->iMaxTimerClock;
	TInt i;
	for (i=0; i<KMaxCpus; ++i)
		{
		TSubScheduler& ss = TheSubSchedulers[i];
		ss.iSSX.iCpuFreqRI.Set(v->iCpuFreqR[i]);
		ss.iSSX.iTimerFreqRI.Set(v->iTimerFreqR[i]);
		v->iCpuFreqR[i] = 0;
		v->iTimerFreqR[i] = 0;
		UPerCpuUncached* u = v->iUncached[i];
		ss.iUncached = u;
		if (u)
			{
			u->iU.iDetachCount = 0;
			u->iU.iAttachCount = 0;
			u->iU.iPowerOffReq = FALSE;
			u->iU.iDetachCompleteFn = &DetachComplete;
			}
		}
	NKern::InterruptInit0();
	}

EXPORT_C SCpuIdleHandler* NKern::CpuIdleHandler()
	{
	return &::CpuIdleHandler;
	}

TUint32 NKern::IdleGenerationCount()
	{
	return TheScheduler.iIdleGenerationCount;
	}

EXPORT_C TInt NKern::TimesliceTicks(TUint32 aMicroseconds)
	{
	TUint64 ticks = TUint64(TheScheduler.iSX.iTimerMax)*TUint64(aMicroseconds) + UI64LIT(999999);
	ticks /= UI64LIT(1000000);
	if (ticks > TUint64(TInt(KMaxTInt)))
		return KMaxTInt;
	return (TInt)ticks;
	}

// The kernel has already printed the fault category and reason. First cut: report where, then halt this CPU.
// shortcut: no crash IPI to the other CPUs and no crash debugger; add both with SMP bring-up (M4.2 step 6)
void NKern::NotifyCrash(const TAny* /*a0*/, TInt /*a1*/)
	{
	NKern::DisableAllInterrupts();
	TUint64 lr;
	__asm__ __volatile__("mov %0, x30" : "=r"(lr));
	A64_DebugPuts("\n*** CRASH: CPU "); A64_DebugHex(NKern::CurrentCpu());
	A64_DebugPuts(" thread "); A64_DebugHex((TLinAddr)NKern::CurrentThread());
	A64_DebugPuts(" from "); A64_DebugHex(lr);
	A64_DebugPuts("\n");
	for (;;) { __asm__ __volatile__("msr daifset, #3\n\twfi"); }
	}

// nccpu.cpp: start one application processor.
// shortcut: the first full-kernel boot runs one CPU (board A64_CpuCount()==1, so A::InitAPs never calls this);
// the PSCI CPU_ON version with a physical boot block is M4.2 step 6
TInt NKern::BootAP(volatile SAPBootInfo*)
	{
	return KErrNotSupported;
	}

// ncthrd.cpp: the user-mode register set of a thread.
// shortcut: no user threads before M4.4, so "no registers available"; M4.4 reads the AArch32 frame from the stack
EXPORT_C void NKern::ThreadGetUserContext(NThread*, TAny*, TUint32& aAvailRegistersMask)
	{
	aAvailRegistersMask = 0;
	}

// ARM ncthrd.cpp / Phorepoc x86-64: queue a callback the thread runs on its way back to user mode. CAS on the
// head (the target may be running on another CPU and draining the list), IPI it so it notices.
TInt NKern::QueueUserModeCallback(NThreadBase* aThread, TUserModeCallback* aCallback)
	{
	__e32_memory_barrier();
	if (aCallback->iNext != KUserModeCallbackUnqueued)
		return KErrInUse;
	if (aThread->i_NThread_Initial)
		return KErrArgument;
	TInt result = KErrDied;
	NKern::Lock();
	TUserModeCallback* listHead = aThread->iUserModeCallbacks;
	do	{
		if (TLinAddr(listHead) & 3)
			goto done;	// thread exiting
		aCallback->iNext = listHead;
		} while (!__e32_atomic_cas_ord_ptr(&aThread->iUserModeCallbacks, &listHead, aCallback));
	result = KErrNone;
	if (!listHead)
		{
		TInt current = aThread->iCurrent;
		if (current)
			{
			TInt cpu = current & NSchedulable::EReadyCpuMask;
			if (cpu != NKern::CurrentCpu())
				send_resched_ipi(cpu);
			}
		}
done:
	NKern::Unlock();
	return result;
	}

/******************************************************************************
 * Spin locks (ncutilf.cia): release and re-acquire. The test-and-set lock can't see waiters, so always flash --
 * correct, merely eager (as Phorepoc's x86-64 port).
 ******************************************************************************/
EXPORT_C TBool TSpinLock::FlashIrq()
	{
	UnlockIrq();
	LockIrq();
	return ETrue;
	}
EXPORT_C TBool TSpinLock::FlashIrqRestore(TInt aIrq)
	{
	UnlockIrqRestore(aIrq);
	LockIrq();
	return ETrue;
	}

// cutils.cia: aRequest ? "are interrupts enabled" : "are interrupts disabled" (both IRQ and FIQ, DAIF.I/F)
TBool InterruptsStatus(TBool aRequest)
	{
	TUint64 daif;
	__asm__ __volatile__("mrs %0, daif" : "=r"(daif));
	TUint32 masked = (TUint32)(daif >> 6) & 3;
	return aRequest ? (masked == 0) : (masked == 3);
	}

// ncutils.cpp: CPU 0's interrupt and exception set-up. The vectors and the GIC are set by the bootstrap/variant on
// AArch64 (A64_InitGic); the generic part is the NIrq hardware init.
void Arm::Init1Interrupts()
	{
	NIrq::HwInit1();
	}

// the NIrq hardware hook for CPU 0: the GICv3 distributor and this CPU's redistributor are set up by the variant
// (A64_InitGic), so nothing is left to do here
void NIrq::HwInit1()
	{
	}

TInt __arm_cpu_number()
	{
	return NKern::CurrentCpu();
	}

/******************************************************************************
 * BTrace (ncutils.cpp). Tracing is off in this port for now: Out* report "not traced"; the handler/filter
 * registration is real so a trace handler can be installed later without touching callers.
 ******************************************************************************/
EXPORT_C TBool BTrace::OutX(TUint32, TUint32, TUint32, TUint32) { return EFalse; }
EXPORT_C TBool BTrace::OutN(TUint32, TUint32, TUint32, const TAny*, TInt) { return EFalse; }
EXPORT_C TBool BTrace::OutNX(TUint32, TUint32, TUint32, const TAny*, TInt) { return EFalse; }

static TInt BTraceDefaultControl(BTrace::TControl, TAny*, TAny*) { return KErrNotSupported; }

EXPORT_C void BTrace::SetHandlers(BTrace::THandler aNewHandler, BTrace::TControlFunction aNewControl, BTrace::THandler& aOldHandler, BTrace::TControlFunction& aOldControl)
	{
	BTrace::TControlFunction nc = aNewControl ? aNewControl : &BTraceDefaultControl;
	TInt irq = NKern::DisableAllInterrupts();
	BTrace::THandler oldh = BTraceData.iHandler;
	BTrace::TControlFunction oldc = BTraceData.iControl;
	BTraceData.iHandler = aNewHandler;
	BTraceData.iControl = nc;
	NKern::RestoreInterrupts(irq);
	aOldHandler = oldh;
	aOldControl = oldc;
	}

EXPORT_C TInt BTrace::SetFilter(TUint aCategory, TInt aValue)
	{
	if (!IsSupported(aCategory))
		return KErrNotSupported;
	TUint8* filter = BTraceData.iFilter+aCategory;
	TUint oldValue = *filter;
	if (TUint(aValue)<=1u)
		oldValue = __e32_atomic_swp_ord8(filter, (TUint8)aValue);
	return oldValue;
	}

/******************************************************************************
 * Small kernel helpers (cutils.cia)
 ******************************************************************************/
// shortcut: no crash debugger in this port yet; a fault halts in NotifyCrash
void A::StartCrashDebugger(const TAny*, TInt)
	{
	for (;;) { __asm__ __volatile__("msr daifset, #3\n\twfi"); }
	}

EXPORT_C TInt Arm::DebugOutJTAG(TUint)
	{
	return KErrNotSupported;
	}

// Busy-wait aInterval nanoseconds on the generic timer (ARM: a calibrated loop, PP::NanoWaitCal)
void K::DoNanoWait(TUint32 aInterval)
	{
	TUint64 f = NKern::TimestampFrequency();
	TUint64 ticks = (f * aInterval + 999999999u) / 1000000000u;
	TUint64 t0 = NKern::Timestamp();
	while (NKern::Timestamp() - t0 < ticks)
		__asm__ __volatile__("yield");
	}

// ARM reads the VFP FPSID, which EL1 AArch64 has no access to (as the x86 baseline: not supported)
TInt K::FloatingPointSystemId(TUint32&)
	{
	return KErrNotSupported;
	}

/******************************************************************************
 * C++ runtime the kernel image needs (the ARM build gets these from its kernel runtime)
 ******************************************************************************/
extern "C" void __cxa_pure_virtual()
	{
	NKFault("pure virtual", 0);
	}

TAny* operator new(unsigned long aSize) __NO_THROW		{ return Kern::Alloc((TInt)aSize); }
TAny* operator new[](unsigned long aSize) __NO_THROW	{ return Kern::Alloc((TInt)aSize); }
void operator delete(TAny* aPtr) __NO_THROW				{ Kern::Free(aPtr); }
void operator delete[](TAny* aPtr) __NO_THROW			{ Kern::Free(aPtr); }
