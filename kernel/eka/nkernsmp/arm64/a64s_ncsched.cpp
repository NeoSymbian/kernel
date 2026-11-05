// SPDX-License-Identifier: Apache-2.0
// Derived from Phorepoc adaptation-x86-64 port/x64smp/x64s_ncsched.cpp (Copyright The Phorepoc Authors, Apache-2.0).
// AArch64 changes: GICv3 SGIs (ICC_SGI1R_EL1) for IPIs, DAIF masking for cli/sti, WFI for hlt, YIELD for pause,
// MPIDR for the APIC id, TPIDR_EL1 = this CPU's TSubScheduler*, and an AArch64 thread start frame.

/* x64s_ncsched.cpp -- Phorepoc amd64 SMP arch backend: the scheduler / per-CPU /
 * IPI / fault core the arch-neutral nkernSMP references but that is inherently
 * machine-specific. The SMP analogue of nkernsmp/x86/{ncglob,ncsched,ncthrd,
 * ncirq,nctimer}.cpp+.cia.
 *
 * STATUS (SMP slice 2b step 2): this CLOSES THE LINK -- every symbol the neutral
 * nkernSMP core needs is defined, so the neutral core + amd64 SMP backend link
 * with zero unresolved references. The genuinely-real pieces here are: the
 * per-CPU globals (TheScheduler / TheSubSchedulers / SubSchedulerLookupTable),
 * the spin locks (test-and-set on the 64-bit iLock, correct on x86 TSO), the
 * LAPIC IPI senders (ICR writes; vectors from nk_plat.h), the kernel preemption
 * lock, and the timestamp bookkeeping in Save/UpdateThreadTimes. The pieces that
 * need the full neutral scheduler graph stood up (SelectNextThread's tied events,
 * NFastSemaphore Dec/Inc, the DFC state machine, NIrq HW dispatch, core control)
 * are honest first-cut stubs marked X64_TODO -- they compile+link and are wired
 * in the per-CPU bring-up (step 3), modelled on the 32-bit x86 backend.
 */
#define __INCLUDE_NTHREADBASE_DEFINES__
#include <e32def.h>
#include <nkern.h>
#include <nk_priv.h>
#include <nk_irq.h>
#include "a64_sysreg.h"

/* The amd64 context-switch primitive (x64_ctxsw.S, shared with uniprocessor). */
extern "C" void __phore_switch(void** aOldSPSlot, void* aNewSP);
extern "C" void __phore_thread_trampoline(void);

/* Weak board hooks (real defs in the SMP board file / ring3.c). */
extern "C" __attribute__((weak)) void fpu_set_ts(void) {}
extern "C" __attribute__((weak)) void board_set_rsp0(void*) {}
extern "C" __attribute__((weak)) void board_switch_as(NThreadBase*) {}
/* Eager FPU/SSE context (full kernel): save the outgoing thread's state before the switch, load the
 * incoming thread's when it resumes (aThread = NULL: a thread's very first run -> default state). */
extern "C" __attribute__((weak)) void board_fpu_save(NThreadBase*) {}
extern "C" __attribute__((weak)) void board_fpu_restore(NThreadBase*) {}
TSubScheduler& SubScheduler();   // a64s_ncutils.cpp
extern "C" TUint32 A64_MpidrIndex(TUint32 aMpidr);   // a64s_ncutils.cpp

/* =====================================================================
 *  Per-CPU / global singletons  (normally nkernsmp/x86/ncglob.cpp)
 * ===================================================================== */
TScheduler TheScheduler;
TSubScheduler TheSubSchedulers[KMaxCpus];
extern "C" { TSubScheduler* SubSchedulerLookupTable[256]; }

SBTraceData BTraceData;
#ifdef __USE_BTRACE_LOCK__
TSpinLock BTraceLock(TSpinLock::EOrderBTrace);
#endif

extern "C" {
SCpuIdleHandler CpuIdleHandler;
}

/* C++ static-dtor registration in a freestanding build: nothing runs atexit. */
extern "C" void* __dso_handle = 0;
extern "C" int __cxa_atexit(void (*)(void*), void*, void*) { return 0; }

/* =====================================================================
 *  Spin locks  (real: test-and-set on iLock; x86 LOCK RMW is a full fence)
 *  Order checking is a debug-only nicety and is elided here.
 * ===================================================================== */
EXPORT_C TSpinLock::TSpinLock(TUint /*aOrder*/) { iLock = 0; }

static inline void spin_acquire(volatile TUint64& l)
    {
    while (__atomic_exchange_n(&l, 1, __ATOMIC_ACQUIRE))
        A64_PAUSE();
    }
static inline void spin_release(volatile TUint64& l)
    { __atomic_store_n(&l, 0, __ATOMIC_RELEASE); }

EXPORT_C void TSpinLock::LockOnly()             { spin_acquire(iLock); }
EXPORT_C void TSpinLock::UnlockOnly()           { spin_release(iLock); }
EXPORT_C void TSpinLock::LockIrq()
    { NKern::DisableAllInterrupts(); spin_acquire(iLock); }
EXPORT_C void TSpinLock::UnlockIrq()
    { spin_release(iLock); NKern::EnableAllInterrupts(); }
EXPORT_C TInt TSpinLock::LockIrqSave()
    { TInt r = NKern::DisableAllInterrupts(); spin_acquire(iLock); return r; }
EXPORT_C void TSpinLock::UnlockIrqRestore(TInt aR)
    { spin_release(iLock); NKern::RestoreInterrupts(aR); }
EXPORT_C TBool TSpinLock::FlashPreempt()
    { return FALSE; }   /* single holder in the demo; no contention flash needed */

/* =====================================================================
 *  LAPIC IPI senders  (real)
 * ===================================================================== */
extern "C" void send_ipi(TUint32 aMpidr, TUint32 aVector)
    {
    A64_SendSgi(aMpidr, aVector);    /* one system-register write: no two-step ICR sequence to protect */
    }

extern "C" void send_resched_ipi(TInt aCpu)
    {
    send_ipi(TheSubSchedulers[aCpu].iSSX.iMpidr, RESCHED_IPI_VECTOR);
    }

/* Declared in nk_plat.h; the neutral core calls it with a CPU bit mask. */
extern "C" void send_resched_ipis(TUint32 aMask)
    {
    for (TInt i = 0; aMask; aMask >>= 1, ++i)
        if (aMask & 1)
            send_resched_ipi(i);
    }

extern "C" void send_resched_ipi_and_wait(TInt aCpu)
    {
    TSubScheduler& ss = TheSubSchedulers[aCpu];
    volatile TUint32& irqc = ss.iSSX.iIrqCount;
    volatile TInt& irqn    = ss.iSSX.iIrqNestCount;
    TUint32 irqc0 = irqc;
    __sync_synchronize();
    send_ipi(ss.iSSX.iMpidr, RESCHED_IPI_VECTOR);
    __sync_synchronize();
    while (!ss.iRescheduleNeededFlag || (irqn < 0 && irqc == irqc0))
        A64_PAUSE();
    __sync_synchronize();
    }

extern "C" void send_self_resched_ipi()
    {
    send_ipi(A64_Mpidr(), RESCHED_IPI_VECTOR);
    }

/* Deliver an arbitrary vector to THIS cpu via the APIC self-IPI shorthand (reliable,
 * unlike a physical-destination IPI aimed at one's own APIC id). Used by the NIrq demo. */
extern "C" void send_self_ipi(TUint32 aVector)
    {
    send_ipi(A64_Mpidr(), aVector);
    }

extern "C" void send_generic_ipis(TUint32 aMask)
    {
    for (TInt i = 0; aMask; aMask >>= 1, ++i)
        if (aMask & 1)
            send_ipi(TheSubSchedulers[i].iSSX.iMpidr, GENERIC_IPI_VECTOR);
    }

extern "C" void send_irq_ipi(TSubScheduler* aSS)
    {
    send_ipi(aSS->iSSX.iMpidr, TRANSFERRED_IRQ_VECTOR);
    }

/* =====================================================================
 *  Per-CPU timer / thread-time bookkeeping  (real, simplified)
 * ===================================================================== */
void TSubScheduler::SaveTimesliceTimer(NThreadBase* aT)
    {
    /* Deliberate no-op for this board's timer model: the LAPIC timeslice timer is armed
     * FIXED-PERIODIC and board_smp_tick zeroes iTime each tick, so there is no per-thread
     * one-shot INITCNT reload with a residual to reclaim. (The x86 backend reads CURRCNT
     * and rescales via iTimerFreqRI; that would only apply under per-thread timer arming,
     * a different model. The timestamp side IS real: NKern::Timestamp()=rdtsc with a
     * PIT-calibrated NKern::TimestampFrequency(), so UpdateThreadTimes accrues true time.) */
    (void)aT;
    }

void TSubScheduler::UpdateThreadTimes(NThreadBase* aOld, NThreadBase* aNew)
    {
    if (!aOld) aOld = iInitialThread;
    if (!aNew) aNew = iInitialThread;
    if (aNew != aOld)
        {
        TUint64 now = NKern::Timestamp();
        TUint64 delta = now - iLastTimestamp.i64;
        iLastTimestamp.i64 = now;
        if (aOld)
            {
            aOld->iLastRunTime.i64 = now;
            aOld->iTotalCpuTime.i64 += delta;
            }
        ++iReschedCount.i64;
        if (aNew)
            ++aNew->iRunCount.i64;
        }
    }

TBool TSubScheduler::Detached()
    {
    /* Platform hook: is this CPU detached from the scheduler (powered down)?
     * This port never powers cores down, so a subscheduler is never detached. */
    return FALSE;
    }

/* =====================================================================
 *  NKern preemption lock + current-thread/context  (real)
 * ===================================================================== */
/* pushfd/cli ... popfd, exactly as ncsched.cia's NKern::CurrentThread: "stop thread migration
 * between reading APIC ID and thread pointer". Without it a thread preempted between the two reads
 * and resumed on another CPU gets THAT CPU's current thread -- found as a spurious
 * NFastMutex::HeldByCurrentThread() failure (kern_priv.h:1025, RMessageK::SetAccepted's
 * __ASSERT_SYSTEM_LOCK) once keyboard input loaded the window server (X4.19). */
extern "C" NThread* NCurrentThreadL()
    {
    TInt irq = NKern::DisableAllInterrupts();
    NThread* t = SubScheduler().iCurrentThread;
    NKern::RestoreInterrupts(irq);
    return t;
    }

EXPORT_C NThread* NKern::CurrentThread()
    {
    return NCurrentThreadL();
    }

EXPORT_C TInt NKern::CurrentContext()
    {
    TInt irq = NKern::DisableAllInterrupts();     /* same migration race as NCurrentThreadL */
    TSubScheduler& ss = SubScheduler();
    TInt c = NKern::EThread;
    if (ss.iSSX.iIrqNestCount >= 0) c = NKern::EInterrupt;
    else if (ss.iInIDFC)            c = NKern::EIDFC;
    NKern::RestoreInterrupts(irq);
    return c;
    }

EXPORT_C void NKern::Lock()
    {
    /* pushfd/cli ... popfd, as ncsched.cia: the caller may already have interrupts off (spin-lock-irq
     * sections lock the kernel too), and reading "this cpu" must not race a migration. */
    TInt irq = NKern::DisableAllInterrupts();
    ++SubScheduler().iKernLockCount;
    NKern::RestoreInterrupts(irq);
    }

EXPORT_C NThread* NKern::LockC()
    {
    NKern::Lock();
    return SubScheduler().iCurrentThread;
    }

EXPORT_C void NKern::Unlock()
    {
    TInt irq = NKern::DisableAllInterrupts();
    TSubScheduler& ss = SubScheduler();
    if (--ss.iKernLockCount == 0)
        {
        /* Enter the scheduler if a thread reschedule OR an IDFC is pending: the x86
         * unlock tests the whole 32-bit word spanning iRescheduleNeededFlag(+0),
         * iDfcPendingFlag(+2) and iExIDfcPendingFlag(+3) (ncsched.cia). Reschedule
         * runs QueueDfcs for the pending IDFCs before selecting a thread. */
        if (ss.iRescheduleNeededFlag || ss.iDfcPendingFlag || ss.iExIDfcPendingFlag)
            {
            /* Reschedule's contract is "enter with the kernel LOCKED" (it enables interrupts while
             * it selects, and an interrupt taken there must not start a nested reschedule on the
             * temporary stack). So the count goes back to 1 for the call -- ncsched.cia does the
             * same; Reschedule returns with it 0. */
            ss.iKernLockCount = 1;
            TScheduler::Reschedule();   /* returns kernel-unlocked, interrupts disabled */
            }
        /* Drain any CROSS-CORE reschedule IPIs the neutral ReadyT queued on THIS
         * cpu while it was kernel-locked: waking a thread whose home cpu != this
         * cpu sets ss0.iReschedIPIs |= targetCpuMask (sched.cpp:637) rather than
         * rescheduling locally. We must kick those remote cpus now that this cpu
         * reenables preemption -- exactly the x86 unlock tail (ncsched.cia:286):
         * atomically swap the mask to 0 and, if non-empty, send the reschedule IPIs.
         * This is what makes a fast-sem signal on one core actually wake a thread
         * blocked on another. */
        /* (SubScheduler() again, not `ss`: Reschedule may have brought us back on another cpu.) */
        TUint32 ipis = __atomic_exchange_n(&SubScheduler().iReschedIPIs, 0, __ATOMIC_SEQ_CST);
        if (ipis)
            send_resched_ipis(ipis);
        }
    NKern::RestoreInterrupts(irq);
    }

EXPORT_C TInt NKern::PreemptionPoint()
    {
    TInt irq = NKern::DisableAllInterrupts();
    TSubScheduler& ss = SubScheduler();
    if (ss.iKernLockCount == 1 && ss.iRescheduleNeededFlag)
        {
        TScheduler::Reschedule();               /* returns unlocked, perhaps on ANOTHER cpu */
        SubScheduler().iKernLockCount = 1;      /* ... so re-lock the cpu we are on now */
        NKern::RestoreInterrupts(irq);
        return TRUE;
        }
    NKern::RestoreInterrupts(irq);
    return FALSE;
    }

/* =====================================================================
 *  TScheduler::Reschedule  (per-CPU; drives the neutral SelectNextThread)
 * ===================================================================== */
/* Second half: runs on THIS cpu's temporary stack (the initial thread's), kernel locked, with the
 * outgoing thread already parked. Interrupts are ENABLED for the selection, as in ncsched.cia: another
 * cpu may be inside send_resched_ipi_and_wait() holding the very thread lock SelectNextThread wants,
 * waiting for us to take its IPI -- with interrupts off here that is a deadlock (seen: ~1 boot in 3
 * on two cpus). Never returns: ends by resuming the selected thread. */
extern "C" void __phore_resched_park(void** aOldSPSlot, void* const* aTempSPSlot, void (*aFn)(void*), void* aArg);
extern "C" void __phore_resume(void* aNewSP);
extern "C" void send_resched_ipis(TUint32 aMask);

extern "C" void A64_ReschedSelect(void* aSS)
    {
    TSubScheduler& ss = *(TSubScheduler*)aSS;
    NThreadBase* next;
    for (;;)
        {
        A64_STI();
        next = ss.SelectNextThread();           /* neutral: bookkeeping + iCurrentThread */
        if (next)
            break;
        /* No thread is ready for this cpu (ncsched.cia no_thread): kick whoever we owe an IPI, then
         * sleep until an interrupt queues an IDFC; run those and select again. */
        do  {
            A64_CLI();
            TUint32 ipis = __atomic_exchange_n(&ss.iReschedIPIs, 0, __ATOMIC_SEQ_CST);
            if (ipis)
                send_resched_ipis(ipis);
            A64_IDLE();
            } while (!ss.iDfcPendingFlag && !ss.iExIDfcPendingFlag);
        A64_CLI();
        ss.QueueDfcs();
        }
    fpu_set_ts();
    board_set_rsp0((void*)(((TLinAddr)next->iStackBase + (TLinAddr)next->iStackSize) & ~(TLinAddr)0xF));
    board_switch_as(next);
    A64_CLI();
    __phore_resume((void*)next->iSavedSP);
    }

extern "C" void do_forced_exit(NThreadBase* aT)     /* nkernsmp/x86/ncsched.cpp */
    {
    __NK_ASSERT_ALWAYS(aT->iFastMutexDefer != 1);
    aT->iFastMutexDefer = 0;
    aT->Exit();
    }

void TScheduler::Reschedule()
    {
    /* Enter: kernel locked on THIS cpu, interrupts enabled or disabled. Return: kernel unlocked,
     * interrupts disabled. amd64 sibling of ncsched.cia TScheduler::Reschedule, same structure:
     *   start_resched: run pending IDFCs; if a reschedule is (still) needed, PARK the current thread
     *   and do the selection on the temporary stack; when this thread is next resumed -- on whatever
     *   cpu -- go round again, because more may have been queued for that cpu meanwhile. */
    A64_CLI();
    for (;;)
        {
        TSubScheduler& ss = SubScheduler();
        if (ss.iDfcPendingFlag || ss.iExIDfcPendingFlag)
            ss.QueueDfcs();
        if (!ss.iRescheduleNeededFlag)
            break;
        NThreadBase* me = ss.iCurrentThread;
        board_fpu_save(me);
        __phore_resched_park((void**)&me->iSavedSP, (void* const*)&ss.iInitialThread->iSavedSP, &A64_ReschedSelect, &ss);
        /* Running again, interrupts off, as the thread that was parked -- possibly on ANOTHER cpu
         * (migration, idle pull), so nothing per-cpu may be carried across the park. */
        board_fpu_restore(me);
        }
    TSubScheduler& ss = SubScheduler();
    NThreadBase* t = ss.iCurrentThread;
    if (t->iCsFunction == NThreadBase::ECSDivertPending)
        {
        /* resched_thread_divert: this thread was killed while it could not run; it exits here. */
        TUint32 ipis = __atomic_exchange_n(&ss.iReschedIPIs, 0, __ATOMIC_SEQ_CST);
        if (ipis)
            send_resched_ipis(ipis);
        A64_STI();
        do_forced_exit(t);                      /* kernel still locked, as it expects; never returns */
        }
    ss.iKernLockCount = 0;
    }

/* =====================================================================
 *  Fault / idle  (real)
 * ===================================================================== */
// weak: a nanokernel-only image uses this; the full kernel (ekern) supplies the real one
extern "C" __attribute__((weak)) void ExcFault(TAny*)
    { for (;;) { A64_CLI(); __asm__ __volatile__("wfi"); } }

// weak: a nanokernel-only image uses this; the full kernel (ekern) supplies the real one
__attribute__((weak)) EXPORT_C void NKFault(const char*, TInt)
    { for (;;) { A64_CLI(); __asm__ __volatile__("wfi"); } }

EXPORT_C TBool BTrace::Out(TUint32, TUint32, TUint32, TUint32) { return EFalse; }
EXPORT_C TBool BTrace::OutBig(TUint32, TUint32, const TAny*, TInt) { return EFalse; }

/* =====================================================================
 *  Thread bring-up  (first-cut; primes the cooperative switch frame)
 * ===================================================================== */
/* First code a new nanothread runs (amd64 sibling of ncthrd.cia __StartThread). It is entered by
 * the `ret` of __phore_switch, i.e. INSIDE the reschedule that selected it: interrupts off, kernel
 * locked, and Reschedule()'s epilogue will never run on this stack -- so do that epilogue's work
 * here (flush pending resched IPIs, drop the kernel lock), enable interrupts, run the thread
 * function, and exit the thread if it ever returns. */
extern "C" void send_resched_ipis(TUint32 aMask);
extern "C" void A64_ThreadStart(TAny* aParameterBlock, NThreadFunction aFunction)
    {
    TSubScheduler& ss = SubScheduler();
    TUint32 ipis = __e32_atomic_swp_ord32(&ss.iReschedIPIs, 0);
    if (ipis)
        send_resched_ipis(ipis);
    board_fpu_restore(0);
    ss.iKernLockCount = 0;
    A64_STI();
    (*aFunction)(aParameterBlock);
    NKern::Exit();
    }
extern "C" void __phore_thread_start(void);   /* a64_ctxsw.S: x19 = parameter block, x20 = function */

TInt NThread::Create(SNThreadCreateInfo& aInfo, TBool aInitial)
    {
    if (!aInfo.iStackBase || aInfo.iStackSize<0x100)
        return KErrArgument;
    new (this) NThread;
    TSubScheduler* ss = 0;
    if (aInitial)
        {
        /* The thread ALREADY RUNNING on this CPU becomes its initial (null) thread: this is what
         * brings a CPU's subscheduler to life (nkernsmp/x86/ncthrd.cpp). */
        TInt cpu = __e32_atomic_add_ord32(&TheScheduler.iNumCpus, 1);
        aInfo.iCpuAffinity = cpu;
        TUint32 mpidr = A64_Mpidr();
        ss = &TheSubSchedulers[cpu];
        ss->iSSX.iMpidr = mpidr;
        A64_SYSREG_WRITE(tpidr_el1, (TLinAddr)ss);   /* SubScheduler() = one mrs from now on */
        ss->iSSX.iIrqNestCount = -1;            /* "not in an ISR" (NKern::CurrentContext tests >= 0) */
        ss->iCurrentThread = this;
        ss->iDeferShutdown = 0;
        SubSchedulerLookupTable[A64_MpidrIndex(mpidr)] = ss;
        iRunCount.i64 = UI64LIT(1);
        iActiveState = 1;
        if (cpu)
            {   /* an application processor joining (X64_TODO: NIrq::HwInit2AP when BootAP is real) */
            __e32_atomic_ior_ord32(&TheScheduler.iThreadAcceptCpus, 1<<cpu);
            __e32_atomic_ior_ord32(&TheScheduler.iIpiAcceptCpus, 1<<cpu);
            __e32_atomic_ior_ord32(&TheScheduler.iCpusNotIdle, 1<<cpu);
            __e32_atomic_add_ord32(&TheScheduler.iCCRequestLevel, 1);
            }
        }
    TInt r = NThreadBase::Create(aInfo, aInitial);
    if (r != KErrNone)
        return r;
    if (!aInitial)
        {
        /* Prime the supervisor stack with the frame __phore_switch pops (r15 r14 r13 r12 rbx rbp,
         * then the return address). A parameter block is COPIED onto the new stack first: the
         * caller's copy usually lives on a stack frame that is gone by the time the thread runs. */
        TLinAddr sp = ((TLinAddr)iStackBase + (TLinAddr)iStackSize) & ~(TLinAddr)0xF;
        TLinAddr pb = (TLinAddr)aInfo.iParameterBlock;
        if (aInfo.iParameterBlockSize)
            {
            sp = (sp - (TLinAddr)aInfo.iParameterBlockSize) & ~(TLinAddr)0xF;
            const TUint8* src = (const TUint8*)aInfo.iParameterBlock;   /* no libc in the nkern-only image */
            for (TInt i=0; i<aInfo.iParameterBlockSize; ++i)
                ((TUint8*)sp)[i] = src[i];
            pb = sp;
            }
        sp -= 12 * 8;                           /* the 96-byte frame __phore_resume pops (sp stays 16-aligned) */
        TUint64* f = (TUint64*)sp;              /* 64-bit slots even though pointers are 32-bit (ILP32) */
        for (TInt i = 0; i < 12; ++i) f[i] = 0;
        f[0]  = pb;                             /* x19 */
        f[1]  = (TLinAddr)aInfo.iFunction;      /* x20 */
        f[11] = (TLinAddr)__phore_thread_start; /* x30: UNPARK's ret lands there */
        iSavedSP = sp;
        }
    else
        {
        NKern::EnableAllInterrupts();
        iActiveState = 1;                       /* nccpu.cpp InitTimestamp, boot-CPU path */
        ss->iLastTimestamp.i64 = 0;
        iLastActivationTime.i64 = 0;
        }
    AddToEnumerateList();
    InitLbInfo();
    return KErrNone;
    }

void NThread::CompleteContextSave() {}                 /* A64_TODO (lazy VFP flush, full kernel) */
void NThread::GetUserContext(TArmRegSet&, TUint32&) {}  /* A64_TODO (full kernel: from SThreadExcStack) */
void NThread::SetUserContext(const TArmRegSet&, TUint32&) {}
void NThread::GetSystemContext(TArmRegSet&, TUint32&) {}

/* NThreadBase hooks + tied-event/fast-mutex interlocks. OnKill/OnExit are empty on IA-32 as well
 * (x86/ncthrd.cpp); the rest are ports (CheckFastMutexDefer X4.27b, tied events X4.38). */
void NThreadBase::OnKill() {}
void NThreadBase::OnExit() {}
/* Port of x86/ncthrd.cia CheckFastMutexDefer (X4.27b). Called with the kernel lock held from
 * SuspendOrKill and the scheduler: if this thread holds a fast mutex, set the mutex's contention flag
 * (so its NFastMutex::Signal takes DoSignalL, which runs the deferred iCsFunction) and return TRUE --
 * the suspend/kill/migration is deferred until the mutex is released. The old `return FALSE` stub let
 * RThread::Suspend() park a thread that was HOLDING THE SYSTEM LOCK (Exec::ThreadSuspend runs under it):
 * the S60 boot hang, where TimerThread and everything else queued behind akniconsrv forever. */
TBool NThreadBase::CheckFastMutexDefer()
	{
	TLinAddr h = (TLinAddr)iHeldFastMutex;
	NFastMutex* m = (NFastMutex*)(h & ~(TLinAddr)3);
	if (!m)
		return FALSE;									// no mutex
	NThreadBase* expect = this;
	NThreadBase* contended = (NThreadBase*)((TLinAddr)this | 1);
	// if m->iHoldingThread==this set the contention flag and return TRUE; TRUE also if it is already set
	if (__e32_atomic_cas_ord_ptr(&m->iHoldingThread, &expect, contended) || expect == contended)
		return TRUE;
	if (h & 1)
		iHeldFastMutex = 0;								// was being released and now is: nothing to defer
	return FALSE;										// being acquired but not yet held
	}
/* Tied events (X4.38): ports of x86/ncthrd.cia. An interrupt/IDFC/timer handler "tied" to a thread or
 * group counts itself into iEventState while it runs, so the thread cannot be made ready on another CPU
 * meanwhile (the ready is deferred and done by the i_IDfcMem IDFC when the last event ends).
 *   iEventState: bits 16-31 event count (EEventCountInc), 15 EEventParent (count lives in the group),
 *   14 EDeferredReady, 8-12 thread CPU, 0-4 event CPU.
 * The IA-32 current-CPU trick (`str` TSS selector - 0x28 >> 3) is SubScheduler().iCpuNum here, as in the
 * DFC state changes below. One deliberate difference: IA-32's EndTiedEvent tests the parent flag as
 * 0x800 -- a typo in the 2010 source (BeginTiedEvent and ARM's EndTiedEvent use EEventParent, 0x8000). */
static inline TBool EventCas(volatile TUint32* aState, TUint32& aOld, TUint32 aNew)
	{
	return __atomic_compare_exchange_n(aState, &aOld, aNew, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
	}

// the group whose event count a thread's tied events use (iParent, or iNewParent while it migrates)
static NSchedulable* EventParent(NSchedulable* aS)
	{
	__e32_memory_barrier();
	NSchedulable* p = aS->iParent;
	if (p && p != aS)
		return p;
	if (p)													// parent not yet updated: iNewParent
		{
		__e32_memory_barrier();
		p = ((NThreadBase*)aS)->iNewParent;
		if (p)
			return p;
		__e32_memory_barrier();
		p = aS->iParent;									// iNewParent cleared: iParent must be set now
		if (p != aS)
			return p;
		}
	NKFault("EventParent", (TInt)(TLinAddr)aS);
	return 0;
	}

TInt NSchedulable::BeginTiedEvent()
	{
	TUint32 orig = __atomic_fetch_add(&iEventState, (TUint32)EEventCountInc, __ATOMIC_SEQ_CST);
	if (orig & EEventParent)
		orig = __atomic_fetch_add(&EventParent(this)->iEventState, (TUint32)EEventCountInc, __ATOMIC_SEQ_CST);
	return orig & EEventCpuMask;
	}

// count one event out of aS; the last one sets event CPU = thread CPU and runs a deferred ready
static void EndTiedEventOn(NSchedulable* aS)
	{
	TUint32 old = __atomic_load_n(&aS->iEventState, __ATOMIC_RELAXED), nw;
	do	{
		nw = old - NSchedulable::EEventCountInc;
		if (nw < NSchedulable::EEventCountInc)
			nw = (nw & ~0xFFu) | ((nw >> NSchedulable::EThreadCpuShift) & NSchedulable::EEventCpuMask);
		} while (!EventCas(&aS->iEventState, old, nw));
	if (nw < NSchedulable::EEventCountInc && (nw & NSchedulable::EDeferredReady))
		((TDfc*)aS->i_IDfcMem)->Add();
	}

void NSchedulable::EndTiedEvent()
	{
	if (__atomic_load_n(&iEventState, __ATOMIC_SEQ_CST) & EEventParent)
		{
		EndTiedEventOn(EventParent(this));
		__atomic_fetch_sub(&iEventState, (TUint32)EEventCountInc, __ATOMIC_SEQ_CST);	// the thread's own count
		}
	else
		EndTiedEventOn(this);
	}

// A lone thread or a group becoming ready on aCpu: record the thread CPU; if events are running and the
// event CPU differs, set EDeferredReady and return TRUE (the ready waits); no events: event CPU = aCpu.
TBool NSchedulable::TiedEventReadyInterlock(TInt aCpu)
	{
	const TUint32 cpu = (TUint32)aCpu & EEventCpuMask;
	TUint32 old = __atomic_load_n(&iEventState, __ATOMIC_RELAXED), nw;
	do	{
		nw = (old & ~EThreadCpuMask) | (cpu << EThreadCpuShift);
		if (nw >= EEventCountInc)
			{
			if ((nw & 0xFFu) != cpu)
				nw |= EDeferredReady;
			}
		else
			nw = (nw & ~0xFFu) | cpu;
		} while (!EventCas(&iEventState, old, nw));
	return ((old ^ nw) & EDeferredReady) != 0;
	}

// A thread leaving its group: if no events are running, both CPUs = this CPU, parent flag clear -> TRUE
TBool NThreadBase::TiedEventLeaveInterlock()
	{
	const TUint32 cpu = SubScheduler().iCpuNum & EEventCpuMask;
	TUint32 old = __atomic_load_n(&iEventState, __ATOMIC_RELAXED);
	do	{
		if (old >= EEventCountInc)
			return FALSE;
		} while (!EventCas(&iEventState, old, cpu | (cpu << EThreadCpuShift)));
	return TRUE;
	}

// A thread joining a group: if no events are running, set the parent flag -> TRUE
TBool NThreadBase::TiedEventJoinInterlock()
	{
	TUint32 old = __atomic_load_n(&iEventState, __ATOMIC_RELAXED);
	do	{
		if (old >= EEventCountInc)
			return FALSE;
		} while (!EventCas(&iEventState, old, old | EEventParent));
	return TRUE;
	}

/* NFastSemaphore block/wake primitives (x86 32-bit: nkerns.cia). SMP slice 3 step 3.
 *
 * The 32-bit .cia packs the waiting thread pointer INTO the 32-bit iCount when the
 * count goes negative: iCount = (thread>>2)|0x80000000. That trick cannot hold a
 * 64-bit amd64 NThreadBase* -- which is precisely why this was a stub. But a fast
 * semaphore has a SINGLE possible waiter: its iOwningThread (documented invariant --
 * "only the thread that owns a fast semaphore can wait on it", nkern.cpp:505). So on
 * amd64 we don't need to encode the pointer at all: iCount<0 is purely the
 * "owner is blocked" flag, and the thread to wake is always iOwningThread.
 *
 * The observable contract is byte-for-byte the .cia's:
 *   Dec  returns the ORIGINAL count (Wait(): !=0 => was signalled, no block; 0 => block)
 *   Inc  returns the thread to wake (or 0); when count was negative it consumes one
 *        signal for the released waiter (new = aCount-1), matching fsinc's "eax<0" path
 *   DoReset returns the blocked thread (or 0) and zeroes the count
 * All are full barriers (SEQ_CST cmpxchg; x86 TSO makes the LOCK RMW a fence), so the
 * iCount transition is correctly ordered against iWaitState in Wait()/Signal(). */
TInt NFastSemaphore::Dec(NThreadBase*)
    {
    TInt old = __atomic_load_n(&iCount, __ATOMIC_RELAXED), nw;
    do  {
        nw = (old > 0) ? (old - 1) : -1;   // count>0: consume a signal; ==0: mark blocked
        } while (!__atomic_compare_exchange_n(&iCount, &old, nw, /*weak=*/false,
                                              __ATOMIC_SEQ_CST, __ATOMIC_RELAXED));
    return old;
    }

NThreadBase* NFastSemaphore::Inc(TInt aCount)
    {
    TInt old = __atomic_load_n(&iCount, __ATOMIC_RELAXED), nw;
    do  {
        nw = (old >= 0) ? (old + aCount) : (aCount - 1);  // <0: consume one for the waiter
        } while (!__atomic_compare_exchange_n(&iCount, &old, nw, /*weak=*/false,
                                              __ATOMIC_SEQ_CST, __ATOMIC_RELAXED));
    return (old < 0) ? iOwningThread : 0;
    }

NThreadBase* NFastSemaphore::DoReset()
    {
    TInt old = __atomic_exchange_n(&iCount, 0, __ATOMIC_SEQ_CST);
    return (old < 0) ? iOwningThread : 0;
    }

/* ============================================================================
 *  IDFC state machine (x86: nkernsmp/x86/ncthrd.cia)
 *
 * The 16-bit iDfcState (=i8816.iHState16) low byte encodes the IDFC phase + owning
 * cpu number (008n = on cpu n's normal IDFC queue, 00Cn = running on cpu n, 00En =
 * running + re-add pending, 006n/002g = idle-queue), high byte = a bitmask of cpus
 * trying to cancel it (always 0 in this port -- we do not exercise Cancel). The x86
 * .cia derives the CURRENT cpu number from the TSS selector (`str dx; sub dl,0x28;
 * shr dl,3`); amd64 loads no per-cpu TSS, so we take it from SubScheduler().iCpuNum.
 *
 * Ported: the three functions on the untied-IDFC-on-home-cpu path
 *   Add -> AddStateChange ; QueueDfcs -> RunIDFCStateChange -> (run) -> EndIDFCStateChange.
 * The cancel / cross-cpu-transfer / idle-queue variants followed in X4.38 (below).
 * ============================================================================ */
#define DFC_CAS(st, oldv, nw) \
    __atomic_compare_exchange_n((st), &(oldv), (TUint16)(nw), false, \
                                __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)

/* Add(): 0000->008n, 00Cn->00En, all other states unchanged. Return original. */
TUint32 TDfc::AddStateChange()
    {
    TUint n = SubScheduler().iCpuNum;
    volatile TUint16* st = &i8816.iHState16;
    TUint16 old = __atomic_load_n(st, __ATOMIC_RELAXED), nw;
    do  {
        if (old == 0)                          nw = (TUint16)(0x80 | n);
        else if (old >= 0xC0 && old <= 0xDF)   nw = (TUint16)(old + 0x20);
        else                                   nw = old;
        } while (!DFC_CAS(st, old, nw));
    return old;
    }

/* Run (in QueueDfcs, just before running the IDFC):
 *   002g->00Cn, 008n->00Cn, 00An->00Cn, XXYY->XX00, XX00->0000. Return original. */
TUint32 TDfc::RunIDFCStateChange()
    {
    TUint n = SubScheduler().iCpuNum;
    volatile TUint16* st = &i8816.iHState16;
    TUint16 old = __atomic_load_n(st, __ATOMIC_RELAXED), nw;
    do  {
        TUint hi = old >> 8, lo = old & 0xFF;
        if (hi)                                     // XXYY / XX00 (cancel pending)
            nw = (TUint16)(lo ? (old & 0xFF00) : 0);
        else if ((lo & 0xFE) == 0x20)               // 002g (idle generation)
            nw = (TUint16)(0xC0 | n);
        else                                        // 008n / 00An -> 00Cn
            {
            if (lo < 0x80 || lo > 0xBF || (lo & 0x1F) != n)
                NKFault("RunIDFCStateChange", (TInt)old);
            nw = (TUint16)(0xC0 | n);
            }
        } while (!DFC_CAS(st, old, nw));
    return old;
    }

/* End (in QueueDfcs, just after running the IDFC): first swap aS->iCurrentIDFC with 0;
 * if it was not this IDFC, return 0xFFFFFFFF and leave the state alone. Else:
 *   00Cn->0000, 00En->008n, 006n->006n, XXCn->XX00, XXEn->XX00, XX6n->XX00, XX00->0000. */
TUint32 TDfc::EndIDFCStateChange(TSubScheduler* aS)
    {
    if (__atomic_exchange_n(&aS->iCurrentIDFC, (TDfc*)0, __ATOMIC_SEQ_CST) != this)
        return 0xFFFFFFFFu;
    TUint n = SubScheduler().iCpuNum;
    volatile TUint16* st = &i8816.iHState16;
    TUint16 old = __atomic_load_n(st, __ATOMIC_RELAXED), nw;
    do  {
        TUint hi = old >> 8, lo = old & 0xFF, ph = lo & 0xE0;
        if (lo == 0)                                // XX00 -> 0000
            nw = 0;
        else
            {
            if ((lo & 0x1F) != n || (ph != 0x60 && ph != 0xC0 && ph != 0xE0))
                NKFault("EndIDFCStateChange", (TInt)old);
            if (hi)                                 // XX6n/XXCn/XXEn -> XX00
                nw = (TUint16)(old & 0xFF00);
            else if (ph == 0xC0)                    // 00Cn -> 0000
                nw = 0;
            else if (ph == 0x60)                    // 006n -> 006n
                nw = (TUint16)lo;
            else                                    // 00En -> 008n
                nw = (TUint16)(lo - 0x60);
            }
        } while (!DFC_CAS(st, old, nw));
    return old;
    }

/* MoveToFinalQStateChange: QueueDfcs calls this on a DFC-with-queue (also a mutating
 * NTimer) to move it from the per-cpu pending IDFC queue onto its DFC queue's final
 * state (dfcs.cpp:727). Ported from ncthrd.cia:
 *   002g -> 0001 (idle generation)     008n -> 0001 (on cpu n's normal queue)  [ok]
 *   XXYY -> XX00 (cancel pending, low nonzero)   XX00 -> 0000                  [not ok]
 * Return the ORIGINAL state; the caller tests (orig>>5)==1 (was 002g) or ==4 (was 008n). */
TUint32 TDfc::MoveToFinalQStateChange()
    {
    volatile TUint16* st = &i8816.iHState16;
    TUint16 old = __atomic_load_n(st, __ATOMIC_RELAXED), nw;
    do  {
        TUint16 hi = (TUint16)(old & 0xFF00);
        if (hi == 0)              nw = 0x0001;      // 002g / 008n -> 0001
        else if (old & 0x00FF)    nw = hi;          // XXYY -> XX00
        else                      nw = 0;           // XX00 -> 0000
        } while (!DFC_CAS(st, old, nw));
    return old;
    }

/* The rest of the IDFC state machine (X4.38, ports of x86/ncthrd.cia -- cancel, cross-CPU transfer,
 * queue-on-idle -- were X64_TODO stubs returning 0, i.e. "state was 0000": a Cancel() of a queued IDFC
 * would have believed it was not queued). */

/* Just after running an IDFC queued on idle: 006n->002g (g = TheScheduler.iIdleGeneration), XX6n->XX00. */
TUint32 TDfc::EndIDFCStateChange2()
    {
    TUint n = SubScheduler().iCpuNum;
    volatile TUint16* st = &i8816.iHState16;
    TUint16 old = __atomic_load_n(st, __ATOMIC_RELAXED), nw;
    do  {
        TUint lo = old & 0xFF;
        if (lo < 0x60 || lo > 0x7F || (lo & 0x1F) != n)
            NKFault("EndIDFCStateChange2", (TInt)old);
        nw = (old & 0xFF00) ? (TUint16)(old & 0xFF00) : (TUint16)(0x20 | TheScheduler.iIdleGeneration);
        } while (!DFC_CAS(st, old, nw));
    return old;
    }

/* Moving an IDFC to CPU aCpu (target's ExIDfcLock held): 002g/008n->00Am, XXYY->XX00, XX00->0000.
 * Follows ARM's version, which admits XX00 (IA-32's faults on it, against its own comment). */
TUint32 TDfc::TransferIDFCStateChange(TInt aCpu)
    {
    TUint n = SubScheduler().iCpuNum;
    volatile TUint16* st = &i8816.iHState16;
    TUint16 old = __atomic_load_n(st, __ATOMIC_RELAXED), nw;
    do  {
        TUint lo = old & 0xFF;
        if (old == 0 || (lo && (lo & 0xFE) != 0x20 && lo != (0x80 | n)))
            NKFault("TransferIDFCStateChange", (TInt)old);
        if (!(old & 0xFF00))       nw = (TUint16)(0xA0 | (aCpu & 0x1F));   // 002g/008n -> 00Am
        else if (lo)               nw = (TUint16)(old & 0xFF00);           // XXYY -> XX00
        else                       nw = 0;                                 // XX00 -> 0000
        } while (!DFC_CAS(st, old, nw));
    return old;
    }

/* Before cancelling: 0000->0000, else set this CPU's bit (8+n) in the high byte. */
TUint32 TDfc::CancelInitialStateChange()
    {
    const TUint16 bit = (TUint16)(1u << (8 + SubScheduler().iCpuNum));
    volatile TUint16* st = &i8816.iHState16;
    TUint16 old = __atomic_load_n(st, __ATOMIC_RELAXED), nw;
    do  {
        nw = old ? (TUint16)(old | bit) : (TUint16)0;
        } while (!DFC_CAS(st, old, nw));
    return old;
    }

/* End of a cancel: XXYY->XX00, XX00->0000. */
TUint32 TDfc::CancelFinalStateChange()
    {
    volatile TUint16* st = &i8816.iHState16;
    TUint16 old = __atomic_load_n(st, __ATOMIC_RELAXED), nw;
    do  {
        nw = (old & 0x00FF) ? (TUint16)(old & 0xFF00) : (TUint16)0;
        } while (!DFC_CAS(st, old, nw));
    return old;
    }

/* QueueOnIdle() (IdleSpinLock held): 0000->002g, 00Cn->006n, anything else unchanged. */
TUint32 TDfc::QueueOnIdleStateChange()
    {
    volatile TUint16* st = &i8816.iHState16;
    TUint16 old = __atomic_load_n(st, __ATOMIC_RELAXED), nw;
    do  {
        if (old == 0)                           nw = (TUint16)(0x20 | TheScheduler.iIdleGeneration);
        else if (old >= 0xC0 && old <= 0xDF)    nw = (TUint16)(old - 0x60);
        else                                    nw = old;
        } while (!DFC_CAS(st, old, nw));
    return old;
    }

/* ResetState: real (x86 ncthrd.cia:786). Atomically zero the 16-bit iDfcState after
 * a DFC has been dequeued+run, so it may be re-enqueued; the .cia crashes (int 0xf8)
 * if the state was already 0 (a DFC that reset twice / was never queued). */
void TDfc::ResetState()
    {
    /* iDfcState is the macro (i8816.iHState16); reference the union field directly since
     * that macro (dfcs.h) is not in scope in this backend TU. */
    volatile TUint16* st = &i8816.iHState16;
    TUint16 old = __atomic_exchange_n(st, (TUint16)0, __ATOMIC_SEQ_CST);
    if (old == 0)
        NKFault("TDfc::ResetState", 0);   /* state was already idle -- as .cia int 0xf8 */
    }

/* Core control (dynamic CPU power up/down). This port keeps all cores online. */
TBool TScheduler::CoreControlSupported() { return FALSE; }
void TScheduler::CCInitiatePowerUp(TUint32) {}
void TScheduler::CCIndirectPowerDown(TAny*) {}
void TScheduler::DoFrequencyChanged(TAny*) {}

/* ============================================================================
 *  NIrq / NIrqHandler dispatch state machine (x86: nkernsmp/x86/ncirq.cia)
 *
 * These are the atomic state transitions the neutral NIrq::HwIsr / NIrqHandler::Activate
 * dispatch path runs on iIState (NIrq: bits0-7 flags, 8-15 running-cpu, 16-31 run count)
 * and iHState (NIrqHandler: byte0 iHType, bits 8-31 flags incl EActive/EDisable/run
 * count). Ported as C++ CAS loops from the .cia; the one amd64 change is in EnterIsr,
 * where the x86 reads the running-cpu number from the TSS selector -- we use
 * SubScheduler().iCpuNum (no per-cpu TSS). The HW glue below drives the local APIC.
 * ============================================================================ */
/* The hardware half is the REAL IA-32 nkernsmp/x86/ncirq.cpp (IOAPIC + local APIC) in the full
 * kernel (make_ekern.sh, X4.21); these weak no-IOAPIC stand-ins only serve the nanokernel demo. */
#define NIRQ_WEAK __attribute__((weak))
NIRQ_WEAK void NIrq::HwInit() {}
NIRQ_WEAK void NIrq::HwInit0() {}
NIRQ_WEAK void NIrq::HwEnable() {}
NIRQ_WEAK void NIrq::HwDisable() {}
NIRQ_WEAK void NIrq::HwEoi() { A64_GicEoi(iVector); }
NIRQ_WEAK TBool NIrq::HwPending() { return FALSE; }
NIRQ_WEAK void NIrq::HwWaitCpus() {}

/* EnterIsr: if run count 0 record the running cpu; increment run count; if it would
 * reach 2 and the IRQ is not counting/raw, don't increment (limit pending to 1). Then
 * spin until EWait is clear. Return iIState immediately before the increment. */
TUint32 NIrq::EnterIsr()
    {
    TUint32 cpu = SubScheduler().iCpuNum;
    for (;;)
        {
        TUint32 old = __atomic_load_n(&iIState, __ATOMIC_RELAXED);
        TUint32 nw  = old;
        if ((nw >> 16) == 0)                          // run count 0: record cpu (bits 8-15)
            nw = (nw & ~0x0000FF00u) | (cpu << 8);
        nw += 0x00010000u;                            // ++run count
        TBool commit = TRUE;
        if ((nw >> 16) >= 2 && !(nw & (ECount | ERaw)))
            commit = FALSE;                           // limit pending count to 1
        if (!commit ||
            __atomic_compare_exchange_n(&iIState, &old, nw, false,
                                        __ATOMIC_SEQ_CST, __ATOMIC_RELAXED))
            {
            while (__atomic_load_n(&iIState, __ATOMIC_RELAXED) & EWait)   // wait for EWait clear
                A64_PAUSE();
            __sync_synchronize();
            return old;
            }
        }
    }

/* IsrDone: atomically decrement run count; return TRUE if still nonzero afterwards. */
TBool NIrq::IsrDone()
    {
    TUint32 old = __atomic_fetch_add(&iIState, 0xFFFF0000u, __ATOMIC_SEQ_CST);
    return (TBool)(((old >> 16) - 1) != 0);
    }

/* Wait/Done: EWait interlock against ISRs (thread context). Wait spins (interrupts on)
 * until run count 0 and EWait clear, then sets EWait; Done clears it. */
void NIrq::Wait()
    {
    for (;;)
        {
        A64_CLI();
        TUint32 old = __atomic_load_n(&iIState, __ATOMIC_RELAXED);
        if ((old & 0xFFFF0001u) == 0 &&
            __atomic_compare_exchange_n(&iIState, &old, old | EWait, false,
                                        __ATOMIC_SEQ_CST, __ATOMIC_RELAXED))
            return;                                   // success: returns interrupts-disabled
        A64_STI(); A64_PAUSE();
        }
    }
void NIrq::Done()
    {
    __atomic_and_fetch(&iIState, ~(TUint32)EWait, __ATOMIC_SEQ_CST);
    A64_STI();
    }

/* DoSetEnabled: if neither EUnbind nor ENotReady set, clear EDisable and EBind.
 * Return original iHState. */
TUint32 NIrqHandler::DoSetEnabled()
    {
    TUint32 old = __atomic_load_n(&iHState, __ATOMIC_RELAXED), nw;
    do  {
        nw = old;
        if (!(nw & (EUnbind | ENotReady)))
            nw &= ~(TUint32)(EDisable | EBind);
        } while (!__atomic_compare_exchange_n(&iHState, &old, nw, false,
                                              __ATOMIC_SEQ_CST, __ATOMIC_RELAXED));
    return old;
    }

/* DoActivate: increment run count by aCount if ECount set or run count was 0; if not
 * EUnbind/EDisable, set EActive. Return original iHState. */
TUint32 NIrqHandler::DoActivate(TInt aCount)
    {
    TUint32 old = __atomic_load_n(&iHState, __ATOMIC_RELAXED), nw;
    do  {
        nw = old;
        if ((nw >> 16) == 0 || (nw & ECount))
            nw += ((TUint32)aCount << 16);
        if (!(nw & (EUnbind | EDisable)))
            nw |= EActive;
        } while (!__atomic_compare_exchange_n(&iHState, &old, nw, false,
                                              __ATOMIC_SEQ_CST, __ATOMIC_RELAXED));
    return old;
    }

/* EventBegin: decrement run count. Return original iHState. */
TUint32 NIrqHandler::EventBegin()
    {
    return __atomic_fetch_add(&iHState, 0xFFFF0000u, __ATOMIC_SEQ_CST);
    }

/* EventDone: if run count now 0 or EDisable/EUnbind set, clear EActive. Return the
 * original iHState but with EActive cleared iff we cleared it (matches the .cia's
 * "or dh,0xDF; and ah,dh"). */
TUint32 NIrqHandler::EventDone()
    {
    TUint32 old = __atomic_load_n(&iHState, __ATOMIC_RELAXED), nw;
    do  {
        nw = old;
        if ((nw >> 16) == 0 || (nw & (EUnbind | EDisable)))
            nw &= ~(TUint32)EActive;
        } while (!__atomic_compare_exchange_n(&iHState, &old, nw, false,
                                              __ATOMIC_SEQ_CST, __ATOMIC_RELAXED));
    return old & (nw | ~(TUint32)EActive);
    }
