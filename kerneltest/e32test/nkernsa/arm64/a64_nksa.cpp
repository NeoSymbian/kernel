// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// SPDX-License-Identifier: EPL-1.0
//
// a64_nksa.cpp -- AArch64 / QEMU virt glue for the standalone nanokernel tests (kerneltest/e32test/nkernsa):
// the AArch64 counterpart of nkernsa/arm/armutils.{cpp,cia} plus the board hooks (Hw_Init1/InitAPs/Init3,
// StartSystemTimer, __timer_period, DebugPrint, __finish) and the exception/interrupt dispatch.
// The IRQ path follows Phorepoc adaptation-x86-64 ekern/xk_irq.cpp X64_Trap (Apache-2.0): EOI early, tick,
// reschedule/generic IPIs, tied-event tail, and preemption when leaving the outermost interrupt.
#define __INCLUDE_NTHREADBASE_DEFINES__		/* iReady/iCurrent/iLastCpu names for the stall dump */
#include <nktest/nkutils.h>
#include <e32rom.h>
#include <kernboot.h>
#include "../a64smp/a64_sysreg.h"
#include "../board/gicv3.h"
#include "../board/board.h"

extern "C" void KernelMain();
extern "C" void generic_ipi_isr(TSubScheduler* aS);		// nkernsmp/nkern.cpp
extern "C" void run_event_handlers(TSubScheduler* aS);	// nkernsmp/nk_irq.cpp
extern "C" void send_resched_ipis(TUint32 aMask);
TSubScheduler& SubScheduler();

static const TInt KTickUs = 1000;						// 1 ms nanokernel tick

/* ======================================================================================================
 *  Exception / interrupt dispatch (called from vectors.S with an SThreadExcStack frame)
 * ====================================================================================================== */
extern "C" { volatile int A64_BrkSeen; }
static TUint64 TickPeriod;				// counter ticks per 1 ms system tick
static volatile TBool TimerQReady;

/* Each CPU's EL1 virtual timer serves two deadlines, programmed as an absolute CNTV_CVAL: the periodic system
 * tick and the current thread's timeslice. Indexed by MPIDR so a secondary CPU can start its tick before
 * NKern::Init has given it a TSubScheduler. */
struct SCpuTimer { TUint64 iNextTick; TUint64 iSliceDeadline; TBool iSliceArmed; };
static SCpuTimer CpuTimer[256];
extern "C" TUint32 A64_MpidrIndex(TUint32 aMpidr);
static inline SCpuTimer& ThisCpuTimer()	{ return CpuTimer[A64_MpidrIndex(A64_Mpidr()) & 0xFF]; }

static void TimerProgram(SCpuTimer& c)
	{
	TUint64 cval = c.iNextTick;
	if (c.iSliceArmed && c.iSliceDeadline < cval)
		cval = c.iSliceDeadline;
	A64_SYSREG_WRITE(cntv_cval_el0, cval);
	A64_SYSREG_WRITE(cntv_ctl_el0, 1);
	__asm__ __volatile__("isb" ::: "memory");
	}

extern "C" void A64_TimesliceSet(TInt aTicks)
	{
	TInt irq = NKern::DisableAllInterrupts();
	SCpuTimer& c = ThisCpuTimer();
	c.iSliceArmed = aTicks > 0;
	if (c.iSliceArmed)
		c.iSliceDeadline = NKern::Timestamp() + (TUint32)aTicks;
	TimerProgram(c);
	NKern::RestoreInterrupts(irq);
	}

extern "C" TInt A64_TimesliceRemaining()
	{
	TInt irq = NKern::DisableAllInterrupts();
	SCpuTimer& c = ThisCpuTimer();
	TUint64 now = NKern::Timestamp();
	TInt r = (c.iSliceArmed && c.iSliceDeadline > now) ? (TInt)(c.iSliceDeadline - now) : 0;
	NKern::RestoreInterrupts(irq);
	return r;
	}
static volatile TUint32 TickCount;		// CPU 0 ticks, for the boot-time tick self-check

/* stall detector: each CPU records its tick count and where its tick interrupted it; if the tests print
 * nothing for 60 s (suspend/kill/migrate cases run ~30 s silently), CPU 0 dumps every CPU (the phone hung in ThreadGroupTest1's migrate loop) */
static volatile TUint32 CpuTicks[KMaxCpus];
static volatile TUint64 CpuLastElr[KMaxCpus];
static volatile TUint32 StallLinesSeen, StallQuietTicks, StallDumped;
extern "C" TUint32 A64_StatusLines();
static void DumpCpus();

/* HijackSystemTimer (tiedevents EInterrupt): CPU 0's tick is raised as software NIrq KTickIrq, whose one handler
 * is tied to a thread/group, so NIrqHandler::Activate runs the tick -- and the untied NTimer callbacks Tick() calls
 * -- on the CPU that thread/group is attached to, never concurrently with it. Vector 1023: no GIC EOI. */
static const TInt KTickIrq = 0;
static volatile TUint32 TickIrqHandle;		// nonzero while hijacked
static void TickIsr(TAny*)	{ TheTimerQ.Tick(); }

static void SystemTick(TSubScheduler& aSS)
	{
	++CpuTicks[aSS.iCpuNum & (KMaxCpus - 1)];
	if (aSS.iCpuNum == 0 && TimerQReady)
		{
		TUint32 lines = A64_StatusLines();
		if (lines != StallLinesSeen) { StallLinesSeen = lines; StallQuietTicks = 0; StallDumped = 0; }	// re-arm on progress
		else if (++StallQuietTicks == 60000 && StallDumped < 3) { ++StallDumped; DumpCpus(); StallQuietTicks = 0; }
		}
	if (aSS.iCpuNum == 0)
		++TickCount;
	if (aSS.iCpuNum == 0 && TimerQReady)
		{
		if (__e32_atomic_load_acq32(&TickIrqHandle))
			NKern::Interrupt(KTickIrq);		// hijacked: the tied handler runs the tick where its thread/group is
		else
			TheTimerQ.Tick();				// the nanokernel millisecond tick (may queue the timer DFC)
		}
	}

static void TimerIrq(TSubScheduler& aSS)
	{
	SCpuTimer& c = ThisCpuTimer();
	TUint64 now = NKern::Timestamp();
	if (now >= c.iNextTick)
		{
		do c.iNextTick += TickPeriod; while (c.iNextTick <= now);	// catch up, never fire twice for one tick
		SystemTick(aSS);
		}
	if (c.iSliceArmed && now >= c.iSliceDeadline)
		{
		c.iSliceArmed = FALSE;				// expired: SaveTimesliceTimer will read 0
		aSS.iRescheduleNeededFlag = TRUE;	// as the ARM do_timeslice_irq: just ask for a reschedule
		}
	TimerProgram(c);
	}

static void Irq(SThreadExcStack* aF)
	{
	TSubScheduler& ss = SubScheduler();
	++ss.iSSX.iIrqNestCount;				// -1 = not in an ISR
	++ss.iSSX.iIrqCount;
	for (;;)
		{
		TUint32 id = Gicv3_Ack();
		if (id >= 1020)
			break;							// nothing (more) pending
		Gicv3_Eoi(id);						// EOI first: a reschedule below may not return for a long time
		if (id == A64_TimerPpi)
			{
			CpuLastElr[ss.iCpuNum & (KMaxCpus - 1)] = aF->iElr;
			TimerIrq(ss);
			}
		else if (id == RESCHED_IPI_VECTOR)
			ss.iRescheduleNeededFlag = TRUE;
		else if (id == GENERIC_IPI_VECTOR)
			generic_ipi_isr(&ss);
		// TRANSFERRED_IRQ_VECTOR: the tail below runs the events
		}
	if (--ss.iSSX.iIrqNestCount >= 0)
		return;
	if (ss.iEventHandlersPending)
		{
		++ss.iSSX.iIrqNestCount;
		run_event_handlers(&ss);
		--ss.iSSX.iIrqNestCount;
		}
	if ((ss.iRescheduleNeededFlag || ss.iDfcPendingFlag || ss.iExIDfcPendingFlag) && ss.iKernLockCount == 0)
		{
		ss.iKernLockCount = 1;
		TScheduler::Reschedule();			// returns (perhaps much later, perhaps on another CPU) unlocked, IRQs off
		TUint32 ipis = __e32_atomic_swp_ord32(&SubScheduler().iReschedIPIs, 0);
		if (ipis)
			send_resched_ipis(ipis);
		}
	}

static void DumpSched(const char* aWhat, NSchedulable* aS)
	{
	A64_DebugPuts(aWhat); A64_DebugPuts(" "); A64_DebugHex((TLinAddr)aS);
	A64_DebugPuts(" pri="); A64_DebugHex(aS->iPriority);
	A64_DebugPuts(" rdy="); A64_DebugHex(aS->iReady);
	A64_DebugPuts(" cur="); A64_DebugHex(aS->iCurrent);
	A64_DebugPuts(" last="); A64_DebugHex(aS->iLastCpu);
	A64_DebugPuts(" aff="); A64_DebugHex(aS->iCpuAffinity);
	A64_DebugPuts(" chg="); A64_DebugHex(aS->iCpuChange);
	A64_DebugPuts(" frz="); A64_DebugHex(aS->iFreezeCpu);
	A64_DebugPuts(" tr="); A64_DebugHex(aS->iTransientCpu);
	A64_DebugPuts(" ev="); A64_DebugHex(aS->iEventState);
	A64_DebugPuts(" act="); A64_DebugHex((TUint32)aS->iActiveState);
	A64_DebugPuts(" pause="); A64_DebugHex(aS->iPauseCount);
	A64_DebugPuts(" susp="); A64_DebugHex(aS->iSuspended);
	if (aS->iParent)	// a thread
		{
		NThreadBase* t = (NThreadBase*)aS;
		A64_DebugPuts(" par="); A64_DebugHex((TLinAddr)t->iParent);
		A64_DebugPuts(" wst="); A64_DebugHex(t->iWaitState.iWtC.iWtStFlags);
		A64_DebugPuts(" cs="); A64_DebugHex(t->iCsFunction);
		A64_DebugPuts(" fmd="); A64_DebugHex(t->iFastMutexDefer);
		A64_DebugPuts(" time="); A64_DebugHex((TUint32)t->iTime);
		}
	else
		{
		A64_DebugPuts(" nthr="); A64_DebugHex(((NThreadGroup*)aS)->iThreadCount);
		}
	A64_DebugPuts("\n");
	}

static void DumpCpus()
	{
	A64_DebugPuts("\n*** STALL: no test output for 60 s. Per CPU: ticks elr thread klock resched irqnest\n");
	for (TInt i = 0; i < TheScheduler.iNumCpus && i < KMaxCpus; ++i)
		{
		TSubScheduler& s = TheSubSchedulers[i];
		NThreadBase* t = s.iCurrentThread;
		A64_DebugPuts(" cpu"); A64_DebugPutc((char)('0' + i));
		A64_DebugPuts(" t="); A64_DebugHex(CpuTicks[i]);
		A64_DebugPuts(" pc="); A64_DebugHex(CpuLastElr[i]);
		A64_DebugPuts(" thr="); A64_DebugHex((TLinAddr)t);
		A64_DebugPuts(" kl="); A64_DebugHex(s.iKernLockCount);
		A64_DebugPuts(" rs="); A64_DebugHex(s.iRescheduleNeededFlag);
		A64_DebugPuts(" in="); A64_DebugHex((TUint32)s.iSSX.iIrqNestCount);
		if (t)
			{
			A64_DebugPuts(" pri="); A64_DebugHex(t->iPriority);
			A64_DebugPuts(" grp="); A64_DebugHex((TLinAddr)t->iParent);
			}
		A64_DebugPuts("\n");
		}
	// the thread group under test (threadbasic.cpp TG1) and its member threads
	extern NThreadGroup TG1;
	DumpSched("group TG1", &TG1);
	for (SIterDQLink* l = TheScheduler.iAllThreads.iA.Next(); l && l != &TheScheduler.iAllThreads.iA; l = l->Next())
		{
		if (!l->IsObject()) continue;
		NThreadBase* t = _LOFF(l, NThreadBase, iEnumerateLink);
		if (t->iParent == &TG1 || t->iNewParent == &TG1)
			DumpSched("  member", t);
		}
	}

extern "C" void A64_ExceptionDispatch(SThreadExcStack* aF, TUint64 aKind)
	{
	if ((aKind & 3) == 1)
		{ Irq(aF); return; }
	TUint32 ec = (TUint32)(aF->iEsr >> 26);
	if ((aKind & 3) == 0 && ec == 0x3C && (aF->iEsr & 0xFFFF) == 0x1234)	// the boot self-test BRK
		{ ++A64_BrkSeen; aF->iElr += 4; return; }
	A64_DebugPuts("\n*** FAULT kind="); A64_DebugHex(aKind);
	A64_DebugPuts(" esr="); A64_DebugHex(aF->iEsr);
	A64_DebugPuts(" elr="); A64_DebugHex(aF->iElr);
	A64_DebugPuts(" far="); A64_DebugHex(aF->iFar);
	A64_DebugPuts(" lr="); A64_DebugHex(aF->iX[30]);
	A64_DebugPuts("\n");
	for (;;) { A64_CLI(); __asm__ __volatile__("wfi"); }
	}

/* ======================================================================================================
 *  Board hooks the nkernsa init.cpp calls
 * ====================================================================================================== */
extern "C" TInt __timer_period()		{ return KTickUs; }

static void InitThisCpuGic()		// this CPU's redistributor: IPIs (SGI 0-7) and the tick (PPI 27)
	{
	TLinAddr sgi = Gicv3_InitCpu(A64_GicrBase, A64_Mpidr());
	if (!sgi)
		{ A64_DebugPuts("*** GICv3: no redistributor for this CPU\n"); return; }
	for (TUint32 i = 0; i < 8; ++i)
		Gicv3_EnablePpiSgi(sgi, i, 0x80);
	Gicv3_EnablePpiSgi(sgi, A64_TimerPpi, 0xA0);
	}

extern "C" void Hw_Init1()
	{
	Gicv3_InitDist(A64_GicdBase);
	InitThisCpuGic();
	TickPeriod = (TUint64)NKern::TimestampFrequency() * KTickUs / 1000000;
	NKern::InterruptInit0();				// the NIrqHandler pool (ARM: NKern::Init's arch part)
	NKern::InterruptInit(KTickIrq, NKern::EIrqInit_RisingEdge, 1023, 0);
	}

extern "C" void StartSystemTimer()		// this CPU: tick from now on, no timeslice deadline yet
	{
	SCpuTimer& c = ThisCpuTimer();
	c.iNextTick = NKern::Timestamp() + TickPeriod;
	c.iSliceArmed = FALSE;
	TimerProgram(c);
	}

extern "C" void ApMainGeneric(volatile SAPBootInfo* aInfo);	// nkernsa init.cpp
extern "C" void A64_ApEntry();									// boot.S

/* PSCI 0.2+ call through the board's conduit (QEMU virt: HVC, Pixel/TrustZone: SMC) */
static TInt Psci(TUint64 aFn, TUint64 aA1, TUint64 aA2, TUint64 aA3)
	{
	register TUint64 x0 __asm__("x0") = aFn;
	register TUint64 x1 __asm__("x1") = aA1;
	register TUint64 x2 __asm__("x2") = aA2;
	register TUint64 x3 __asm__("x3") = aA3;
	if (A64_PsciSmc)
		__asm__ __volatile__("smc #0" : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3) :: "memory");
	else
		__asm__ __volatile__("hvc #0" : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3) :: "memory");
	return (TInt)x0;
	}
static TInt PsciCpuOn(TUint64 aMpidr, TUint64 aEntry, TUint64 aContext)	{ return Psci(0xC4000003, aMpidr, aEntry, aContext); }

/* a secondary CPU, on its own stack with the MMU on: its GIC, its tick, then join the nanokernel */
extern "C" void A64_ApMain(volatile SAPBootInfo* aInfo)
	{
	InitThisCpuGic();
	StartSystemTimer();
	ApMainGeneric(aInfo);				// NKern::Init for this CPU's initial thread, then idles; never returns
	}

extern "C" void Hw_InitAPs()
	{
	const TInt KApStack = 0x4000;
	for (TInt cpu = 1; cpu < KMaxCpus; ++cpu)
		{
		SAPBootInfo* info = new SAPBootInfo;
		NThread* t = new NThread;
		TAny* stack = malloc(KApStack);
		if (!info || !t || !stack)
			{ KPrintf("AP%d: out of memory", cpu); return; }
		memclr(info, sizeof(*info));
		info->iCpu = cpu;
		info->iInitStackBase = (TLinAddr)stack;
		info->iInitStackSize = KApStack;
		info->iArgs[0] = t;
		// the AP reads its boot info, and pushes onto its stack, with MMU and caches OFF: put the info in RAM
		// and evict the stack from this core's cache (a dirty line written back later would corrupt it)
		A64_DCacheRange((TLinAddr)info, sizeof(*info), FALSE);
		A64_DCacheRange((TLinAddr)stack, KApStack, TRUE);
		TInt r = PsciCpuOn(A64_CpuMpidr(cpu), (TLinAddr)&A64_ApEntry, (TLinAddr)info);
		if (r != 0)
			{
			KPrintf("PSCI CPU_ON(%d) = %d: %d CPU(s)", cpu, r, cpu);		// -2 INVALID_PARAMETERS: no such CPU
			free(stack); delete t; delete info;
			return;
			}
		TUint64 t0 = fast_counter(), limit = (TUint64)NKern::TimestampFrequency() * 5;
		while (!__e32_atomic_load_acq32(&info->iArgs[1]))
			if (fast_counter() - t0 > limit)
				{ KPrintf("AP%d: did not check in", cpu); return; }
		KPrintf("AP%d up", cpu);
		}
	}

extern "C" void Hw_Init3()
	{
	TimerQReady = TRUE;
	StartSystemTimer();
#ifdef __SMP__
	// SMP load balancing, as kernel/sinit.cpp does in the full kernel (nkernsa never started it): the rebalance
	// DFC queue at KRebalancePriority 26, InitLB, then the periodic balance timer. Without it a running thread is
	// never moved off a CPU -- ThreadGroupTest1's migrate case hung with the group pinned behind the Main thread.
	TheScheduler.iRebalanceDfcQ = CreateDfcQ("Rebalance", 26, KCpuAffinityAny);
	if (TheScheduler.iRebalanceDfcQ)
		{
		TheScheduler.InitLB();
		TheScheduler.StartPeriodicBalancing();
		}
	else
		KPrintf("*** no rebalance DFC queue: SMP load balancing off");
#endif
	// self-check: 20 ms of busy-wait must see ticks, or every NKern::Sleep() hangs (wrong timer PPI, GIC setup)
	TUint32 t0 = TickCount;
	TUint64 c0 = fast_counter(), wait = (TUint64)NKern::TimestampFrequency() / 50;
	while (fast_counter() - c0 < wait) {}
	TUint32 n = TickCount - t0;
	if (n)
		KPrintf("tick ok: %d ticks in 20 ms (timer PPI %d)", n, A64_TimerPpi);
	else
		KPrintf("NO TICKS on timer PPI %d: cntv_ctl=%x daif=%x", A64_TimerPpi,
				(TUint32)A64_SYSREG_READ(cntv_ctl_el0), (TUint32)A64_SYSREG_READ(daif));
	}

/* ======================================================================================================
 *  armutils equivalents (nkernsa/utils.h)
 * ====================================================================================================== */
extern "C" {

/* status bar bookkeeping: the last "Testing ..." / "Basic..." headline, and the TEST_RESULT failure lines */
static TUint32 StatusLines, StatusFails;
extern "C" TUint32 A64_StatusLines() { return StatusLines; }
static char StatusTest[80] = "boot";
static char StatusShown[96];				// what the bar shows: test name, plus the last failure site
/* failure sites ("threadbasic.cpp:437") with counts, printed as a summary by __finish */
struct SFailSite { char iWhere[40]; TUint32 iCount; char iTest[48]; char iMsg[64]; char iCase[48]; };
/* What a failure line refers to, kept per printing thread (skmpar runs several DoRWTest cases at once):
 *   iLine   the TEST_RESULT message line printed just before "Line N File ..."
 *   iCase   the last case header: "TT2: ...", "TiedEventTest ...", DoRWTest's "Type ..."
 *   iDetail DoRWTest's "F0 n F1 n ... F6 n" counts, shown instead of its bare "FAILED" */
struct SCaseSlot { NThreadBase* iThread; char iLine[64]; char iCase[48]; char iDetail[64]; };
static SCaseSlot CaseSlots[8];
static TUint32 CaseSlotNext;
static SCaseSlot& CaseSlot()				// the calling thread's slot; a new thread takes over the oldest
	{
	NThreadBase* t = NKern::CurrentThread();
	for (TUint32 i = 0; i < sizeof(CaseSlots) / sizeof(CaseSlots[0]); ++i)
		if (CaseSlots[i].iThread == t)
			return CaseSlots[i];
	SCaseSlot& c = CaseSlots[CaseSlotNext++ % (sizeof(CaseSlots) / sizeof(CaseSlots[0]))];
	c.iThread = t; c.iLine[0] = c.iCase[0] = c.iDetail[0] = 0;
	return c;
	}

/* per-test durations for __finish: each "Testing ..." headline (and "Tests Completed") with its start time */
struct STestTime { char iName[48]; TUint32 iStartMs; };
static STestTime TestTimes[64];
static TInt TestTimeCount;
static TUint32 NowMs()
	{
	TUint32 f = NKern::TimestampFrequency();
	return f ? (TUint32)(NKern::Timestamp() * 1000 / f) : 0;
	}

static void CopyLine(char* aDst, TInt aMax, const char* s, TInt l)	// without the "c:NKTEST: " prefix
	{
	for (TInt i = 0; i + 8 <= l; ++i)
		if (s[i]=='N' && s[i+1]=='K' && s[i+2]=='T' && s[i+3]=='E' && s[i+4]=='S' && s[i+5]=='T' && s[i+6]==':' && s[i+7]==' ')
			{ s += i + 8; l -= i + 8; break; }
	TInt k = 0;
	while (k < l && k < aMax - 1 && s[k] && s[k] != '\r' && s[k] != '\n') { aDst[k] = s[k]; ++k; }
	aDst[k] = 0;
	}
static SFailSite FailSites[24];
static TInt FailSiteCount;

static void NoteFailure(const char* s, TInt l, SCaseSlot& aC)	// s = "c:Line <n> File <path>/<name>"
	{
	const char* LastCase = aC.iCase;
	char where[40]; TInt w = 0;
	const char* name = s; const char* line = 0;
	for (TInt i = 0; i < l && s[i]; ++i)
		{
		if (s[i] == '/') name = s + i + 1;
		if (!line && i + 5 <= l && s[i]=='L' && s[i+1]=='i' && s[i+2]=='n' && s[i+3]=='e' && s[i+4]==' ') line = s + i + 5;
		}
	for (const char* p = name; *p && *p != '\r' && *p != '\n' && w < 30; ++p) where[w++] = *p;
	where[w++] = ':';
	for (const char* p = line; p && *p >= '0' && *p <= '9' && w < 39; ++p) where[w++] = *p;
	where[w] = 0;
	TInt i;
	for (i = 0; i < FailSiteCount; ++i)
		{
		TInt k = 0, c = 0;
		while (where[k] && where[k] == FailSites[i].iWhere[k]) ++k;
		while (c < 47 && LastCase[c] && LastCase[c] == FailSites[i].iCase[c]) ++c;	// one site per (line, case)
		if (!where[k] && !FailSites[i].iWhere[k] && (c == 47 || (!LastCase[c] && !FailSites[i].iCase[c]))) break;
		}
	if (i == FailSiteCount && FailSiteCount < (TInt)(sizeof(FailSites)/sizeof(FailSites[0])))
		{
		TInt k;
		for (k = 0; where[k]; ++k) FailSites[i].iWhere[k] = where[k];
		FailSites[i].iWhere[k] = 0;
		for (k = 0; StatusTest[k] && k < 47; ++k) FailSites[i].iTest[k] = StatusTest[k];
		FailSites[i].iTest[k] = 0;
		const char* msg = aC.iDetail[0] ? aC.iDetail : aC.iLine;
		for (k = 0; msg[k] && k < 63; ++k) FailSites[i].iMsg[k] = msg[k];
		FailSites[i].iMsg[k] = 0;
		for (k = 0; LastCase[k] && k < 47; ++k) FailSites[i].iCase[k] = LastCase[k];
		FailSites[i].iCase[k] = 0;
		++FailSiteCount;
		}
	if (i < FailSiteCount)
		{
		++FailSites[i].iCount;
		// two pin rows per site: "<count>x <file:line> | <case or test>", then "   <message or counts>"
		char pin[96]; TInt n = 0;
		TUint32 v = FailSites[i].iCount; char d[12]; TInt j = 0;
		do { d[j++] = (char)('0' + v % 10); v /= 10; } while (v);
		while (j) pin[n++] = d[--j];
		const char* parts[] = { "x ", FailSites[i].iWhere, " | ", FailSites[i].iCase[0] ? FailSites[i].iCase : FailSites[i].iTest };
		for (TInt p = 0; p < 4; ++p) for (const char* c = parts[p]; *c && n < 95; ++c) pin[n++] = *c;
		pin[n] = 0;
		A64_ConsolePin(2 * i, pin);
		n = 0;
		for (const char* c = "   "; *c; ++c) pin[n++] = *c;
		for (const char* c = FailSites[i].iMsg; *c && n < 95; ++c) pin[n++] = *c;
		pin[n] = 0;
		A64_ConsolePin(2 * i + 1, pin);
		}
	// bar: "<test>  last fail <where>"
	TInt n = 0;
	for (const char* p = StatusTest; *p && n < 50; ++p) StatusShown[n++] = *p;
	const char* t = "  last fail ";
	for (const char* p = t; *p; ++p) StatusShown[n++] = *p;
	for (const char* p = where; *p && n < 95; ++p) StatusShown[n++] = *p;
	StatusShown[n] = 0;
	}

static TBool Contains(const char* s, TInt l, const char* w)
	{
	for (TInt i = 0; i < l && s[i]; ++i)
		{
		TInt j = 0;
		while (w[j] && i + j < l && s[i + j] == w[j]) ++j;
		if (!w[j]) return TRUE;
		}
	return FALSE;
	}

static volatile TUint32 PrintLock;			// one line at a time across CPUs (skmpar prints from four)

void DebugPrint(const char* s, int l)
	{
	TInt irq = NKern::DisableAllInterrupts();	// keep one line together across CPUs/interrupts
	while (__e32_atomic_swp_acq32(&PrintLock, 1))
		A64_PAUSE();
	SCaseSlot& cs = CaseSlot();
	++StatusLines;
	static TUint32 pipeRounds;
	if (Contains(s, l, "Line ") && Contains(s, l, " File "))
		{
		++StatusFails;
		NoteFailure(s, l, cs);
		}
	else if (Contains(s, l, "waits out of"))		// fastbuf.cpp: 0x01000000 writes / 131072 per round = 128 rounds
		{
		const char* h = "wait-free pipe, round ";
		TInt k = 0;
		while (h[k]) { StatusTest[k] = h[k]; ++k; }
		TUint32 v = ++pipeRounds; char d[12]; TInt i = 0;
		do { d[i++] = (char)('0' + v % 10); v /= 10; } while (v);
		while (i) StatusTest[k++] = d[--i];
		const char* t = "/128";
		for (TInt j = 0; t[j]; ++j) StatusTest[k++] = t[j];
		StatusTest[k] = 0;
		}
	else if (Contains(s, l, "TT2:") || Contains(s, l, "TiedEventTest") || (Contains(s, l, "Type ") && Contains(s, l, "ISEQ"))
			|| Contains(s, l, "NKTEST: DFCTest") || Contains(s, l, "NKTEST: Mode "))
		{
		CopyLine(cs.iCase, sizeof(cs.iCase), s, l);
		cs.iDetail[0] = 0;
		}
	else if (Contains(s, l, "F0 ") && Contains(s, l, " F6 "))
		{
		char* d = cs.iDetail;
		CopyLine(d, sizeof(cs.iDetail), s, l);
		TInt w = 0;									// collapse the %6d padding so all seven counts fit one row
		for (TInt r = 0; d[r]; ++r)
			if (d[r] != ' ' || (w && d[w - 1] != ' ')) d[w++] = d[r];
		d[w] = 0;
		}
	else if (Contains(s, l, "Testing") || Contains(s, l, "Tests Completed") || Contains(s, l, "Kill") || Contains(s, l, "Thread on CPU"))
		{
		const char* t = s; TInt n = l;
		for (TInt i = 0; i + 8 <= n; ++i)			// drop the "c:NKTEST: " prefix
			if (Contains(s + i, 8, "NKTEST: ")) { t = s + i + 8; n -= i + 8; break; }
		TInt k = 0;
		while (k < n && k < (TInt)sizeof(StatusTest) - 1 && t[k] && t[k] != '\r' && t[k] != '\n') { StatusTest[k] = t[k]; ++k; }
		StatusTest[k] = 0;
		cs.iCase[0] = 0; cs.iDetail[0] = 0;					// a new test: the last case header belongs to the previous one
		if ((Contains(s, l, "Testing") || Contains(s, l, "Tests Completed")) && TestTimeCount < (TInt)(sizeof(TestTimes) / sizeof(TestTimes[0])))
			{
			STestTime& tt = TestTimes[TestTimeCount++];
			CopyLine(tt.iName, sizeof(tt.iName), s, l);
			tt.iStartMs = NowMs();
			}
		}
	if (!(Contains(s, l, "Line ") && Contains(s, l, " File ")))
		CopyLine(cs.iLine, sizeof(cs.iLine), s, l);
	while (l-- > 0 && *s) A64_DebugPutc(*s++);
	A64_DebugPutc('\n');
	TUint32 freq = NKern::TimestampFrequency();
	A64_ConsoleStatus(StatusFails ? StatusShown : StatusTest, StatusLines, StatusFails, freq ? (TUint32)(NKern::Timestamp() / freq) : 0);
	__e32_atomic_store_rel32(&PrintLock, 0);
	NKern::RestoreInterrupts(irq);
	}

void __cpu_idle()					{ A64_IDLE(); }
void __cpu_yield()					{ A64_PAUSE(); }
TLinAddr __stack_pointer()			{ TLinAddr sp; __asm__ __volatile__("mov %w0, wsp" : "=r"(sp)); return sp; }
TUint32 __cpu_status_reg()			{ return (TUint32)A64_SYSREG_READ(daif); }
TUint32 __cpsr()					{ return __cpu_status_reg(); }
TUint32 __cpu_id()					{ return A64_Mpidr(); }
TUint32 __trace_cpu_num()			{ return NKern::CurrentCpu(); }
TUint64 fast_counter()				{ return NKern::Timestamp(); }
TUint32 norm_fast_counter();		// below, after nfcfs

static void PutDec(TUint32 v)
	{
	char d[12]; TInt i = 0;
	do { d[i++] = (char)('0' + v % 10); v /= 10; } while (v);
	while (i) { char c[2] = { d[--i], 0 }; A64_DebugPuts(c); }
	}

void __finish()
	{
	// where the time went: every headline that lasted >= 2 s (the last one runs until now)
	TUint32 end = NowMs();
	A64_DebugPuts("Time per test (>= 2 s):\n");
	for (TInt i = 0; i < TestTimeCount; ++i)
		{
		TUint32 ms = (i + 1 < TestTimeCount ? TestTimes[i + 1].iStartMs : end) - TestTimes[i].iStartMs;
		if (ms < 2000)
			continue;
		A64_DebugPuts("  "); PutDec(ms / 1000); A64_DebugPuts("s  "); A64_DebugPuts(TestTimes[i].iName); A64_DebugPuts("\n");
		}
	A64_DebugPuts("  total "); PutDec(TestTimeCount ? (end - TestTimes[0].iStartMs) / 1000 : 0); A64_DebugPuts("s\n");
	A64_DebugPuts("EKA2/arm64: nkernsa finished, failures: ");
	A64_DebugHex(StatusFails);
	A64_DebugPuts("\n");
	for (TInt i = 0; i < FailSiteCount; ++i)		// one line per failure site: count, where, during which test
		{
		A64_DebugPuts("  "); A64_DebugHex(FailSites[i].iCount);
		A64_DebugPuts(" x "); A64_DebugPuts(FailSites[i].iWhere);
		A64_DebugPuts(" "); A64_DebugPuts(FailSites[i].iMsg);
		A64_DebugPuts("  ("); A64_DebugPuts(FailSites[i].iCase[0] ? FailSites[i].iCase : FailSites[i].iTest); A64_DebugPuts(")\n");
		}
	StallDumped = 3;						// the hold below is quiet on purpose: no stall dumps over the summary
	// the phone keeps the summary on screen until the user powers it off (a 2-minute hold lost an overnight run);
	// shortcut: a static screen for hours on an OLED, add a dim/blank timer if this becomes routine
	if (!A64_HoldResults)
		Psci(0x84000008, 0, 0, 0);		// PSCI SYSTEM_OFF (QEMU exits)
	for (;;) { A64_CLI(); __asm__ __volatile__("wfi"); }
	}

void* memcpy(void* aDest, const void* aSrc, unsigned aLen)
	{
	volatile TUint8* d = (volatile TUint8*)aDest;	// volatile: keep GCC from turning this into a memcpy call
	const TUint8* s = (const TUint8*)aSrc;
	while (aLen--) *d++ = *s++;
	return aDest;
	}

void* wordmove(void* aDest, const void* aSrc, unsigned aLen)	// overlapping-safe word copy
	{
	TUint32* d = (TUint32*)aDest; const TUint32* s = (const TUint32*)aSrc;
	TInt n = aLen >> 2;
	if (d <= s) { while (n--) *d++ = *s++; }
	else { d += n; s += n; while (n--) *--d = *--s; }
	return aDest;
	}

/* --- from nkernsa/arm/armutils.cpp (EPL-1.0), unchanged apart from the timer-watchdog test (ARM11 only) --- */
const TUint32 KPageSize = 0x1000u;
TUint64 fcf;
TUint32 nfcf;
TUint32 nfcfs;

TUint32 round_to_page(TUint32 x)		{ return (x + KPageSize - 1) &~ (KPageSize - 1); }
TLinAddr __initial_stack_base()			{ return __stack_pointer() & ~0xfff; }
TInt __initial_stack_size()				{ return 0x1000; }
TUint64 fast_counter_freq()				{ return fcf; }
TUint32 norm_fast_counter_freq()		{ return nfcf; }
TUint32 norm_fast_counter()				{ return (TUint32)(fast_counter() >> nfcfs); }

void init_fast_counter()
	{
	NKern::Sleep(30);
	TUint64 initial = fast_counter();
	NKern::Sleep(1000);
	TUint64 final = fast_counter();
	fcf = final - initial;
	TUint64 f = fcf;
	nfcfs = 0;
	while (f > 2000000)
		f>>=1, ++nfcfs;
	nfcf = (TUint32)(fcf >> nfcfs);
	DEBUGPRINT("fcf=%lx",fcf);
	DEBUGPRINT("nfcf=%d",nfcf);
	}

TInt __microseconds_to_fast_counter(TInt us)
	{
	TUint64 x = TUint64(TUint32(us));
	x *= fcf; x += TUint64(500000); x /= TUint64(1000000);
	return (TInt)x;
	}

TInt __microseconds_to_norm_fast_counter(TInt us)
	{
	TUint64 x = TUint64(TUint32(us));
	x *= nfcf; x += TUint64(500000); x /= TUint64(1000000);
	return (TInt)x;
	}

void fcfspin(TUint64 aTicks)
	{
	TUint64 t0 = fast_counter();
	TUint64 t1;
	do	{ t1 = fast_counter(); t1 -= t0; } while (t1<aTicks);
	}

void nfcfspin(TUint32 aTicks)
	{
	TUint64 ticks = aTicks;
	ticks <<= nfcfs;
	fcfspin(ticks);
	}

void CheckPoint()						{ KPrintf("CPSR=%08x", __cpsr()); }

/* timeslice ticks = generic-timer counts (the fast counter is the same counter). Use the frequency the tests
 * calibrated (init_fast_counter: fcf = counts in NKern::Sleep(1000)) once it exists, so a slice of N us and the
 * test's own __microseconds_to_*_fast_counter(N) agree to the count. Sleep guarantees AT LEAST N ticks, so fcf
 * reads ~0.05-0.1% above CNTFRQ; with the nominal CNTFRQ every slice expired a few counts before the test's
 * expected point (phone: TT2 pointE 449/437 and pointB 448, the only failures left). */
TInt __microseconds_to_timeslice_ticks(TInt us)
	{
	TUint64 f = fcf ? fcf : NKern::TimestampFrequency();
	return (TInt)(((TUint64)(TUint32)us * f + 500000) / 1000000);
	}
TInt __fast_counter_to_timeslice_ticks(TUint64 aFCdelta)	{ return (TInt)aFCdelta; }

/* armutils.cia, as compare-and-swap loops with full barriers */
TUint32 set_bit0_if_nonnull(TUint32& a)
	{
	TUint32 o = __atomic_load_n(&a, __ATOMIC_SEQ_CST);
	while (o && !__atomic_compare_exchange_n(&a, &o, o | 1, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)) {}
	return o;
	}
void flip_bit0(TUint32& a)						{ __atomic_fetch_xor(&a, 1u, __ATOMIC_SEQ_CST); }
TUint32 swap_out_if_bit0_clear(TUint32& a)
	{
	TUint32 o = __atomic_load_n(&a, __ATOMIC_SEQ_CST);
	while (!(o & 1) && !__atomic_compare_exchange_n(&a, &o, 0, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)) {}
	return o;
	}

/* tie the system tick to aTieTo (NULL: back to the plain tick); see KTickIrq */
void HijackSystemTimer(NSchedulable* aTieTo)
	{
	TInt h = (TInt)__e32_atomic_swp_ord32(&TickIrqHandle, 0);
	if (h)
		NKern::InterruptUnbind(h);			// waits out a tick in flight
	if (!aTieTo)
		return;
	h = NKern::InterruptBind(KTickIrq, &TickIsr, 0, NKern::EIrqBind_Tied, aTieTo);
	TInt r = h > 0 ? NKern::InterruptEnable(h) : h;
	if (r != KErrNone)
		{ KPrintf("HijackSystemTimer: bind/enable %d", r); return; }
	__e32_atomic_store_rel32(&TickIrqHandle, (TUint32)h);
	}
void thread_request_signal(NThread* aThread)	{ NKern::ThreadRequestSignal(aThread); }

} // extern "C"

/* armutils.cia: atomically take the request status pointer; if there was one, complete it and signal */
void RequestComplete(NThread* aThread, NRequestStatus*& aStatus, TInt aValue)
	{
	NRequestStatus* s = __atomic_exchange_n(&aStatus, (NRequestStatus*)0, __ATOMIC_SEQ_CST);
	if (s)
		{
		*(TInt*)s = aValue;
		thread_request_signal(aThread);
		}
	}

TVersion::TVersion() : iMajor(0), iMinor(0), iBuild(0) {}	// euser's, needed by a header-only use in the tests

TAny* operator new(TUint aSize) __NO_THROW		{ return malloc(aSize); }
TAny* operator new[](TUint aSize) __NO_THROW	{ return malloc(aSize); }
void operator delete(TAny* aPtr) __NO_THROW		{ free(aPtr); }
void operator delete[](TAny* aPtr) __NO_THROW	{ free(aPtr); }
void operator delete(TAny* aPtr, TUint) __NO_THROW	{ free(aPtr); }	// sized delete (GCC 14 emits it)

/* ======================================================================================================
 *  ROM header / super page stand-ins: SetupMemoryAllocator (tlsf.cpp) derives the heap from them
 * ====================================================================================================== */
extern "C" TLinAddr RomHeaderAddress;
extern "C" TLinAddr SuperPageAddress;

static TRomHeader TheRomHeader;
static SSuperPageBase TheSuperPage;
static TRomEntry ThePrimaryEntry;
static TRomImageHeader ThePrimaryImage;

extern "C" void A64_NkSaStart()
	{
	TheRomHeader.iKernDataAddress = A64_HeapBase;	// heap = iKernDataAddress + svdata(0) + primary stack(0)
	TheRomHeader.iTotalSvDataSize = 0;
	ThePrimaryImage.iStackSize = 0;
	ThePrimaryEntry.iAddressLin = (TLinAddr)&ThePrimaryImage;
	TheSuperPage.iPrimaryEntry = (TLinAddr)&ThePrimaryEntry;
	TheSuperPage.iInitialHeapSize = A64_HeapSize;
	RomHeaderAddress = (TLinAddr)&TheRomHeader;
	SuperPageAddress = (TLinAddr)&TheSuperPage;
	A64_DebugPuts("EKA2/arm64: -> KernelMain()\n");
	KernelMain();
	}
