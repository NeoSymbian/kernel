// SPDX-License-Identifier: Apache-2.0
// a64_platform.cpp -- the AArch64 platform layer shared by the nkernsa test image and the full kernel (see
// a64_platform.h). Moved unchanged from nksa/a64_nksa.cpp, where the nanokernel bring-up proved it on QEMU and the
// Pixel; the IRQ path follows Phorepoc adaptation-x86-64 ekern/xk_irq.cpp X64_Trap (EOI early, tick, IPIs,
// tied-event tail, preemption when leaving the outermost interrupt).
#include <nkern.h>
#include <nk_priv.h>
#include "../a64smp/a64_sysreg.h"
#include "../board/gicv3.h"
#include "../board/board.h"
#include "a64_platform.h"

extern "C" void generic_ipi_isr(TSubScheduler* aS);		// nkernsmp/nkern.cpp
extern "C" void run_event_handlers(TSubScheduler* aS);	// nkernsmp/nk_irq.cpp
extern "C" void resched_ipi_ack(TSubScheduler* aSS);		// a64s_ncsched.cpp
extern "C" void send_resched_ipis(TUint32 aMask);
extern "C" TUint32 A64_MpidrIndex(TUint32 aMpidr);
TSubScheduler& SubScheduler();

/* ---- GIC ----------------------------------------------------------------------------------------------- */
extern "C" void A64_InitThisCpuGic()		// this CPU's redistributor: IPIs (SGI 0-7) and the tick PPI
	{
	TLinAddr sgi = Gicv3_InitCpu(A64_Io(A64_GicrBase), A64_Mpidr());
	if (!sgi)
		{ A64_DebugPuts("*** GICv3: no redistributor for this CPU\n"); return; }
	for (TUint32 i = 0; i < 8; ++i)
		Gicv3_EnablePpiSgi(sgi, i, 0x80);
	Gicv3_EnablePpiSgi(sgi, A64_TimerPpi, 0xA0);
	}

extern "C" void A64_InitGic()
	{
	Gicv3_InitDist(A64_Io(A64_GicdBase));
	A64_InitThisCpuGic();
	}

/* ---- timer ----------------------------------------------------------------------------------------------
 * Each CPU's EL1 virtual timer serves two deadlines, programmed as an absolute CNTV_CVAL: the periodic system tick
 * and the current thread's timeslice. Indexed by MPIDR so a secondary CPU can start its tick before NKern::Init has
 * given it a TSubScheduler. */
static TUint64 TickPeriod;				// counter ticks per system tick
struct SCpuTimer { TUint64 iNextTick; TUint64 iSliceDeadline; TBool iSliceArmed; };
static SCpuTimer CpuTimer[256];
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

extern "C" void A64_TimerInit(TInt aTickUs)
	{
	TickPeriod = (TUint64)NKern::TimestampFrequency() * aTickUs / 1000000;
	}

extern "C" void A64_StartSystemTimer()		// this CPU: tick from now on, no timeslice deadline yet
	{
	SCpuTimer& c = ThisCpuTimer();
	c.iNextTick = NKern::Timestamp() + TickPeriod;
	c.iSliceArmed = FALSE;
	TimerProgram(c);
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

static void TimerIrq(TSubScheduler& aSS, TUint64 aElr)
	{
	SCpuTimer& c = ThisCpuTimer();
	TUint64 now = NKern::Timestamp();
	if (now >= c.iNextTick)
		{
		do c.iNextTick += TickPeriod; while (c.iNextTick <= now);	// catch up, never fire twice for one tick
		A64_TickHook(aSS, aElr);
		}
	if (c.iSliceArmed && now >= c.iSliceDeadline)
		{
		c.iSliceArmed = FALSE;				// expired: SaveTimesliceTimer will read 0
		aSS.iRescheduleNeededFlag = TRUE;	// as the ARM do_timeslice_irq: just ask for a reschedule
		}
	TimerProgram(c);
	}

/* ---- IRQ ----------------------------------------------------------------------------------------------- */
extern "C" void A64_Irq(SThreadExcStack* aF)
	{
	TSubScheduler& ss = SubScheduler();
	// release: a CPU in send_resched_ipi_and_wait that sees us inside an ISR also sees the interrupted thread's stores
	__atomic_store_n(&ss.iSSX.iIrqNestCount, ss.iSSX.iIrqNestCount + 1, __ATOMIC_RELEASE);	// -1 = not in an ISR
	++ss.iSSX.iIrqCount;
	for (;;)
		{
		TUint32 id = Gicv3_Ack();
		if (id >= 1020)
			break;							// nothing (more) pending
		Gicv3_Eoi(id);						// EOI first: a reschedule below may not return for a long time
		if (id == A64_TimerPpi)
			TimerIrq(ss, aF->iElr);
		else if (id == RESCHED_IPI_VECTOR)
			{
			ss.iRescheduleNeededFlag = TRUE;
			resched_ipi_ack(&ss);			// after the flag: an acknowledged CPU reschedules on exit
			}
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

/* ---- PSCI ---------------------------------------------------------------------------------------------- */
extern "C" TInt A64_Psci(TUint64 aFn, TUint64 aA1, TUint64 aA2, TUint64 aA3)
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

extern "C" TInt A64_PsciCpuOn(TUint64 aMpidr, TUint64 aEntry, TUint64 aContext)
	{
	return A64_Psci(0xC4000003, aMpidr, aEntry, aContext);
	}
