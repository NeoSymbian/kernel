/* SPDX-License-Identifier: Apache-2.0
 *
 * a64_platform.h -- the AArch64 platform layer shared by the nanokernel test image (nksa) and the full kernel
 * (ekern): GICv3 per-CPU setup, the per-CPU generic-timer multiplexing (system tick + exact timeslice deadline),
 * the interrupt dispatch core, and PSCI calls. What differs per image -- the work done on each tick -- comes back
 * through A64_TickHook, which each image defines.
 */
#ifndef A64_PLATFORM_H
#define A64_PLATFORM_H

struct SThreadExcStack;
class TSubScheduler;

extern "C" {
/* GIC: the distributor (once) and this CPU's redistributor (IPIs SGI 0-7 + the timer PPI) */
void A64_InitGic();
void A64_InitThisCpuGic();

/* timer: set the tick period once, then start each CPU's tick; timeslice deadlines for the scheduler backend */
void A64_TimerInit(TInt aTickUs);
void A64_StartSystemTimer();
void A64_TimesliceSet(TInt aTicks);
TInt A64_TimesliceRemaining();

/* the IRQ dispatch core: GIC ack/EOI, tick + timeslice, IPIs, tied events, preemption on the way out */
void A64_Irq(SThreadExcStack* aFrame);

/* defined by the image: work for one tick on this CPU (aElr = where the tick interrupted it) */
void A64_TickHook(TSubScheduler& aSS, TUint64 aElr);

/* device interrupts (SPIs): an ISR runs in A64_Irq, interrupt context, after the GIC EOI; the line is enabled and
 * routed to CPU 0 */
typedef void (*TA64Isr)(TAny* aPtr);
TInt A64_BindSpi(TUint32 aIntId, TA64Isr aIsr, TAny* aPtr);
/* a software doorbell: SGI 5-7 bound to an ISR; A64_RaiseSgi (any context) runs it on CPU 0, in interrupt context */
TInt A64_BindSgi(TUint32 aIntId, TA64Isr aIsr, TAny* aPtr);
void A64_RaiseSgi(TUint32 aIntId);

/* PSCI 0.2+ through the board's conduit (QEMU virt: HVC, Pixel/TrustZone: SMC) */
TInt A64_Psci(TUint64 aFn, TUint64 aA1, TUint64 aA2, TUint64 aA3);
TInt A64_PsciCpuOn(TUint64 aMpidr, TUint64 aEntry, TUint64 aContext);
}

#endif
