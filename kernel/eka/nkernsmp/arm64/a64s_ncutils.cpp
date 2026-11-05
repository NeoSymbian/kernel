// SPDX-License-Identifier: Apache-2.0
// Derived from Phorepoc adaptation-x86-64 port/x64smp/x64s_ncutils.cpp (Copyright The Phorepoc Authors, Apache-2.0).

/* a64s_ncutils.cpp -- AArch64 SMP backend: the per-CPU NKern primitives the arch-neutral nkernsmp core
 * references (interrupt masking, CPU identity, timestamp, idle, mem fill). The scheduler-entangled entry
 * points (Lock/Unlock/Reschedule) live in a64s_ncsched.cpp.
 */
#include <e32def.h>
#include <nkern.h>
#include <nk_priv.h>
#include "a64_sysreg.h"

extern "C" {

// weak: a nanokernel-only image uses these; the full kernel (ekern) supplies the real ones
__attribute__((weak)) EXPORT_C TAny* memclr(TAny* aTrg, unsigned int aLength)
	{
	volatile TUint8* d = (volatile TUint8*)aTrg;	// volatile: keep GCC from turning this back into a memset call
	while (aLength--) *d++ = 0;
	return aTrg;
	}

__attribute__((weak)) EXPORT_C TAny* memset(TAny* aTrg, TInt aValue, unsigned int aLength)
	{
	volatile TUint8* d = (volatile TUint8*)aTrg;
	while (aLength--) *d++ = (TUint8)aValue;
	return aTrg;
	}

} // extern "C"

/* --- NKern interrupt control (token = the saved DAIF I/F bits) -------------------------------------------- */
EXPORT_C TInt NKern::DisableAllInterrupts()
	{
	TUint64 daif = A64_SYSREG_READ(daif);
	A64_CLI();
	return (TInt)(daif & A64_DAIF_IF);
	}

EXPORT_C void NKern::EnableAllInterrupts()
	{
	A64_STI();
	}

EXPORT_C void NKern::RestoreInterrupts(TInt aRestoreData)
	{
	if (aRestoreData & A64_DAIF_IF)
		A64_CLI();
	else
		A64_STI();
	}

/* --- Per-CPU identity --------------------------------------------------------------------------------------
 * TPIDR_EL1 holds this CPU's TSubScheduler* (set when the CPU's initial thread is created, NThread::Create):
 * one register read, unlike the amd64 port's TSS/APIC dance. Before that, fall back to the MPIDR lookup. */
static inline TSubScheduler* SubSchedulerFromTpidr()
	{
	return (TSubScheduler*)(TLinAddr)A64_SYSREG_READ(tpidr_el1);
	}

extern "C" TUint32 A64_MpidrIndex(TUint32 aMpidr)	// SubSchedulerLookupTable index: Aff1:Aff0 folded into 8 bits
	{
	return ((aMpidr >> 4) & 0xF0) | (aMpidr & 0x0F);
	}

/* Before NKern::Init gives this CPU its pointer, only the boot CPU runs kernel code (KernelMain's early
 * KPrintf calls NKern::CurrentThread): it is CPU 0, as the 2010 ARM bootstrap assumed by presetting TPIDRPRW
 * to &TheSubSchedulers[0]. (Found on the Pixel: QEMU's flash at VA 0 had hidden the NULL dereference.) */
static inline TSubScheduler* ThisSubScheduler()
	{
	TSubScheduler* ss = SubSchedulerFromTpidr();
	if (!ss)
		ss = SubSchedulerLookupTable[A64_MpidrIndex(A64_Mpidr())];
	return ss ? ss : &TheSubSchedulers[0];
	}

EXPORT_C TInt NKern::CurrentCpu()
	{
	return (TInt)ThisSubScheduler()->iCpuNum;
	}

TSubScheduler& SubScheduler()
	{
	return *ThisSubScheduler();
	}

/* --- Timestamp: the generic timer's virtual count (CNTVCT_EL0) at CNTFRQ_EL0 ------------------------------- */
EXPORT_C TUint64 NKern::Timestamp()
	{
	__asm__ __volatile__("isb" ::: "memory");	// keep the counter read from being speculated early
	return A64_SYSREG_READ(cntvct_el0);
	}

EXPORT_C TUint32 NKern::TimestampFrequency()
	{
	return (TUint32)A64_SYSREG_READ(cntfrq_el0);
	}

EXPORT_C TUint32 NKern::CpuTimeMeasFreq()
	{
	return NKern::TimestampFrequency();
	}

/* --- Idle -------------------------------------------------------------------------------------------------- */
EXPORT_C void NKern::DoIdle()
	{
	A64_IDLE();
	}
