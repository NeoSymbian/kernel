/* SPDX-License-Identifier: Apache-2.0
 *
 * a64_sysreg.h -- the few AArch64 system-register operations the nanokernel backend needs:
 * interrupt masking (DAIF), CPU identity (MPIDR_EL1, TPIDR_EL1 = this CPU's TSubScheduler*),
 * the generic timer counter, and the GICv3 CPU interface (ICC_*_EL1, system-register access).
 */
#ifndef A64_SYSREG_H
#define A64_SYSREG_H

#define A64_SYSREG_READ(reg)		({ TUint64 _v; __asm__ __volatile__("mrs %0, " #reg : "=r"(_v) :: "memory"); _v; })
#define A64_SYSREG_WRITE(reg, v)	__asm__ __volatile__("msr " #reg ", %0" :: "r"((TUint64)(v)) : "memory")

/* DAIF.I (bit 7) and DAIF.F (bit 6): masking both is "all interrupts off" (the ARM port's CPSIDIF) */
#define A64_DAIF_IF		0xC0
static inline void A64_CLI()	{ __asm__ __volatile__("msr daifset, #3" ::: "memory"); }
static inline void A64_STI()	{ __asm__ __volatile__("msr daifclr, #3" ::: "memory"); }
/* wait for an interrupt with interrupts MASKED (a pending one still wakes the core), then take it: the AArch64
 * form of x86 "sti; hlt", race-free because WFI completes on a pending-but-masked interrupt */
static inline void A64_IDLE()	{ __asm__ __volatile__("dsb sy\n\twfi\n\tmsr daifclr, #3" ::: "memory"); }
static inline void A64_PAUSE()	{ __asm__ __volatile__("yield" ::: "memory"); }

/* MPIDR affinity packed as Aff3:Aff2:Aff1:Aff0 in 32 bits (the form ICC_SGI1R_EL1 and PSCI want split out) */
static inline TUint32 A64_Mpidr()
	{
	TUint64 m = A64_SYSREG_READ(mpidr_el1);
	return (TUint32)(m & 0x00FFFFFF) | (TUint32)((m >> 8) & 0xFF000000);
	}

/* GICv3: send SGI aIntId to the CPU with packed affinity aMpidr (one target, IRM=0) */
static inline void A64_SendSgi(TUint32 aMpidr, TUint32 aIntId)
	{
	TUint64 aff0 = aMpidr & 0xFF, aff1 = (aMpidr >> 8) & 0xFF, aff2 = (aMpidr >> 16) & 0xFF, aff3 = aMpidr >> 24;
	TUint64 v = (aff3 << 48) | (aff2 << 32) | ((TUint64)(aIntId & 0xF) << 24) | (aff1 << 16) | (1ull << (aff0 & 0xF));
	__asm__ __volatile__("dsb ishst" ::: "memory");		/* make prior stores visible before the target runs */
	A64_SYSREG_WRITE(S3_0_C12_C11_5, v);					/* ICC_SGI1R_EL1 */
	__asm__ __volatile__("isb" ::: "memory");
	}
static inline TUint32 A64_GicAck()			{ return (TUint32)A64_SYSREG_READ(S3_0_C12_C12_0); }	/* ICC_IAR1_EL1 */
static inline void A64_GicEoi(TUint32 aIntId)	{ A64_SYSREG_WRITE(S3_0_C12_C12_1, aIntId); }		/* ICC_EOIR1_EL1 */

#endif
