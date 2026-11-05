/* Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
 * SPDX-License-Identifier: EPL-1.0
 *
 * port/arm64/a64smp/ncern.h -- AArch64 variant interface: shadows e32/include/nkernsmp/arm/ncern.h.
 * Same shape as the 2010 ARM header, with the GICv2/SCU/A9-timer addresses replaced by GICv3 + generic timer.
 * The __ASM_CLI/__ASM_STI macros of the ARM header are AArch32 asm and have no users in the C++ backend.
 */
#ifndef __NCERN_H__
#define __NCERN_H__

/** Information needed to boot an AP (CPU_ON via PSCI) */
struct SArmAPBootInfo : public SAPBootInfo
	{
	TUint64		iMpidr;				// MPIDR affinity of the AP (PSCI target_cpu)
	T_UintPtr	iEntryPhys;			// physical entry point handed to PSCI CPU_ON
	TLinAddr	iInitSp;			// initial EL1 stack of the AP
	TLinAddr	iInitIrqSp;			// top of the AP's IRQ stack
	};

typedef void (*TDetachComplete)(void);

struct SPerCpuUncached
	{
	volatile TUint32	iDetachCount;
	volatile TUint32	iAttachCount;
	volatile TBool		iPowerOffReq;
	volatile TBool		iPowerOnReq;
	TDetachComplete		iDetachCompleteFn;
	volatile TUint32	iDetachCompleteCpus;
	};

union UPerCpuUncached
	{
	SPerCpuUncached		iU;
	volatile TUint64	i__Dummy[8];
	};
__ASSERT_COMPILE(sizeof(SPerCpuUncached) <= 8*sizeof(TUint64));

typedef void (*TCpuPowerUpFn)(TInt aCpu, SPerCpuUncached* aU);
typedef void (*TCpuPowerDownFn)(TInt aCpu, SPerCpuUncached* aU);
typedef TInt (*TFrequencyChangeFn)();

/** Variant interface block */
struct SVariantInterfaceBlock : public SInterfaceBlockBase
	{
	TUint64		iMaxCpuClock;				// maximum possible CPU clock frequency on this system
	TUint16		iTimerGap1;
	TUint16		iTimerGap2;
	TUint32		iMaxTimerClock;				// generic timer frequency (CNTFRQ_EL0)
	TLinAddr	iGicDistAddr;				// GICv3 distributor
	TLinAddr	iGicRedistAddr;				// first GICv3 redistributor (frames are contiguous, 128 KB stride)
	TUint32		iPsciConduit;				// 0 = SMC, 1 = HVC (from the DTB /psci node)
	TUint32		iSpare;
	SRatio*		iTimerFreqR[KMaxCpus];
	SRatio*		iCpuFreqR[KMaxCpus];
	UPerCpuUncached*		iUncached[KMaxCpus];
	TCpuPowerUpFn			iCpuPowerUpFn;
	TCpuPowerDownFn			iCpuPowerDownFn;
	SRatio*		iGTimerFreqR;
	TFrequencyChangeFn		iFrqChgFn;
	};

#endif
