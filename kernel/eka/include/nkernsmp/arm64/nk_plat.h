/* Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
 * SPDX-License-Identifier: EPL-1.0
 *
 * port/arm64/a64smp/nk_plat.h -- AArch64 (ILP32) SMP nanokernel platform header.
 *
 * Shadows e32/include/nkernsmp/arm/nk_plat.h (selected by -I port/arm64/a64smp ahead of the baseline includes,
 * so the baseline tree is untouched). Modelled on Phorepoc's x64smp/nk_plat.h: under ILP32 pointers are 4 bytes,
 * so the per-CPU extras keep the 2010 ARM field layout and the neutral core's size asserts (256 / 128) hold.
 * Register frames are AArch64: 64-bit slots. A thread's user side is AArch32 at EL0 (r0-r14 = x0-x14).
 */
#ifndef __NK_A64SMP_H__
#define __NK_A64SMP_H__
#include <nk_cpu.h>

// TExcTrap/TPagingExcTrap::iState is TUint32[EXC_TRAP_CTX_SZ]: callee-saved x19-x30 + sp = 13 x 8 bytes = 26 words.
#undef  EXC_TRAP_CTX_SZ
#define EXC_TRAP_CTX_SZ	26

class TSubScheduler;
class TScheduler;
struct Gicv3Redist;			// this CPU's GICv3 redistributor frame (defined by the backend)
struct SFullArmRegSet;		// a CPU's full register set in the AArch32 crash/exception layout (ekern)

// --- Per-CPU / global arch extras: field layout of the 2010 ARM header where it still applies ------------
struct TSubSchedulerX
	{
	TUint32				iSSXP[3];
	TUint32				iMpidr;					// MPIDR_EL1 affinity of this CPU (PSCI / ICC_SGI1R target)
	TUint32				iSSXP1;
	TAny*				iGicDistAddr;			// GICv3 distributor (also in TScheduler)
	Gicv3Redist*		iGicRedistAddr;			// this CPU's redistributor
	TUint32				iSSXP4;
	volatile TUint32	iIrqCount;				// count of interrupts handled
	volatile TInt		iIrqNestCount;			// IRQ nest count for this CPU (starts at -1)
	TAny*				iExcInfo;				// pointer to exception info for crash debugger
	volatile TInt		iCrashState;			// 0=normal, 1=this CPU faulted, 2=this CPU has received a crash IPI
	union {										// the ARM layout: ekern's kernel/arm/{ckernel,cutils}.cpp reach a
		TLinAddr		iSSXP5_0;				// faulting CPU's saved register set through iRegs
		SFullArmRegSet*	iRegs;
		};
	TLinAddr			iSSXP5[2];
	TLinAddr			iIrqStackTop;			// Top of IRQ stack for this CPU
	SRatioInv* volatile	iNewCpuFreqRI;			// set when CPU frequency has been changed
	SRatioInv* volatile	iNewTimerFreqRI;		// set when CPU local timer frequency has been changed
	SRatioInv			iCpuFreqRI;				// Ratio of CPU frequency to maximum possible CPU frequency
	SRatioInv			iTimerFreqRI;			// Ratio of CPU local timer frequency to maximum possible
	TUint32				iSSXP2[36];
	TUint64				iSSXP3;					// one 64 bit value to guarantee alignment
	};

struct TSchedulerX
	{
	TUint64				iTimerMax;				// Maximum per-CPU timer frequency (after prescaling)
	TUint32				iSXP[3];
	TAny*				iGicDistAddr;			// GICv3 distributor
	TAny*				iGicRedistBase;			// first redistributor frame
	TUint32				iSXP1;
	SRatioInv			iGTimerFreqRI;			// ratio of generic timer frequency to maximum possible
	TUint64				iCount0;				// generic timer count at last frequency change
	TUint64				iTimestamp0;			// timestamp at last frequency change
	TUint32				iSXP2[16];
	};

// --- IPI (SGI) and per-CPU interrupt numbers --------------------------------------------------------------------
#define	RESCHED_IPI_VECTOR				0x00
#define	GENERIC_IPI_VECTOR				0x01
#define	TRANSFERRED_IRQ_VECTOR			0x02
#define	CRASH_IPI_VECTOR				0x03
#define	BOOT_IPI_VECTOR					0x04
#define	INDIRECT_POWERDOWN_IPI_VECTOR	0x04
#define RESERVED_IPI_VECTOR_1			0x05
#define RESERVED_IPI_VECTOR_2			0x06
#define IDLE_WAKEUP_IPI_VECTOR			0x07
#define	TIMESLICE_VECTOR				27		// PPI 27 = EL1 virtual timer (CNTV), as Phorepoc's nkernsmp-gt

extern "C" TSubScheduler* SubSchedulerLookupTable[256];	// MPIDR (A64_MpidrIndex) -> subscheduler, before TPIDR_EL1 is set

#define IRQ_STACK_SIZE	4096

// The nanokernel scheduler/DFC/etc. are C++ (not machine-coded) in this port: leave every __*_MACHINE_CODED__ off.

const TUint32 KNThreadContextFlagThumbBit0=1;

// --- Per-thread arch context ----------------------------------------------------------------------------------
class TArmRegSet;
class NThread : public NThreadBase
	{
public:
	TInt Create(SNThreadCreateInfo& anInfo, TBool aInitial);
	void Stillborn();
	void GetUserContext(TArmRegSet& aContext, TUint32& aAvailRegMask);
	void SetUserContext(const TArmRegSet& aContext, TUint32& aRegMask);
	void GetSystemContext(TArmRegSet& aContext, TUint32& aAvailRegMask);
	void CompleteContextSave();
public:
	TUint64	iCoprocessorState[34];	// AArch32 user VFP: D0-D31 + FPSR/FPCR (saved lazily from EL1)
	};
__ASSERT_COMPILE(!(_FOFF(NThread,iCoprocessorState)&7));

// --- On-stack register frames (AArch64) -------------------------------------------------------------------------

// Saved by the scheduler (context switch): callee-saved registers plus the EL0 thread-ID registers.
struct SThreadReschedStack
	{
	TUint64	iX19_X28[10];
	TUint64	iFp;			// x29
	TUint64	iLr;			// x30: return address from Reschedule()
	TUint64	iTpidrEl0;		// AArch32 TPIDRURW
	TUint64	iTpidrroEl0;	// AArch32 TPIDRURO
	TUint64	iReschedFlag;
	TUint64	iSpare;
	};

// Saved on any exception, interrupt or system call. For an AArch32 EL0 thread x0-x14 hold r0-r14_usr.
struct SThreadExcStack
	{
	enum	TType
		{
		EPrefetch	=0,		// instruction abort
		EData		=1,		// data abort
		EUndef		=2,		// undefined instruction
		EIrq		=3,		// IRQ
		EFiq		=4,		// FIQ
		ESvc		=5,		// SVC (exec call)
		EInit		=6,		// Thread has never run
		EStub		=7,		// Stub indicating parameter block still on stack
		};
	TUint64	iX[31];			// x0-x30
	TUint64	iSpEl0;			// user stack pointer (AArch64 EL0); unused for AArch32 (r13_usr is iX[13])
	TUint64	iElr;			// return address
	TUint64	iSpsr;			// return PSTATE (M[4]=1 for AArch32 EL0)
	TUint64	iEsr;
	TUint64	iFar;
	TUint64	iExcCode;		// TType
	TUint64	iPad;			// keep the frame a multiple of 16 bytes
	};
__ASSERT_COMPILE(!(sizeof(SThreadExcStack)&15));

struct SThreadStackStub
	{
	TLinAddr iPBlock;		// pointer to parameter block
	TUint32	iPad;
	TUint64	iExcCode;		// always EStub
	};

struct SThreadInitStack
	{
	SThreadReschedStack		iR;
	SThreadExcStack			iX;
	};

extern "C" {
GLREF_D SCpuIdleHandler CpuIdleHandler;
}

/** Ordering of explicit memory writes / accesses. */
#define	wmb()		__asm__ __volatile__("dmb ishst" ::: "memory")
#define	smp_wmb()	wmb()
#define	mb()		__asm__ __volatile__("dmb ish" ::: "memory")
#define smp_mb()	mb()

/** Send reschedule IPIs to the CPUs identified by the bit mask. */
extern "C" void send_resched_ipis(TUint32 aMask);

#endif
