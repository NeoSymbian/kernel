// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// SPDX-License-Identifier: EPL-2.0
//
// a64e_exc.cpp -- exceptions, traps and user-memory access for the AArch64 ekern: the C++ half of ARM's
// kernel/arm/ckernel.cia, csched.cia, cutils.cia, klib/arm/cumem.cia and nkernsmp/arm/ncutils.cia (Save/UpdateState).
//
// The generic kernel (kernel/arm/ckernel.cpp Exc::Dispatch, compiled unmodified) works on the AArch32 TArmExcInfo.
// The shim below describes an AArch64 exception in it and writes the result back:
//   - from EL0 (AArch32 user code): r0-r14 = x0-x14, r15 = ELR, CPSR = SPSR (the AArch32 layout);
//   - from EL1 (kernel): "SVC mode" (ESvcMode) with the PC in r15, NZCV/I/F from SPSR, r0-r12 = x0-x12;
//   - iExcCode: ARM prefetch abort (EC 0x20/0x21), data abort (0x24/0x25), else undefined instruction;
//   - iFaultStatus = ESR_EL1, iFaultAddress = FAR_EL1 (the contract mm/xmmu.cpp M::DemandPagingFault decodes).
// Dispatch runs on the faulting thread's own stack with interrupts as the faulting code had them; TExcTrap::
// Exception may longjmp out of it.
//
// "Did this call come from user mode?" (kumem*, KUSafeWrite) reads the exec frame at the top of the thread's
// supervisor stack, never the live SPSR_EL1, which any interrupt overwrites.
#define __INCLUDE_NTHREADBASE_DEFINES__
#include <kernel/kern_priv.h>
#include <arm.h>
#include <mmboot.h>
#include "board.h"
#include "a64_platform.h"

TSubScheduler& SubScheduler();

extern "C" {
TInt A64_TrapSaveExc(TExcTrap*, TExcTrapHandler);
TInt A64_TrapSaveIpc(TIpcExcTrap*, DThread*);
TInt A64_TrapSavePaging(TPagingExcTrap*);
__attribute__((noreturn)) void A64_TrapRestoreExc(TExcTrap*, TInt);
__attribute__((noreturn)) void A64_TrapRestorePaging(TPagingExcTrap*, TInt);
TAny* A64_SafeReadKern(const TAny* aSrc, TAny* aDest, TInt aSize);
TAny* A64_SafeReadUser(const TAny* aSrc, TAny* aDest, TInt aSize);
TAny* A64_SafeWriteUser(TAny* aDest, const TAny* aSrc, TInt aSize);
TAny* A64_SafeWriteKern(TAny* aDest, const TAny* aSrc, TInt aSize);
extern char A64_MagicStart[], A64_MagicEnd[];
}

/******************************************************************************
 * Traps (ckernel.cia): naked forwarders to a64e_trap.S, which saves/restores the context; the bookkeeping is here
 ******************************************************************************/
// Trap() must reach the context save with the caller's callee-saved registers and sp untouched: naked, and never
// inlined (GCC copies a naked body verbatim into a caller, dropping the argument set-up around the call)
__attribute__((naked, noinline)) EXPORT_C TInt TExcTrap::Trap()						{ asm("mov x1, xzr\n\tb A64_TrapSaveExc"); }
__attribute__((naked, noinline)) EXPORT_C TInt TExcTrap::Trap(TExcTrapHandler)		{ asm("b A64_TrapSaveExc"); }
__attribute__((naked, noinline)) TInt TIpcExcTrap::Trap(DThread*)					{ asm("b A64_TrapSaveIpc"); }
__attribute__((naked, noinline)) TInt TPagingExcTrap::Trap()						{ asm("b A64_TrapSavePaging"); }
// Exception() restores everything from the trap object, so an ordinary call is fine
EXPORT_C void TExcTrap::Exception(TInt aResult)		{ A64_TrapRestoreExc(this, aResult); __builtin_unreachable(); }
void TPagingExcTrap::Exception(TInt aResult)		{ A64_TrapRestorePaging(this, aResult); __builtin_unreachable(); }

static void DefaultExcTrapHandler(TExcTrap* aTrap, DThread*, TAny*)	// Trap() with no handler: a bad descriptor
	{
	aTrap->Exception(KErrBadDescriptor);
	}

extern "C" TInt A64TrapSetExc(TExcTrap* aTrap, TExcTrapHandler aHandler)
	{
	DThread* t = TheCurrentThread;
	aTrap->iHandler = aHandler ? aHandler : &DefaultExcTrapHandler;
	aTrap->iThread = t;
	t->iExcTrap = aTrap;
	return 0;
	}

extern "C" TInt A64TrapSetIpc(TIpcExcTrap* aTrap, DThread* aClient)
	{
	DThread* t = TheCurrentThread;
	aTrap->iHandler = &DThread::IpcExcHandler;
	aTrap->iThread = t;
	t->iExcTrap = aTrap;
	t->iIpcClient = aClient;
	return 0;
	}

extern "C" TInt A64TrapSetPaging(TPagingExcTrap* aTrap)
	{
	DThread* t = TheCurrentThread;
	aTrap->iThread = t;
	t->iPagingExcTrap = aTrap;
	return 0;
	}

extern "C" void A64TrapClearExc(TExcTrap* aTrap)
	{
	aTrap->iThread->iExcTrap = NULL;
	aTrap->iThread->iPagingExcTrap = NULL;
	}

extern "C" void A64TrapClearPaging(TPagingExcTrap* aTrap)
	{
	aTrap->iThread->iPagingExcTrap = NULL;
	}

TBool TIpcExcTrap::IsTIpcExcTrap()
	{
	return iHandler == &DThread::IpcExcHandler;
	}

/******************************************************************************
 * Magic accessors (csched.cia): a fault inside them returns its address instead of killing anyone
 ******************************************************************************/
TBool Exc::IsMagic(TLinAddr aPc)
	{
	return aPc >= (TLinAddr)A64_MagicStart && aPc < (TLinAddr)A64_MagicEnd;
	}

static TBool CallerIsUser()
	{
	DThread* t = TheCurrentThread;
	if (t->iThreadType != EThreadUser)
		return EFalse;
	NThreadBase* n = &t->iNThread;
	const SThreadExcStack* f = (const SThreadExcStack*)((TLinAddr)n->iStackBase + n->iStackSize) - 1;
	return (f->iSpsr & 0x1F) == 0x10;				// AArch32 EL0 user mode entered this kernel call
	}

EXPORT_C TAny* Kern::SafeRead(const TAny* aSrc, TAny* aDest, TInt aSize)	{ return A64_SafeReadKern(aSrc, aDest, aSize); }
TAny* K::USafeRead(const TAny* aSrc, TAny* aDest, TInt aSize)				{ return A64_SafeReadUser(aSrc, aDest, aSize); }
TAny* K::USafeWrite(TAny* aDest, const TAny* aSrc, TInt aSize)				{ return A64_SafeWriteUser(aDest, aSrc, aSize); }
EXPORT_C TAny* Kern::KUSafeWrite(TAny* aDest, const TAny* aSrc, TInt aSize)
	{
	return CallerIsUser() ? A64_SafeWriteUser(aDest, aSrc, aSize) : A64_SafeWriteKern(aDest, aSrc, aSize);
	}

// decrement aValue if it is > 0; return its old value (ARM's version was "NO GOOD ON SMP": a CAS loop instead)
EXPORT_C TInt Kern::KUSafeDec(TInt& aValue)
	{
	TInt old = __atomic_load_n(&aValue, __ATOMIC_RELAXED);
	while (old > 0 && !__atomic_compare_exchange_n(&aValue, &old, old - 1, false, __ATOMIC_SEQ_CST, __ATOMIC_RELAXED)) {}
	return old;
	}

/******************************************************************************
 * User memory (cumem.cia). User addresses are reached with LDTR/STTR (EL0 permissions, so a user pointer can't
 * reach kernel memory). The IPC alias window is EL1-only (APTable, mm/xmmu.cpp), so a pointer into it -- the
 * remote side of an IPC copy -- is reached with plain loads/stores. A fault ends in Exc::Dispatch: an XTRAP
 * catches it, otherwise the thread is panicked KERN-EXEC 3, as on ARM.
 ******************************************************************************/
static inline TBool InIpcAlias(TLinAddr a)	{ return a - KIPCAlias < KIPCAliasAreaSize; }

static void CopyFromUser(TAny* aDest, const TAny* aSrc, TInt aLength)
	{
	TUint8* d = (TUint8*)aDest;
	TLinAddr s = (TLinAddr)aSrc;
	if (InIpcAlias(s))
		{ memcpy(d, (const TAny*)s, aLength); return; }
	for (; aLength > 0; --aLength, ++s, ++d)
		{
		TUint32 v;
		__asm__ __volatile__("ldtrb %w0, [%1]" : "=r"(v) : "r"((TUint64)s) : "memory");
		*d = (TUint8)v;
		}
	}

static void CopyToUser(TAny* aDest, const TAny* aSrc, TInt aLength)
	{
	TLinAddr d = (TLinAddr)aDest;
	const TUint8* s = (const TUint8*)aSrc;
	if (InIpcAlias(d))
		{ memcpy((TAny*)d, s, aLength); return; }
	for (; aLength > 0; --aLength, ++s, ++d)
		__asm__ __volatile__("sttrb %w0, [%1]" :: "r"((TUint32)*s), "r"((TUint64)d) : "memory");
	}

static void SetUser(TAny* aDest, TUint8 aValue, TInt aLength)
	{
	TLinAddr d = (TLinAddr)aDest;
	if (InIpcAlias(d))
		{ memset((TAny*)d, aValue, aLength); return; }
	for (; aLength > 0; --aLength, ++d)
		__asm__ __volatile__("sttrb %w0, [%1]" :: "r"((TUint32)aValue), "r"((TUint64)d) : "memory");
	}

extern "C" {
EXPORT_C void umemput(TAny* aUserAddr, const TAny* aKernAddr, TInt aLength)	{ CopyToUser(aUserAddr, aKernAddr, aLength); }

EXPORT_C void kumemget(TAny* aKernAddr, const TAny* aAddr, TInt aLength)
	{
	if (CallerIsUser()) CopyFromUser(aKernAddr, aAddr, aLength); else memcpy(aKernAddr, aAddr, aLength);
	}
EXPORT_C void kumemget32(TAny* aKernAddr, const TAny* aAddr, TInt aLength)	{ kumemget(aKernAddr, aAddr, aLength); }
EXPORT_C void kumemget_no_paging_assert(TAny* aKernAddr, const TAny* aAddr, TInt aLength) { kumemget(aKernAddr, aAddr, aLength); }

EXPORT_C void kumemput(TAny* aAddr, const TAny* aKernAddr, TInt aLength)
	{
	if (CallerIsUser()) CopyToUser(aAddr, aKernAddr, aLength); else memcpy(aAddr, aKernAddr, aLength);
	}
EXPORT_C void kumemput32(TAny* aAddr, const TAny* aKernAddr, TInt aLength)	{ kumemput(aAddr, aKernAddr, aLength); }
EXPORT_C void kumemput_no_paging_assert(TAny* aAddr, const TAny* aKernAddr, TInt aLength) { kumemput(aAddr, aKernAddr, aLength); }

EXPORT_C void kumemset(TAny* aAddr, const TUint8 aValue, TInt aLength)
	{
	if (CallerIsUser()) SetUser(aAddr, aValue, aLength); else memset(aAddr, aValue, aLength);
	}

void pagecpy(TAny* aDest, const TAny* aSrc)
	{
	memcpy(aDest, aSrc, KPageSize);
	}
}

// Touch a user address with user permissions on purpose (the memory model uses them to raise the fault a user
// access would): LDTRB/STTRB, as ARM's LDRBT/STRBT in xmmu.cia
void UserReadFault(TLinAddr aAddr)
	{
	TUint32 v;
	__asm__ __volatile__("ldtrb %w0, [%1]" : "=r"(v) : "r"((TUint64)aAddr) : "memory");
	(void)v;
	}

void UserWriteFault(TLinAddr aAddr)
	{
	__asm__ __volatile__("sttrb wzr, [%0]" :: "r"((TUint64)aAddr) : "memory");
	}

// The 84-byte frame a user exception handler gets (euser uc_exe.cia unwinds it): from low to high address
// {type, iExcCode, iFaultAddress, iFaultStatus, iCpsr, r0..r15}, pushed below the user sp
void PushExcInfoOnUserStack(TArmExcInfo* aContext, TInt aType)
	{
	TUint32 w[21];
	w[0] = (TUint32)aType;
	w[1] = (TUint32)aContext->iExcCode;
	w[2] = aContext->iFaultAddress;
	w[3] = aContext->iFaultStatus;
	w[4] = aContext->iCpsr;
	const TArmReg* r = &aContext->iR0;		// iR0..iR14 are consecutive in TArmExcInfo
	for (TInt i = 0; i < 15; ++i)
		w[5 + i] = r[i];
	w[20] = aContext->iR15;
	TLinAddr sp = aContext->iR13 - sizeof(w);
	CopyToUser((TAny*)sp, w, sizeof(w));
	aContext->iR13 = sp;
	}

/******************************************************************************
 * Register state for the crash debugger / Exc::Fault (ncutils.cia, ncglob.cpp, ncthrd.cpp)
 ******************************************************************************/
SFullArmRegSet DefaultRegSet;

// The current CPU's state as an AArch32 "SVC mode" register set; returns with interrupts off, as ARM's does
void Arm::SaveState(SFullArmRegSet& a)
	{
	TUint64 sp, lr, nzcv, daif;
	__asm__ __volatile__("mov %0, sp\n\tmov %1, x30\n\tmrs %2, nzcv\n\tmrs %3, daif" : "=r"(sp), "=r"(lr), "=r"(nzcv), "=r"(daif));
	__asm__ __volatile__("msr daifset, #3" ::: "memory");
	memclr(&a, sizeof(a));
	a.iN.iR13Svc = (TArmReg)sp;
	a.iN.iR14Svc = (TArmReg)lr;
	a.iN.iFlags = (TArmReg)((nzcv & 0xF0000000u) | (daif & 0xC0) | ESvcMode);
	a.iExcCode = KMaxTInt;
	}

void Arm::UpdateState(SFullArmRegSet& a, TArmExcInfo& e)
	{
	TArmReg* n = &a.iN.iR0;
	const TArmReg* r = &e.iR0;
	for (TInt i = 0; i < 15; ++i)
		n[i] = r[i];
	a.iN.iR15 = e.iR15;
	a.iN.iFlags = e.iCpsr;
	a.iN.iR13Svc = e.iR13Svc;
	a.iN.iR14Svc = e.iR14Svc;
	a.iN.iSpsrSvc = e.iSpsrSvc;
	a.iExcCode = e.iExcCode;
	}

void DumpExcInfo(TArmExcInfo& a)
	{
	Kern::Printf("Exc %d Cpsr=%08x FAR=%08x ESR=%08x", a.iExcCode, a.iCpsr, a.iFaultAddress, a.iFaultStatus);
	Kern::Printf(" R0=%08x  R1=%08x  R2=%08x  R3=%08x", a.iR0, a.iR1, a.iR2, a.iR3);
	Kern::Printf(" R4=%08x  R5=%08x  R6=%08x  R7=%08x", a.iR4, a.iR5, a.iR6, a.iR7);
	Kern::Printf(" R8=%08x  R9=%08x R10=%08x R11=%08x", a.iR8, a.iR9, a.iR10, a.iR11);
	Kern::Printf("R12=%08x R13=%08x R14=%08x R15=%08x", a.iR12, a.iR13, a.iR14, a.iR15);
	Kern::Printf("R13Svc=%08x R14Svc=%08x SpsrSvc=%08x", a.iR13Svc, a.iR14Svc, a.iSpsrSvc);
	}

void DumpFullRegSet(SFullArmRegSet& a)
	{
	SNormalRegs& r = a.iN;
	Kern::Printf("MODE_USR: R0=%08x R1=%08x R2=%08x R3=%08x", r.iR0, r.iR1, r.iR2, r.iR3);
	Kern::Printf("MODE_USR: R4=%08x R5=%08x R6=%08x R7=%08x", r.iR4, r.iR5, r.iR6, r.iR7);
	Kern::Printf("MODE_USR: R8=%08x R9=%08x R10=%08x R11=%08x", r.iR8, r.iR9, r.iR10, r.iR11);
	Kern::Printf("MODE_USR: R12=%08x R13=%08x R14=%08x R15=%08x", r.iR12, r.iR13, r.iR14, r.iR15);
	Kern::Printf("CPSR=%08x R13Svc=%08x R14Svc=%08x ExcCode=%d", r.iFlags, r.iR13Svc, r.iR14Svc, a.iExcCode);
	}

/******************************************************************************
 * The exception shim and the vector dispatcher of the ekern image
 ******************************************************************************/
static void SyncException(SThreadExcStack* aF, TUint32 aGroup)
	{
	TBool fromUser = (aGroup >= 2);
	TUint32 ec = (TUint32)(aF->iEsr >> 26);
	TArmExcInfo e;
	memclr(&e, sizeof(e));
	TArmReg* r = &e.iR0;
	TInt nregs = fromUser ? 15 : 13;					// EL1: r13/r14 are the AArch64 kernel's own sp/lr
	for (TInt i = 0; i < nregs; ++i)
		r[i] = (TArmReg)aF->iX[i];
	e.iR15 = (TArmReg)aF->iElr;
	if (fromUser)
		e.iCpsr = (TArmReg)aF->iSpsr;					// AArch32 EL0: SPSR_EL1 is the CPSR
	else
		{
		e.iCpsr = (TArmReg)((aF->iSpsr & 0xF00000C0u) | ESvcMode);	// NZCV, I, F
		e.iR13Svc = (TArmReg)((TLinAddr)(aF + 1));		// sp before the exception (the frame sits right below it)
		e.iR14Svc = (TArmReg)aF->iX[30];
		}
	e.iExcCode = (ec == 0x20 || ec == 0x21) ? EArmExceptionPrefetchAbort
			   : (ec == 0x24 || ec == 0x25) ? EArmExceptionDataAbort : EArmExceptionUndefinedOpcode;
	e.iFaultAddress = (TArmReg)aF->iFar;
	e.iFaultStatus = (TArmReg)aF->iEsr;

	TSubScheduler& ss = SubScheduler();
	if (ss.iKernLockCount || ss.iSSX.iIrqNestCount >= 0)
		Exc::Fault(&e);									// can't run the exception handlers with the kernel locked or in an ISR
	if (!(aF->iSpsr & 0xC0))
		__asm__ __volatile__("msr daifclr, #3\n\tisb" ::: "memory");	// interrupts as the faulting code had them

	if (fromUser && e.iExcCode != EArmExceptionUndefinedOpcode)	// bring-up aid: where a user fault really was
		Kern::Printf("[exc] user %s abort: pc %08x far %08x esr %08x",
			e.iExcCode == EArmExceptionDataAbort ? "data" : "prefetch", e.iR15, e.iFaultAddress, (TUint32)aF->iEsr);
	Exc::Dispatch(&e, NKern::CurrentThread());			// may not return (TExcTrap::Exception, thread panic)

	__asm__ __volatile__("msr daifset, #3" ::: "memory");
	for (TInt i = 0; i < nregs; ++i)					// write back what the handlers changed (r4, r12, r13 ...)
		if ((TArmReg)aF->iX[i] != r[i])
			aF->iX[i] = r[i];
	aF->iElr = e.iR15;
	if (fromUser)
		aF->iSpsr = (e.iCpsr & ~0x1Fu) | 0x10;			// stay in user mode whatever the handler wrote
	else
		aF->iSpsr = (aF->iSpsr & ~0xF0000000ull) | (e.iCpsr & 0xF0000000u);	// e.g. the magic accessor's Z flag
	}

// vectors.S: kind = (group << 2) | type; group 0 EL1/SP0, 1 EL1/SPx, 2 EL0 AArch64, 3 EL0 AArch32;
// type 0 sync, 1 IRQ, 2 FIQ, 3 SError
extern "C" void A64_ExecGate(SThreadExcStack* aF);		// a64e_exec.cpp
extern "C" void A64_ReturnToUser();						// a64e_nkern.cpp: user-mode callbacks
extern "C" TBool A64_A32Fixup(SThreadExcStack* aF);		// a64e_a32fix.cpp: instructions ARMv8 dropped

extern "C" void A64_ExceptionDispatch(SThreadExcStack* aF, TUint64 aKind)
	{
	TUint32 type = (TUint32)aKind & 3, group = (TUint32)aKind >> 2;
	if (group == 3 && type <= 1)						// AArch32 EL0: the user side
		{
		if (type == 1)
			A64_Irq(aF);
		else if ((aF->iEsr >> 26) == 0x11)				// SVC: an exec call
			A64_ExecGate(aF);
		else if (!A64_A32Fixup(aF))
			SyncException(aF, group);
		A64_ReturnToUser();								// interrupts off again here
		return;
		}
	if (type == 1)
		{ A64_Irq(aF); return; }
	if (type == 0 && group < 2)							// EL1; there is no AArch64 user code
		{ SyncException(aF, group); return; }
	A64_DebugPuts("\n*** unexpected exception kind="); A64_DebugHex(aKind);
	A64_DebugPuts(" esr="); A64_DebugHex(aF->iEsr);
	A64_DebugPuts(" elr="); A64_DebugHex(aF->iElr);
	A64_DebugPuts(" far="); A64_DebugHex(aF->iFar);
	A64_DebugPuts("\n");
	for (;;) { __asm__ __volatile__("msr daifset, #3\n\twfi"); }
	}
