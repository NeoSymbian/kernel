// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// SPDX-License-Identifier: EPL-1.0
//
// a64e_exec.cpp -- the executive tables, the exec-call gate and thread entry for the AArch64 ekern
// (kernel/arm/cexec.cia, nkernsmp/arm/vectors.cia __ArmVectorSwi, kernel/arm/ckernel.cia
// DThread::EpocThreadFunction). port/arm64/ekern/execs.h emits the tables as C arrays (EpocFastExecTable /
// EpocSlowExecTable, genexec.pl's layout); entries are 32-bit addresses, valid under ILP32.
//
// The user side is unchanged 32-bit ARM code at AArch32 EL0: r0-r14 are x0-x14 of the exception frame, the pc is
// ELR_EL1 and the CPSR is SPSR_EL1.
#define __INCLUDE_NTHREADBASE_DEFINES__
#include <kernel/kern_priv.h>
#include <nk_priv.h>
#include <arm.h>

#ifndef __USERSIDE_THREAD_DATA__
#error "EpocThreadFunction below sets up the user-side thread data (TPIDRURW), as the ARMv7 kernel does"
#endif

GLREF_C TInt CalcKernelHeapUsed();
GLREF_C void InvalidFastExec();
void GetLatencyValues(TInt aMode, TInt& aCount, TInt* aDest);
void KernMsgTest();
void InvalidExecHandler();
void PreprocessHandler();

#define __GEN_KERNEL_EXEC_CODE__
#include "execs.h"

extern "C" {
TAny* A64_SafeReadUser(const TAny* aSrc, TAny* aDest, TInt aSize);			// a64e_trap.S
__attribute__((noreturn)) void A64_EnterUser(TUint32 aR4, TUint32 aSp, TUint32 aPc, TLinAddr aSvcStackTop);
}

void InvalidFastExec()
	{
	InvalidExecHandler();
	}

// The slow-exec table's preprocess slot. The gate below calls A64PreprocessHandler directly with the arguments in
// memory (as Win32 and Phorepoc x86-64 do), so nothing jumps here.
void PreprocessHandler()
	{
	NKFault("a64 PreprocessHandler: called through the table", 0);
	}

// Kernel-side callers of the HAL (kernel/arm/cexec.cia, kernel/win32/cexec.cpp): straight into the exec handler
EXPORT_C TInt Kern::HalFunction(TInt aGroup, TInt aFunction, TAny* a1, TAny* a2)
	{
	CHECK_PRECONDITIONS(MASK_THREAD_STANDARD,"Kern::HalFunction(TInt, TInt, TAny*, TAny*)");
	return ExecHandler::HalFunction(aGroup, aFunction, a1, a2);
	}

EXPORT_C TInt Kern::HalFunction(TInt aGroup, TInt aFunction, TAny* a1, TAny* a2, TInt aDeviceNumber)
	{
	CHECK_PRECONDITIONS(MASK_THREAD_STANDARD,"Kern::HalFunction(TInt, TInt, TAny*, TAny*, TInt)");
	return ExecHandler::HalFunction(aGroup | (aDeviceNumber << 16), aFunction, a1, a2);
	}

// handle -> object for exec calls flagged "preprocess" (kernel/win32/cexec.cpp Win32PreprocessHandler; ARM's
// cexec.cia does the same in assembler). Called with the system lock held.
static void A64PreprocessHandler(TInt* aArgs, TUint32 aFlags)
	{
	DThread* t = TheCurrentThread;
	aFlags &= 0x3f;
	if (aFlags >= EIpcMessageD)
		{
		RMessageK* m = RMessageK::MessageK(aArgs[0], t);
		if (m == NULL || (aFlags != EIpcMessageD && m->iFunction == RMessage2::EDisConnect))
			K::PanicCurrentThread(EBadMessageHandle);
		if (aFlags == EIpcClient)
			aArgs[0] = TInt(m->iClient);
		return;
		}
	DObject* obj = Kern::ObjectFromHandle(t, aArgs[0], TObjectType(TInt(aFlags) - 1));
	if (obj)
		aArgs[0] = TInt(obj);
	else
		K::PanicCurrentThread(EBadHandle);
	}

/******************************************************************************
 * The exec-call gate: an SVC from AArch32 EL0 (ESR EC 0x11), entered from A64_ExceptionDispatch with interrupts off
 * on the thread's supervisor stack. The call number is the SVC's 24-bit immediate (bit 23 = fast), read from the
 * instruction itself: ESR keeps only its low 16 bits.
 ******************************************************************************/
typedef TInt (*TFastExecFn)(TInt, NThread*);
typedef TInt (*TSlowExecFn)(TInt, TInt, TInt, TInt);

extern "C" void A64_ExecGate(SThreadExcStack* aF)
	{
	NThread* t = NKern::CurrentThread();
	TUint32 insn = 0x7FFFFF;							// unreadable, or a Thumb SVC (8-bit immediate): an invalid call
	if (!(aF->iSpsr & 0x20))
		{
		TUint32 w;
		if (!A64_SafeReadUser((const TAny*)(TLinAddr)(aF->iElr - 4), &w, 4))
			insn = w;
		}
	TUint32 n = insn & 0x7FFFFF;
	if (insn & 0x00800000)
		{
		if (n == 0)										// EFastExecWaitForAnyRequest: blocks, so interrupts on
			{
			__asm__ __volatile__("msr daifclr, #3" ::: "memory");
			NKern::WaitForAnyRequest();
			__asm__ __volatile__("msr daifset, #3" ::: "memory");
			return;
			}
		const SFastExecTable* ft = t->iFastExecTable;
		if (n < (TUint32)ft->iFastExecCount)
			{
			// fast execs run with interrupts off and return r0 only (ARM fast_swi_exit)
			aF->iX[0] = (TUint32)((TFastExecFn)ft->iFunction[n - 1])((TInt)aF->iX[0], t);
			return;
			}
		n = 0x7FFFFF;									// on to the slow table's invalid handler
		}

	const SSlowExecEntry* e = t->iSlowExecTable;		// -> iEntries[0]: count, invalid, preprocess sit before it
	const TLinAddr* hdr = (const TLinAddr*)e;
	TInt a[4] = { (TInt)aF->iX[0], (TInt)aF->iX[1], (TInt)aF->iX[2], (TInt)aF->iX[3] };
	__asm__ __volatile__("msr daifclr, #3" ::: "memory");
	TInt r;
	if (n >= (TUint32)(TInt)hdr[-3])
		r = ((TSlowExecFn)hdr[-2])(a[0], a[1], a[2], a[3]);	// InvalidExecHandler: panics the thread
	else
		{
		TUint32 flags = e[n].iFlags;
		TUint32 extra[8];
		if (flags & KExecFlagClaim)
			NKern::LockSystem();
		if (flags & KExecFlagExtraArgMask)
			{
			// ARM passes r2 = &frame.iR4: the user's r4.. as consecutive words (u32exec.h stubs load them)
			for (TInt i = 0; i < 8; ++i)
				extra[i] = (TUint32)aF->iX[4 + i];
			a[2] = (TInt)extra;
			}
		if (flags & KExecFlagPreprocess)
			A64PreprocessHandler(a, flags);
		r = ((TSlowExecFn)e[n].iFunction)(a[0], a[1], a[2], a[3]);
		if (flags & KExecFlagRelease)
			NKern::UnlockSystem();
		}
	__asm__ __volatile__("msr daifset, #3" ::: "memory");
	aF->iX[0] = (TUint32)r;
	}

/******************************************************************************
 * First code a Symbian thread runs. A kernel thread calls its function and exits with the result. A user thread
 * (ckernel.cia) gets its create-info block copied below the top of its user stack, then enters AArch32 EL0 at the
 * process entry point: r4 = entry reason, r13 = the copied block, USR mode, ARM state, interrupts on.
 ******************************************************************************/
void DThread::EpocThreadFunction(TAny* aPtr)
	{
	SThreadCreateInfo& info = *(SThreadCreateInfo*)aPtr;
	DThread* t = TheCurrentThread;
	if (t->iThreadType != EThreadUser)
		Kern::Exit((*info.iFunction)(info.iPtr));		// never returns

	// TLocalThreadData at the very top of the user stack, TPIDRURW (TPIDR_EL0) pointing at it, its thread id set;
	// then the create-info block right below it, which r13_usr points at (ckernel.cia). Unprivileged stores:
	// a bad user stack faults cleanly.
	TLinAddr top = t->iUserStackRunAddress + t->iUserStackSize;
	TLinAddr tld = top - KLocalThreadDataSize;
	kumemset((TAny*)tld, 0, KLocalThreadDataSize);
	TUint32 id = t->iId;
	kumemput32((TAny*)(tld + _FOFF(TLocalThreadData, iThreadId)), &id, sizeof(id));
	NThread& n = t->iNThread;
	n.iUserTid[0] = tld;
	__asm__ __volatile__("msr tpidr_el0, %0" :: "r"((TUint64)tld));
	t->iFlags |= KThreadFlagLocalThreadDataValid;
	TInt size = info.iTotalSize;
	TLinAddr usp = tld - size;
	kumemput((TAny*)usp, &info, size);
	TBool first = t->iFlags & KThreadFlagOriginal;
	DProcess* p = t->iOwningProcess;
	TLinAddr pc = first ? p->iCodeSeg->iEntryPtVeneer : p->iReentryPoint;
	TUint32 reason = first ? KModuleEntryReasonProcessInit : KModuleEntryReasonThreadInit;
	t->iUserThreadState = EUserThreadRunning;
	// the supervisor stack is empty while user code runs: every exception from EL0 then pushes its frame at the top
	A64_EnterUser(reason, (TUint32)usp, (TUint32)pc, (TLinAddr)n.iStackBase + n.iStackSize);
	}
