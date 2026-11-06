// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// SPDX-License-Identifier: EPL-1.0
//
// a64e_exec.cpp -- the executive tables and thread entry for the AArch64 ekern (kernel/arm/cexec.cia,
// kernel/arm/ckernel.cia DThread::EpocThreadFunction). port/arm64/ekern/execs.h emits the tables as C arrays
// (EpocFastExecTable / EpocSlowExecTable, genexec.pl's layout); entries are 32-bit addresses, valid under ILP32.
#include <kernel/kern_priv.h>
#include <arm.h>

GLREF_C TInt CalcKernelHeapUsed();
GLREF_C void InvalidFastExec();
void GetLatencyValues(TInt aMode, TInt& aCount, TInt* aDest);
void KernMsgTest();
void InvalidExecHandler();
void PreprocessHandler();

#define __GEN_KERNEL_EXEC_CODE__
#include "execs.h"

void InvalidFastExec()
	{
	InvalidExecHandler();
	}

// shortcut: the slow-exec preprocessor (handle -> object) belongs to the exec gate, which is M4.4 (no user code yet)
void PreprocessHandler()
	{
	NKFault("a64 PreprocessHandler: exec gate not implemented (M4.4)", 0);
	}

// First code a Symbian thread runs. A kernel thread calls its function and exits with the result.
// shortcut: user threads (copy SThreadCreateInfo to the user stack top, ERET to AArch32 EL0 at the entry point
// with r4 = the entry reason) are M4.4; until then a user thread faults here, loudly
void DThread::EpocThreadFunction(TAny* aPtr)
	{
	SThreadCreateInfo& info = *(SThreadCreateInfo*)aPtr;
	DThread* t = TheCurrentThread;
	if (t->iThreadType != EThreadUser)
		Kern::Exit((*info.iFunction)(info.iPtr));		// never returns
	NKFault("a64 EpocThreadFunction: user threads not implemented (M4.4)", 0);
	}
