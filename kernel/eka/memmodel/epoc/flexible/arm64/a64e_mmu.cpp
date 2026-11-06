// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// SPDX-License-Identifier: EPL-2.0
//
// a64e_mmu.cpp -- address-space switching for the AArch64 ekern: memmodel/epoc/flexible/arm/xsched.cia
// (DoProcessSwitch, DMemModelThread::RestoreAddressSpace) in C++.
//
// An address space is a VMSAv8 level-1 root (mm/mpdalloc.cpp): TTBR0_EL1 = A64RootTablePhys + (OS ASID << 5), with
// the ASID in TTBR0_EL1[63:48] (TCR_EL1.A1 = 0). Global (kernel) mappings carry nG=0, so they survive the switch.
// The nanokernel backend calls board_switch_as() for the thread it is about to run (a64s_ncsched.cpp
// A64_ReschedSelect), which is where ARM's reschedule called the registered process handler.
#define __INCLUDE_NTHREADBASE_DEFINES__
#include <kernel/kern_priv.h>
#include <memmodel.h>
#include <mmboot.h>
#include "mm.h"
#include "mmu.h"
#include "cache_maintenance.inl"

TSubScheduler& SubScheduler();
extern TPhysAddr A64RootTablePhys;						// mm/mpdalloc.cpp

static inline void SetAddressSpace(TUint aOsAsid)
	{
	TUint64 ttbr = (TUint64)(A64RootTablePhys + (aOsAsid << KRootTableShift)) | ((TUint64)aOsAsid << 48);
	__asm__ __volatile__("msr ttbr0_el1, %0\n\tisb" :: "r"(ttbr) : "memory");
	}

// a thread with an IPC alias in place gets its alias PDE back in the (per-CPU slot of its) page directory, and
// the stale translation of the alias page dropped (xsched.cia got_alias)
static void RestoreAlias(DMemModelThread* aThread, TUint aOsAsid)
	{
	TInt irq = NKern::DisableAllInterrupts();			// set the PDE before Mmu::RemoveAliasesForPageTable's IPI can run
	*aThread->iAliasPdePtr = aThread->iAliasPde;
	NKern::RestoreInterrupts(irq);
	SinglePdeUpdated(aThread->iAliasPdePtr);
	LocalInvalidateTLBForPage((aThread->iAliasLinAddr & ~KPageMask) | aOsAsid);
	}

extern "C" void board_switch_as(NThreadBase* aNext)
	{
	DMemModelProcess* p = (DMemModelProcess*)aNext->iAddressSpace;
	if (!p)
		return;											// a nanothread with no process: keep the current space
	TSubScheduler& ss = SubScheduler();
	TUint asid = (TUint)p->OsAsid();
	if (ss.iAddressSpace != p)
		{
		SetAddressSpace(asid);
		ss.iAddressSpace = p;
		}
	DMemModelThread* t = _LOFF(aNext, DMemModelThread, iNThread);
	if (t->iAliasLinAddr)
		RestoreAlias(t, asid);
	}

// the process handler registered by M::Init1 (minit.cpp); here it does for the current thread what the
// reschedule does for the next one
void DoProcessSwitch()
	{
	board_switch_as(NKern::CurrentThread());
	}

// make the current thread's address space match its owning process (after an IPC alias is torn down etc.)
void DMemModelThread::RestoreAddressSpace()
	{
	TInt irq = NKern::DisableAllInterrupts();
	TSubScheduler& ss = SubScheduler();
	NThreadBase* n = ss.iCurrentThread;
	DMemModelProcess* p = (DMemModelProcess*)_LOFF(n, DThread, iNThread)->iOwningProcess;
	SetAddressSpace((TUint)p->OsAsid());
	ss.iAddressSpace = p;
	n->iAddressSpace = p;
	NKern::RestoreInterrupts(irq);
	}
