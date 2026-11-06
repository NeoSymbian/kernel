// Copyright (c) 1996-2009 Nokia Corporation and/or its subsidiary(-ies).
// All rights reserved.
// This component and the accompanying materials are made available
// under the terms of the License "Eclipse Public License v1.0"
// which accompanies this distribution, and is available
// at the URL "http://www.eclipse.org/legal/epl-v10.html".
//
// Initial Contributors:
// Nokia Corporation - initial contribution.
//
// Contributors:
// AArch64 port: marked-delta copy (port/arm64/mm/xinit.cpp); changes are marked "a64 port".
//
// Description:
//

#include "arm_mem.h"
#include "../mmu/mm.h"

// Set up virtual addresses used for cache flushing if this is
// done by data read or line allocate
void M::SetupCacheFlushPtr(TInt aCache, SCacheInfo& aInfo)
	{
	}

#ifdef __SMP__
extern DMemoryObject* ExceptionStacks;

// a64 port: AArch64 takes every EL1 exception on SP_EL1, so the four AArch32 mode stacks (IRQ/FIQ/UND/ABT) become
// an initial kernel stack and an IRQ stack, in the same ExceptionStacks slots ARM used for IRQ (1) and FIQ (3).
void M::GetAPBootInfo(TInt aCpu, volatile SAPBootInfo* aInfo)
	{
	SArmAPBootInfo* b = (SArmAPBootInfo*)aInfo;
	TInt i;
	for (i=1; i<=3; i+=2)
		{
		TUint ix = (TUint)(aCpu*8+i);
		TInt r = MM::MemoryAlloc(ExceptionStacks, ix, 1);
		__NK_ASSERT_ALWAYS(r==KErrNone);
		TLinAddr addr = KExcptStacksLinearBase + (ix<<KPageShift);
		TLinAddr top = addr + KPageSize;
		TUint8 fill = (i==1) ? 0xaa : 0xbb;
		if (i==1)
			b->iInitIrqSp = top;
		else
			b->iInitSp = top;
		memset((TAny*)addr, fill, top-addr);
		}
	}

void M::Init2AP()
	{
	__KTRACE_OPT(KBOOT,Kern::Printf("M::Init2AP()"));
	DThread& t = Kern::CurrentThread();
	SubScheduler().iAddressSpace = t.iOwningProcess;
	t.iNThread.SetAddressSpace(t.iOwningProcess);
	t.iNThread.SetAttributes(KThreadAttAddressSpace);
	}
#endif
