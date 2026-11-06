// Copyright (c) 2007-2009 Nokia Corporation and/or its subsidiary(-ies).
// All rights reserved.
// This component and the accompanying materials are made available
// under the terms of the License "Eclipse Public License v1.0"
// which accompanies this distribution, and is available
// at the URL "http://www.eclipse.org/legal/epl-v10.html".
// SPDX-License-Identifier: EPL-2.0 (changed by the NeoSymbian Authors)
// Initial Contributors:
// Nokia Corporation - initial contribution.
//
// Contributors:
// The NeoSymbian Authors (2026): the AArch64 per-address-space level-1 root under __CPU_ARM64.
// Description:
//

#include "memmodel.h"
#include "mm.h"
#include "mmu.h"

#include "mpdalloc.h"
#include "mobject.h"
#include "cache_maintenance.inl"


// check enough space for page directories...
__ASSERT_COMPILE(KNumOsAsids <= (KPageDirectoryEnd-KPageDirectoryBase)/KPageDirectorySize);


PageDirectoryAllocator PageDirectories;

#ifdef __CPU_ARM64
// NeoSymbian arm64: the per-address-space VMSAv8 level-1 root (TTBR0_EL1 target, 4 table descriptors: the address space's own
// two level-2 tables for 0-2 GB, then the kernel's two for 2-4 GB). Slot = KRootTableBase + OS ASID * 32; the
// bootstrap maps the 8 KB and fills the kernel's (ASID 0) slot. The process switch loads
// TTBR0_EL1 = A64RootTablePhys + (asid<<KRootTableShift) | asid<<48.
TPhysAddr A64RootTablePhys;

static inline TPde* A64RootSlot(TUint aOsAsid)
	{
	return (TPde*)(KRootTableBase + (aOsAsid<<KRootTableShift));
	}

static void A64SetRoot(TUint aOsAsid, TPhysAddr aLocalPd, TPhysAddr aKernelPd)
	{
	TPde* root = A64RootSlot(aOsAsid);
	root[0] = (TPde)aLocalPd | KA64DescTable;
	root[1] = (TPde)(aLocalPd+KPageSize) | KA64DescTable;
	root[2] = (TPde)(aKernelPd+2*KPageSize) | KA64DescTable;
	root[3] = (TPde)(aKernelPd+3*KPageSize) | KA64DescTable;
	__asm__ __volatile__("dsb ishst" ::: "memory");		// visible to the table walker before the ASID is used
	}
#endif

const TUint KLocalPdShift = KPageDirectoryShift > KPageShift ? KPageDirectoryShift-1 : KPageShift;
const TUint KLocalPdSize = 1<<KLocalPdShift;
const TUint KLocalPdPages = 1<<(KLocalPdShift-KPageShift);


__ASSERT_COMPILE((KPageDirectoryBase&(31*KPageDirectorySize))==0); // following code assumes this alignment

void PageDirectoryAllocator::GlobalPdeChanged(TPde* aPde)
	{
	__NK_ASSERT_DEBUG(MmuLock::IsHeld());
	__NK_ASSERT_DEBUG(TLinAddr(aPde)>=KPageDirectoryBase);
	__NK_ASSERT_DEBUG(TLinAddr(aPde)<KPageDirectoryEnd);
	__NK_ASSERT_DEBUG(KLocalPdSize==(TUint)KPageDirectorySize); // shouldn't be called if we have separate global PDs

	TLinAddr addr = (TLinAddr(aPde)&KPageDirectoryMask)*(KChunkSize/sizeof(TPde));
	if(addr<KGlobalMemoryBase)
		return; // change was in local part of PD, so nothing to do
	if(addr-KIPCAlias<KIPCAliasAreaSize)
		return; // change was in IPC alias area, so nothing to do
	if(!iAllocator)
		return; // not yet initialised

	TRACE2(("PageDirectoryAllocator::GlobalPdeChanged(0x%08x)",aPde));
	TPde pde = *aPde;
	TLinAddr pPde = KPageDirectoryBase+(TLinAddr(aPde)&KPageDirectoryMask); // first page directory

	// copy PDE to all allocated page directories
	pPde -= KPageDirectorySize; // start off at PD minus one
	TLinAddr lastPd = KPageDirectoryBase+(KNumOsAsids-1)*KPageDirectorySize;
	TUint32* ptr = iAllocator->iMap;
	do
		{
		TUint32 bits = ~*ptr++;
		do
			{
			pPde += KPageDirectorySize; // step to next page directory
			if(bits&0x80000000u)
				{
				TRACE2(("!PDE %x=%x",pPde,pde));
				*(TPde*)pPde = pde;
				CacheMaintenance::SinglePdeUpdated(pPde);
				}
			}
		while(bits<<=1);
		pPde |= 31*KPageDirectorySize; // step to next group of 32 PDs
		}
	while(pPde<lastPd);
	}


void PageDirectoryAllocator::Init2()
	{
	TRACEB(("PageDirectoryAllocator::Init2()"));

	// construct memory object for page directories...
#if defined(__CPU_PAGE_TABLES_FULLY_CACHED)
	TMemoryAttributes memAttr = EMemoryAttributeStandard;
#else
	TMemoryAttributes memAttr = (TMemoryAttributes)(EMemoryAttributeNormalUncached|EMemoryAttributeDefaultShareable);
#endif
	TInt r = MM::InitFixedKernelMemory(iPageDirectoryMemory, KPageDirectoryBase, KPageDirectoryEnd, KPageDirectorySize, EMemoryObjectHardware, EMemoryCreateNoWipe, memAttr, EMappingCreateFixedVirtual);
	__NK_ASSERT_ALWAYS(r==KErrNone);

	// initialise kernel page directory...
	TPhysAddr kernelPd = Mmu::LinearToPhysical((TLinAddr)Mmu::PageDirectory(KKernelOsAsid));
	iKernelPageDirectory = kernelPd;
	((DMemModelProcess*)K::TheKernelProcess)->iPageDir = kernelPd;
	AssignPages(KKernelOsAsid*(KPageDirectorySize>>KPageShift),KPageDirectorySize>>KPageShift,kernelPd);

#ifdef __CPU_ARM64
	// NeoSymbian arm64: the bootstrap built the kernel's root; check it describes this page directory
	A64RootTablePhys = Mmu::LinearToPhysical(KRootTableBase);
	__NK_ASSERT_ALWAYS(A64RootTablePhys != KPhysAddrInvalid);
	for (TInt i=0; i<4; ++i)
		__NK_ASSERT_ALWAYS(A64RootSlot(KKernelOsAsid)[i] == ((TPde)(kernelPd+i*KPageSize) | KA64DescTable));
#endif
	// construct allocator...
	iAllocator = TBitMapAllocator::New(KNumOsAsids,ETrue);
	__NK_ASSERT_ALWAYS(iAllocator);
	iAllocator->Alloc(KKernelOsAsid,1); // kernel page directory already allocated

	TRACEB(("PageDirectoryAllocator::Init2 done"));
	}


void PageDirectoryAllocator::AssignPages(TUint aIndex, TUint aCount, TPhysAddr aPhysAddr)
	{
	__NK_ASSERT_DEBUG(aCount<=KMaxPageInfoUpdatesInOneGo);
	MmuLock::Lock();
	SPageInfo* pi = SPageInfo::FromPhysAddr(aPhysAddr);
	SPageInfo* piEnd = pi+aCount;
	while(pi<piEnd)
		{
		pi->SetPhysAlloc(iPageDirectoryMemory,aIndex);
		++pi;
		++aIndex;
		}
	MmuLock::Unlock();
	}


TInt PageDirectoryAllocator::Alloc(TUint aOsAsid, TPhysAddr& aPageDirectory)
	{
	TRACE(("PageDirectoryAllocator::Alloc(%d)",aOsAsid));

	// get memory for local page directory...
	Mmu& m = TheMmu;
	TUint offset = aOsAsid*KPageDirectorySize;
	TPhysAddr pdPhys;
	RamAllocLock::Lock();
	TInt r = m.AllocContiguousRam(pdPhys, KLocalPdPages, KLocalPdShift-KPageShift, iPageDirectoryMemory->RamAllocFlags());
	if(r==KErrNone)
		{
		AssignPages(offset>>KPageShift,KLocalPdPages,pdPhys);

#ifdef BTRACE_KERNEL_MEMORY
		BTrace4(BTrace::EKernelMemory, BTrace::EKernelMemoryMiscAlloc, KLocalPdPages << KPageShift);
		Epoc::KernelMiscPages += KLocalPdPages;
#endif
		}
	RamAllocLock::Unlock();

	if(r==KErrNone)
		{
		TRACE(("PageDirectoryAllocator::Alloc pdPhys = 0x%08x",pdPhys));

		// map local page directory...
		r = MM::MemoryAddContiguous(iPageDirectoryMemory,MM::BytesToPages(offset),KLocalPdPages,pdPhys);
		if(r!=KErrNone)
			{
			RamAllocLock::Lock();
			m.FreeContiguousRam(pdPhys,KLocalPdPages);

#ifdef BTRACE_KERNEL_MEMORY
			BTrace4(BTrace::EKernelMemory, BTrace::EKernelMemoryMiscFree, KLocalPdPages << KPageShift);
			Epoc::KernelMiscPages -= KLocalPdPages;
#endif
			RamAllocLock::Unlock();
			}
		else
			{
			aPageDirectory = pdPhys;

			TPde* pd = Mmu::PageDirectory(aOsAsid);
			const TUint globalOffset = (KGlobalMemoryBase>>KChunkShift)*sizeof(TPde); // start of global part

			// clear local entries in page directory...
			memclr(pd,globalOffset);
			CacheMaintenance::PdesInitialised((TLinAddr)pd,globalOffset);

			if(KLocalPdSize<(TUint)KPageDirectorySize)
				{
				// map global page directory after local part...
				__NK_ASSERT_DEBUG(KLocalPdSize==globalOffset);
				r = MM::MemoryAddContiguous(iPageDirectoryMemory, MM::BytesToPages(offset+KLocalPdSize), 
						(KPageDirectorySize-KLocalPdSize)/KPageSize, iKernelPageDirectory+KLocalPdSize);
				__NK_ASSERT_DEBUG(r==KErrNone); // can't fail
#ifdef __CPU_ARM64
				A64SetRoot(aOsAsid, pdPhys, iKernelPageDirectory);	// NeoSymbian arm64: this address space's level-1 root
#endif
				MmuLock::Lock(); // need lock because allocator not otherwise atomic
				iAllocator->Alloc(aOsAsid,1);
				MmuLock::Unlock();
				}
			else
				{
				// copy global entries to local page directory...
				TPde* globalPd = Mmu::PageDirectory(KKernelOsAsid);
				MmuLock::Lock(); // need lock because allocator not otherwise atomic, also  to make sure GlobalPdeChanged() only accesses extant PDs
				memcpy((TUint8*)pd+globalOffset,(TUint8*)globalPd+globalOffset,KPageDirectorySize-globalOffset);
				iAllocator->Alloc(aOsAsid,1);
				MmuLock::Unlock();
				CacheMaintenance::PdesInitialised((TLinAddr)((TUint8*)pd+globalOffset),KPageDirectorySize-globalOffset);
				}
			}
		}
	TRACE(("PageDirectoryAllocator::Alloc returns %d",r));
	return r;
	}


void PageDirectoryAllocator::Free(TUint aOsAsid)
	{
	TRACE(("PageDirectoryAllocator::Free(%d)",aOsAsid));

	MmuLock::Lock(); // need lock because allocator not otherwise atomic, also to make sure GlobalPdeChanged() only accesses extant PDs
	iAllocator->Free(aOsAsid, 1);
	MmuLock::Unlock();

#ifdef __CPU_ARM64
	// NeoSymbian arm64: retire the level-1 root before its page directory pages go (the caller has already switched every
	// CPU away from this ASID; DAddressSpace's destructor invalidates its TLB entries afterwards)
	memclr(A64RootSlot(aOsAsid), KRootTableSize);
	__asm__ __volatile__("dsb ishst" ::: "memory");
#endif
	const TUint KPageDirectoryPageCount = KPageDirectorySize>>KPageShift;
	TPhysAddr pages[KPageDirectoryPageCount];
	TUint n = MM::MemoryRemovePages(iPageDirectoryMemory,aOsAsid*KPageDirectoryPageCount,KPageDirectoryPageCount,pages);
	(void)n;
	__NK_ASSERT_DEBUG(n==KPageDirectoryPageCount);

	RamAllocLock::Lock();
	Mmu& m = TheMmu;
	// Page directories are fixed.
	m.FreeRam(pages, KLocalPdPages, EPageFixed);

#ifdef BTRACE_KERNEL_MEMORY
	BTrace4(BTrace::EKernelMemory, BTrace::EKernelMemoryMiscFree, KLocalPdPages << KPageShift);
	Epoc::KernelMiscPages -= KLocalPdPages;
#endif
	RamAllocLock::Unlock();
	}

