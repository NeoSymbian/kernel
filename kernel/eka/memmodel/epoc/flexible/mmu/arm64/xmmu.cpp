// Copyright (c) 1997-2009 Nokia Corporation and/or its subsidiary(-ies).
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
// AArch64 port: marked-delta copy of memmodel/epoc/flexible/mmu/arm/xmmu.cpp for VMSAv8-64 descriptors (mmboot.h, xmmu.h);
// changes are marked "a64 port".
//
// Description:
//

#include "arm_mem.h"
#include "mm.h"
#include "mmu.h"
#include "mpager.h"

#include "cache_maintenance.inl"
#include "execs.h"


#ifdef BROADCAST_TLB_MAINTENANCE
class TTLBIPI : public TGenericIPI
	{
public:
	TTLBIPI();
	static void InvalidateIsr(TGenericIPI*);
	static void WaitAndInvalidateIsr(TGenericIPI*);
	void AddArg(TLinAddr aArg);
public:
	volatile TInt	iFlag;
	TLinAddr		iArg;
	};

TTLBIPI::TTLBIPI()
	:	iFlag(0), iArg(0)
	{
	}

void TTLBIPI::InvalidateIsr(TGenericIPI* aPtr)
	{
	TRACE2(("TLBInv"));
	TTLBIPI& a = *(TTLBIPI*)aPtr;
	TLinAddr arg = a.iArg;
	if (arg==0)
		LocalInvalidateTLB();
	else if (arg<256)
		LocalInvalidateTLBForAsid(arg);
	else
		LocalInvalidateTLBForPage(arg);
	}

void TTLBIPI::WaitAndInvalidateIsr(TGenericIPI* aPtr)
	{
	TRACE2(("TLBWtInv"));
	TTLBIPI& a = *(TTLBIPI*)aPtr;
	while (!a.iFlag)
		{ __chill(); }
	InvalidateIsr(aPtr);
	}

void TTLBIPI::AddArg(TLinAddr aArg)
	{
	iArg = aArg;
	NKern::Lock();
	InvalidateIsr(this);
	QueueAllOther(&InvalidateIsr);
	NKern::Unlock();
	WaitCompletion();
	}

void BroadcastInvalidateTLB(TLinAddr aLinAddrAndAsid)
	{
	TTLBIPI ipi;
	ipi.AddArg(aLinAddrAndAsid);
	}
#endif	// BROADCAST_TLB_MAINTENANCE

//
// Functions for class Mmu
//

/**
Return the physical address of the memory mapped by a Page Table Entry (PTE).

@param aPte			The value contained in the PTE.
@param aPteIndex	The index of the PTE within its page table.
*/
TPhysAddr Mmu::PtePhysAddr(TPte aPte, TUint aPteIndex)
	{
	// a64 port: a valid level-3 page descriptor (bits 1:0 = 11); every PTE holds its own address (no ARMv6 large page)
	(void)aPteIndex;
	if((aPte&KPteTypeMask)==KA64DescTable)
		return (TPhysAddr)(aPte & KPteSmallPageAddrMask);
	return KPhysAddrInvalid;
	}

/**
Return the virtual address of the page table referenced by the given
Page Directory Entry (PDE) \a aPde. If the PDE doesn't refer to a
page table then the null-pointer is returned.

If the page table was not one allocated by the kernel then the
results are unpredictable and may cause a system fault.

@pre #MmuLock held.
*/
TPte* Mmu::PageTableFromPde(TPde aPde)
	{
	if((aPde&KPdeTypeMask)==KA64DescTable)		// a64 port: level-2 table descriptor
		{
		SPageInfo* pi = SPageInfo::FromPhysAddr((TPhysAddr)aPde);
		return (TPte*)(KPageTableBase+(pi->Index()<<KPageShift)+(aPde&(KPageMask&~KPageTableMask)));
		}
	return 0;
	}


/**
Perform the action of #PageTableFromPde but without the possibility of
a system fault caused the page table not being one allocated by the kernel.

@pre #MmuLock held.
*/
TPte* Mmu::SafePageTableFromPde(TPde aPde)
	{
	if((aPde&KPdeTypeMask)==KA64DescTable)		// a64 port: level-2 table descriptor
		{
		SPageInfo* pi = SPageInfo::SafeFromPhysAddr((TPhysAddr)aPde&~KPageMask);
		if(pi)
			return (TPte*)(KPageTableBase+(pi->Index()<<KPageShift)+(aPde&(KPageMask&~KPageTableMask)));
		}
	return 0;
	}


/**
Return the base phsical address of the section table referenced by the given
Page Directory Entry (PDE) \a aPde. If the PDE doesn't refer to a
section then KPhysAddrInvalid is returned.

@pre #MmuLock held.
*/
TPhysAddr Mmu::SectionBaseFromPde(TPde aPde)
	{
	if(PdeMapsSection(aPde))
		return aPde&KPdeSectionAddrMask;
	return KPhysAddrInvalid;
	}


/**
Return a pointer to the Page Table Entry (PTE) which maps the
virtual address \a aAddress in the address space \a aOsAsid.

If no page table exists or it was not one allocated by the kernel
then the results are unpredictable and may cause a system fault.

@pre #MmuLock held.
*/
TPte* Mmu::PtePtrFromLinAddr(TLinAddr aAddress, TInt aOsAsid)
	{
	TPde pde = PageDirectory(aOsAsid)[aAddress>>KChunkShift];
	SPageInfo* pi = SPageInfo::FromPhysAddr(pde);
	TPte* pt = (TPte*)(KPageTableBase+(pi->Index()<<KPageShift)+(pde&(KPageMask&~KPageTableMask)));
	pt += (aAddress>>KPageShift)&(KChunkMask>>KPageShift);
	return pt;
	}


/**
Perform the action of #PtePtrFromLinAddr but without the possibility
of a system fault. If the page table is not present or not one
allocated by the kernel then the null-pointer is returned.

@pre #MmuLock held.
*/
TPte* Mmu::SafePtePtrFromLinAddr(TLinAddr aAddress, TInt aOsAsid)
	{
	TPde pde = PageDirectory(aOsAsid)[aAddress>>KChunkShift];
	TPte* pt = SafePageTableFromPde(pde);
	if(pt)
		pt += (aAddress>>KPageShift)&(KChunkMask>>KPageShift);
	return pt;
	}


/**
Return the physical address for the page table whose virtual
address is \a aPt.

If the page table was not one allocated by the kernel then the
results are unpredictable and may cause a system fault.

@pre #MmuLock held.
*/
TPhysAddr Mmu::PageTablePhysAddr(TPte* aPt)
	{
	__NK_ASSERT_DEBUG(MmuLock::IsHeld() || PageTablesLockIsHeld());

	TInt pdeIndex = ((TLinAddr)aPt)>>KChunkShift;
	TPde pde = PageDirectory(KKernelOsAsid)[pdeIndex];
	__NK_ASSERT_DEBUG((pde&KPdeTypeMask)==KA64DescTable);		// a64 port

	SPageInfo* pi = SPageInfo::FromPhysAddr(pde);
	TPte* pPte = (TPte*)(KPageTableBase+(pi->Index(true)<<KPageShift)+(pde&(KPageMask&~KPageTableMask)));
	TPte pte = pPte[(((TLinAddr)aPt)&KChunkMask)>>KPageShift];
	__NK_ASSERT_DEBUG((pte&KPteTypeMask)==KA64DescTable);		// a64 port

	return (TPhysAddr)(pte&KPteSmallPageAddrMask);			// a64 port: a page table is exactly one page
	}


/**
Perform a page table walk to return the physical address of
the memory mapped at virtual address \a aLinAddr in the
address space \a aOsAsid.

If the page table used was not one allocated by the kernel
then the results are unpredictable and may cause a system fault.

Use of this function should be avoided, use instead Mmu::LinearToPhysical
which contains debug assertions for its preconditions.

@pre #MmuLock held.
*/
TPhysAddr Mmu::UncheckedLinearToPhysical(TLinAddr aLinAddr, TInt aOsAsid)
	{
	TRACE2(("Mmu::UncheckedLinearToPhysical(%08x,%d)",aLinAddr,aOsAsid));
	TInt pdeIndex = aLinAddr>>KChunkShift;
	TPde pde = PageDirectory(aOsAsid)[pdeIndex];
	// a64 port: table (bits 1:0 = 11) or 2 MB block (01); level-3 pages only (no ARMv6 large pages)
	if ((pde&KPdeTypeMask)==KA64DescTable)
		{
		SPageInfo* pi = SPageInfo::FromPhysAddr((TPhysAddr)pde);
		TPte* pPte = (TPte*)(KPageTableBase+(pi->Index(true)<<KPageShift));
		TPte pte = pPte[(aLinAddr&KChunkMask)>>KPageShift];
		if ((pte&KPteTypeMask)==KA64DescTable)
			{
			TPhysAddr pa=((TPhysAddr)pte&KPteSmallPageAddrMask)|(aLinAddr&~KPteSmallPageAddrMask);
			__KTRACE_OPT(KMMU,Kern::Printf("Mapped with small page - returning %08x",pa));
			return pa;
			}
		}
	else if ((pde&KPdeTypeMask)==KA64DescBlock)
		{
		TPhysAddr pa=((TPhysAddr)pde&KPdeSectionAddrMask)|(aLinAddr&~KPdeSectionAddrMask);
		__KTRACE_OPT(KMMU,Kern::Printf("Mapped with section - returning %08x",pa));
		return pa;
		}
	return KPhysAddrInvalid;
	}




void Mmu::Init1()
	{
	TRACEB(("Mmu::Init1"));

	// a64 port: one 4 GB TTBR0 space (TCR_EL1.T0SZ = 32); the local/global split is in the level-1 root instead of
	// ARM's TTBCR.N, see mmboot.h. (ARM: __NK_ASSERT_ALWAYS(TTCR()==1).)
	TUint64 tcr;
	__asm__ __volatile__("mrs %0, tcr_el1" : "=r"(tcr));
	__NK_ASSERT_ALWAYS((tcr & 0x3f) == 32);

	// a64 port: ARMv8 data caches behave as PIPT, so no data colouring. An instruction cache may be VIPT (CTR_EL0.L1Ip
	// = 2); code changes invalidate it with IC IALLUIS, which has no aliasing problem, so no colouring either.
	// (ARM: the v6/v7 CTR/CLIDR/CCSIDR colour probe, ID_MMFR checks and the domain access default.)
	TUint64 ctr;
	__asm__ __volatile__("mrs %0, ctr_el0" : "=r"(ctr));
	TRACEB(("CTR_EL0 = %08x (L1Ip %d)", (TUint32)ctr, (TUint32)(ctr>>14)&3));

	// a64 port (as Phorepoc's x86-64 port): 4 KB supervisor stacks overflow with 64-bit frames
	K::SupervisorThreadStackSize = 0x4000;

#ifdef __SMP__
	TInt i;
	for (i=0; i<KMaxCpus; ++i)
		{
		TSubScheduler& ss = TheSubSchedulers[i];
		TLinAddr a = KIPCAlias + (i<<KChunkShift);
		ss.i_AliasLinAddr = (TAny*)a;
		ss.i_AliasPdePtr = (TAny*)(KPageDirectoryBase + (a>>KChunkShift)*sizeof(TPde));
		}
#endif

	Init1Common();
	}

void Mmu::Init2()
	{
	TRACEB(("Mmu::Init2"));

	Init2Common();
	}

DMemoryObject* ExceptionStacks;

void Mmu::Init2Final()
	{
	TRACEB(("Mmu::Init2Final"));

	Init2FinalCommon();

	// initialise memory object for exception stacks...
	TMappingCreateFlags mapFlags = (TMappingCreateFlags)(EMappingCreateFixedVirtual|EMappingCreateReserveAllResources);
	TMemoryAttributes memAttr = EMemoryAttributeStandard;
	TUint size = 4*2*KPageSize; // 4 exception stacks each of one guard page and one mapped page
	size |= 1; // lower bit of size is set if region to be claimed contains gaps
	TInt r = MM::InitFixedKernelMemory(ExceptionStacks, KExcptStacksLinearBase, KExcptStacksLinearEnd, size, EMemoryObjectUnpaged, EMemoryCreateNoWipe, memAttr, mapFlags);
	__NK_ASSERT_ALWAYS(r==KErrNone);
	}


/**
Return the page directory entry (PDE) value to use for when mapping page tables intended
to map memory with the given attributes.
The returned value has the physical address component being zero, so a page table's physical
address can be simply ORed in.
*/
TPde Mmu::BlankPde(TMemoryAttributes aAttributes)
	{
	// a64 port: a level-2 table descriptor (no ARM ECC bit)
	(void)aAttributes;
	TPde pde = KA64DescTable;
	TRACE2(("Mmu::BlankPde(%x) returns 0x%x",aAttributes,(TUint32)pde));
	return pde;
	}

/**
Return the page directory entry (PDE) value to use for when creating a section mapping for memory
with the given attributes and #TPteType.
The returned value has the physical address component being zero, so the section's physical address
can be simply ORed in.
*/
TPde Mmu::BlankSectionPde(TMemoryAttributes aAttributes, TUint aPteType)
	{
	// reuse existing functions rather than duplicating the logic
	TPde pde = BlankPde(aAttributes);
	TPte pte = BlankPte(aAttributes, aPteType);
	return PageToSectionEntry(pte, pde);
	}


/**
Return the page table entry (PTE) to use when mapping memory pages
with the given attributes and #TPteType.
This value has the physical address component being zero, so a page's physical
address can be simply ORed in.
*/

TPte Mmu::BlankPte(TMemoryAttributes aAttributes, TUint aPteType)
	{
	TUint attr = CanonicalMemoryAttributes(aAttributes);

	// a64 port: a VMSAv8 level-3 page descriptor. The memory type (TMemoryType 0-7) is the MAIR_EL1 attribute index
	// (KA64MairValue in mmboot.h: the bootstrap programs MAIR_EL1 with it), as ARM's TEX-remap path used the type as
	// its PRRR/NMRR index. The access flag is always set (no AF faults).
	TUint type = attr&EMemoryAttributeTypeMask;
	TPte pte = KA64DescTable | KA64AF | ((TPte)type<<KA64AttrIndxShift);
	if(type>=EMemAttNormalUncached)
		pte |= KA64SHInner;						// normal memory: inner shareable (SMP coherence); device: SH ignored
	if(aPteType&EPteTypeUserAccess)
		pte |= KA64AP1;							// AP[1] = EL0 access
	if((aPteType&EPteTypeWritable)==false)
		pte |= KA64AP2;							// AP[2] = read-only
	if((aPteType&EPteTypeGlobal)==false)
		pte |= KA64nG;							// tagged with the ASID
	if((aPteType&EPteTypeExecutable)==false || (aPteType&EPteTypeUserAccess)==false)
		pte |= KA64UXN;							// EL0 may execute only user executable pages
	// the kernel never executes non-executable pages or a process's own user pages; global user code (the ROM,
	// global code chunks) stays executable at EL1 because the kernel's own code is in the ROM, as on ARM
	if((aPteType&EPteTypeExecutable)==false || ((aPteType&EPteTypeUserAccess) && (aPteType&EPteTypeGlobal)==false))
		pte |= KA64PXN;

	TRACE2(("Mmu::BlankPte(%x,%x) returns 0x%x%08x",aAttributes,aPteType,(TUint32)(pte>>32),(TUint32)pte));
	return pte;
	}

/**
Calculate PDE and PTE which represent a page table mapping for an existing
section mapping.

@param[in] aPde The PDE for the existing section mapping.
@param[out] aPde A PDE for a page table mapping, with physical address == 0.

@return The PTE value for the first entry in the page table.
*/
TPte Mmu::SectionToPageEntry(TPde& aPde)
	{
	// a64 port: block and page descriptors carry their attributes in the same bits; only the type bits (01 -> 11) and
	// the address change. The new PDE is a plain table descriptor (no ARM IMP/domain bits to keep).
	TPde pde = aPde;
	TPte pte = (pde & ~(KA64OutAddrMask|KA64DescTypeMask|KA64Contiguous)) | KA64DescTable;
	aPde = KA64DescTable;
	return pte;
	}

/**
Calculate a PDE entry which represents a section mapping for an existing
page table mapping.

@pre The existing page table contains mappings for a chunk sized and
	 aligned contiguous region.

@param aPte A PTE from the existing page table.
@param aPde The existing PDE for the page table mappings.
			(Physical address portion is ignored.)

@return A PDE entry value for a section mapping.
*/
TPde Mmu::PageToSectionEntry(TPte aPte, TPde aPde)
	{
	// a64 port: keep the page's attributes, take its address aligned to the 2 MB block, block type (01)
	(void)aPde;
	TPde pde = (aPte & ~(KA64OutAddrMask|KA64DescTypeMask|KA64Contiguous)) | (aPte & KPdeSectionAddrMask) | KA64DescBlock;
	return pde;
	}

/**
Tranform the specified memory attributes into the canonical form relevant to
the platform the code is running on. This applies defaults and overrides to
the attributes to return what should be used with the MMU.
*/
TMemoryAttributes Mmu::CanonicalMemoryAttributes(TMemoryAttributes aAttr)
	{
	TUint attr = aAttr;
	if(attr&EMemoryAttributeDefaultShareable)
		{
		// sharing not specified, use default...
#if defined	(__CPU_USE_SHARED_MEMORY)
		attr |= EMemoryAttributeShareable;
#else
		attr &= ~EMemoryAttributeShareable;
#endif
		}

#if defined(FAULTY_NONSHARED_DEVICE_MEMORY)
	if((attr&(EMemoryAttributeShareable|EMemoryAttributeTypeMask))==EMemoryAttributeDevice)
		{
		// make unshared device memory into shared strongly ordered memory...
		attr ^= EMemoryAttributeShareable;
		attr ^= EMemoryAttributeDevice^EMemoryAttributeStronglyOrdered;
		}
#endif

#if	defined(__SMP__) || defined(__CPU_FORCE_SHARED_MEMORY_IF_CACHED)
	TMemoryType type = (TMemoryType)(attr&KMemoryTypeMask);
	if(CacheMaintenance::IsCached(type))
		{
		// force cached memory to be shared memory on SMP systems...
		attr |= EMemoryAttributeShareable;
		}
#endif

	return (TMemoryAttributes)(attr&EMemoryAttributeMask);
	}

/**
Method called to initialise RAM pages when they are allocated for a new use.
This performs any cache synchronisation required to remove old entries
and also wipes the contents of the memory (if requested via \a aFlags).

@param aPageList	Pointer to a list of physical addresses for the RAM pages,
					or, if the least significant bit of this value is set, then
					the rest of the value is the physical address of a contiguous
					region of RAM pages being allocated.

@param aCount		The number of pages.

@param aFlags		A set of flag values from #TRamAllocFlags which indicate
					the memory type the pages will be used for and whether
					the contents should be wiped.

@param aReallocate	True, if the RAM pages have already been previously allocated
					and are being reinitilised e.g. by DMemoryManager::ReAllocDecommitted.
					False, to indicate that these pages have been newly allocated (are in
					the SPageInfo::EUnused state.)

@pre #RamAllocLock held.
*/
void Mmu::PagesAllocated(TPhysAddr* aPageList, TUint aCount, TRamAllocFlags aFlags, TBool aReallocate)
	{
	TRACE2(("Mmu::PagesAllocated(0x%08x,%d,0x%x,%d)",aPageList, aCount, aFlags, (bool)aReallocate));
	__NK_ASSERT_DEBUG(RamAllocLock::IsHeld());

	TBool wipe = !(aFlags&EAllocNoWipe); // do we need to wipe page contents?
	TUint8 wipeByte = (aFlags&EAllocUseCustomWipeByte) ? (aFlags>>EAllocWipeByteShift)&0xff : 0x03; // value to wipe memory with

	// process each page in turn...
	while(aCount--)
		{
		// get physical address of next page...
		TPhysAddr pagePhys;
		if((TPhysAddr)aPageList&1)
			{
			// aPageList is actually the physical address to use...
			pagePhys = (TPhysAddr)aPageList&~1;
			*(TPhysAddr*)&aPageList += KPageSize;
			}
		else
			pagePhys = *aPageList++;
		__NK_ASSERT_DEBUG((pagePhys&KPageMask)==0);

		// get info about page...
		SPageInfo* pi = SPageInfo::FromPhysAddr(pagePhys);
		TMemoryType oldType = (TMemoryType)(pi->Flags(true)&KMemoryTypeMask);
		TBool oldTypeNormal = CacheMaintenance::IsNormal(oldType);

		TRACE2(("Mmu::PagesAllocated page=0x%08x, oldType=%d, wipe=%d, colour=%d",pagePhys,oldType,wipe,pi->Index(true)&KPageColourMask));
		if(wipe || oldTypeNormal)
			{
			// work out temporary mapping values...
			TUint colour = pi->Index(true)&KPageColourMask;
			TLinAddr tempLinAddr = iTempMap[0].iLinAddr+colour*KPageSize;
			TPte* tempPte = iTempMap[0].iPtePtr+colour;

			if(oldTypeNormal)
				{
				// cache maintenance required. Prepare temporary mapping.
				*tempPte = pagePhys | iTempPteCacheMaintenance;
				CacheMaintenance::SinglePteUpdated((TLinAddr)tempPte);
				InvalidateTLBForPage(tempLinAddr|KKernelOsAsid);

				// will hold additional arguments in CacheMaintenance::PageToReuse call
				TInt pageToReuseMask = 0;
				
				// check if old and new mappings are the same. (Wiping needs temporary
				// mapping which may not be the same as the old and new mapping.)
				TMemoryType newType = (TMemoryType)(aFlags&KMemoryTypeMask); // memory type that pages will be used for
				if (!wipe && (newType ==oldType))
					pageToReuseMask |= CacheMaintenance::EOldAndNewMappingMatch;

				MmuLock::Lock();

				// decide wether to trigger maintenance of entire cache(s).
				if(CacheMaintenance::IsPageToReuseThresholdReached(iCacheInvalidatePageCount))
					{
					// enough pages to make it worth triggering maintenance of entire cache(s)
					pageToReuseMask |= CacheMaintenance::EThresholdReached;
					++iCacheInvalidateCounter;
					iCacheInvalidatePageCount = 0; // all pages will be partially synced 
					}
				
				if(CacheMaintenance::IsCached(oldType) && !aReallocate)
					{
					if(pi->CacheInvalidateCounter()==(TUint32)iCacheInvalidateCounter)
						{
						// one less unused page in the L1 cache...
						__NK_ASSERT_DEBUG(iCacheInvalidatePageCount);
						--iCacheInvalidatePageCount;
						}
					else
						{
						// our page has been already partially maintained in cache
						// by a previous PageToReuse call.
						pageToReuseMask |= CacheMaintenance::EPageHasBeenPartiallySynced;
						}
					}
				
				MmuLock::Unlock();
				
				TBool pageRemovedFromCache = CacheMaintenance::PageToReuse(tempLinAddr, oldType, pagePhys, pageToReuseMask);
				if(pageRemovedFromCache && !aReallocate)
					pi->SetUncached();
				}

			if(wipe)
				{
				//We need uncached normal temporary mapping to wipe. Change it if necessary.
				//or , in case of !oldTypeNormal it is not configured yet.
				if (!oldTypeNormal || (CacheMaintenance::TemporaryMapping()!=EMemAttNormalUncached))
					{
					*tempPte = pagePhys | iTempPteUncached;
					CacheMaintenance::SinglePteUpdated((TLinAddr)tempPte);
					InvalidateTLBForPage(tempLinAddr|KKernelOsAsid);
					}
				// wipe contents of memory...
				memset((TAny*)tempLinAddr, wipeByte, KPageSize);
				CacheMaintenance::PageToReuse(tempLinAddr, EMemAttNormalUncached, pagePhys);
				}

			// invalidate temporary mapping...
			*tempPte = KPteUnallocatedEntry;
			CacheMaintenance::SinglePteUpdated((TLinAddr)tempPte);
			InvalidateTLBForPage(tempLinAddr|KKernelOsAsid);
			}

		// indicate page has been allocated...
		if(!aReallocate)
			pi->SetAllocated();

		// loop round for next page...
		} // end of while(aCount--)
	}


/**
Method called to update the state of a RAM page when it is freed.
This sets the page state to SPageInfo::EUnused.

@param aPageInfo	The page information structure for the RAM page.

@pre #MmuLock held.
*/
void Mmu::PageFreed(SPageInfo* aPageInfo)
	{
	__NK_ASSERT_DEBUG(MmuLock::IsHeld());

	if(aPageInfo->Type()==SPageInfo::EUnused)
		return;

	aPageInfo->SetUnused();

	TMemoryType type = (TMemoryType)(aPageInfo->Flags()&KMemoryTypeMask);
	if(CacheMaintenance::IsCached(type))
		{
		// another unused page with L1 cache entries...
		aPageInfo->SetCacheInvalidateCounter(iCacheInvalidateCounter);
		++iCacheInvalidatePageCount;
		}

	TRACE2(("Mmu::PageFreed page=0x%08x type=%d colour=%d",aPageInfo->PhysAddr(),aPageInfo->Flags()&KMemoryTypeMask,aPageInfo->Index()&KPageColourMask));
	}

/**
Remove the contents of RAM pages from any memory caches.

@param aPages		Pointer to a list of physical addresses for the RAM pages,
					or, if the least significant bit of this value is set, then
					the rest of the value is the physical address of a contiguous
					region of RAM pages.

@param aCount		The number of pages.

@param aAttributes	The memory attributes of the pages.

@param aColour 		The colour for the first page;
					consecutive pages will be coloured accordingly.
					Only #KPageColourShift least significant bits are used,
					therefore an index into a memory object's memory can be
					used for this value.
*/
void Mmu::CleanAndInvalidatePages(TPhysAddr* aPages, TUint aCount, TMemoryAttributes aAttributes, TUint aColour)
	{
	TMemoryType type = (TMemoryType)(aAttributes&EMemoryAttributeTypeMask);

	if(!CacheMaintenance::IsNormal(type))
		{
		TRACE2(("Mmu::CleanAndInvalidatePages - nothing to do"));
		return;
		}
	
	RamAllocLock::Lock();

	while(aCount--)
		{
		TPhysAddr pagePhys = *aPages++;
		TRACE2(("Mmu::CleanAndInvalidatePages 0x%08x",pagePhys));

		// work out temporary mapping values...
		aColour &= KPageColourMask;
		TLinAddr tempLinAddr = iTempMap[0].iLinAddr+aColour*KPageSize;
		TPte* tempPte = iTempMap[0].iPtePtr+aColour;
		++aColour;

		// temporarily map page...
		*tempPte = pagePhys | iTempPteCacheMaintenance;
		CacheMaintenance::SinglePteUpdated((TLinAddr)tempPte);
		InvalidateTLBForPage(tempLinAddr|KKernelOsAsid);

		// preserve memory content and remove from cache...
		CacheMaintenance::PageToPreserveAndReuse(tempLinAddr, type, pagePhys);

		// invalidate temporary mapping...
		*tempPte = KPteUnallocatedEntry;
		CacheMaintenance::SinglePteUpdated((TLinAddr)tempPte);
		InvalidateTLBForPage(tempLinAddr|KKernelOsAsid);

		RamAllocLock::Flash();
		}
	RamAllocLock::Unlock();
	}


// a64 port: ARM toggles domain 2 (the IPC alias region) in DACR around an alias; VMSAv8 has no domains. The alias PDE
// itself is made EL1-only below (APTable[0]), so there is nothing to lock or unlock.
static inline void UnlockIPCAlias() {}
static inline void LockIPCAlias() {}


TInt DMemModelThread::Alias(TLinAddr aAddr, DMemModelProcess* aProcess, TInt aSize, TLinAddr& aAliasAddr, TUint& aAliasSize)
//
// Set up an alias mapping starting at address aAddr in specified process.
// Note: Alias is removed if an exception is trapped by DThread::IpcExcHandler.
//
	{
	TRACE2(("Thread %O Alias %08x+%x Process %O",this,aAddr,aSize,aProcess));
	__NK_ASSERT_DEBUG(this==TheCurrentThread); // many bad things can happen if false
	// If there is an existing alias it should be on the same process otherwise
	// the os asid reference may be leaked.
	__NK_ASSERT_DEBUG(!iAliasLinAddr || aProcess == iAliasProcess);

	if(TUint(aAddr^KIPCAlias)<TUint(KIPCAliasAreaSize))
		return KErrBadDescriptor; // prevent access to alias region

#ifdef _DEBUG
	if (KDebugNum(KFORCEKUPAGEFAULTS))
		{
		TInt r = ThePager.FlushRegion(aProcess, aAddr, aSize);
		if (r != KErrNone)
			return r;
		}
#endif
	
	// Grab the mmu lock before opening a reference on os asid so that this thread 
	// is in an implicit critical section and therefore can't leak the reference by
	// dying before iAliasLinAddr is set.
	MmuLock::Lock();

	TInt osAsid;
	if (!iAliasLinAddr)
		{// There isn't any existing alias.
		// Open a reference on the aProcess's os asid so that it is not freed and/or reused
		// while we are aliasing an address belonging to it.
		osAsid = aProcess->TryOpenOsAsid();
		if (osAsid < 0)
			{// Couldn't open os asid so aProcess is no longer running.
			MmuLock::Unlock();
			return KErrBadDescriptor;
			}
		}
	else
		{
		// Just read the os asid of the process being aliased we already have a reference on it.
		osAsid = aProcess->OsAsid();
		}

	// Now we have the os asid check access to kernel memory.
	if(aAddr >= KUserMemoryLimit && osAsid != (TUint)KKernelOsAsid)
		{
		if (!iAliasLinAddr)
			{// Close the new reference as RemoveAlias won't do as iAliasLinAddr is not set.
			aProcess->AsyncCloseOsAsid();	// Asynchronous close as this method should be quick.
			}
		MmuLock::Unlock();
		return KErrBadDescriptor; // prevent access to supervisor only memory
		}

	// Now we know all accesses to global memory are safe so check if aAddr is global.
	if(aAddr >= KGlobalMemoryBase)
		{
		// address is in global section, don't bother aliasing it...
		if (!iAliasLinAddr)
			{// Close the new reference as not required.
			aProcess->AsyncCloseOsAsid(); // Asynchronous close as this method should be quick.
			MmuLock::Unlock();
			}
		else
			{// Remove the existing alias as it is not required.
			DoRemoveAlias(iAliasLinAddr);	// Releases mmulock.
			}
		aAliasAddr = aAddr;
		TInt maxSize = KChunkSize-(aAddr&KChunkMask);
		aAliasSize = aSize<maxSize ? aSize : maxSize;
		TRACE2(("DMemModelThread::Alias() abandoned as memory is globally mapped"));
		return KErrNone;
		}

	TPde* pd = Mmu::PageDirectory(osAsid);
	TInt pdeIndex = aAddr>>KChunkShift;
	TPde pde = pd[pdeIndex];
	// a64 port: instead of moving the PDE into domain 2, deny EL0 everything below it -- the PDE points at the target
	// process's page table, whose PTEs grant EL0 access (APTable exists only in table descriptors; a 2 MB block
	// loses its own AP[1] instead)
	if((pde&KPdeTypeMask)==KA64DescTable)
		pde |= KA64APTableNoEL0;
	else
		pde &= ~KA64AP1;
	// Get os asid, this is the current thread's process so no need for reference.
	TUint32 local_asid = ((DMemModelProcess*)iOwningProcess)->OsAsid();
#ifdef __SMP__
	TLinAddr aliasAddr;
#else
	TLinAddr aliasAddr = KIPCAlias+(aAddr&(KChunkMask & ~KPageMask));
#endif
	if(pde==iAliasPde && iAliasLinAddr)
		{
		// pde already aliased, so just update linear address...
#ifdef __SMP__
		__NK_ASSERT_DEBUG(iCpuRestoreCookie>=0);
		aliasAddr = iAliasLinAddr & ~KChunkMask;
		aliasAddr |= (aAddr & (KChunkMask & ~KPageMask));
#endif
		iAliasLinAddr = aliasAddr;
		}
	else
		{
		// alias PDE changed...
		if(!iAliasLinAddr)
			{
			UnlockIPCAlias();
			TheMmu.iAliasList.Add(&iAliasLink); // add to list if not already aliased
#ifdef __SMP__
			__NK_ASSERT_DEBUG(iCpuRestoreCookie==-1);
			iCpuRestoreCookie = NKern::FreezeCpu();	// temporarily lock current thread to this processor
#endif
			}
		iAliasPde = pde;
		iAliasProcess = aProcess;
#ifdef __SMP__
		TSubScheduler& ss = SubScheduler();		// OK since we are locked to this CPU
		aliasAddr = TLinAddr(ss.i_AliasLinAddr) + (aAddr & (KChunkMask & ~KPageMask));
		iAliasPdePtr = (TPde*)(TLinAddr(ss.i_AliasPdePtr) + (local_asid << KPageDirectoryShift));
#endif
		iAliasLinAddr = aliasAddr;
		*iAliasPdePtr = pde;
		SinglePdeUpdated(iAliasPdePtr);
		}

	TRACE2(("DMemModelThread::Alias() PDEntry=%x, iAliasLinAddr=%x",pde, aliasAddr));
	LocalInvalidateTLBForPage(aliasAddr | local_asid);
	TInt offset = aAddr&KPageMask;
	aAliasAddr = aliasAddr | offset;
	TInt maxSize = KPageSize - offset;
	aAliasSize = aSize<maxSize ? aSize : maxSize;
	iAliasTarget = aAddr & ~KPageMask;

	MmuLock::Unlock();

	return KErrNone;
	}


void DMemModelThread::RemoveAlias()
//
// Remove alias mapping (if present)
//
	{
	TRACE2(("Thread %O RemoveAlias", this));
	__NK_ASSERT_DEBUG(this==TheCurrentThread); // many bad things can happen if false

	TLinAddr addr = iAliasLinAddr;
	if(addr)
		{
		MmuLock::Lock();

		DoRemoveAlias(addr);	// Unlocks mmulock.
		}
	}


/**
Remove the alias mapping.

@pre Mmulock held
@post MmuLock released.
*/
void DMemModelThread::DoRemoveAlias(TLinAddr aAddr)
	{
	LockIPCAlias();
	iAliasLinAddr = 0;
	iAliasPde = KPdeUnallocatedEntry;
	*iAliasPdePtr = KPdeUnallocatedEntry;
	SinglePdeUpdated(iAliasPdePtr);
	__NK_ASSERT_DEBUG((aAddr&KPageMask)==0);
	// Invalidate the tlb using os asid, no need to open a reference as this
	// is the current thread's process os asid.
	LocalInvalidateTLBForPage(aAddr | ((DMemModelProcess*)iOwningProcess)->OsAsid());
	iAliasLink.Deque();
#ifdef __SMP__
	__NK_ASSERT_DEBUG(iCpuRestoreCookie>=0);
	NKern::EndFreezeCpu(iCpuRestoreCookie);
	iCpuRestoreCookie = -1;
#endif

	// Must close the os asid while holding MmuLock so we are in an implicit critical section.
	iAliasProcess->AsyncCloseOsAsid(); // Asynchronous close as this method should be quick.
	MmuLock::Unlock();
	}


TInt M::DemandPagingFault(TAny* aExceptionInfo)
	{
	TArmExcInfo& exc=*(TArmExcInfo*)aExceptionInfo;

	// permissions required by faulting memory access...
	TUint accessPermissions = EUser; // we only allow paging of user memory

	// get faulting address...
	TLinAddr faultAddress = exc.iFaultAddress;
	if(exc.iExcCode==EArmExceptionPrefetchAbort)
		{
		// fault trying to read code to execute...
		accessPermissions |= EExecute;
		}
	else if(exc.iExcCode!=EArmExceptionDataAbort)
		return KErrUnknown; // not prefetch or data abort

	// a64 port: the exception shim reports an AArch64 instruction/data abort as ARM's prefetch/data abort with
	// ESR_EL1 in iFaultStatus and FAR_EL1 in iFaultAddress. Fault status code (ESR[5:0]): translation fault
	// 0b0001xx or permission fault 0b0011xx, any level.
	TUint32 fsc = exc.iFaultStatus & 0x3f;
	if((fsc&0x3c) != 0x04 && (fsc&0x3c) != 0x0c)
		return KErrUnknown; // not translation or permission fault

	// check access type... (data abort ESR.WnR, bit 6)
	if(exc.iExcCode==EArmExceptionDataAbort && (exc.iFaultStatus&(1<<6)))
		accessPermissions |= EReadWrite;

	// let TheMmu handle the fault...
	return TheMmu.HandlePageFault(exc.iR15, faultAddress, accessPermissions, aExceptionInfo);
	}


