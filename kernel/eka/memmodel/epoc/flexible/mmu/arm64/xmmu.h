// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// SPDX-License-Identifier: EPL-2.0
//
// xmmu.h -- shadow of memmodel/epoc/flexible/mmu/arm/xmmu.h for the AArch64 kernel (mmu/arm64 first on -I;
// the generic memory model includes "xmmu.h" from mmu.h). Same interface as the ARM header -- barrier helpers, TLB
// maintenance, page-directory access, PTE permission edits, the TPdeType/TPteType enums -- implemented for VMSAv8-64
// descriptors (layout and geometry: mmboot.h).
//
// TLB maintenance: VMSAv8 broadcasts the *IS forms to every core in the Inner Shareable domain in hardware, so the
// ARM11MP software broadcast (BroadcastInvalidateTLB) isn't needed. EKA2 passes "page address | ASID" as one
// TLinAddr (pages are 4 KB aligned, ASIDs < 256); TLBI takes VA[55:12] in bits 43:0 and the ASID in bits 63:48.
// A global (nG=0) entry matches any ASID, so VAE1 also removes kernel/global mappings.
#ifndef __A64_XMMU_H__
#define __A64_XMMU_H__

#define	__VOLATILE__	volatile

#include <kernel/cache_maintenance.h>

FORCE_INLINE void __arm_dmb()	{ __asm__ __volatile__("dmb ish" ::: "memory"); }
FORCE_INLINE void __arm_dsb()	{ __asm__ __volatile__("dsb ish" ::: "memory"); }
extern "C" void __e32_instruction_barrier();
FORCE_INLINE void __arm_isb()	{ __asm__ __volatile__("isb" ::: "memory"); }
// branch predictor maintenance after page table changes is not required on ARMv8 (the TLBI + DSB + ISB suffice)
FORCE_INLINE void __arm_bpiall()	{}
#ifdef __SMP__
FORCE_INLINE void __arm_bpiallis()	{}
#endif

/**
Make a single change to the page directory visible to the hardware table walker.
*/
FORCE_INLINE void SinglePdeUpdated(TPde* aPde)
	{
	CacheMaintenance::SinglePdeUpdated((TLinAddr)aPde);
	}

FORCE_INLINE TUint64 A64TlbiOperand(TLinAddr aLinAddrAndAsid)
	{
	return ((TUint64)(aLinAddrAndAsid >> KPageShift)) | ((TUint64)(aLinAddrAndAsid & KMmuAsidMask) << 48);
	}

/** Invalidate the TLB entries for one page (VA | ASID) on this CPU only. */
FORCE_INLINE void LocalInvalidateTLBForPage(TLinAddr aLinAddrAndAsid)
	{
	__asm__ __volatile__("dsb ishst\n\ttlbi vae1, %0\n\tdsb nsh\n\tisb" :: "r"(A64TlbiOperand(aLinAddrAndAsid)) : "memory");
	}

/** Invalidate the TLB entries for one page (VA | ASID) on all CPUs. */
FORCE_INLINE void InvalidateTLBForPage(TLinAddr aLinAddrAndAsid)
	{
	__asm__ __volatile__("dsb ishst\n\ttlbi vae1is, %0\n\tdsb ish\n\tisb" :: "r"(A64TlbiOperand(aLinAddrAndAsid)) : "memory");
	}

/** Invalidate the entire TLB on this CPU only. */
FORCE_INLINE void LocalInvalidateTLB()
	{
	__asm__ __volatile__("dsb ishst\n\ttlbi vmalle1\n\tdsb nsh\n\tisb" ::: "memory");
	}

/** Invalidate the entire TLB on all CPUs. */
FORCE_INLINE void InvalidateTLB()
	{
	__asm__ __volatile__("dsb ishst\n\ttlbi vmalle1is\n\tdsb ish\n\tisb" ::: "memory");
	}

__ASSERT_COMPILE(KKernelOsAsid==0); // InvalidateTLBForAsid assumes this

/** Invalidate the TLB entries of one address space on this CPU only; the kernel's (ASID 0) means everything. */
FORCE_INLINE void LocalInvalidateTLBForAsid(TUint aAsid)
	{
	if (aAsid == KKernelOsAsid)
		LocalInvalidateTLB();
	else
		__asm__ __volatile__("dsb ishst\n\ttlbi aside1, %0\n\tdsb nsh\n\tisb" :: "r"((TUint64)aAsid << 48) : "memory");
	}

/** Invalidate the TLB entries of one address space on all CPUs; the kernel's (ASID 0) means everything. */
FORCE_INLINE void InvalidateTLBForAsid(TUint aAsid)
	{
	if (aAsid == KKernelOsAsid)
		InvalidateTLB();
	else
		__asm__ __volatile__("dsb ishst\n\ttlbi aside1is, %0\n\tdsb ish\n\tisb" :: "r"((TUint64)aAsid << 48) : "memory");
	}


/**
Return the virtual address of the page directory used for address space \a aOsAsid. As on ARM, the global part
(level-2 tables 2-3) is mapped right after each address space's own part (tables 0-1), so the result can be indexed
by any virtual address.
*/
FORCE_INLINE TPde* Mmu::PageDirectory(TInt aOsAsid)
	{
	return (TPde*)(KPageDirectoryBase+(aOsAsid<<KPageDirectoryShift));
	}

/**
Return the virtual address of the PDE mapping \a aAddress in address space \a aOsAsid.
*/
FORCE_INLINE TPde* Mmu::PageDirectoryEntry(TInt aOsAsid, TLinAddr aAddress)
	{
	return PageDirectory(aOsAsid) + (aAddress>>KChunkShift);
	}

/**
Return the physical address mapped by a section (2 MB block) PDE, else KPhysAddrInvalid.
*/
FORCE_INLINE TPhysAddr Mmu::PdePhysAddr(TPde aPde)
	{
	if((aPde&KPdeTypeMask)==KA64DescBlock)
		return (TPhysAddr)(aPde&KPdeSectionAddrMask);
	return KPhysAddrInvalid;
	}


/*
Access permissions in a PTE (AP[2:1]):
	AP2 AP1		EL0		write
	0	0		n		y
	0	1		y		y
	1	0		n		n
	1	1		y		n
"No access" is an invalid descriptor (bit 0 clear) whose other bits are kept, so MakePteAccessible can restore it.
*/

/**
Modify a PTE so it restricts access: read-only if \a aReadOnly, else no access.
*/
FORCE_INLINE TPte Mmu::MakePteInaccessible(TPte aPte, TBool aReadOnly)
	{
	__NK_ASSERT_DEBUG(!(aPte&KA64Contiguous));
	if(aPte&KPtePresentMask)
		{
		if(aReadOnly)
			aPte |= KA64AP2;			// make read only
		else
			aPte &= ~KPtePresentMask;	// make inaccessible
		}
	return aPte;
	}

/**
Modify a PTE so it allows more access: read/write if \a aWrite, else read-only.
*/
FORCE_INLINE TPte Mmu::MakePteAccessible(TPte aPte, TBool aWrite)
	{
	__NK_ASSERT_DEBUG(!(aPte&KA64Contiguous));
	if((aPte&KPtePresentMask)==0)
		{
		aPte |= KA64DescTable;			// valid page descriptor again (bits 1:0 = 11)
		aPte |= KA64AP2;				// make read only
		}
	if(aWrite)
		aPte &= ~KA64AP2;				// make writable
	return aPte;
	}

/**
Return true if a present PTE only allows read access.
*/
FORCE_INLINE TBool Mmu::IsPteReadOnly(TPte aPte)
	{
	__NK_ASSERT_DEBUG(aPte&KPtePresentMask); // read-only state is ambiguous if pte not present
	return (aPte&KA64AP2)!=0;
	}

/**
Return true if a PTE doesn't allow any access.
*/
FORCE_INLINE TBool Mmu::IsPteInaccessible(TPte aPte)
	{
	return !(aPte&KPtePresentMask);
	}

/**
Return true if \a aNewPte allows greater access (read/write, read-only, none) than \a aOldPte.
*/
FORCE_INLINE TBool Mmu::IsPteMoreAccessible(TPte aNewPte, TPte aOldPte)
	{
	if(aNewPte&aOldPte&KPtePresentMask)				// if ptes both present
		return ((aOldPte&~aNewPte)&KA64AP2)!=0;		//   check for more writable
	else											// else
		return (aNewPte&KPtePresentMask)!=0;		//   check for new pte being present
	}


/**
Memory mapping differences governed by PDEs: memory differing in TPdeType can't share a page table.
*/
enum TPdeType
	{
	/** Legacy ARM attribute, kept so DMemoryMapping::PdeType() works unchanged. */
	EPdeTypeECC				= 1<<0,

	/** Total number of combinations of #TPdeType values. */
	ENumPdeTypes			= 2
	};

/**
Memory mapping differences governed by PTEs.
*/
enum TPteType
	{
	/** PTE grants user mode access to memory. */
	EPteTypeUserAccess		= EUser,

	/** PTE grants write access to memory. */
	EPteTypeWritable		= EReadWrite,

	/** PTE grants execute access to memory. */
	EPteTypeExecutable		= EExecute,

	/** PTE is global (nG=0): valid for all ASIDs, for mappings at or above KGlobalMemoryBase. */
	EPteTypeGlobal			= 1<<3,

	/** Total number of combinations of #TPteType values. */
	ENumPteTypes			= 16
	};

__ASSERT_COMPILE(EPteTypeUserAccess==(1<<0));
__ASSERT_COMPILE(EPteTypeWritable==(1<<1));
__ASSERT_COMPILE(EPteTypeExecutable==(1<<2));


#define MMU_SUPPORTS_EXECUTE_NEVER


/**
Return the #TPdeType for memory with the given attributes.
*/
FORCE_INLINE TUint Mmu::PdeType(TMemoryAttributes aAttributes)
	{
	return aAttributes&EMemoryAttributeUseECC ? EPdeTypeECC : 0;
	}

/**
Return the #TPteType for mappings with the given permissions and global attribute.
*/
FORCE_INLINE TUint Mmu::PteType(TMappingPermissions aPermissions, TBool aGlobal)
	{
	__NK_ASSERT_DEBUG(aPermissions&EUser || aGlobal); // can't have supervisor local memory

	TUint pteType =	(aPermissions&(EUser|EReadWrite|EExecute));
	if(aGlobal)
		pteType |= EPteTypeGlobal;

	__NK_ASSERT_DEBUG(pteType<ENumPteTypes);

	return pteType;
	}

/**
Test whether an access with \a aAccessPermissions is allowed by a mapping of type \a aPteType.
*/
FORCE_INLINE TBool Mmu::CheckPteTypePermissions(TUint aPteType, TUint aAccessPermissions)
	{
	aAccessPermissions &= EUser|EReadWrite|EExecute;
	return (aPteType&aAccessPermissions)==aAccessPermissions;
	}

/**
Extract the #TMappingPermissions of a #TPteType.
*/
FORCE_INLINE TMappingPermissions Mmu::PermissionsFromPteType(TUint aPteType)
	{
	return (TMappingPermissions)(aPteType&(EPteTypeUserAccess|EPteTypeWritable|EPteTypeExecutable));
	}

extern void UserWriteFault(TLinAddr aAddr);
extern void UserReadFault(TLinAddr aAddr);


FORCE_INLINE void inline_DisableAllInterrupts()	{ __asm__ __volatile__("msr daifset, #3" ::: "memory"); }
FORCE_INLINE void inline_EnableAllInterrupts()	{ __asm__ __volatile__("msr daifclr, #3\n\tisb" ::: "memory"); }

#ifndef	__SMP__
#undef __SPIN_LOCK_IRQ
#define __SPIN_LOCK_IRQ(lock)					(inline_DisableAllInterrupts())
#undef __SPIN_UNLOCK_IRQ
#define __SPIN_UNLOCK_IRQ(lock)					(inline_EnableAllInterrupts())
#undef __SPIN_FLASH_IRQ
#define __SPIN_FLASH_IRQ(lock)					(inline_EnableAllInterrupts(),inline_DisableAllInterrupts(),((TBool)TRUE))
#endif


/**
Whether a PDE maps a page table (level-2 table descriptor).
*/
FORCE_INLINE TBool Mmu::PdeMapsPageTable(TPde aPde)
	{
	return (aPde & KPdeTypeMask) == KA64DescTable;
	}

/**
Whether a PDE maps a section (level-2 2 MB block descriptor).
*/
FORCE_INLINE TBool Mmu::PdeMapsSection(TPde aPde)
	{
	return (aPde & KPdeTypeMask) == KA64DescBlock;
	}

#endif // __A64_XMMU_H__
