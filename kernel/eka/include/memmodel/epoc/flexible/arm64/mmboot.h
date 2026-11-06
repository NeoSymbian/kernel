// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// SPDX-License-Identifier: EPL-2.0
//
// mmboot.h -- shadow of include/memmodel/epoc/flexible/arm/mmboot.h for the AArch64 ILP32 kernel (its arm64 dir
// first on -I).
//
// The LINEAR MAP is the ARM flexible model's, unchanged: the AArch32 user side is laid out by rombuild for it (ROM at
// 0x80000000, user data below 0x7F000000), and the kernel's 32-bit pointers fit it.
//
// The GEOMETRY is VMSAv8-64 with a 4 KB granule and TCR_EL1.T0SZ=32 (one 4 GB space, TTBR0 only):
//   level 1  4 entries x 1 GB  -- the per-address-space ROOT (KRootTableBase), the TTBR0 target
//   level 2  512 x 2 MB each   -- EKA2's "page directory": 4 tables = 2048 PDEs = 16 KB per address space
//   level 3  512 x 4 KB each   -- EKA2's "page table": exactly one page, maps one 2 MB "chunk"
// This is the same shape Phorepoc's x86-64 port used for long mode (x64mm/mmboot.h): KChunkShift 21, 8-byte
// entries, a page table per page, a 16 KB page directory.
//
// ARM's split of each page directory into a local half (TTBR0, per ASID) and a global half (TTBR1) maps onto the
// four level-2 tables: tables 0-1 (0-2 GB, local data) belong to the address space, tables 2-3 (2-4 GB: ROM,
// global and kernel memory) are the SAME physical pages in every address space. Mapped back to back at
// PageDirectory(asid) they still look like one contiguous 16 KB directory to the generic code.
//
// ARM domains don't exist in VMSAv8. The only domain user was the IPC alias region (domain 2). It stays at
// 0x7F000000, but an alias PDE points at the TARGET process's page table, whose PTEs grant EL0 access -- so the
// alias PDE carries APTable[0] (KA64APTableNoEL0: no EL0 access anywhere below it), which keeps other threads of the
// current process out of it the way domain 2 did (xmmu.cpp, DMemModelThread::Alias). Consequence: the kernel's
// user-memory copies must reach alias addresses with privileged loads/stores, not LDTR/STTR.
//
// The per-address-space level-1 roots live in the fixed bootstrap area, in the slot ARM used for its AArch32 AP boot
// page directory (PSCI enters the kernel directly, so the AP boot page/trampoline pages are not used).
#ifndef __MMBOOT_H__
#define __MMBOOT_H__

#include <nk_cpu.h>

// Linear address map (the ARM flexible model's)
// 00000000-003FFFFF	Unmapped
// 00400000-7EFFFFFF	Local data
// 7F000000-7FFFFFFF	IPC Alias region (EL1-only)
// 80000000-			ROM
//         -8FFFFFFF	Global user area
// 90000000-EFFFFFFF	Kernel memory
// F0000000-FFF00000	Fixed kernel mappings
// FFF00000-FFFFFFFF	(ARM: exception vectors; AArch64 uses VBAR_EL1, the window stays reserved)

const TLinAddr KUserLocalDataBase		=0x00400000u;
const TLinAddr KUserLocalDataEnd		=0x7f000000u;

const TLinAddr KIPCAlias				=0x7f000000u;
const TLinAddr KIPCAliasAreaSize		=0x01000000u;

/** Everything above here is global (visible to all processes).*/
const TLinAddr KGlobalMemoryBase		=0x80000000u;

const TLinAddr KRomLinearBase			=0x80000000u;

/** Everything below here has user access permissions, everything above is supervisor only. */
const TLinAddr KUserMemoryLimit			=0x90000000u;

const TLinAddr KKernelSectionBase		=0x90000000u;

const TLinAddr KPrimaryIOBase			=0xC6000000u; // XXX This magic constant is hard coded into baseports!
const TLinAddr KPrimaryIOEnd			=0xC8000000u;

const TLinAddr KKernelDataBase			=0xC8000000u;

const TLinAddr KKernelSectionEnd		=0xF0000000u;

const TLinAddr KSuperPageLinAddr		=0xF0000000u;
const TLinAddr KMachineConfigLinAddr	=0xF0000800u;
const TLinAddr KRootTableBase			=0xF0004000u;	// AArch64: 256 x 32-byte level-1 roots = 8 KB (bootstrap-mapped)
const TLinAddr KRootTableEnd			=0xF0006000u;
const TLinAddr KDummyUncachedAddr		=0xF000F000u;
const TLinAddr KPageInfoMap				=0xF0010000u;
const TLinAddr KExcptStacksLinearBase	=0xF0040000u;
const TLinAddr KExcptStacksLinearEnd	=0xF0080000u;
const TLinAddr KTempAddr				=0xF0080000u;
const TLinAddr KTempAddrEnd				=0xF0100000u;

const TLinAddr KPageTableInfoBase		=0xF0C00000u;
const TLinAddr KPageTableInfoEnd		=0xF1000000u;

const TLinAddr KPageArraySegmentBase	=0xF1000000u;
const TLinAddr KPageArraySegmentEnd		=0xF2000000u;

const TLinAddr KPageInfoLinearBase		=0xF2000000u;
const TLinAddr KPageInfoLinearEnd		=0xF4000000u;

const TLinAddr KPageDirectoryBase		=0xF4000000u;	// 256 x 16 KB directories = 4 MB
const TLinAddr KPageDirectoryEnd		=0xF8000000u;

const TLinAddr KPageTableBase			=0xF8000000u;
const TLinAddr KPageTableEnd			=0xFFE00000u;	// 2 MB aligned: a fixed kernel region must be whole chunks (DMemoryManager::New)

// VMSAv8-64, 4 KB granule, T0SZ=32
const TInt KPageShift=12;
const TInt KPageSize=1<<KPageShift;
const TInt KPageMask=KPageSize-1;
const TInt KChunkShift=21;								// one level-3 table (512 x 8-byte PTEs) maps 2 MB
const TInt KChunkSize=1<<KChunkShift;
const TInt KChunkMask=KChunkSize-1;
const TInt KPageTableShift=KChunkShift-KPageShift+3;	// PTE is 8 bytes: a page table is exactly one page
const TInt KPageTableSize=1<<KPageTableShift;
const TInt KPageTableMask=KPageTableSize-1;
const TInt KPtClusterShift=KPageShift-KPageTableShift;	// 0: one page table per page
const TInt KPtClusterSize=1<<KPtClusterShift;
const TInt KPtClusterMask=KPtClusterSize-1;
const TInt KPtBlockShift=KPageShift-4;					/**< @internalTechnology */	// sizeof(SPageTableInfo)=16
const TInt KPtBlockSize=1<<KPtBlockShift;				/**< @internalTechnology */
const TInt KPtBlockMask=KPtBlockSize-1;					/**< @internalTechnology */
const TInt KPagesInPDEShift=KChunkShift-KPageShift;
const TInt KPagesInPDE=1<<KPagesInPDEShift;
const TInt KPagesInPDEMask=KPagesInPDE-1;
const TInt KLargePageShift=16;							// 16 PTEs with the Contiguous bit (ARMv6's 64 KB large page)
const TInt KLargePageSize=1<<KLargePageShift;
const TInt KLargePageMask=KLargePageSize-1;

const TInt KPageDirectoryShift=32-KChunkShift+3;		// PDE is 8 bytes: 2048 PDEs = 16 KB, as on ARM
const TInt KPageDirectorySize=1<<KPageDirectoryShift;
const TInt KPageDirectoryMask=KPageDirectorySize-1;
const TInt KLocalPdTables=2;							// level-2 tables 0-1 (0-2 GB) are per address space

const TInt KRootTableShift=5;							// 4 level-1 entries x 8 bytes
const TInt KRootTableSize=1<<KRootTableShift;

const TInt KMmuAsidCount=256;							// 8-bit ASIDs (TCR_EL1.AS=0)
const TInt KMmuAsidMask=KMmuAsidCount-1;

const TUint KNumOsAsids=KMmuAsidCount;
const TUint KKernelOsAsid=0;

// Translation table descriptor bits (ARM ARM D5.3). Bit 0 = valid: a PTE/PDE with bit 0 clear is ignored by the
// walker, so EKA2's "inaccessible but remembered" PTEs keep all their other bits (MakePteInaccessible).
const TUint64 KA64DescValid			=0x1;
const TUint64 KA64DescTable			=0x3;			// level 1/2: next-level table; level 3: page
const TUint64 KA64DescBlock			=0x1;			// level 1/2: block (2 MB at level 2 = EKA2's "section")
const TUint64 KA64DescTypeMask		=0x3;
const TUint64 KA64AttrIndxShift		=2;				// [4:2] index into MAIR_EL1
const TUint64 KA64AttrIndxMask		=0x7<<2;
const TUint64 KA64AP1				=1<<6;			// AP[1]: EL0 access
const TUint64 KA64AP2				=1<<7;			// AP[2]: read-only
const TUint64 KA64SHInner			=3<<8;			// SH[1:0]=11 inner shareable
const TUint64 KA64AF				=1<<10;			// access flag (always set: no AF faults)
const TUint64 KA64nG				=1<<11;			// not global: TLB entry tagged with the ASID
const TUint64 KA64Contiguous		=1ull<<52;
const TUint64 KA64PXN				=1ull<<53;		// privileged execute never
const TUint64 KA64UXN				=1ull<<54;		// unprivileged execute never
const TUint64 KA64OutAddrMask		=0x0000fffffffff000ull;	// output address [47:12]
const TUint64 KA64APTableNoEL0		=1ull<<61;		// table descriptor APTable[0]: no EL0 access at any level below

// MAIR_EL1: attribute index = EKA2 TMemoryType (BlankPte puts the type in AttrIndx); the bootstrap programs this value.
//   0 EMemAttStronglyOrdered  Device-nGnRnE			0x00
//   1 EMemAttDevice           Device-nGnRE				0x04
//   2 EMemAttNormalUncached   Normal non-cacheable		0x44
//   3 EMemAttNormalCached     Normal WB RA/WA			0xFF
//   4 EMemAttKernelInternal4  Normal WB RA/WA			0xFF
//   5-7 platform specific     Normal WT RA (5), WB RA no-WA (6), WB RA/WA (7)	0xAA 0xEE 0xFF
const TUint64 KA64MairValue			=0xFFEEAAFFFF440400ull;

// ARM domains don't exist: generic code that tags PDEs with a domain (__USER_MEMORY_GUARDS_ENABLED__) leaves them as is
#define	PDE_IN_DOMAIN(aPde, aDomain)	(aPde)

const TUint64 KPdePresentMask		=KA64DescValid;
const TUint64 KPdeTypeMask			=KA64DescTypeMask;
const TUint32 KPdeSectionAddrMask	=0xffe00000u;	// 2 MB block (physical addresses are 32-bit: TPhysAddr)
const TUint32 KPdePageTableAddrMask	=0xfffff000u;
const TUint32 KPteLargePageAddrMask	=0xffff0000u;
const TUint32 KPteSmallPageAddrMask	=0xfffff000u;
const TInt KLargeSmallPageRatio=KLargePageSize/KPageSize;	/**< @internalTechnology */
const TUint64 KPdeUnallocatedEntry	=0;
const TUint64 KPteUnallocatedEntry	=0;
const TUint64 KPdeMatchMask			=0;
const TUint64 KPteMatchMask			=0;
const TUint64 KPtePresentMask		=KA64DescValid;
const TUint64 KPteTypeMask			=KA64DescTypeMask;

const TInt KPageInfoShift = 5;

const TInt KPageColourShift=0;						// PIPT data caches: no colouring
const TInt KPageColourCount=(1<<KPageColourShift);
const TInt KPageColourMask=KPageColourCount-1;

// Exception stacks per CPU: AArch64 takes every exception on SP_EL1, these keep the ARM layout's reservation
const TInt KAbtStackSize=KPageSize;		/**< @internalComponent */
const TInt KUndStackSize=KPageSize;		/**< @internalComponent */
const TInt KIrqStackSize=KPageSize;		/**< @internalComponent */
const TInt KFiqStackSize=KPageSize;		/**< @internalComponent */
const TInt KExcStackAddressSpace = KIrqStackSize + KFiqStackSize + KUndStackSize + KAbtStackSize + 4*KPageSize;	/**< @internalComponent */

const TInt KMaxCPUs = 8;
__ASSERT_COMPILE(KExcptStacksLinearEnd-KExcptStacksLinearBase >= (TLinAddr)KExcStackAddressSpace * (TUint)KMaxCPUs);
__ASSERT_COMPILE(KRootTableBase+KNumOsAsids*KRootTableSize <= KRootTableEnd);
__ASSERT_COMPILE(KRootTableEnd <= KDummyUncachedAddr);

#endif	// __MMBOOT_H__
