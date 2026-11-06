// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// Portions Copyright The Phorepoc Authors
// SPDX-License-Identifier: EPL-2.0
//
// a64boot.cpp -- the BOOTSTRAP of the AArch64 full kernel, for the flexible memory model. A port of Phorepoc's
// x86-64 bootstrap (adaptation-x86-64 ekern/xboot.cpp) to VMSAv8-64 tables (port/arm64/mm/mmboot.h).
//
// boot.S calls A64_BootMain at EL1 with the MMU and caches off, at the image's physical (= link) address. It
// leaves the machine as EKA2's kernel entry expects:
//   - a ROM header + image records at KRomLinearBase (primary = this image, the static variant, one extension);
//   - the kernel address space: the 16 KB page directory of OS ASID 0 (four level-2 tables) at
//     KPageDirectoryBase; page tables contiguous at KPageTableBase, mapped through PT0; the ASID 0 level-1 root at
//     KRootTableBase; super page + CPU page; page-table-info page; initial page-array segment; KPageInfoMap + the
//     SPageInfo array for the RAM bank; the board's devices in the IO window at KPrimaryIOBase;
//   - kernel .data copied, .bss zero, the initial thread stack and the initial heap right above it;
//   - MAIR/TCR/TTBR0/SCTLR set, then _E32Startup(ROM header, super page) on the initial stack.
// The kernel learns which RAM pages are in use only by WALKING these tables (Mmu::Init2Common): every page taken
// here is mapped, or lies outside the RAM bank.
//
// Runs physical: nothing outside the boot stub may be called (ROM code sits at its linear address, unmapped until
// the switch), and the ROM is read and written through its physical address. Built -mstrict-align (with the MMU
// off every data access is Device memory, which faults on an unaligned access) and without libc.
#include <e32rom.h>
#include <kernel/kernboot.h>
#include <mmboot.h>
#include "../board/board.h"

typedef TUint64 TEntry;

extern "C" {
extern char _start[];
extern char __rom_lin[], __rom_phys[], __rom_size[];
extern char __code_base[], __code_end[];
extern char __data_run[], __data_load[], __data_size[], __svdata_size[];
void _E32Startup(TLinAddr aRomHeader, TLinAddr aSuperPage);	// ROM: ekern/a64e_entry.cpp
TInt A64VariantEntry(TInt aReason);								// ROM: board/a64_variant.cpp
TAny* A64VariantInitialise(TInt aMode);
TInt A64ExtensionEntry(TInt aReason);
}
#define SYMVAL(s)	((TUint32)(TLinAddr)(s))

// RAM the kernel may use: from above the image to the board's limit
#if defined(BOARD_QEMU)
const TUint32 KRamEnd = 0x60000000u;			// run-qemu.sh: -m 512 from 0x40000000
#elif defined(BOARD_BONITO)
const TUint32 KRamEnd = 0x85700000u;			// shortcut: below hyp-mem@85700000 (DTB); the M4.3 RAM map adds the rest
#else
#error "BOARD_QEMU or BOARD_BONITO"
#endif

/******************************************************************************
 * Console (physical; boards without a UART stay silent until the kernel's console)
 ******************************************************************************/
static void PutC(char c)
	{
#ifdef BOARD_QEMU
	volatile TUint32* uart = (volatile TUint32*)0x09000000u;	// PL011
	while (uart[0x18 / 4] & (1u << 5)) {}
	uart[0] = (TUint8)c;
#else
	(void)c;
#endif
	}
static void Put(const char* s)
	{ while (*s) { if (*s == '\n') PutC('\r'); PutC(*s++); } }
static void PutHex(TUint32 v)
	{ for (TInt i = 28; i >= 0; i -= 4) PutC("0123456789abcdef"[(v >> i) & 15]); }
static void Trace(const char* aWhat, TUint32 aVal)
	{ Put("  [a64boot] "); Put(aWhat); Put(" = "); PutHex(aVal); Put("\n"); }
// Boards without a UART show progress as a colour band across the top of the boot framebuffer
static void Band(TUint32 aArgb)
	{
#ifdef BOARD_BONITO
	volatile TUint32* fb = (volatile TUint32*)0x9c000000u;		// 1080 x 2160, stride 4320 (board/pixel_bonito.cpp)
	for (TInt i = 0; i < 1080 * 64; ++i)
		fb[i] = aArgb;
	__asm__ __volatile__("dsb sy" ::: "memory");
#else
	(void)aArgb;
#endif
	}
static void Die(const char* aWhy)
	{ Put("  [a64boot] FATAL: "); Put(aWhy); Put("\n"); Band(0xFFFF0000); for (;;) __asm__ __volatile__("wfe"); }

static void Zero(TUint32 aPhys, TUint32 aSize)
	{ volatile TUint64* p = (volatile TUint64*)aPhys; for (aSize >>= 3; aSize; --aSize) *p++ = 0; }
static void Copy(TUint32 aDst, TUint32 aSrc, TUint32 aSize)
	{ volatile TUint8* d = (volatile TUint8*)aDst; const volatile TUint8* s = (const volatile TUint8*)aSrc; while (aSize--) *d++ = *s++; }
static void Fill32(TUint32 aPhys, TUint32 aSize, TUint32 aVal)
	{ volatile TUint32* p = (volatile TUint32*)aPhys; for (aSize >>= 2; aSize; --aSize) *p++ = aVal; }

/******************************************************************************
 * Physical pages: bump allocation upward from the base of the RAM bank, zeroed
 ******************************************************************************/
static TUint32 BankBase, BankEnd, AllocNext;

static TUint32 AllocPages(TUint aCount)
	{
	TUint32 p = AllocNext;
	AllocNext += aCount << KPageShift;
	if (AllocNext > BankEnd)
		Die("out of RAM");
	Zero(p, aCount << KPageShift);
	return p;
	}

/******************************************************************************
 * Mapping. Leaf entries are what Mmu::BlankPte (mm/xmmu.cpp) composes for the same memory, so a kernel memory
 * object claiming a boot mapping finds the entry it would have written. Everything mapped here is global (nG=0).
 ******************************************************************************/
const TEntry KAttrCached	= (TEntry)3 << KA64AttrIndxShift;		// EMemAttNormalCached (MAIR index = TMemoryType)
const TEntry KPermPde		= KA64DescTable;
const TEntry KPermKernel	= KA64DescTable | KA64AF | KAttrCached | KA64SHInner | KA64PXN | KA64UXN;	// EL1 RW
// ROM: read-only at EL0 and EL1 and executable by both (kernel and user code share it, as on ARM). PSTATE.PAN
// stays 0 (SCTLR_EL1.SPAN set below): the kernel reads this EL0-visible memory with ordinary loads.
const TEntry KPermRom		= KA64DescTable | KA64AF | KAttrCached | KA64SHInner | KA64AP1 | KA64AP2;
static TEntry PermIo(TUint32 aAttr)		// 1 Device-nGnRE, 2 Normal non-cacheable (shareable)
	{ return KA64DescTable | KA64AF | ((TEntry)aAttr << KA64AttrIndxShift) | KA64PXN | KA64UXN | (aAttr >= 2 ? KA64SHInner : 0); }

static TEntry*	KernPd;			// physical: the 4 contiguous level-2 tables of OS ASID 0
static TEntry*	Pt0;			// the page table that maps the page-table window
static TUint	PtCount;

static TEntry* PageTableFor(TLinAddr aLin)
	{
	TEntry& pde = KernPd[aLin >> KChunkShift];
	if (pde & KA64DescValid)
		return (TEntry*)(TUint32)(pde & KA64OutAddrMask);
	if (PtCount >= (TUint)(KChunkSize >> KPageShift))
		Die("page-table window full");
	TUint32 pt = AllocPages(1);
	Pt0[PtCount++] = (TEntry)pt | KPermKernel;		// appears at KPageTableBase + n*4K: contiguous
	pde = (TEntry)pt | KPermPde;
	return (TEntry*)pt;
	}

static void MapContiguous(TLinAddr aLin, TUint32 aPhys, TUint32 aSize, TEntry aPerm)
	{
	for (TUint32 off = 0; off < aSize; off += KPageSize)
		{
		TEntry& pte = PageTableFor(aLin + off)[((aLin + off) >> KPageShift) & (KChunkMask >> KPageShift)];
		if (pte)
			Die("double mapping");
		pte = (TEntry)(aPhys + off) | aPerm;
		}
	}

static TUint32 AllocAndMap(TLinAddr aLin, TUint32 aSize, TEntry aPerm)
	{
	aSize = (aSize + KPageMask) & ~KPageMask;
	TUint32 p = AllocPages(aSize >> KPageShift);
	MapContiguous(aLin, p, aSize, aPerm);
	return p;
	}

// ARM's super page + CPU page (include/kernel/arm/bootdefs.h): the bank tables live in the CPU page
const TUint32 KSuperCpuPages	= 2;
const TUint32 KRamTableOffset	= 0x1900;		// CpuRamTableOffset
const TUint32 KInitStackSize	= 0x4000;
const TUint32 KHeapSizeMin		= 0x00100000;
const TUint32 KHeapSizeMax		= 0x01000000;

extern "C" void A64_BootMain(TUint64 /*aDtb*/)
	{
	Put("\n[a64boot] EKA2 AArch64 bootstrap: flexible memory model, VMSAv8 4K tables\n");
	const TUint32 romLin	= SYMVAL(__rom_lin);
	const TUint32 romPhys	= SYMVAL(__rom_phys);
	const TUint32 romSize	= (SYMVAL(__rom_size) + KPageMask) & ~KPageMask;
	const TUint32 svData	= SYMVAL(__svdata_size);
	Trace("ROM phys", romPhys); Trace("ROM size", romSize); Trace("SvData size", svData);
#define ROM_PHYS(lin)	(romPhys + (SYMVAL(lin) - romLin))

	// ---- RAM: one bank, from the first 2 MB boundary above the ROM image (so no SPageInfo block holds both bank
	// and ROM pages: Mmu::Init2Common would find ROM pages "in RAM" but in no zone) to the board's limit
	BankBase = (romPhys + romSize + KChunkMask) & ~(TUint32)KChunkMask;
	BankEnd = KRamEnd;
	AllocNext = BankBase;
	Trace("RAM bank base", BankBase); Trace("RAM bank end", BankEnd);

	// ---- the page directory, PT0 self-mapping the page-table window, the directory mapped
	KernPd = (TEntry*)AllocPages(KPageDirectorySize >> KPageShift);
	Pt0 = (TEntry*)AllocPages(1);
	Pt0[0] = (TEntry)(TUint32)Pt0 | KPermKernel;
	PtCount = 1;
	KernPd[KPageTableBase >> KChunkShift] = (TEntry)(TUint32)Pt0 | KPermPde;
	MapContiguous(KPageDirectoryBase, (TUint32)KernPd, KPageDirectorySize, KPermKernel);

	// ---- the level-1 roots (one 32-byte root per OS ASID; mm/mpdalloc.cpp fills the others), OS ASID 0's now
	const TUint32 rootPhys = AllocAndMap(KRootTableBase, KRootTableEnd - KRootTableBase, KPermKernel);
	for (TInt i = 0; i < 4; ++i)
		((TEntry*)rootPhys)[i] = ((TEntry)(TUint32)KernPd + i * KPageSize) | KA64DescTable;

	// ---- ROM, super + CPU page, page-table info, initial page-array segment
	MapContiguous(romLin, romPhys, romSize, KPermRom);
	const TUint32 superPhys = AllocAndMap(KSuperPageLinAddr, KSuperCpuPages << KPageShift, KPermKernel);
	AllocAndMap(KPageTableInfoBase, KPageSize, KPermKernel);
	AllocAndMap(KPageArraySegmentBase, KPageSize, KPermKernel);

	// ---- the board's devices, back to back in the IO window (board.h; console_io.cpp A64_Io computes the same)
	const SA64IoRegion* io = (const SA64IoRegion*)ROM_PHYS(A64_IoRegions);
	const TInt nio = *(const TInt*)ROM_PHYS(&A64_IoRegionCount);
	TLinAddr ioLin = KPrimaryIOBase;
	for (TInt i = 0; i < nio; ++i)
		{
		if ((io[i].iPhys | io[i].iSize) & KPageMask)
			Die("IO region not page aligned");
		if (ioLin + io[i].iSize > KPrimaryIOEnd)
			Die("IO window full");
		MapContiguous(ioLin, io[i].iPhys, io[i].iSize, PermIo(io[i].iAttr));
		ioLin += io[i].iSize;
		}

	// ---- kernel .data/.bss + initial thread stack + initial heap, contiguous at iKernDataAddress
	TUint32 heap = ((BankEnd - BankBase) >> (KPageShift - 1)) + 24 * 1024;		// ~2 bytes per RAM page + 24K
	if (heap < KHeapSizeMin)
		heap = KHeapSizeMin;
	heap = (heap + KPageMask) & ~KPageMask;
	const TUint32 svRounded = (svData + KPageMask) & ~KPageMask;
	const TLinAddr kernData = SYMVAL(__data_run);
	const TUint32 kdPhys = AllocAndMap(kernData, svRounded + KInitStackSize + heap, KPermKernel);
	Copy(kdPhys, ROM_PHYS(__data_load), SYMVAL(__data_size));		// .data; .bss is already zero
	Fill32(kdPhys + svRounded, KInitStackSize, 0xFFFFFFFFu);
	const TLinAddr initSp = kernData + svRounded + KInitStackSize;
	Trace("kernel data phys", kdPhys); Trace("initial heap", heap); Trace("initial SP", initSp);

	// ---- the RAM bank list (CPU page; [1] = {0,0} ends RAM, [2] = {0,0} ends the reserved banks), then the SPageInfo
	// array: one page of 32-byte records per 128 RAM pages; bit n of KPageInfoMap set <=> page n of the array exists
	SRamBank* ram = (SRamBank*)(superPhys + KRamTableOffset);
	ram[0].iBase = BankBase; ram[0].iSize = BankEnd - BankBase;
	const TUint32 mapPhys = AllocAndMap(KPageInfoMap, KPageSize, KPermKernel);
	const TUint blockShift = KPageShift + KPageShift - KPageInfoShift;		// RAM covered by one SPageInfo page
	TUint first = BankBase >> blockShift, last = (BankEnd - 1) >> blockShift;
	for (TUint n = first; n <= last; ++n)
		((volatile TUint8*)mapPhys)[n >> 3] |= (TUint8)(1u << (n & 7));
	AllocAndMap(KPageInfoLinearBase + (first << KPageShift), (last - first + 1) << KPageShift, KPermKernel);

	// ---- the ROM header + image records, written through the ROM's physical address
	TRomHeader& rh = *(TRomHeader*)romPhys;
	struct SRecords												// at ROM + sizeof(TRomHeader)
		{
		TRomEntry			iPrimaryEntry;	TUint8 iPad1[32];	// iName[] extends past the struct
		TRomImageHeader		iPrimary;
		TRomEntry			iVariantEntry;	TUint8 iPad2[32];
		TRomImageHeader		iVariant;
		TUint32				iVariantExports[1];
		TRomEntry			iExtensionEntry; TUint8 iPad4[32];
		TRomImageHeader		iExtension;
		TRomRootDirectoryList iRootList;
		TRomDir				iRootDir;		TUint8 iPad3[32];
		};
	const TUint32 recOff = (sizeof(TRomHeader) + 15) & ~15u;
	if (recOff + sizeof(SRecords) > (TUint32)KPageSize)
		Die("ROM header area too small");
	SRecords& rec = *(SRecords*)(romPhys + recOff);
	const TLinAddr recLin = romLin + recOff;
	#define REC_LIN(f)	(recLin + (TUint32)((TUint8*)&rec.f - (TUint8*)&rec))

	// ROM build time, microseconds since 0 AD: P::InitSystemTime falls back to it on a cold start and panics
	// (PLAT 0) if it is not a valid date. 730497 days to 2000-01-01 (stimer.cpp) + 9806 to 2026-11-06.
	rh.iTime = (TInt64)(730497 + 9806) * 86400 * 1000000;
	rh.iRomBase = romLin;
	rh.iRomSize = romSize;
	rh.iUncompressedSize = romSize;
	rh.iRomHeaderSize = sizeof(TRomHeader);
	rh.iKernDataAddress = kernData;
	rh.iKernelLimit = (kernData + svRounded + KInitStackSize + KHeapSizeMax + 0xFFFFFu) & ~0xFFFFFu;
	rh.iTotalSvDataSize = svData;
	rh.iPrimaryFile = REC_LIN(iPrimaryEntry);
	rh.iVariantFile = REC_LIN(iVariantEntry);
	rh.iExtensionFile = REC_LIN(iExtensionEntry);
	rh.iSecondaryFile = 0;										// shortcut: no user side yet (M4.4)
	rh.iUserDataAddress = KUserLocalDataEnd;
	rh.iKernelConfigFlags = 0;

	TRomImageHeader& k = rec.iPrimary;
	k.iUid1 = 0x1000007Au;  k.iUid2 = 0x1000008Bu;  k.iUid3 = 0x10287037u;	// EXE / kernel / SMP kernel
	k.iEntryPoint = SYMVAL(_E32Startup);
	k.iCodeAddress = SYMVAL(__code_base);
	k.iCodeSize = k.iTextSize = SYMVAL(__code_end) - SYMVAL(__code_base);
	k.iDataAddress = SYMVAL(__data_load);
	k.iDataSize = SYMVAL(__data_size);
	k.iBssSize = svData - SYMVAL(__data_size);
	k.iTotalDataSize = svData;
	k.iDataBssLinearBase = kernData;
	k.iHeapSizeMin = heap;  k.iHeapSizeMax = KHeapSizeMax;  k.iStackSize = KInitStackSize;
	k.iFlags = KRomImageFlagPrimary;
	k.iPriority = EPrioritySupervisor;
	k.iS.iCaps[0] = 0xFFFFFFFFu;  k.iS.iCaps[1] = 0xFFFFFFFFu;
	k.iHardwareVariant = 0x01000000u;							// top byte <= 3: compatible with anything
	rec.iPrimaryEntry.iSize = romSize;
	rec.iPrimaryEntry.iAddressLin = REC_LIN(iPrimary);
	rec.iPrimaryEntry.iNameLength = 5;
	{ const char* n = "ekern"; for (TInt i = 0; i < 5; ++i) rec.iPrimaryEntry.iName[i << 1] = (TUint8)n[i]; }

	TRomImageHeader& v = rec.iVariant;							// the variant is linked into the kernel image
	v.iUid1 = 0x10000079u;										// DLL
	v.iEntryPoint = SYMVAL(A64VariantEntry);
	v.iExportDirCount = 1;
	v.iExportDir = REC_LIN(iVariantExports);
	rec.iVariantExports[0] = SYMVAL(A64VariantInitialise);
	v.iFlags = KRomImageFlagVariant;
	v.iS.iCaps[0] = 0xFFFFFFFFu;  v.iS.iCaps[1] = 0xFFFFFFFFu;
	v.iHardwareVariant = 0x01000000u;
	rec.iVariantEntry.iAddressLin = REC_LIN(iVariant);
	rec.iVariantEntry.iNameLength = 5;
	{ const char* n = "ecust"; for (TInt i = 0; i < 5; ++i) rec.iVariantEntry.iName[i << 1] = (TUint8)n[i]; }

	// at least ONE kernel extension must exist: P::StartExtensions sizes an RArray by the count (0 panics)
	TRomImageHeader& x = rec.iExtension;
	x.iUid1 = 0x10000079u;
	x.iEntryPoint = SYMVAL(A64ExtensionEntry);
	x.iFlags = KRomImageFlagExtension;
	x.iS.iCaps[0] = 0xFFFFFFFFu;  x.iS.iCaps[1] = 0xFFFFFFFFu;
	x.iHardwareVariant = 0x01000000u;
	rec.iExtensionEntry.iAddressLin = REC_LIN(iExtension);
	rec.iExtensionEntry.iNameLength = 4;
	{ const char* n = "xext"; for (TInt i = 0; i < 4; ++i) rec.iExtensionEntry.iName[i << 1] = (TUint8)n[i]; }

	// an empty Z: root directory (iSize counts the TRomEntry region: 0 entries -> 0)
	rec.iRootList.iNumRootDirs = 1;
	rec.iRootList.iRootDir[0].iHardwareVariant = 0x01000000u;
	rec.iRootList.iRootDir[0].iAddressLin = REC_LIN(iRootDir);
	rec.iRootDir.iSize = 0;
	rh.iRomRootDirectoryList = REC_LIN(iRootList);

	// ---- super page
	SSuperPageBase& sp = *(SSuperPageBase*)superPhys;
	sp.iTotalRomSize = romSize;
	sp.iTotalRamSize = BankEnd - BankBase;
	sp.iPageDir = KPageDirectoryBase;
	sp.iRamBootData = KSuperPageLinAddr + KRamTableOffset;
	sp.iBootFlags = KBootFlagRunFromRAM;
	sp.iMachineData = KMachineConfigLinAddr;
	sp.iActiveVariant = 0x01000000u;
	sp.iPrimaryEntry = REC_LIN(iPrimaryEntry);
	sp.iInitialHeapSize = heap;
	sp.iDebugPort = 0;
	sp.iRootDirList = REC_LIN(iRootList);
	sp.iKernelLimit = rh.iKernelLimit;
	sp.iRomHeaderPhys = romPhys;

	// ---- a temporary identity window: one 2 MB block over the boot stub (ekern.lds keeps it inside one), so the
	// instruction after the MMU switch is still mapped. _E32Startup removes it before the memory model walks.
	const TUint32 bootPhys = SYMVAL(_start) & ~(TUint32)KChunkMask;
	TEntry& idPde = KernPd[bootPhys >> KChunkShift];
	if (idPde)
		Die("the identity window collides with the kernel map (M4.3: boot from a different address)");
	idPde = (TEntry)bootPhys | KA64DescBlock | KA64AF | KAttrCached | KA64SHInner | KA64UXN;

	Trace("page tables used", PtCount); Trace("RAM used by boot", AllocNext - BankBase);
	Trace("kernel entry", k.iEntryPoint); Trace("super page phys", superPhys);
	Put("[a64boot] MMU on, calling _E32Startup(ROM header, super page)\n");
	Band(0xFF0040FF);											// blue: the tables are built, switching

	// Written with the MMU off (non-cacheable): drop any stale cache lines over it before the caches see it.
	for (TUint32 a = romPhys; a < romPhys + KPageSize; a += 64)
		__asm__ __volatile__("dc ivac, %0" :: "r"((TUint64)a) : "memory");
	for (TUint32 a = BankBase; a < AllocNext; a += 64)
		__asm__ __volatile__("dc ivac, %0" :: "r"((TUint64)a) : "memory");
	__asm__ __volatile__("dsb sy\n\tic iallu\n\tdsb sy\n\tisb" ::: "memory");

	const TUint64 tcr = 32 | (1u << 8) | (1u << 10) | (3u << 12) | (1u << 23);	// T0SZ 32, WBWA ISH walks, 4K, EPD1, IPS 4 GB
	TUint64 sctlr;
	__asm__ __volatile__("mrs %0, sctlr_el1" : "=r"(sctlr));
	sctlr |= (1u << 0) | (1u << 2) | (1u << 12) | (1u << 23);	// M, C, I; SPAN: exceptions leave PSTATE.PAN alone
	sctlr &= ~(TUint64)((1u << 1) | (1u << 19) | (1u << 25));	// no alignment checks, no WXN, little-endian EL1
	__asm__ __volatile__(
		"msr	pan, #0\n\t"
		"msr	mair_el1, %0\n\t"
		"msr	tcr_el1, %1\n\t"
		"msr	ttbr0_el1, %2\n\t"				// OS ASID 0's root, ASID field 0
		"isb\n\t"
		"tlbi	vmalle1\n\t"
		"dsb	nsh\n\t"
		"isb\n\t"
		"msr	sctlr_el1, %3\n\t"
		"isb\n\t"
		"mov	sp, %4\n\t"
		"mov	x0, %5\n\t"
		"mov	x1, %6\n\t"
		"mov	x29, xzr\n\t"
		"mov	x30, xzr\n\t"
		"br		%7"
		: : "r"(KA64MairValue), "r"(tcr), "r"((TUint64)rootPhys), "r"(sctlr), "r"((TUint64)initSp),
			"r"((TUint64)romLin), "r"((TUint64)KSuperPageLinAddr), "r"((TUint64)k.iEntryPoint)
		: "x0", "x1", "x29", "x30", "memory");
	Die("kernel entry returned");
	}
