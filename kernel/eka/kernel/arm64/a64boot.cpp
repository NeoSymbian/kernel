// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// Portions Copyright The Phorepoc Authors
// SPDX-License-Identifier: EPL-2.0
//
// a64boot.cpp -- the BOOTSTRAP of the AArch64 full kernel, for the flexible memory model. A port of Phorepoc's
// x86-64 bootstrap (adaptation-x86-64 ekern/xboot.cpp) to VMSAv8-64 tables (arm64/mmboot.h).
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
#include "board.h"

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
TInt A64ExstartShim(TInt aReason);								// ROM: board/a64_variant.cpp -> kernel/exstart.cpp
extern char A32Test[], A32TestEnd[];							// ROM .utext: boot/a32test.S, AArch32 EL0 code
extern char __uimage_base[], __uimage_size[];					// ekern.lds: the user-side ROM section, if any
}
#define SYMVAL(s)	((TUint32)(TLinAddr)(s))

// RAM the kernel may use: the DTB's map above the image (RamMap below), else from above the image to the board's limit
// The boot stub runs with the MMU off and may not call into the board: the board's variant.conf passes its
// constants (BOOT_DEFS): A64_BOOT_RAM_END (the end of RAM when there is no usable DTB), and optionally A64_BOOT_PL011
// (an early UART) and A64_BOOT_FB, A64_BOOT_FB_W (a boot framebuffer for boards without one).
#ifndef A64_BOOT_RAM_END
#error "the board's variant.conf sets A64_BOOT_RAM_END in BOOT_DEFS"
#endif
const TUint32 KRamEnd = A64_BOOT_RAM_END;

/******************************************************************************
 * Console (physical; boards without a UART stay silent until the kernel's console)
 ******************************************************************************/
static void PutC(char c)
	{
#ifdef A64_BOOT_PL011
	volatile TUint32* uart = (volatile TUint32*)A64_BOOT_PL011;	// PL011
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
#ifdef A64_BOOT_FB
	volatile TUint32* fb = (volatile TUint32*)A64_BOOT_FB;		// 32 bpp, A64_BOOT_FB_W pixels a line
	for (TInt i = 0; i < A64_BOOT_FB_W * 64; ++i)
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
 * The RAM map: the DTB's /memory, below 4 GB and above the image, minus its /reserved-memory regions, its /memreserve/
 * entries and the board's IO regions (A64_IoRegions: RAM the drivers map themselves, e.g. ABL's DSI packet page).
 * The DTB is read before the first page is allocated: it may lie in the RAM we hand out.
 ******************************************************************************/
struct SRange { TUint64 iBase, iEnd; };
const TInt KMaxRanges = 128, KMaxBanks = 64;		// the CPU page's bank list has room for 224
// shortcut: TPhysAddr is 32-bit, so RAM above 4 GB is left out; widen it (roadmap M4) to use the rest. The last page
// stays out too: Mmu::Init2Common rejects a bank whose end wraps to 0.
const TUint64 KPhysTop = 0xFFFFF000u;
const TUint64 KMinBank = 0x100000;					// smaller pieces cost more SPageInfo than they give
static SRange MemRanges[KMaxRanges], Holes[KMaxRanges];
static TInt MemCount, HoleCount;
static SRamBank Banks[KMaxBanks];
static TInt BankCount;
static TUint64 InitrdStart, InitrdEnd;	// the DTB's /chosen linux,initrd-*: the bootloader's ramdisk (ROFS=ramdisk: Z:)

static TUint32 Be32(TUint32 aPhys)
	{ return __builtin_bswap32(*(const volatile TUint32*)aPhys); }
static TUint64 Cells(TUint32& aPhys, TUint32 aCells)
	{ TUint64 v = 0; for (; aCells; --aCells, aPhys += 4) v = (v << 32) | Be32(aPhys); return v; }
static TUint8 Byte(TUint32 aPhys)
	{ return *(const volatile TUint8*)aPhys; }
static TBool StrIs(TUint32 aPhys, const char* aStr)			// a DTB string equals aStr
	{ for (; *aStr; ++aStr, ++aPhys) if (Byte(aPhys) != (TUint8)*aStr) return EFalse; return Byte(aPhys) == 0; }
static TBool NodeIs(TUint32 aPhys, const char* aName)		// a node name is aName or aName@unit
	{ for (; *aName; ++aName, ++aPhys) if (Byte(aPhys) != (TUint8)*aName) return EFalse; return Byte(aPhys) == 0 || Byte(aPhys) == '@'; }

static void AddRange(SRange* aList, TInt& aCount, TUint64 aBase, TUint64 aSize)
	{
	if (!aSize)
		return;
	if (aCount >= KMaxRanges)
		Die("too many RAM ranges in the DTB");		// never drop a hole: its RAM would be handed out
	aList[aCount].iBase = aBase;
	aList[aCount++].iEnd = aBase + aSize < aBase ? ~(TUint64)0 : aBase + aSize;
	}

static void Sort(SRange* aList, TInt aCount)
	{
	for (TInt i = 1; i < aCount; ++i)
		for (TInt j = i; j && aList[j - 1].iBase > aList[j].iBase; --j)
			{ const SRange t = aList[j]; aList[j] = aList[j - 1]; aList[j - 1] = t; }
	}

// Fills MemRanges and Holes from a flattened device tree; EFalse if there is none or it is malformed
static TBool ParseDtb(TUint64 aDtb)
	{
	if (!aDtb || aDtb >= KPhysTop || (aDtb & 3) || Be32((TUint32)aDtb) != 0xd00dfeedu)
		return EFalse;
	const TUint32 fdt = (TUint32)aDtb, total = Be32(fdt + 4);
	if (total < 40 || total > 0x400000 || fdt + total < fdt)
		return EFalse;
	const TUint32 end = fdt + total, strings = fdt + Be32(fdt + 12);
	for (TUint32 r = fdt + Be32(fdt + 16); r + 16 <= end; )		// /memreserve/: {address, size} pairs, {0, 0} ends
		{
		const TUint64 base = Cells(r, 2), size = Cells(r, 2);
		if (!base && !size)
			break;
		AddRange(Holes, HoleCount, base, size);
		}
	TUint32 rootAc = 2, rootSc = 1, resAc = 2, resSc = 1;		// #address-cells, #size-cells (the spec's defaults)
	TInt depth = 0;
	TBool inReserved = EFalse;
	TInt kind = 0;							// the current node: 1 a /memory node, 2 a /reserved-memory child, 3 /chosen
	for (TUint32 p = fdt + Be32(fdt + 8); p + 4 <= end; )
		{
		const TUint32 token = Be32(p);
		p += 4;
		if (token == 1)						// FDT_BEGIN_NODE, then the NUL-terminated name, padded to 4
			{
			++depth;
			kind = 0;
			if (depth == 2)
				{
				inReserved = NodeIs(p, "reserved-memory");
				kind = NodeIs(p, "memory") ? 1 : StrIs(p, "chosen") ? 3 : 0;
				}
			else if (depth == 3 && inReserved)
				kind = 2;
			while (p < end && Byte(p))
				++p;
			p = (p + 4) & ~3u;
			}
		else if (token == 2)				// FDT_END_NODE (properties come before subnodes: none follow here)
			{
			if (--depth < 2)
				inReserved = EFalse;
			kind = 0;
			}
		else if (token == 3)				// FDT_PROP: length, name offset, value padded to 4
			{
			if (p + 8 > end)
				return EFalse;
			const TUint32 len = Be32(p), name = strings + Be32(p + 4);
			TUint32 v = p + 8;
			p = (v + len + 3) & ~3u;
			if (p > end || p < v)
				return EFalse;
			const TBool root = depth == 1, res = depth == 2 && inReserved;
			if ((root || res) && len == 4 && StrIs(name, "#address-cells"))
				(root ? rootAc : resAc) = Be32(v);
			else if ((root || res) && len == 4 && StrIs(name, "#size-cells"))
				(root ? rootSc : resSc) = Be32(v);
			else if (kind == 3 && (len == 4 || len == 8) && StrIs(name, "linux,initrd-start"))
				InitrdStart = Cells(v, len >> 2);
			else if (kind == 3 && (len == 4 || len == 8) && StrIs(name, "linux,initrd-end"))
				InitrdEnd = Cells(v, len >> 2);
			else if ((kind == 1 || kind == 2) && StrIs(name, "reg"))
				{
				const TUint32 ac = kind == 1 ? rootAc : resAc, sc = kind == 1 ? rootSc : resSc;
				if (ac < 1 || ac > 2 || sc < 1 || sc > 2)
					return EFalse;
				for (const TUint32 e = v + len; v + 4 * (ac + sc) <= e; )
					{
					const TUint64 base = Cells(v, ac), size = Cells(v, sc);
					AddRange(kind == 1 ? MemRanges : Holes, kind == 1 ? MemCount : HoleCount, base, size);
					}
				}
			}
		else if (token == 9)				// FDT_END
			break;
		else if (token != 4)				// FDT_NOP
			return EFalse;
		}
	if (InitrdEnd > InitrdStart)
		AddRange(Holes, HoleCount, InitrdStart, InitrdEnd - InitrdStart);	// never handed out: a64_medrofs.cpp reads it
	return MemCount > 0;
	}

static void AddBank(TUint64 aBase, TUint64 aEnd)
	{
	aBase = (aBase + KPageMask) & ~(TUint64)KPageMask;
	aEnd &= ~(TUint64)KPageMask;
	if (aEnd <= aBase || aEnd - aBase < KMinBank || BankCount >= KMaxBanks)
		return;								// leaving RAM out is safe
	Banks[BankCount].iBase = (TUint32)aBase;
	Banks[BankCount++].iSize = (TUint32)(aEnd - aBase);
	}

// The kernel's RAM banks, ascending: the DTB's map, else [aLow, A64_BOOT_RAM_END)
static TBool RamMap(TUint64 aDtb, TUint32 aLow, const SA64IoRegion* aIo, TInt aIoCount)
	{
	const TBool dtb = ParseDtb(aDtb);
	if (dtb)
		{
		for (TInt i = 0; i < aIoCount; ++i)
			AddRange(Holes, HoleCount, aIo[i].iPhys, aIo[i].iSize);
		Sort(MemRanges, MemCount);
		Sort(Holes, HoleCount);
		for (TInt i = 0; i < MemCount; ++i)
			{
			TUint64 cur = MemRanges[i].iBase < aLow ? aLow : MemRanges[i].iBase;
			if (BankCount && cur < (TUint64)Banks[BankCount - 1].iBase + Banks[BankCount - 1].iSize)
				cur = (TUint64)Banks[BankCount - 1].iBase + Banks[BankCount - 1].iSize;	// overlapping /memory ranges
			const TUint64 end = MemRanges[i].iEnd > KPhysTop ? KPhysTop : MemRanges[i].iEnd;
			for (TInt h = 0; h < HoleCount && cur < end && Holes[h].iBase < end; ++h)
				if (Holes[h].iEnd > cur)
					{
					if (Holes[h].iBase > cur)
						AddBank(cur, Holes[h].iBase);
					cur = Holes[h].iEnd;
					}
			if (cur < end)
				AddBank(cur, end);
			}
		}
	if (BankCount)
		return ETrue;
	AddBank(aLow, KRamEnd);
	return EFalse;
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

extern "C" void A64_BootMain(TUint64 aDtb)
	{
	Put("\n[a64boot] EKA2 AArch64 bootstrap: flexible memory model, VMSAv8 4K tables\n");
	const TUint32 romLin	= SYMVAL(__rom_lin);
	const TUint32 romPhys	= SYMVAL(__rom_phys);
	const TUint32 romSize	= (SYMVAL(__rom_size) + KPageMask) & ~KPageMask;
	const TUint32 svData	= SYMVAL(__svdata_size);
	Trace("ROM phys", romPhys); Trace("ROM size", romSize); Trace("SvData size", svData);
#define ROM_PHYS(lin)	(romPhys + (SYMVAL(lin) - romLin))

	// ---- RAM: the banks above the first 2 MB boundary over the ROM image (so no SPageInfo block holds both bank
	// and ROM pages: Mmu::Init2Common would find ROM pages "in RAM" but in no zone)
	const TUint32 ramLow = (romPhys + romSize + KChunkMask) & ~(TUint32)KChunkMask;
	const SA64IoRegion* io = (const SA64IoRegion*)ROM_PHYS(A64_IoRegions);
	const TInt nio = *(const TInt*)ROM_PHYS(&A64_IoRegionCount);
	const TBool fromDtb = RamMap(aDtb, ramLow, io, nio);
	TUint32 totalRam = 0;
	for (TInt b = 0; b < BankCount; ++b)
		{
		totalRam += Banks[b].iSize;
		Trace("RAM bank base", Banks[b].iBase); Trace("RAM bank size", Banks[b].iSize);
		}
	Put(fromDtb ? "[a64boot] RAM map from the DTB\n" : "[a64boot] no usable DTB: RAM up to A64_BOOT_RAM_END\n");
	// the boot's own pages: the first bank big enough for the SPageInfo array (32 bytes a page) and the rest
	TInt allocBank = 0;
	while (allocBank < BankCount - 1 && Banks[allocBank].iSize < (totalRam >> 6) + 0x800000)
		++allocBank;
	BankBase = Banks[allocBank].iBase;
	BankEnd = BankBase + Banks[allocBank].iSize;
	AllocNext = BankBase;

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
	TUint32 heap = (totalRam >> (KPageShift - 1)) + 24 * 1024;		// ~2 bytes per RAM page + 24K
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
	for (TInt b = 0; b < BankCount; ++b)
		ram[b] = Banks[b];
	ram[BankCount].iBase = ram[BankCount].iSize = 0;
	ram[BankCount + 1].iBase = ram[BankCount + 1].iSize = 0;		// no reserved banks
	const TUint32 mapPhys = AllocAndMap(KPageInfoMap, KPageSize, KPermKernel);
	const TUint blockShift = KPageShift + KPageShift - KPageInfoShift;		// RAM covered by one SPageInfo page
	for (TInt b = 0; b < BankCount; ++b)
		for (TUint n = Banks[b].iBase >> blockShift; n <= (Banks[b].iBase + Banks[b].iSize - 1) >> blockShift; ++n)
			{
			volatile TUint8& bits = ((volatile TUint8*)mapPhys)[n >> 3];
			if (bits & (1u << (n & 7)))
				continue;							// a block shared with the bank before
			bits |= (TUint8)(1u << (n & 7));
			AllocAndMap(KPageInfoLinearBase + (n << KPageShift), KPageSize, KPermKernel);
			}

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
		TRomEntry			iExstartEntry;	TUint8 iPad5[32];
		TRomImageHeader		iExstart;
		TRomEntry			iSecondaryEntry; TUint8 iPad6[32];
		TRomImageHeader		iSecondary;
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
	rh.iUserDataAddress = KUserLocalDataEnd;
	// SMP-unsafe compatibility, as Symbian^3 shipped it: a process that loads an image without KImageSMPSafe (an old
	// .sis app, built for single-core phones) runs its threads one CPU at a time (Frozen Bubble's mixer thread raced
	// its creator on 8 cores). The build marks every image of ours SMP-safe.
	rh.iKernelConfigFlags = 1u << 12;		// EKernelConfigSMPUnsafeCompat (u32std.h)

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
	x.iNextExtension = REC_LIN(iExstartEntry);

	// the second extension is the real exstart (kernel/exstart.cpp, linked in): it starts the ROM's secondary process
	TRomImageHeader& xs = rec.iExstart;
	xs.iUid1 = 0x10000079u;
	xs.iEntryPoint = SYMVAL(A64ExstartShim);
	xs.iFlags = KRomImageFlagExtension;
	xs.iS.iCaps[0] = 0xFFFFFFFFu;  xs.iS.iCaps[1] = 0xFFFFFFFFu;
	xs.iHardwareVariant = 0x01000000u;
	rec.iExstartEntry.iAddressLin = REC_LIN(iExstart);
	rec.iExstartEntry.iNameLength = 7;
	{ const char* n = "exstart"; for (TInt i = 0; i < 7; ++i) rec.iExstartEntry.iName[i << 1] = (TUint8)n[i]; }

	// the secondary (efile.exe's slot): for now the AArch32 EL0 test program, XIP from the ROM's .utext. No data/bss,
	// no imports; exstart creates the process from this header (Epoc::RomProcessInfo) and resumes it.
	TRomImageHeader& u = rec.iSecondary;
	u.iUid1 = 0x1000007Au;										// EXE
	u.iEntryPoint = SYMVAL(A32Test);
	u.iCodeAddress = SYMVAL(A32Test);
	u.iCodeSize = u.iTextSize = SYMVAL(A32TestEnd) - SYMVAL(A32Test);
	u.iDataBssLinearBase = KUserLocalDataBase;
	u.iHeapSizeMin = 0x1000;  u.iHeapSizeMax = 0x100000;  u.iStackSize = 0x2000;
	u.iPriority = EPriorityHigh;
	u.iFlags = KRomImageEpt_Eka2;								// CHECK_ROM_ENTRY_POINT: an EKA2 entry point
	u.iS.iCaps[0] = 0xFFFFFFFFu;  u.iS.iCaps[1] = 0xFFFFFFFFu;
	u.iHardwareVariant = 0x01000000u;
	rec.iSecondaryEntry.iSize = u.iCodeSize;
	rec.iSecondaryEntry.iAddressLin = REC_LIN(iSecondary);
	rec.iSecondaryEntry.iNameLength = 9;
	{ const char* n = "efile.exe"; for (TInt i = 0; i < 9; ++i) rec.iSecondaryEntry.iName[i << 1] = (TUint8)n[i]; }
	rh.iSecondaryFile = REC_LIN(iSecondaryEntry);

	// an empty Z: root directory (iSize counts the TRomEntry region: 0 entries -> 0)
	rec.iRootList.iNumRootDirs = 1;
	rec.iRootList.iRootDir[0].iHardwareVariant = 0x01000000u;
	rec.iRootList.iRootDir[0].iAddressLin = REC_LIN(iRootDir);
	rec.iRootDir.iSize = 0;
	TLinAddr rootDirList = REC_LIN(iRootList);

	// ---- the user side: a prelinked ROM SECTION (tools/build-userrom.sh, pdk rombuild.py --section) holds real
	// TRomEntry + TRomImageHeader records for the 2010 user side, laid out for this address, and a Z: directory tree.
	// Its header {magic 'PRSC', version 4, base, size, count, rootDirList, secondary TRomEntry, ROM DLL data size,
	// built-in device table}: efile.exe there becomes the secondary, and its tree the ROM's root directory.
	if (SYMVAL(__uimage_size))
		{
		const TUint32* uh = (const TUint32*)ROM_PHYS(__uimage_base);
		if (uh[0] != 0x43535250u || uh[1] != 4)
			Die("the ROM section is not a version 4 'PRSC' section (tools/build-userrom.sh)");
		if (uh[2] != SYMVAL(__uimage_base))
			Die("the ROM section was laid out for a different linear address");
		if (uh[8] && *(const TUint32*)ROM_PHYS(uh[8]))
			Die("the ROM section lists built-in device drivers: not supported by this kernel");
		rh.iSecondaryFile = uh[6];
		rootDirList = uh[5];
		rh.iTotalUserDataSize = uh[7];						// ROM DLL static data below KUserLocalDataEnd
		Trace("user-side section images", uh[4]);
		Trace("ROM DLL static data", uh[7]);
		Put("[a64boot] secondary = efile.exe from the user-side ROM section\n");
		}
	else
		Put("[a64boot] no user-side section: secondary = boot/a32test.S\n");
	rh.iRomRootDirectoryList = rootDirList;

	// ---- super page
	SSuperPageBase& sp = *(SSuperPageBase*)superPhys;
	sp.iTotalRomSize = romSize;
	sp.iTotalRamSize = totalRam;
	sp.iPageDir = KPageDirectoryBase;
	sp.iRamBootData = KSuperPageLinAddr + KRamTableOffset;
	sp.iBootFlags = KBootFlagRunFromRAM;
	sp.iMachineData = KMachineConfigLinAddr;
	sp.iActiveVariant = 0x01000000u;
	sp.iPrimaryEntry = REC_LIN(iPrimaryEntry);
	sp.iInitialHeapSize = heap;
	sp.iDebugPort = 0;
	sp.iRootDirList = rootDirList;		// FindRomRootDirectory prefers this over the ROM header's
	sp.iKernelLimit = rh.iKernelLimit;
	sp.iRomHeaderPhys = romPhys;
	// the bootloader's ramdisk, kept out of the RAM banks above: iReserved[0] its physical base, [1] its size (0: none,
	// or above 4 GB); a64_medrofs.cpp maps it as Z:'s ROFS when it starts with the header this kernel carries
	if (fromDtb && InitrdEnd > InitrdStart && InitrdEnd <= KPhysTop)
		{
		sp.iReserved[0] = (TInt)(TUint32)InitrdStart;
		sp.iReserved[1] = (TInt)(TUint32)(InitrdEnd - InitrdStart);
		Trace("ramdisk phys", (TUint32)InitrdStart); Trace("ramdisk size", (TUint32)(InitrdEnd - InitrdStart));
		}

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
	sctlr |= (1u << 5);											// CP15BEN: AArch32 EL0 may use the ARMv6 CP15 barriers
	sctlr &= ~(TUint64)((1u << 1) | (1u << 19) | (1u << 25));	// no alignment checks, no WXN, little-endian EL1
	sctlr &= ~(TUint64)((1u << 7) | (1u << 8));					// ITD, SED: AArch32 EL0 keeps IT and SETEND
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
		"msr	cpacr_el1, %8\n\t"				// NeoSymbian WB1: FPEN, FP/SIMD for AArch32 EL0 (switched per thread)
		"msr	cntkctl_el1, %9\n\t"			// NeoSymbian WB2c: EL0VCTEN, EL0 reads CNTVCT/CNTFRQ (P.I.P.S. CLOCK_MONOTONIC)
		"isb\n\t"
		"mov	sp, %4\n\t"
		"mov	x0, %5\n\t"
		"mov	x1, %6\n\t"
		"mov	x29, xzr\n\t"
		"mov	x30, xzr\n\t"
		"br		%7"
		: : "r"(KA64MairValue), "r"(tcr), "r"((TUint64)rootPhys), "r"(sctlr), "r"((TUint64)initSp),
			"r"((TUint64)romLin), "r"((TUint64)KSuperPageLinAddr), "r"((TUint64)k.iEntryPoint), "r"(3ull << 20),
			"r"(1ull << 1)
		: "x0", "x1", "x29", "x30", "memory");
	Die("kernel entry returned");
	}
