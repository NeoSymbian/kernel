// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// SPDX-License-Identifier: EPL-1.0
//
// a64e_cache.cpp -- the InternalCache primitives of kernel/arm/cache_maintenance{,v7}.cia for AArch64. The C++
// policy above them (kernel/arm/cache_maintenance.cpp, compiled unmodified) uses the ARMv7 model, which ARMv8 keeps:
// CTR/CLIDR/CCSIDR have the v7 layout (no FEAT_CCIDX), maintenance by VA to the PoC/PoU, set/way for "all".
// VA-based maintenance is broadcast in the Inner Shareable domain by hardware (no __BROADCAST_CACHE_MAINTENANCE__).
#include <kernel/kern_priv.h>
#include <kernel/cache_maintenance.h>

static inline TUint32 DLine()	{ TUint64 c; __asm__ __volatile__("mrs %0, ctr_el0" : "=r"(c)); return 4u << ((c >> 16) & 0xF); }

#define A64_DC_RANGE(op, aBase, aSize)											\
	do {																		\
		TUint32 line = DLine();													\
		TLinAddr a = (aBase) & ~(TLinAddr)(line - 1), e = (aBase) + (aSize);	\
		for (; a < e; a += line)												\
			__asm__ __volatile__("dc " op ", %0" :: "r"((TUint64)a) : "memory");	\
		__asm__ __volatile__("dsb ish" ::: "memory");							\
	} while (0)

TUint32 InternalCache::TypeRegister()
	{
	TUint64 v; __asm__ __volatile__("mrs %0, ctr_el0" : "=r"(v)); return (TUint32)v;
	}

TUint32 InternalCache::LevelIDRegister()
	{
	TUint64 v; __asm__ __volatile__("mrs %0, clidr_el1" : "=r"(v)); return (TUint32)v;
	}

// aType 0 = data/unified, 1 = instruction; aLevel 0-based (CSSELR_EL1 = level<<1 | InD)
TUint32 InternalCache::SizeIdRegister(TUint32 aType, TUint32 aLevel)
	{
	TUint64 v;
	TInt irq = NKern::DisableAllInterrupts();		// CSSELR/CCSIDR pair must not be split by a migration
	__asm__ __volatile__("msr csselr_el1, %1\n\tisb\n\tmrs %0, ccsidr_el1" : "=r"(v) : "r"((TUint64)((aLevel<<1)|(aType&1))) : "memory");
	NKern::RestoreInterrupts(irq);
	return (TUint32)v;
	}

void InternalCache::DrainBuffers()
	{
	__asm__ __volatile__("dsb sy" ::: "memory");
	}

void InternalCache::Clean_DCache_Region(TLinAddr aBase, TUint aSize)				{ A64_DC_RANGE("cvac", aBase, aSize); }
void InternalCache::Clean_PoU_DCache_Region(TLinAddr aBase, TUint aSize)			{ A64_DC_RANGE("cvau", aBase, aSize); }
void InternalCache::Invalidate_DCache_Region(TLinAddr aBase, TUint aSize)			{ A64_DC_RANGE("ivac", aBase, aSize); }
void InternalCache::CleanAndInvalidate_DCache_Region(TLinAddr aBase, TUint aSize)	{ A64_DC_RANGE("civac", aBase, aSize); }

// An instruction cache may be VIPT (CTR_EL0.L1Ip = 2), where invalidating by one VA can miss an alias of the same
// physical code: invalidate it all, in every core of the Inner Shareable domain. Code changes are rare.
void InternalCache::Invalidate_ICache_All()
	{
	__asm__ __volatile__("dsb ish\n\tic ialluis\n\tdsb ish\n\tisb" ::: "memory");
	}

void InternalCache::Invalidate_ICache_Region(TLinAddr, TUint)
	{
	Invalidate_ICache_All();
	}

// Make one modified line of code executable: clean the D line to the PoU, then drop stale instructions
void InternalCache::IMB_CacheLine(TLinAddr aAddr)
	{
	__asm__ __volatile__("dc cvau, %0" :: "r"((TUint64)aAddr) : "memory");
	Invalidate_ICache_All();
	}

// Set/way clean+invalidate of every data/unified cache level up to the Level of Coherency (this CPU only)
void InternalCache::CleanAndInvalidate_DCache_All()
	{
	TInt irq = NKern::DisableAllInterrupts();
	TUint64 clidr;
	__asm__ __volatile__("mrs %0, clidr_el1" : "=r"(clidr));
	TUint32 loc = (TUint32)(clidr >> 24) & 7;
	for (TUint32 level = 0; level < loc; ++level)
		{
		TUint32 ctype = (TUint32)(clidr >> (level * 3)) & 7;
		if (ctype < 2)
			continue;										// no data or unified cache at this level
		TUint64 ccsidr;
		__asm__ __volatile__("msr csselr_el1, %1\n\tisb\n\tmrs %0, ccsidr_el1" : "=r"(ccsidr) : "r"((TUint64)(level << 1)) : "memory");
		TUint32 lineShift = (TUint32)(ccsidr & 7) + 4;
		TUint32 ways = (TUint32)((ccsidr >> 3) & 0x3FF);	// ways - 1
		TUint32 sets = (TUint32)((ccsidr >> 13) & 0x7FFF);	// sets - 1
		TUint32 wayShift = ways ? __builtin_clz(ways) : 0;
		for (TUint32 w = 0; w <= ways; ++w)
			for (TUint32 s = 0; s <= sets; ++s)
				{
				TUint64 op = ((TUint64)w << wayShift) | ((TUint64)s << lineShift) | (level << 1);
				__asm__ __volatile__("dc cisw, %0" :: "r"(op) : "memory");
				}
		}
	__asm__ __volatile__("dsb sy\n\tisb" ::: "memory");
	NKern::RestoreInterrupts(irq);
	}

// ARMv7 PRRR/NMRR describing our MAIR_EL1 (KA64MairValue in mmboot.h) so TypeToCachingAttributes decodes the
// memory types 4-7 correctly. PRRR.TRn (bits 2n+1:2n): 0 strongly ordered, 1 device, 2 normal.
// NMRR.IRn / ORn (bits 2n+1:2n / 16+2n+1:16+2n): 0 non-cacheable, 1 WB-WA, 2 WT, 3 WB no-WA.
//   0 SO, 1 device, 2 normal NC, 3 WB-WA, 4 WB-WA, 5 WT, 6 WB no-WA, 7 WB-WA
TUint32 InternalCache::PrimaryRegionRemapRegister()
	{
	return (0u<<0) | (1u<<2) | (2u<<4) | (2u<<6) | (2u<<8) | (2u<<10) | (2u<<12) | (2u<<14);
	}

TUint32 InternalCache::NormalMemoryRemapRegister()
	{
	const TUint32 ir = (1u<<6) | (1u<<8) | (2u<<10) | (3u<<12) | (1u<<14);
	return ir | (ir << 16);
	}

#if defined(__SMP__) && !defined(__BROADCAST_CACHE_MAINTENANCE__)
void T_ISB_IPI::ISBIsr(TGenericIPI* aPtr)
	{
	__asm__ __volatile__("isb" ::: "memory");
	(void)aPtr;
	}
#endif
