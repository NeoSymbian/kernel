// SPDX-License-Identifier: Apache-2.0
//
// qcom_smmu.cpp -- the apps SMMU (DTB iommu@15000000, "qcom,sdm670-smmu-500", Arm MMU-500): one context bank that
// maps IOVA == PA, for a device whose streams fault today (unmatched streams fault: sCR0.USFCFG = 1). Telephony C5b:
// IPA (streams 0x720/0x722) reaches our rings, SMEM item 497 and IMEM through it. The hypervisor lets us write the
// stream-match registers (the monitor's "smmu probe", I0), but it only routes exact matches (mask 0): a masked
// match was ignored, and IPA's reads faulted elsewhere (C5b).
// Written from the Arm SMMUv2 architecture (register map, AArch64 context format, VMSAv8-64 descriptors); the
// sequence follows Linux arm-smmu.c (context bank, then S2CR, then SMR).
// shortcut: 1 GB blocks over the low 4 GB, no per-buffer mapping (IPA sees all of RAM); a proper IOMMU API comes
// with more devices.
#include <kernel/kern_priv.h>
#include <platform.h>
#include "board.h"
#include "qcom_smmu.h"

const TUint32 KSmmu = 0x15000000;
// GR0
enum { ECr0 = 0x000, EIdr0 = 0x020, EIdr1 = 0x024, ETlbiAllNsnh = 0x068, ETlbGSync = 0x070, ETlbGStatus = 0x074,
	ESmr = 0x800, ES2cr = 0xc00 };
// GR1 (one page up)
enum { ECbar = 0x000, ECba2r = 0x800 };
// a context bank
enum { ESctlr = 0x000, EActlr = 0x004, ETcr2 = 0x010, ETtbr0 = 0x020, ETcr = 0x030, EMair0 = 0x038, EMair1 = 0x03c,
	EFsr = 0x058, EFar = 0x060, EFsynr0 = 0x068, ETlbiAll = 0x618, ETlbSync = 0x7f0, ETlbStatus = 0x7f4 };

static TUint32 Page, NumPage;
static TInt Cb = -1;
static volatile TUint64* Table;
static TPhysAddr Tbl;

static volatile TUint32& G0(TUint32 aOff) { return *(volatile TUint32*)A64_Io(KSmmu + aOff); }
static volatile TUint32& G1(TUint32 aOff) { return *(volatile TUint32*)A64_Io(KSmmu + Page + aOff); }
static volatile TUint32& C(TInt aCb, TUint32 aOff)
	{
	return *(volatile TUint32*)A64_Io(KSmmu + NumPage * Page + aCb * Page + aOff);
	}

static void Wait(volatile TUint32& aStatus)
	{
	for (TInt i = 0; (aStatus & 1) && i < 100000; ++i)
		Kern::NanoWait(1000);
	}

// the context bank: an identity stage-1 map of the low 4 GB (IOVA == PA, normal non-cacheable), laid out and
// programmed as Linux's working context for IPA on this phone (C5b, read live from C1): T0SZ 16, so the walk starts
// at level 0 with a table descriptor. A level-1-start table (T0SZ 32, 1 GB blocks at the top) faulted at level 0.
static TInt BankSetup()
	{
	NKern::ThreadEnterCS();
	TInt r = Epoc::AllocPhysicalRam(0x2000, Tbl);				// level 0, then level 1
	DPlatChunkHw* chunk = NULL;
	if (r == KErrNone)
		r = DPlatChunkHw::New(chunk, Tbl, 0x2000, EMapAttrSupRw | EMapAttrBufferedNC);
	NKern::ThreadLeaveCS();
	if (r != KErrNone)
		return r;
	volatile TUint64* l0 = (volatile TUint64*)chunk->LinearAddress();
	volatile TUint64* l1 = l0 + 512;
	for (TInt i = 0; i < 1024; ++i)
		l0[i] = 0;
	l0[0] = (Tbl + 0x1000) | 3;									// table descriptor -> level 1
	// block: valid (bit 0, bit 1 clear), AttrIndx 0 (MAIR attr 0), AP 01 (read/write at any level), SH 10 (outer),
	// AF (bit 10)
	const TUint64 KBlock = 1 | (0ull << 2) | (1ull << 6) | (2ull << 8) | (1ull << 10);
	for (TUint64 i = 0; i < 4; ++i)
		l1[i] = (i << 30) | KBlock;
	Table = l0;
	__e32_memory_barrier();

	// the bank: AArch64 format, stage 1 with stage 2 bypassed (the hypervisor nests its own stage 2)
	G1(ECba2r + 4 * Cb) = 1;									// VA64
	G1(ECbar + 4 * Cb) = (0xfu << 12) | (1u << 16) | (3u << 8);	// TYPE S1 + S2 bypass, BPSHCFG NSH, MEMATTR WB (weakest)
	C(Cb, ESctlr) = 0;
	C(Cb, EActlr) = 1;											// Linux's (MMU-500 next-page prefetch off)
	C(Cb, ETcr2) = 0x00038075;									// Linux's: PASize 48 bits, 16-bit ASID, SEP upstream
	*(volatile TUint64*)&C(Cb, ETtbr0) = Tbl;					// ASID 0
	C(Cb, ETcr) = 16 | (1u << 23);								// T0SZ 16, 4 KB granule, walks NC, EPD1 (no TTBR1)
	C(Cb, EMair0) = 0xf404ff44;									// Linux's: attr0 normal non-cacheable (ours), WB, device
	C(Cb, EMair1) = 0;
	C(Cb, EFsr) = 0xffffffff;									// clear stale faults
	C(Cb, ETlbiAll) = 0;
	C(Cb, ETlbSync) = 0;
	Wait(C(Cb, ETlbStatus));
	C(Cb, ESctlr) = 1 | (1u << 1) | (1u << 2) | (1u << 5);		// M, TRE, AFE, CFRE (status, no IRQ)
	__e32_memory_barrier();
	return KErrNone;
	}

TInt Smmu_IdentityMap(TUint32 aId, TUint32 aMask)
	{
	TBool fresh = Cb < 0;
	TUint32 idr0 = G0(EIdr0), idr1 = G0(EIdr1);
	Page = (idr1 >> 31) ? 0x10000 : 0x1000;
	NumPage = 1u << (((idr1 >> 28) & 7) + 1);
	TInt smrs = idr0 & 0xff, cbs = idr1 & 0xff;
	// which context banks the existing stream matches use, and a free SMR
	TUint64 used = 0;
	TInt freeSmr = -1;
	for (TInt n = 0; n < smrs; ++n)
		{
		TUint32 smr = G0(ESmr + 4 * n);
		if (!(smr >> 31))
			{
			freeSmr = n;										// the last free one
			continue;
			}
		TUint32 id = smr & 0x7fff, mask = (smr >> 16) & 0x7fff;
		if (!fresh && id == aId && mask == aMask && (G0(ES2cr + 4 * n) & 0xff) == (TUint32)Cb)
			return KErrNone;									// ours already (a retry)
		if (!((id ^ aId) & ~(mask | aMask) & 0x7fff))
			return KErrInUse;									// someone already routes these streams
		TUint32 s2cr = G0(ES2cr + 4 * n);
		if (((s2cr >> 16) & 3) == 0)
			used |= 1ull << (s2cr & 0x3f);
		}
	for (TInt cb = cbs - 1; cb >= 0 && Cb < 0; --cb)			// the highest bank nobody routes to and not enabled
		if (!(used >> cb & 1) && !(C(cb, ESctlr) & 1))
			Cb = cb;
	if (Cb < 0 || freeSmr < 0)
		return KErrOverflow;
	if (fresh)
		{
		TInt r = BankSetup();
		if (r != KErrNone)
			{
			Cb = -1;
			return r;
			}
		}

	// then the stream routing: S2CR (translate through Cb), SMR valid last
	G0(ES2cr + 4 * freeSmr) = (TUint32)Cb;
	G0(ESmr + 4 * freeSmr) = (1u << 31) | ((aMask & 0x7fff) << 16) | (aId & 0x7fff);
	G0(ETlbiAllNsnh) = 0;
	G0(ETlbGSync) = 0;
	Wait(G0(ETlbGStatus));
	// read back: under the hypervisor a refused write would show here (C5b: GSI's reads missed our context)
	Kern::Printf("[smmu] streams %x/%x -> SMR %d, context %d: identity, table at %08x; read back smr %08x s2cr %08x "
		"cbar %08x sctlr %08x tcr %08x tcr2 %08x ttbr0 %08x; l0[0] %08x l1[2] %08x", aId, aMask, freeSmr, Cb,
		(TUint32)Tbl, G0(ESmr + 4 * freeSmr), G0(ES2cr + 4 * freeSmr), G1(ECbar + 4 * Cb), C(Cb, ESctlr), C(Cb, ETcr),
		C(Cb, ETcr2), C(Cb, ETtbr0), (TUint32)Table[0], (TUint32)Table[512 + 2]);
	return KErrNone;
	}

void Smmu_GlobalFault(TUint32& aGfsr, TUint64& aGfar, TUint32& aGfsynr0)
	{
	aGfsr = G0(0x048);
	aGfar = *(volatile TUint64*)&G0(0x040);
	aGfsynr0 = G0(0x050);
	}

TInt Smmu_Fault(TUint32& aFsr, TUint64& aFar, TUint32& aFsynr)
	{
	if (Cb < 0)
		return KErrNotReady;
	aFsr = C(Cb, EFsr);
	aFar = *(volatile TUint64*)&C(Cb, EFar);
	aFsynr = C(Cb, EFsynr0);
	return KErrNone;
	}
