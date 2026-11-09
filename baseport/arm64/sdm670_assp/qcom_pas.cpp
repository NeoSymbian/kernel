// SPDX-License-Identifier: Apache-2.0
//
// qcom_pas.cpp -- TrustZone's peripheral authentication service (PAS): the firmware's metadata to TZ (init_image),
// the carve-out declared (mem_setup, for relocatable images), the segments copied in, then auth_and_reset. Used for
// IPA's GSI firmware (telephony C5b, PAS 15); the ADSP (C6) is next.
// Written from the sequence in Linux drivers/soc/qcom/mdt_loader.c (__qcom_mdt_load) and qcom_scm.c (the PIL calls:
// SCM service 2, init_image 1, mem_setup 2, auth_and_reset 5, is_supported 7).
#include <kernel/kern_priv.h>
#include <platform.h>
#include "board.h"
#include "qcom_scm.h"
#include "qcom_pas.h"

enum { ESvcPil = 2, EInitImage = 1, EMemSetup = 2, EAuthReset = 5, EIsSupported = 7 };
const TUint32 KArgVal = 0, KArgRw = 2;						// Linux QCOM_SCM_VAL, QCOM_SCM_RW
static TUint32 Args(TUint32 aCount, TUint32 aT0 = 0, TUint32 aT1 = 0) { return aCount | aT0 << 4 | aT1 << 6; }

struct SPhdr { TUint32 iType, iOffset, iVaddr, iPaddr, iFilesz, iMemsz, iFlags, iAlign; };
const TUint32 KTypeMask = 7u << 24, KTypeHash = 2u << 24, KRelocatable = 1u << 27;
static TBool Loadable(const SPhdr& aP) { return aP.iType == 1 && (aP.iFlags & KTypeMask) != KTypeHash && aP.iMemsz; }

static TUint8* Map(TUint32 aPhys, TUint32 aSize, DPlatChunkHw*& aChunk)
	{
	aChunk = NULL;
	NKern::ThreadEnterCS();
	TInt r = DPlatChunkHw::New(aChunk, aPhys, (aSize + 0xfff) & ~0xfffu, EMapAttrSupRw | EMapAttrBufferedNC);
	NKern::ThreadLeaveCS();
	return r == KErrNone ? (TUint8*)aChunk->LinearAddress() : NULL;
	}

static void Unmap(DPlatChunkHw* aChunk)
	{
	__e32_memory_barrier();
	NKern::ThreadEnterCS();
	if (aChunk)
		aChunk->Close(NULL);
	NKern::ThreadLeaveCS();
	}

static TInt Pil(TUint aCmd, TUint32 aArgInfo, const TUint64* aArgs, TInt aCount, const char* aWhat)
	{
	TUint64 res[3] = { 0, 0, 0 };
	TInt r = Scm_Call(ESvcPil, aCmd, aArgInfo, aArgs, aCount, res);
	if (r == KErrNone && res[0])
		r = KErrGeneral;									// the call ran; TZ refused
	if (r != KErrNone)
		Kern::Printf("[pas] %s failed: %d (x0 %d, x1 %d)", aWhat, r, r == KErrGeneral && !res[0] ? (TInt)Scm_LastError : 0,
			(TInt)res[0]);
	return r;
	}

TInt Pas_Load(TUint32 aPas, const TUint8* aImage, TUint32 aLen, TUint32 aMdtLen, TUint32 aPhys, TUint32 aSize)
	{
	if (aLen < 0x34 || aMdtLen > aLen || memcompare(aImage, 4, (const TUint8*)"\x7f" "ELF", 4) != 0)
		return KErrCorrupt;
	TUint32 phoff = *(const TUint32*)(aImage + 28), phnum = *(const TUint16*)(aImage + 44);
	if (phoff + phnum * sizeof(SPhdr) > aMdtLen)
		return KErrCorrupt;
	const SPhdr* ph = (const SPhdr*)(aImage + phoff);
	TUint64 args[3] = { aPas, 0, 0 };
	TUint64 res[3] = { 0, 0, 0 };
	// is_supported: logged only. Linux's IPA path never asks, and this TZ answers it with an odd error (0xffcfffc0)
	TInt sup = Scm_Call(ESvcPil, EIsSupported, Args(1), args, 1, res);
	Kern::Printf("[pas] %d: is_supported -> %d (%d)", aPas, sup, (TInt)res[0]);
	TUint32 minAddr = 0xffffffff, maxAddr = 0;
	TBool reloc = EFalse;
	for (TUint32 i = 0; i < phnum; ++i)
		if (Loadable(ph[i]))
			{
			minAddr = Min(minAddr, ph[i].iPaddr);
			maxAddr = Max(maxAddr, (ph[i].iPaddr + ph[i].iMemsz + 0xfff) & ~0xfffu);	// 4 KB up, as mdt_loader
			reloc |= (ph[i].iFlags & KRelocatable) != 0;
			}
	if (minAddr > maxAddr || maxAddr - minAddr > aSize)
		return KErrTooBig;

	// 1. the metadata, in physically contiguous memory TZ reads (the .mdt exactly: headers + hash segment)
	TPhysAddr md;
	TUint32 mdSize = (aMdtLen + 0xfff) & ~0xfffu;
	NKern::ThreadEnterCS();
	TInt r = Epoc::AllocPhysicalRam(mdSize, md);
	NKern::ThreadLeaveCS();
	if (r != KErrNone)
		return r;
	DPlatChunkHw* c;
	TUint8* p = Map(md, mdSize, c);
	if (p)
		{
		memcpy(p, aImage, aMdtLen);
		Unmap(c);
		args[1] = md;
		Kern::Printf("[pas] %d: init_image, metadata %d bytes at %08x", aPas, aMdtLen, (TUint32)md);
		r = Pil(EInitImage, Args(2, KArgVal, KArgRw), args, 2, "init_image");
		}
	else
		r = KErrNoMemory;
	NKern::ThreadEnterCS();
	Epoc::FreePhysicalRam(md, mdSize);
	NKern::ThreadLeaveCS();
	if (r != KErrNone)
		return r;

	// 2. a relocatable image: tell TZ where it goes
	TUint32 base = reloc ? minAddr : aPhys;
	if (reloc)
		{
		args[1] = aPhys;
		args[2] = maxAddr - minAddr;
		Kern::Printf("[pas] %d: mem_setup %08x + %x", aPas, aPhys, maxAddr - minAddr);
		if ((r = Pil(EMemSetup, Args(3), args, 3, "mem_setup")) != KErrNone)
			return r;
		}

	// 3. the segments
	if ((p = Map(aPhys, aSize, c)) == NULL)
		return KErrNoMemory;
	for (TUint32 i = 0; i < phnum && r == KErrNone; ++i)
		{
		const SPhdr& h = ph[i];
		if (!Loadable(h))
			continue;
		TUint32 off = h.iPaddr - base;
		if (h.iPaddr < base || off + h.iMemsz > aSize || h.iFilesz > h.iMemsz || h.iOffset + h.iFilesz > aLen)
			r = KErrCorrupt;
		else
			{
			memcpy(p + off, aImage + h.iOffset, h.iFilesz);
			memclr(p + off + h.iFilesz, h.iMemsz - h.iFilesz);
			}
		}
	Unmap(c);
	if (r != KErrNone)
		return r;

	// 4. TZ checks the segments against the hash and starts the peripheral
	Kern::Printf("[pas] %d: segments copied; auth_and_reset", aPas);
	r = Pil(EAuthReset, Args(1), args, 1, "auth_and_reset");
	Kern::Printf("[pas] %d: %d bytes in %08x..%08x%s -> %d", aPas, maxAddr - minAddr, aPhys, aPhys + maxAddr - minAddr,
		reloc ? " (relocated)" : "", r);
	return r;
	}
