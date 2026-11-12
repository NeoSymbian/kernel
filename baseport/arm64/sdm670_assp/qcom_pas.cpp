// SPDX-License-Identifier: Apache-2.0
//
// qcom_pas.cpp -- TrustZone's peripheral authentication service (PAS): the firmware's metadata to TZ (init_image),
// the carve-out declared (mem_setup, for relocatable images), the segments copied in, then auth_and_reset. Used for
// IPA's GSI firmware (telephony C5b, PAS 15, one image in memory) and the ADSP (C6, PAS 1, 17 MB streamed segment
// by segment from MODEMD: Pas_Begin, Pas_Segment..., Pas_End).
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

// one load at a time (IPA at MODEMD's start, the ADSP later): the program headers and where the image goes
static struct
	{
	TBool iBusy;
	TUint32 iPas, iBase, iPhys, iSize, iPhnum;
	SPhdr iPh[32];
	} L;

static void Done() { L.iBusy = EFalse; }

TInt Pas_Begin(TUint32 aPas, const TUint8* aMdt, TUint32 aMdtLen, TUint32 aPhys, TUint32 aSize)
	{
	if (L.iBusy)
		return KErrInUse;
	if (aMdtLen < 0x34 || memcompare(aMdt, 4, (const TUint8*)"\x7f" "ELF", 4) != 0)
		return KErrCorrupt;
	TUint32 phoff = *(const TUint32*)(aMdt + 28), phnum = *(const TUint16*)(aMdt + 44);
	if (phoff + phnum * sizeof(SPhdr) > aMdtLen || phnum > sizeof(L.iPh) / sizeof(L.iPh[0]))
		return KErrCorrupt;
	memcpy(L.iPh, aMdt + phoff, phnum * sizeof(SPhdr));
	const SPhdr* ph = L.iPh;
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
		memcpy(p, aMdt, aMdtLen);
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
	if (reloc)
		{
		args[1] = aPhys;
		args[2] = maxAddr - minAddr;
		Kern::Printf("[pas] %d: mem_setup %08x + %x", aPas, aPhys, maxAddr - minAddr);
		if ((r = Pil(EMemSetup, Args(3), args, 3, "mem_setup")) != KErrNone)
			return r;
		}
	L.iBusy = ETrue;
	L.iPas = aPas;
	L.iBase = reloc ? minAddr : aPhys;
	L.iPhys = aPhys;
	L.iSize = aSize;
	L.iPhnum = phnum;
	return KErrNone;
	}

// a window over [aPhys, aPhys + aLen) in the carve-out, mapped for this one copy
static TInt Copy(TUint32 aPhys, const TUint8* aData, TUint32 aLen)
	{
	TUint32 page = aPhys & ~0xfffu;
	DPlatChunkHw* c;
	TUint8* p = Map(page, aPhys - page + aLen, c);
	if (!p)
		return KErrNoMemory;
	if (aData)
		memcpy(p + (aPhys - page), aData, aLen);
	else
		memclr(p + (aPhys - page), aLen);
	Unmap(c);
	return KErrNone;
	}

TInt Pas_Segment(TUint32 aIndex, TUint32 aOffset, const TUint8* aData, TUint32 aLen)
	{
	if (!L.iBusy || aIndex >= L.iPhnum || !Loadable(L.iPh[aIndex]))
		return KErrArgument;
	const SPhdr& h = L.iPh[aIndex];
	TUint32 off = h.iPaddr - L.iBase;
	if (h.iPaddr < L.iBase || off + h.iMemsz > L.iSize || h.iFilesz > h.iMemsz || aOffset > h.iFilesz
		|| aLen > h.iFilesz - aOffset)
		return KErrCorrupt;
	return Copy(L.iPhys + off + aOffset, aData, aLen);
	}

TInt Pas_End()
	{
	if (!L.iBusy)
		return KErrNotReady;
	TInt r = KErrNone;
	for (TUint32 i = 0; i < L.iPhnum && r == KErrNone; ++i)	// each segment's tail past its file data is zeros
		{
		const SPhdr& h = L.iPh[i];
		if (Loadable(h) && h.iMemsz > h.iFilesz)
			for (TUint32 at = h.iFilesz; at < h.iMemsz && r == KErrNone; at += 0x100000)
				r = Copy(L.iPhys + h.iPaddr - L.iBase + at, NULL, Min(h.iMemsz - at, 0x100000u));
		}
	TUint64 args[1] = { L.iPas };
	if (r == KErrNone)
		{
		// TZ checks the segments against the hash and starts the peripheral
		Kern::Printf("[pas] %d: segments copied; auth_and_reset", L.iPas);
		r = Pil(EAuthReset, Args(1), args, 1, "auth_and_reset");
		Kern::Printf("[pas] %d: in %08x -> %d", L.iPas, L.iPhys, r);
		}
	Done();
	return r;
	}

void Pas_Abort() { Done(); }

TInt Pas_Load(TUint32 aPas, const TUint8* aImage, TUint32 aLen, TUint32 aMdtLen, TUint32 aPhys, TUint32 aSize)
	{
	if (aMdtLen > aLen)
		return KErrCorrupt;
	TInt r = Pas_Begin(aPas, aImage, aMdtLen, aPhys, aSize);
	for (TUint32 i = 0; r == KErrNone && i < L.iPhnum; ++i)
		{
		const SPhdr& h = L.iPh[i];
		if (!Loadable(h))
			continue;
		r = h.iOffset + h.iFilesz > aLen ? KErrCorrupt : Pas_Segment(i, 0, aImage + h.iOffset, h.iFilesz);
		}
	if (r == KErrNone)
		return Pas_End();
	if (L.iBusy)
		Pas_Abort();
	return r;
	}

// Linux qcom_scm_pas_shutdown: PIL service 2, command 6, one value argument (peripheral ID).
TInt Pas_Shutdown(TUint32 aPas)
	{
	if (L.iBusy)
		{
		if (L.iPas != aPas)
			return KErrInUse;
		Done();
		}
	TUint64 args[1] = { aPas };
	return Pil(6, Args(1), args, 1, "shutdown");
	}
