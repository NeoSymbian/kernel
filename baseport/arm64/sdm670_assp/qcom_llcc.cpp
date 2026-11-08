// SPDX-License-Identifier: Apache-2.0
//
// qcom_llcc.cpp -- the SDM670's last-level system cache (DTB system-cache-controller@1100000, "qcom,sdm670-llcc",
// LLCC v1): each client (CPUs, GPU, modem, audio, ...) gets a slice; its attributes (capacity, priority, ways) are
// written through the broadcast base to every bank, and some slices are activated at boot. Linux does this at probe
// on sdm670 (only the secure registers are skipped there, skip_llcc_cfg); without it the modem's slice (8) has no
// capacity. Telephony C4: the modem's RF task faulted right after ready while Linux, which had programmed the LLCC,
// ran it fine.
// Written from the slice table and sequence in Linux drivers/soc/qcom/llcc-qcom.c (sdm670_data,
// _qcom_llcc_cfg_program, llcc_update_act_ctrl).
#include <kernel/kern_priv.h>
#include <platform.h>
#include "board.h"
#include "qcom_llcc.h"

const TUint32 KLlcc0 = 0x01100000, KBroadcast = 0x01300000, KBroadcastSize = 0x50000;
const TUint32 KCommonStatus0 = 0x3000c;
const TUint32 KActivate = 1, KDeactivate = 2;

struct SSlice { TUint8 iId; TUint16 iMaxCapKB; TUint8 iPriority, iFixed, iMode; TUint16 iBonus, iRes; TBool iActivate; };
static const SSlice KSlices[] = {		// Linux sdm670_data (probe_target_ways 0 for all)
	{ 1, 512, 1, 0, 0, 0xf, 0x0, ETrue },		// CPUSS
	{ 4, 384, 2, 1, 2, 0x0, 0xe, EFalse },		// ROTATOR
	{ 5, 512, 1, 0, 0, 0xf, 0x0, EFalse },		// VOICE
	{ 6, 512, 1, 0, 0, 0xf, 0x0, EFalse },		// AUDIO
	{ 8, 512, 1, 0, 0, 0xf, 0x0, EFalse },		// MDM (the modem)
	{ 12, 384, 1, 1, 0, 0x0, 0x0, EFalse },		// GPU
	{ 13, 512, 1, 0, 0, 0xf, 0x0, ETrue },		// MMUHWT
	{ 22, 512, 1, 1, 0, 0xf, 0x0, EFalse },		// AUDHW
	};

static TBool Done;

static TUint8* MapRegion(TUint32 aPhys, TUint32 aSize)
	{
	DPlatChunkHw* c = NULL;
	NKern::ThreadEnterCS();
	TInt r = DPlatChunkHw::New(c, aPhys, aSize, EMapAttrSupRw | EMapAttrFullyBlocking);
	NKern::ThreadLeaveCS();
	return r == KErrNone ? (TUint8*)c->LinearAddress() : NULL;	// kept mapped: a one-off at boot
	}

TInt Llcc_Init()
	{
	if (Done)
		return KErrNone;
	TUint8* b = MapRegion(KBroadcast, KBroadcastSize);
	TUint8* l0 = MapRegion(KLlcc0 + (KCommonStatus0 & ~0xfffu), 0x1000);
	if (!b || !l0)
		return KErrNoMemory;
	TUint32 banks = *(volatile TUint32*)(l0 + (KCommonStatus0 & 0xfff)) >> 28;
	if (!banks)
		banks = 2;												// llcc0 + llcc1 in the DTB
	for (TUint i = 0; i < sizeof(KSlices) / sizeof(KSlices[0]); ++i)
		{
		const SSlice& s = KSlices[i];
		TUint32 lines = (TUint32)s.iMaxCapKB * 1024 / banks >> 6;	// 64-byte cache lines per bank
		TUint32 attr1 = s.iMode | (TUint32)s.iFixed << 3 | (TUint32)s.iPriority << 4 | lines << 16;
		TUint32 attr0 = s.iRes | (TUint32)s.iBonus << 16;
		*(volatile TUint32*)(b + 0x21004 + 8 * s.iId) = attr1;
		*(volatile TUint32*)(b + 0x21000 + 8 * s.iId) = attr0;
		}
	TInt failed = 0;
	for (TUint i = 0; i < sizeof(KSlices) / sizeof(KSlices[0]); ++i)
		{
		const SSlice& s = KSlices[i];
		if (!s.iActivate)
			continue;
		volatile TUint32* ctrl = (volatile TUint32*)(b + 0x1000 * s.iId);
		volatile TUint32* status = (volatile TUint32*)(b + 0x1000 * s.iId + 4);
		*ctrl = KActivate << 1 | 1;								// opcode activate + trigger
		*ctrl = KActivate << 1;									// trigger cleared
		TInt n = 0;
		while ((*status & KDeactivate) && ++n < 100)
			Kern::NanoWait(1000);
		failed += n >= 100;
		}
	Done = ETrue;
	Kern::Printf("[llcc] %d banks, %d slices programmed, activation failures %d", banks,
		(TInt)(sizeof(KSlices) / sizeof(KSlices[0])), failed);
	return failed ? KErrTimedOut : KErrNone;
	}
