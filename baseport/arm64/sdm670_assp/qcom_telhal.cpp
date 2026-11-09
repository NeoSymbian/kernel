// SPDX-License-Identifier: Apache-2.0
//
// qcom_telhal.cpp -- what the user-side modem daemon (MODEMD.EXE, NeoSymbian/telephony) needs from the kernel,
// as NeoSymbian EHalGroupVariant functions (docs/telephony.md C4, docs/rust-integration.md HAL table). All need
// CommDD (ROM EXEs have every capability). SIo = { TUint32 a, b, len, buf } in user memory.
//   0x4e0d ModemBoot   boot the modem (qcom_mss.cpp; its own thread)
//   0x4e0e ModemState  a1 = TUint32[4] out: boot step (8 = running), error, SMP2P slave-kernel bits, IPCRTR open
//   0x4e0f QrtrSend    a1 = the packet, a2 = its length (<= 8 KB) -> GLINK IPCRTR; KErrNotReady without an intent
//   0x4e10 QrtrRecv    a1 = buffer, a2 = its size -> the next queued packet's length, KErrNotReady if none
//   0x4e11 PartRead    a1 = SIo { partition 0..4 = modemst1, modemst2, fsg, fsc, vendor_a; byte offset; len; buf },
//                      read-only, sector-aligned (vendor_a: the IPA/ADSP firmware, through MODEMD's ext4 reader)
//   0x4e12 RmtfsMem    a1 = SIo { op 0 info (b = phys, len = size out), 1 read, 2 write; offset; len; buf }: the
//                      rmtfs carve-out the modem reads its EFS through (DTB rmtfs-mem@fc201000, VMID 15)
//   0x4e13 IpaStart    a1 = SIo { .mdt length; 0; image length; image }: IPA's GSI firmware; the kernel brings IPA up
//                      (qcom_ipa.cpp: power, PAS 15, SMMU, config, GSI). MODEMD calls it before it boots the modem
//   0x4e14 AdspLoad    a1 = SIo { op | segment << 8; offset; len; buf }: the ADSP's firmware through TrustZone PAS 1
//                      (qcom_adsp.cpp): op 0 = the .mdt (QMP, SMP2P, init_image), 1 = len bytes of program header
//                      `segment` at `offset`, 2 = done (auth_and_reset, then ready/GLINK on the ADSP's thread)
//   0x4e15 AdspState   a1 = TUint32[4] out: step (4 = running), error, SMP2P slave-kernel bits, open channels mask
//   0x4e16 ChanSend    a1 = SIo { channel 1 = the ADSP's IPCRTR, 2 = APR (apr_audio_svc); 0; len; buf }
//   0x4e17 ChanRecv    a1 = SIo { channel; 0; size; buf } -> the next queued message's length, KErrNotReady if none
//   0x4e18 Audio       a1 = SIo { op; arg; 0; 0 }: the board's codecs and amps (A64Audio_Control, a64_audio.cpp)
// The receive paths are polled by the daemon; messages wait in a 64 KB ring per channel filled by the GLINK thread.
// shortcut: HAL polling, not an LDD with async notification; that comes with power management (C12).
#include <kernel/kern_priv.h>
#include <platform.h>
#include "board.h"
#include "a64_blk.h"
#include "qcom_scm.h"
#include "qcom_smp2p.h"
#include "qcom_smem.h"
#include "qcom_glink.h"
#include "qcom_mss.h"
#include "qcom_fat16.h"
#include "qcom_ipa.h"
#include "qcom_adsp.h"
#include "qcom_telhal.h"

TInt A64BlkDev_Probe();										// qcom_sdhci.cpp (the monitor's "blk init")
// the board's codecs (bonito a64_audio.cpp); a board without call audio has none
__attribute__((weak)) TInt A64Audio_Control(TInt, TInt) { return KErrNotSupported; }

enum { EBoot = 0x4e0d, EState, EQrtrSend, EQrtrRecv, EPartRead, ERmtfsMem, EIpaStart, EAdspLoad, EAdspState, EChanSend,
	EChanRecv, EAudio };
struct SIo { TUint32 iA, iB, iLen, iBuf; };
const TInt KMaxMsg = 0x2000, KChunk = 0x10000;
const TUint32 KRmtfsPhys = 0xfc201000, KRmtfsSize = 0x200000;

// ---- the receive rings, one per channel: [u32 length][bytes, padded to 4] records. 0 = the modem's IPCRTR,
// 1 = the ADSP's IPCRTR, 2 = APR (apr_audio_svc)
const TUint32 KRing = 0x10000;
struct SRing
	{
	TUint8 iBuf[KRing];
	TUint32 iHead, iTail, iDrops;							// byte counters, wrap at 2^32
	NFastMutex iLock;
	TInt iChannel;											// its GLINK channel id once open, else -1
	};
static SRing Rings[KTelRings];
static TBool RingsInit;

TAny* Tel_Ring(TInt aRing)
	{
	if (!RingsInit)
		{
		for (TInt i = 0; i < KTelRings; ++i)
			Rings[i].iChannel = -1;
		RingsInit = ETrue;
		}
	return aRing >= 0 && aRing < KTelRings ? &Rings[aRing] : NULL;
	}
void Tel_SetChannel(TInt aRing, TInt aChannel) { ((SRing*)Tel_Ring(aRing))->iChannel = aChannel; }
void Qrtr_SetChannel(TInt aChannel) { Tel_SetChannel(0, aChannel); }
TBool Tel_Open(TInt aRing) { return ((SRing*)Tel_Ring(aRing))->iChannel >= 0; }

static void RingCopyIn(SRing& aR, TUint32 aPos, const TAny* aSrc, TUint32 aLen)
	{
	for (TUint32 i = 0; i < aLen; ++i)
		aR.iBuf[(aPos + i) % KRing] = ((const TUint8*)aSrc)[i];
	}
static void RingCopyOut(SRing& aR, TAny* aDst, TUint32 aPos, TUint32 aLen)
	{
	for (TUint32 i = 0; i < aLen; ++i)
		((TUint8*)aDst)[i] = aR.iBuf[(aPos + i) % KRing];
	}

void Qrtr_Rx(TAny* aRing, const TUint8* aData, TInt aLen)	// aRing NULL: the modem's (qcom_mss.cpp)
	{
	SRing& r = *(SRing*)(aRing ? aRing : Tel_Ring(0));
	TUint32 rec = 4 + ((aLen + 3) & ~3);
	NKern::FMWait(&r.iLock);
	if (aLen <= 0 || aLen > KMaxMsg || KRing - (r.iHead - r.iTail) < rec)
		++r.iDrops;
	else
		{
		TUint32 len = aLen;
		RingCopyIn(r, r.iHead, &len, 4);
		RingCopyIn(r, r.iHead + 4, aData, aLen);
		r.iHead += rec;
		}
	NKern::FMSignal(&r.iLock);
	}

static TInt RingPop(SRing& aR, TUint8* aBuf)				// aBuf holds KMaxMsg; the length, or KErrNotReady
	{
	NKern::FMWait(&aR.iLock);
	TInt r = KErrNotReady;
	if (aR.iHead != aR.iTail)
		{
		TUint32 len;
		RingCopyOut(aR, &len, aR.iTail, 4);
		RingCopyOut(aR, aBuf, aR.iTail + 4, len);
		aR.iTail += 4 + ((len + 3) & ~3);
		r = len;
		}
	NKern::FMSignal(&aR.iLock);
	return r;
	}

// a send or a receive between user memory and ring/channel aRing, through a kernel buffer
static TInt ChanIo(TBool aSend, TInt aRing, TAny* aBuf, TInt aLen)
	{
	SRing* ring = (SRing*)Tel_Ring(aRing);
	if (!ring || aLen <= 0 || (aSend && aLen > KMaxMsg))
		return KErrArgument;
	NKern::ThreadEnterCS();
	TUint8* k = (TUint8*)Kern::Alloc(KMaxMsg);
	NKern::ThreadLeaveCS();
	if (!k)
		return KErrNoMemory;
	TInt r;
	if (aSend)
		{
		kumemget(k, aBuf, aLen);
		r = ring->iChannel >= 0 ? Glink_Send(ring->iChannel, k, aLen) : KErrNotReady;
		}
	else if ((r = RingPop(*ring, k)) > 0)
		{
		if (r > aLen)
			r = KErrOverflow;								// dropped: the caller's buffer is too small
		else
			kumemput(aBuf, k, r);
		}
	NKern::ThreadEnterCS();
	Kern::Free(k);
	NKern::ThreadLeaveCS();
	return r;
	}

// ---- the rmtfs carve-out
static TUint8* Rmtfs;

static TInt RmtfsMap()
	{
	if (Rmtfs)
		return KErrNone;
	TUint64 perm = 1ull << KScmVmidHlos;					// Linux rmtfs_mem: HLOS + the modem's VMID 15, both RW
	SScmVmPerm next[2] = { { KScmVmidHlos, KScmPermRw }, { KScmVmidMssMsa, KScmPermRw } };
	TInt r = Scm_AssignMem(KRmtfsPhys, KRmtfsSize, perm, next, 2);
	if (r != KErrNone)
		return r;
	DPlatChunkHw* c = NULL;
	NKern::ThreadEnterCS();
	r = DPlatChunkHw::New(c, KRmtfsPhys, KRmtfsSize, EMapAttrSupRw | EMapAttrBufferedNC);
	NKern::ThreadLeaveCS();
	if (r == KErrNone)
		Rmtfs = (TUint8*)c->LinearAddress();
	return r;
	}

static const char* const KParts[] = { "modemst1", "modemst2", "fsg", "fsc", "vendor_a" };

// copy between user memory and the device/carve-out in 64 KB steps through a kernel buffer
static TInt PartRead(const SIo& aIo)
	{
	if (aIo.iA >= sizeof(KParts) / sizeof(KParts[0]) || (aIo.iB | aIo.iLen) % KA64BlkSector)
		return KErrArgument;
	static TBool up;
	if (!up)												// a RAM C: leaves the eMMC down: bring it up once
		{
		static TUint8 s[KA64BlkSector];
		if (A64BlkDev_Read(1, 1, s) == KErrArgument)
			A64BlkDev_Probe();
		up = ETrue;
		}
	TUint64 first, count;
	TInt r = Gpt_Find(KParts[aIo.iA], first, count);
	if (r != KErrNone)
		return r;
	if ((TUint64)aIo.iB + aIo.iLen > count * KA64BlkSector)
		return KErrArgument;
	NKern::ThreadEnterCS();
	TUint8* k = (TUint8*)Kern::Alloc(KChunk);
	NKern::ThreadLeaveCS();
	if (!k)
		return KErrNoMemory;
	for (TUint32 done = 0; r == KErrNone && done < aIo.iLen; )
		{
		TUint32 n = Min(aIo.iLen - done, (TUint32)KChunk);
		r = A64BlkDev_Read(first + (aIo.iB + done) / KA64BlkSector, n / KA64BlkSector, k);
		if (r == KErrNone)
			kumemput((TAny*)(aIo.iBuf + done), k, n);
		done += n;
		}
	NKern::ThreadEnterCS();
	Kern::Free(k);
	NKern::ThreadLeaveCS();
	return r;
	}

static TInt RmtfsMem(SIo& aIo, TAny* aUserIo)
	{
	TInt r = RmtfsMap();
	if (r != KErrNone)
		return r;
	if (aIo.iA == 0)
		{
		aIo.iB = KRmtfsPhys;
		aIo.iLen = KRmtfsSize;
		kumemput32(aUserIo, &aIo, sizeof(aIo));
		return KErrNone;
		}
	if (aIo.iA > 2 || aIo.iB > KRmtfsSize || aIo.iLen > KRmtfsSize - aIo.iB)
		return KErrArgument;
	if (aIo.iA == 1)
		kumemput((TAny*)aIo.iBuf, Rmtfs + aIo.iB, aIo.iLen);
	else
		kumemget(Rmtfs + aIo.iB, (const TAny*)aIo.iBuf, aIo.iLen);
	__e32_memory_barrier();
	return KErrNone;
	}

TInt A64Tel_Hal(TInt aFunction, TAny* a1, TAny* a2)
	{
	if (!Kern::CurrentThreadHasCapability(ECapabilityCommDD, __PLATSEC_DIAGNOSTIC_STRING("NeoSymbian modem")))
		return KErrPermissionDenied;
	switch (aFunction)
		{
	case EBoot:
		return Mss_StartBoot();
	case EState:
		{
		TInt step, err;
		TUint32 rmb[4], out[4] = { 0, 0, 0, Tel_Open(0) };
		Mss_State(step, err, rmb);
		out[0] = step;
		out[1] = err;
		Smp2p_In(KSmemHostModem, "slave-kernel", out[2]);
		kumemput32(a1, out, sizeof(out));
		return KErrNone;
		}
	case EQrtrSend:
	case EQrtrRecv:
		return ChanIo(aFunction == EQrtrSend, 0, a1, (TInt)a2);
	case EPartRead:
	case ERmtfsMem:
		{
		SIo io;
		kumemget32(&io, a1, sizeof(io));
		return aFunction == EPartRead ? PartRead(io) : RmtfsMem(io, a1);
		}
	case EIpaStart:
		{
		SIo io;
		kumemget32(&io, a1, sizeof(io));
		if (!io.iLen || io.iLen > 0x40000 || io.iA > io.iLen)
			return KErrArgument;
		NKern::ThreadEnterCS();
		TUint8* k = (TUint8*)Kern::Alloc(io.iLen);
		NKern::ThreadLeaveCS();
		if (!k)
			return KErrNoMemory;
		kumemget(k, (const TAny*)io.iBuf, io.iLen);
		TInt r = Ipa_SetFirmware(k, io.iLen, io.iA);
		if (r == KErrNone)
			r = Ipa_Start();	// before MODEMD boots the modem, as Linux; on failure MODEMD stays voice-only
		Kern::Printf("[ipa] start from MODEMD -> %d", r);
		NKern::ThreadEnterCS();
		Kern::Free(k);
		NKern::ThreadLeaveCS();
		return r;
		}
	case EAdspLoad:
		{
		SIo io;
		kumemget32(&io, a1, sizeof(io));
		if (io.iLen > KChunk)
			return KErrArgument;
		TUint8* k = NULL;
		if (io.iLen)
			{
			NKern::ThreadEnterCS();
			k = (TUint8*)Kern::Alloc(io.iLen);
			NKern::ThreadLeaveCS();
			if (!k)
				return KErrNoMemory;
			kumemget(k, (const TAny*)io.iBuf, io.iLen);
			}
		TInt r = Adsp_Load(io.iA & 0xff, io.iA >> 8, io.iB, k, io.iLen);
		NKern::ThreadEnterCS();
		Kern::Free(k);
		NKern::ThreadLeaveCS();
		return r;
		}
	case EAdspState:
		{
		TUint32 out[4] = { 0, 0, 0, 0 };
		TInt step, err;
		Adsp_State(step, err);
		out[0] = step;
		out[1] = err;
		Smp2p_In(KSmemHostAdsp, "slave-kernel", out[2]);
		out[3] = Tel_Open(1) | Tel_Open(2) << 1;
		kumemput32(a1, out, sizeof(out));
		return KErrNone;
		}
	case EChanSend:
	case EChanRecv:
		{
		SIo io;
		kumemget32(&io, a1, sizeof(io));
		if (io.iA < 1 || io.iA >= (TUint32)KTelRings)
			return KErrArgument;
		return ChanIo(aFunction == EChanSend, io.iA, (TAny*)io.iBuf, io.iLen);
		}
	case EAudio:
		{
		SIo io;
		kumemget32(&io, a1, sizeof(io));
		return A64Audio_Control(io.iA, io.iB);
		}
		}
	return KErrNotSupported;
	}
