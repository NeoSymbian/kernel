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
//   0x4e1b FwRead      a1 = SIo { 0; byte offset; len <= 64 KB; buf }, a2 = "IMAGE/NAME.EXT" (8.3, in a 32-byte
//                      buffer): a file of modem_a's FAT16, read-only, for MODEMD's TFTP reads (the modem's WLAN
//                      domain loads IMAGE/WLANMDSP.MBN, docs wifi.md W0). len 0 -> its size; else the bytes read
//   0x4e1c WlanMsa     a1 = SIo { op 1 assign, 0 release; phys; size; secure }: a region of the Wi-Fi firmware's MSA
//                      carve-out (DTB wlan-msa 0x94d00000 + 1 MB) handed from HLOS to MSS_MSA + WLAN (+ WLAN_CE unless
//                      secure) and back, as Linux ath10k_qmi_map_msa_permission (docs wifi.md W1)
//   0x4e1d WlanStart   after WLAN cfg + mission mode: the copy engines, HTC and WMI up to READY on the driver's own
//                      thread (qcom_wcn3990.cpp, docs wifi.md W2); KErrInUse while it runs
//   0x4e23 Prng        a1 = TDes8* (length 1..256 = the bytes wanted): the SoC's hardware PRNG (Wi-Fi W4 nonces)
//   C7 (calls): 0x4e24 RprocStop SIo{a = mask 1 MSS, 2 ADSP}, asynchronous; poll 0x4e25 RprocState
//                      [phase, error, mss_gen, adsp_gen], phase 0 idle, 1 stopping, 2 stopped, 3 failed (never reload
//                      after 3 until a later stop succeeds).
//                      0x4e26 AudioMem SIo: 0 info -> {b = phys, len = 64 KiB, buf = adsp_gen}; 1 copy {b = offset,
//                      len, buf}; 2 publish -> {b = ticket, len = 64 KiB, buf = adsp_gen}; 3 complete {b = ticket,
//                      buf = adsp_gen} after the DSP is done.
//                      0x4e27 Haptic SIo{a = ms 0..2000, b = amplitude 0..127}; zero stops, a timer bounds each pulse.
// The receive paths are polled by the daemon; messages wait in a 64 KB ring per channel filled by the GLINK thread.
// shortcut: HAL polling, not an LDD with async notification; that comes with power management (C12).
#include <kernel/kern_priv.h>
#include <platform.h>
#include "board.h"
#include "a64_mutex.h"
#include "a64_blk.h"
#include "qcom_scm.h"
#include "qcom_smp2p.h"
#include "qcom_smem.h"
#include "qcom_smmu.h"
#include "qcom_glink.h"
#include "qcom_mss.h"
#include "qcom_fat16.h"
#include "qcom_ipa.h"
#include "qcom_adsp.h"
#include "qcom_telhal.h"
#include "qcom_wcn3990.h"

TInt A64BlkDev_Probe();										// qcom_sdhci.cpp (the monitor's "blk init")
// the board's codecs (bonito a64_audio.cpp); a board without call audio has none
__attribute__((weak)) TInt A64Haptic_Control(TInt, TInt) { return KErrNotSupported; }
__attribute__((weak)) TInt A64Audio_Control(TInt, TInt) { return KErrNotSupported; }

enum { EBoot = 0x4e0d, EState, EQrtrSend, EQrtrRecv, EPartRead, ERmtfsMem, EIpaStart, EAdspLoad, EAdspState, EChanSend,
	EChanRecv, EAudio, EFwRead = 0x4e1b, EWlanMsa, EWlanStart, EPrng = 0x4e23, ERprocStop, ERprocState, EAudioMem,
	EHaptic };
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
static TBool FatalSaid;
static TUint32 Generations[2];
static volatile TInt StopPhase, StopError;
static TUint32 StopMask;
static NFastMutex StopLock;
#if defined(TEL_DIAGNOSTICS) && TEL_DIAGNOSTICS
static TUint32 DiagnosticFaultMask, DiagnosticFaultId;

TInt Tel_DiagnosticFault(TUint32 aMask)
	{
	if (aMask != 1 && aMask != 2)
		return KErrArgument;
	// Reject a request against a booting or genuinely failed processor. This never writes remote state.
	TInt step, error;
	TUint32 bits = 0, rmb[4];
	if (aMask == 1)
		Mss_State(step, error, rmb);
	else
		Adsp_State(step, error);
	TInt r = Smp2p_In(aMask == 1 ? KSmemHostModem : KSmemHostAdsp, "slave-kernel", bits);
	if (r != KErrNone || error || step != (aMask == 1 ? 8 : 4) || (bits & KMssFatal))
		return KErrNotReady;
	NKern::FMWait(&StopLock);
	if (DiagnosticFaultMask || StopPhase == 1 || StopPhase == 3)
		r = KErrInUse;
	else
		{
		DiagnosticFaultMask = aMask;
		if (!++DiagnosticFaultId)
			++DiagnosticFaultId;
		}
	TUint32 id = DiagnosticFaultId;
	NKern::FMSignal(&StopLock);
	if (r == KErrNone)
		Kern::Printf("[tel] synthetic detection id %d mask %d latched; MODEMD owns recovery", id, aMask);
	return r;
	}

static TUint32 DiagnosticFatal(TUint32 aMask)
	{
	NKern::FMWait(&StopLock);
	TUint32 bits = DiagnosticFaultMask & aMask ? KMssFatal : 0;
	NKern::FMSignal(&StopLock);
	return bits;
	}

static void DiagnosticStopped(TUint32 aMask)
	{
	NKern::FMWait(&StopLock);
	TUint32 done = DiagnosticFaultMask & aMask, id = DiagnosticFaultId;
	DiagnosticFaultMask &= ~aMask;
	NKern::FMSignal(&StopLock);
	if (done)
		Kern::Printf("[tel] synthetic detection id %d mask %d cleared after acknowledged stop", id, done);
	}
#endif

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
	TUint32 drops = 0;
	NKern::FMWait(&r.iLock);
	if (aLen <= 0 || aLen > KMaxMsg || KRing - (r.iHead - r.iTail) < rec)
		drops = ++r.iDrops;
	else
		{
		TUint32 len = aLen;
		RingCopyIn(r, r.iHead, &len, 4);
		RingCopyIn(r, r.iHead + 4, aData, aLen);
		r.iHead += rec;
		}
	NKern::FMSignal(&r.iLock);
	if (drops && (drops == 1 || drops % 64 == 0))						// a lost confirm_rx stalls the sender for good: say so
		Kern::Printf("[tel] ring %d: message of %d bytes dropped (%d so far)", &r - Rings, aLen, drops);
	}

void Tel_Dump(void (*aSay)(const char*, ...))
	{
	for (TInt i = 0; i < KTelRings; ++i)
		aSay("tel ring %d: channel %x, %d bytes queued, %d dropped", i, Rings[i].iChannel,
			Rings[i].iHead - Rings[i].iTail, Rings[i].iDrops);
	NKern::FMWait(&StopLock);
	TInt phase = StopPhase, error = StopError;
	TUint32 mask = StopMask, mss = Generations[0], adsp = Generations[1];
	NKern::FMSignal(&StopLock);
	aSay("tel stop phase %d error %d mask %d generations %d/%d", phase, error, mask, mss, adsp);
	}

// the modem's own reason, once, when it raises FATAL while running (the boot path prints it for boot failures)
static void FatalReason(TUint32 aSlaveKernel)
	{
	if (!(aSlaveKernel & KMssFatal) || FatalSaid)
		return;
	FatalSaid = ETrue;
	TUint32 len = 0;
	const char* why = (const char*)Smem_Get(KSmemHostGlobal, 421, &len);	// SMEM_SSR_REASON_MSS0
	char buf[160];											// a short copy: monitor output over 256 bytes faults
	TUint32 n = why ? Min(len, (TUint32)sizeof(buf) - 1) : 0;
	for (TUint32 i = 0; i < n; ++i)
		buf[i] = why[i] >= 0x20 && why[i] < 0x7f ? why[i] : '.';
	buf[n] = 0;
	Kern::Printf("[tel] modem FATAL (slave-kernel %08x): %s", aSlaveKernel, n ? buf : "(no reason)");
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

// a file of the modem's firmware partition (modem_a, FAT16) by its 8.3 path under IMAGE/
static TInt FwRead(const SIo& aIo, const TAny* aUserPath)
	{
	char path[32];
	kumemget(path, aUserPath, sizeof(path) - 1);
	path[sizeof(path) - 1] = 0;
	TInt n = 0;
	for (; path[n]; ++n)
		{
		char c = path[n];
		if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-' ||
			(c == '.' && n && path[n - 1] != '.') || (c == '/' && n == 5)))
			return KErrArgument;
		}
	if (n < 7 || memcompare((const TUint8*)path, 6, (const TUint8*)"IMAGE/", 6) || aIo.iLen > (TUint32)KChunk)
		return KErrArgument;
	TInt r = Fat_Mount("modem_a");						// one volume at a time; the modem's boot is past it
	SFatFile f;
	if (r == KErrNone)
		r = Fat_Find(path, f);
	if (r != KErrNone)
		return r;
	if (!aIo.iLen)
		return (TInt)f.iSize;
	if (aIo.iB >= f.iSize)
		return 0;
	TUint32 len = Min(aIo.iLen, f.iSize - aIo.iB);
	NKern::ThreadEnterCS();
	TUint8* k = (TUint8*)Kern::Alloc(len);
	NKern::ThreadLeaveCS();
	if (!k)
		return KErrNoMemory;
	r = Fat_Read(f, aIo.iB, k, len);
	if (r == KErrNone)
		kumemput((TAny*)aIo.iBuf, k, len);
	NKern::ThreadEnterCS();
	Kern::Free(k);
	NKern::ThreadLeaveCS();
	return r == KErrNone ? (TInt)len : r;
	}

// the Wi-Fi firmware's MSA regions (ath10k qmi.c): at most two, inside the carve-out, each with its current owners
const TUint32 KWlanMsaPhys = 0x94d00000, KWlanMsaSize = 0x100000, KVmidWlan = 0x18, KVmidWlanCe = 0x19;
static struct { TUint32 iPhys, iSize; TUint64 iOwners; } Msa[2];

static TInt WlanMsa(const SIo& aIo)
	{
	TUint32 phys = aIo.iB, size = aIo.iLen;
	if (aIo.iA > 1 || !size || (phys | size) & 0xfff || phys < KWlanMsaPhys || size > KWlanMsaSize ||
		phys - KWlanMsaPhys > KWlanMsaSize - size)
		return KErrArgument;
	TInt slot = -1;
	for (TInt i = 0; i < 2; ++i)
		if (Msa[i].iSize && Msa[i].iPhys == phys && Msa[i].iSize == size)
			slot = i;
	if (aIo.iA == 1)											// assign
		{
		if (slot >= 0)
			return KErrNone;									// already the firmware's
		for (TInt i = 0; i < 2 && slot < 0; ++i)
			if (!Msa[i].iSize)
				slot = i;
		if (slot < 0)
			return KErrOverflow;
		SScmVmPerm dst[3] = { { KScmVmidMssMsa, KScmPermRw }, { KVmidWlan, KScmPermRw }, { KVmidWlanCe, KScmPermRw } };
		TUint64 owners = 1ull << KScmVmidHlos;
		TInt r = Scm_AssignMem(phys, size, owners, dst, aIo.iBuf ? 2 : 3);	// iBuf = secure: no WLAN_CE
		if (r != KErrNone)
			return r;
		Msa[slot].iPhys = phys;
		Msa[slot].iSize = size;
		Msa[slot].iOwners = owners;
		return KErrNone;
		}
	if (slot < 0)
		return KErrNotFound;
	SScmVmPerm hlos = { KScmVmidHlos, KScmPermRw };				// release: back to us
	TInt r = Scm_AssignMem(phys, size, Msa[slot].iOwners, &hlos, 1);
	if (r == KErrNone)
		Msa[slot].iSize = 0;
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

// One bounded physical region retained for this boot. The daemon owns ASM map/write acknowledgements;
// a published region cannot be overwritten until its exact ticket is completed or PAS shutdown succeeds.
const TUint32 KAudioBytes = 0x40000;					// telephony apr::RING_REGION: a 2 s 48 kHz ringtone (192,000 bytes)
static TPhysAddr AudioPhys;
static DPlatChunkHw* AudioChunk;
static TUint32 AudioTicket, AudioGeneration;
static TBool AudioBusy;
static TBool AudioSmmu;
static DMutex* AudioLock;
// The ADSP reads the region by DMA through the apps SMMU on stream 0x1801 (DTB q6asm-dais iommus <&apps_smmu 0x1801
// 0>). Unmapped, that stream faults (sCR0.USFCFG = 1) and the ringtone's first ASM write froze the whole phone
// (2026-11-14, lastlog: "apr: AsmRun ok", then nothing); mapped like IPA's and WLAN's streams.
const TUint32 KAudioStream = 0x1801;
static NFastMutex AudioInit;
static TInt AudioMem(SIo& aIo)
	{
	TInt r = A64Mutex_Init(AudioLock, AudioInit, _L("AudioMem"), KMutexOrdGeneral0);
	if (r != KErrNone)
		return r;
	// op 1's bytes come from the caller first, outside any critical section or lock: kumemget may fault on a bad
	// pointer, which must not happen holding AudioLock (the kernel's rule for kumemget)
	TUint8* copy = NULL;
	if (aIo.iA == 1)
		{
		if (aIo.iB > KAudioBytes || aIo.iLen > KAudioBytes - aIo.iB)
			return KErrArgument;
		NKern::ThreadEnterCS();
		copy = (TUint8*)Kern::Alloc(aIo.iLen ? aIo.iLen : 1);
		NKern::ThreadLeaveCS();
		if (!copy)
			return KErrNoMemory;
		kumemget(copy, (const TAny*)aIo.iBuf, aIo.iLen);
		}
	NKern::ThreadEnterCS();
	Kern::MutexWait(*AudioLock);
	if (!AudioChunk && aIo.iA == 0)
		{
		r = Epoc::AllocPhysicalRam(KAudioBytes, AudioPhys);
		if (r == KErrNone)
			{
			r = DPlatChunkHw::New(AudioChunk, AudioPhys, KAudioBytes, EMapAttrSupRw | EMapAttrBufferedNC);
			if (r != KErrNone)
				Epoc::FreePhysicalRam(AudioPhys, KAudioBytes);
			else
				memclr((TAny*)AudioChunk->LinearAddress(), KAudioBytes);
			}
		}
	if (r == KErrNone && AudioChunk && !AudioSmmu && aIo.iA == 0)
		{
		r = Smmu_IdentityMap(KAudioStream, 0);
		AudioSmmu = r == KErrNone;
		Kern::Printf("[audio] ADSP stream %x through the apps SMMU -> %d", KAudioStream, r);
		}
	if (r == KErrNone && !AudioChunk)
		r = KErrNotReady;
	if (r == KErrNone)
		switch (aIo.iA)
			{
		case 0:
			aIo.iB = AudioPhys;
			aIo.iLen = KAudioBytes;
			aIo.iBuf = Generations[1];
			break;
		case 1:
			if (AudioBusy)
				r = KErrInUse;
			else
				memcpy((TUint8*)AudioChunk->LinearAddress() + aIo.iB, copy, aIo.iLen);
			break;
		case 2:
			if (AudioBusy)
				r = KErrInUse;
			else
				{
				__e32_memory_barrier(); // PCM reaches noncached DRAM before the daemon sends ASM WRITE
				AudioBusy = ETrue;
				AudioGeneration = Generations[1];
				if (!++AudioTicket)
					++AudioTicket;
				aIo.iB = AudioTicket;
				aIo.iLen = KAudioBytes;
				aIo.iBuf = AudioGeneration;
				}
			break;
		case 3:
			if (!AudioBusy || aIo.iB != AudioTicket || aIo.iBuf != AudioGeneration || aIo.iBuf != Generations[1])
				r = KErrNotReady;
			else
				AudioBusy = EFalse;
			break;
		default:
			r = KErrArgument;
			}
	Kern::MutexSignal(*AudioLock);
	Kern::Free(copy);
	NKern::ThreadLeaveCS();
	return r;
	}

static void ResetRings(TInt aFirst, TInt aEnd)
	{
	for (TInt i = aFirst; i < aEnd; ++i)
		{
		SRing& ring = *(SRing*)Tel_Ring(i);
		NKern::FMWait(&ring.iLock);
		ring.iChannel = -1;
		ring.iHead = ring.iTail = 0;
		NKern::FMSignal(&ring.iLock);
		}
	}

static TDynamicDfcQue* StopQueue;
// One monotonic retry-wait budget across both processors. Calls into the boot, GLINK or secure-firmware
// paths can themselves block beyond it: expiry never cancels those calls or claims their memory is ours.
static TUint32 StopElapsedMs(TUint32 aStarted)
	{
	return (TUint32)((TUint64)(TUint32)(NKern::TickCount() - aStarted) * NKern::TickPeriod() / 1000);
	}

static TInt StopProcessor(TUint32 aMask, TUint32 aStarted)
	{
	Kern::Printf("[tel] stop stage %d enter at %d ms", aMask, StopElapsedMs(aStarted));
	TInt r = KErrTimedOut;
	while ((TUint32)(NKern::TickCount() - aStarted) < (TUint32)NKern::TimerTicks(30000))
		{
		r = aMask == 1 ? Mss_Stop() : Adsp_Stop();
		if (r != KErrInUse)
			break;
		NKern::Sleep(NKern::TimerTicks(1));
		}
	if (r == KErrInUse)
		r = KErrTimedOut;
	Kern::Printf("[tel] stop stage %d -> %d at %d ms", aMask, r, StopElapsedMs(aStarted));
	return r;
	}

static void StopDfcFn(TAny*)
	{
	TInt r = KErrNone;
	TUint32 started = NKern::TickCount(), mask = StopMask;
	Kern::Printf("[tel] stop mask %d begin, generations %d/%d", mask, Generations[0], Generations[1]);
#if defined(TEL_DIAGNOSTICS) && TEL_DIAGNOSTICS
	NKern::FMWait(&StopLock);
	TUint32 fault = DiagnosticFaultMask, id = DiagnosticFaultId;
	NKern::FMSignal(&StopLock);
	if (fault)
		Kern::Printf("[tel] synthetic detection id %d mask %d selected stop mask %d", id, fault, mask);
#endif
	// MODEMD has stopped sending before requesting this. Both operations wait for their existing boot DFC
	// to finish; an unresponsive processor never authorizes memory reuse or another firmware load.
	if (mask & 1)
		r = StopProcessor(1, started);
	if (r == KErrNone && (mask & 1))
		{
		ResetRings(0, 1);
		++Generations[0];
		Wcn3990_ModemDown();							// the Wi-Fi firmware ran inside the modem
		FatalSaid = EFalse;
#if defined(TEL_DIAGNOSTICS) && TEL_DIAGNOSTICS
		DiagnosticStopped(1);
#endif
		}
	if (r == KErrNone && (mask & 2))
		{
		A64Audio_Control(2, 0);
		A64Haptic_Control(0, 0);
		r = StopProcessor(2, started);
		if (r == KErrNone)
			{
			ResetRings(1, 3);
			if (AudioLock)
				{
				NKern::ThreadEnterCS();
				Kern::MutexWait(*AudioLock);
				++Generations[1];
				AudioBusy = EFalse; // successful PAS shutdown is the DSP ownership boundary
				Kern::MutexSignal(*AudioLock);
				NKern::ThreadLeaveCS();
				}
			else
				++Generations[1];
#if defined(TEL_DIAGNOSTICS) && TEL_DIAGNOSTICS
			DiagnosticStopped(2);
#endif
			}
		}
	if (r == KErrInUse)
		r = KErrTimedOut;
	__e32_memory_barrier();
	NKern::FMWait(&StopLock);
	StopError = r;
	StopPhase = r == KErrNone ? 2 : 3;
	NKern::FMSignal(&StopLock);
	Kern::Printf("[tel] stop mask %d -> %d at %d ms, generations %d/%d", mask, r,
		StopElapsedMs(started), Generations[0], Generations[1]);
	}
static TDfc StopDfc(StopDfcFn, NULL, 1);

static TInt QueueStop(TUint32 aMask)
	{
	if (!aMask || aMask > 3)
		return KErrArgument;
	NKern::FMWait(&StopLock);
	if (StopPhase == 1)
		{
		NKern::FMSignal(&StopLock);
		return KErrInUse;
		}
	StopPhase = 1; // reserve the request before queue creation can sleep
	StopMask = aMask;
	StopError = 0;
	NKern::FMSignal(&StopLock);
	TInt r = StopQueue ? KErrNone : Kern::DynamicDfcQCreate(StopQueue, 24, _L8("TelStop"));
	if (r == KErrNone)
		{
		StopDfc.SetDfcQ(StopQueue);
		StopDfc.Enque();
		}
	else
		{
		NKern::FMWait(&StopLock);
		StopError = r;
		StopPhase = 3;
		NKern::FMSignal(&StopLock);
		}
	return r;
	}

// ---- Prng (0x4e23): a1 = TDes8* whose length (1..256) is the bytes wanted, filled from the SoC's PRNG (the
// execution-environment view at 0x793000, as Linux qcom-rng "prng-ee": DATA_OUT +0, STATUS +4 bit 0 = data
// available, nothing to configure). Its AHB clock is the modem's proxy "prng" vote (GCC 0x152004 bit 13), which
// qcom_mss.cpp drops after boot, so it is voted again here. For the Wi-Fi handshake's nonces (W4).
const TUint32 KPrng = 0x00793000, KPrngClkVote = 0x00152004;
static TInt Prng(TAny* aDes)
	{
	TInt len = Kern::ThreadGetDesLength(&Kern::CurrentThread(), aDes);
	if (len < 1 || len > 256)
		return KErrArgument;
	volatile TUint32* prng = (volatile TUint32*)A64_Io(KPrng);
	*(volatile TUint32*)A64_Io(KPrngClkVote) |= 1u << 13;
	for (TInt i = 0; i < len; i += 4)
		{
		TUint32 v = 0;
		for (TInt t = 0; t < 1000 && !v; ++t)		// a word takes well under 1 ms; 0 counts as "not yet", as Linux
			{
			if (prng[1] & 1)
				v = prng[0];
			if (!v)
				Kern::NanoWait(1000);
			}
		if (!v)
			return KErrNotReady;
		TPtrC8 word((const TUint8*)&v, Min(4, len - i));
		TInt r = Kern::ThreadDesWrite(&Kern::CurrentThread(), aDes, word, i, KChunkShiftBy0, NULL);
		if (r != KErrNone)
			return r;
		}
	return KErrNone;
	}

TInt A64Tel_Hal(TInt aFunction, TAny* a1, TAny* a2)
	{
	if (!Kern::CurrentThreadHasCapability(ECapabilityCommDD, __PLATSEC_DIAGNOSTIC_STRING("NeoSymbian modem")))
		return KErrPermissionDenied;
	if ((StopPhase == 1 || StopPhase == 3) && aFunction != ERprocState && aFunction != ERprocStop &&
		aFunction != EState && aFunction != EAdspState && aFunction != EHaptic && aFunction != EPrng)
		return KErrNotReady;
	switch (aFunction)
		{
	case ERprocStop:
		{
		SIo io;
		kumemget32(&io, a1, sizeof(io));
		return QueueStop(io.iA);
		}
	case ERprocState:
		{
		NKern::FMWait(&StopLock);
		TUint32 out[4] = { (TUint32)StopPhase, (TUint32)StopError, Generations[0], Generations[1] };
		NKern::FMSignal(&StopLock);
		kumemput32(a1, out, sizeof(out));
		return KErrNone;
		}
	case EAudioMem:
		{
		SIo io;
		kumemget32(&io, a1, sizeof(io));
		TInt r = AudioMem(io);
		if (r == KErrNone)
			kumemput32(a1, &io, sizeof(io));
		return r;
		}
	case EHaptic:
		{
		SIo io;
		kumemget32(&io, a1, sizeof(io));
		if (io.iA > 2000 || io.iB > 127)
			return KErrArgument;
		return A64Haptic_Control(io.iA, io.iB);
		}
	case EBoot:
		return Mss_StartBoot();
	case EState:
		{
		TInt step, err;
		TUint32 rmb[4], out[4] = { 0, 0, 0, Tel_Open(0) };
		Mss_State(step, err, rmb);
		out[0] = step;
		out[1] = err;
		TInt r = Smp2p_In(KSmemHostModem, "slave-kernel", out[2]);
		if (step == 8 && r != KErrNone)
			return r; // a running processor's unreadable remote state must not be reported healthy
		FatalReason(out[2]);
#if defined(TEL_DIAGNOSTICS) && TEL_DIAGNOSTICS
		out[2] |= DiagnosticFatal(1); // output snapshot only; never remote SMP2P/SMEM
#endif
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
		TInt r = Smp2p_In(KSmemHostAdsp, "slave-kernel", out[2]);
		if (step >= 3 && r != KErrNone)
			return r;
#if defined(TEL_DIAGNOSTICS) && TEL_DIAGNOSTICS
		out[2] |= DiagnosticFatal(2); // output snapshot only; never remote SMP2P/SMEM
#endif
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
	case EFwRead:
		{
		SIo io;
		kumemget32(&io, a1, sizeof(io));
		return FwRead(io, a2);
		}
	case EWlanStart:
		return Wcn3990_Start();
	case EPrng:
		return Prng(a1);
	case EWlanMsa:
		{
		SIo io;
		kumemget32(&io, a1, sizeof(io));
		return WlanMsa(io);
		}
		}
	return KErrNotSupported;
	}
