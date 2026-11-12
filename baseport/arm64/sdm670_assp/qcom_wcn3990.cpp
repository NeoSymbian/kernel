// SPDX-License-Identifier: Apache-2.0
//
// qcom_wcn3990.cpp -- the WCN3990 Wi-Fi data path, bring-up stage W2 (docs wifi.md): the copy engines (CE, polled, no
// IRQs yet), HTC (ready, connect HTT + WMI, setup complete) and WMI (service ready -> init -> ready: the MAC).
// The firmware is already up: MODEMD ran the QMI WLFW handshake, WLAN cfg and mission mode (telephony wlfw.rs), then
// asks for this with HAL WlanStart (qcom_telhal.cpp 0x4e1d). Everything runs on our own DFC thread.
// Register layout, ring and message formats are facts from Linux ath10k (ISC: snoc.c, ce.c/h, hw.c, htc.c/h,
// wmi-tlv.c/h, wmi.c/h); private/wifi/w2-spec.md cites each one. This file is a new implementation.
// W3a: after READY, a first passive scan (PDEV_SET_PARAM chain masks, SCAN_CHAN_LIST, VDEV_CREATE, START_SCAN; beacons
// from MGMT_RX logged), facts in private/wifi/w3a-scan-spec.md.
// shortcut: polled for a fixed window, a few rx buffers per ring, no HTT; IRQs and the WHA PDD come with W3.
#include <kernel/kern_priv.h>
#include <platform.h>
#include "board.h"
#include "a64_device.h"
#include "qcom_smmu.h"
#include "qcom_wcn3990.h"

const TUint32 KWlan = 0x18800000;
static inline TUint32 CeReg(TInt aCe, TUint32 aOff) { return KWlan + 0x240000 + 0x1000 * aCe + aOff; }
static volatile TUint32& R(TUint32 aPhys) { return *(volatile TUint32*)A64_Io(aPhys); }
enum	// per CE (w2-spec.md 1)
	{
	ESrBaseLo = 0x00, ESrBaseHi = 0x04, ESrSize = 0x08, EDrBaseLo = 0x0c, EDrBaseHi = 0x10, EDrSize = 0x14,
	ECtrl1 = 0x18, EHostIe = 0x2c, EHostIs = 0x30, ESrWrIdx = 0x3c, EDstWrIdx = 0x40, ESrri = 0x44, EDrri = 0x48,
	ESrcWm = 0x4c, EDstWm = 0x50
	};
const TUint32 KRriLow = KWlan + 0x24c004, KRriHigh = KWlan + 0x24c008;
static inline TUint32 Shadow(TInt aCe) { return KWlan + 0x32000 + 4 * aCe; }	// the source write index doorbell
const TUint32 KCtrl1SrcSwap = 1u << 17, KCtrl1DstSwap = 1u << 18, KCtrl1Rri = 1u << 19;

// the host's copy engines (ath10k snoc.c host_ce_config_wlan): source entries / max bytes, destination entries
struct SCeCfg { TUint16 iSrcN, iMax, iDstN; };
const SCeCfg KCe[12] = {
	{ 16, 2048, 0 },	// 0 HTC control (EP0) to the target
	{ 0, 2048, 512 },	// 1 HTT + HTC from the target
	{ 0, 2048, 64 },	// 2 WMI from the target
	{ 32, 2048, 0 },	// 3 WMI to the target
	{ 2048, 256, 0 },	// 4 HTT to the target (no interrupt)
	{ 0, 512, 512 },	// 5 HTT from the target
	{ 0, 0, 0 },		// 6 the target's own
	{ 2, 2048, 2 },		// 7 diagnostics
	{ 0, 2048, 128 },	// 8 target to uMC (posted, never reaped)
	{ 0, 2048, 512 },	// 9 HTT
	{ 0, 2048, 512 },	// 10 HTT
	{ 0, 2048, 512 },	// 11 pktlog
	};
const TInt KRxPost = 8;			// rx buffers posted per destination ring (RAM: our bank is ~87 MB)
const TInt KDesc = 16;			// struct ce_desc_64

struct SDesc { TUint32 iAddrLo, iAddrHi; TUint16 iNbytes, iFlags; TUint32 iHash; };
struct SRing { SDesc* iDesc; TPhysAddr iPhys; TUint32 iMask, iSw, iWr; TUint8* iBuf; TPhysAddr iBufPhys; };
static SRing Src[12], Dst[12];

// The pool is mapped as device memory: only aligned 32-bit accesses (the compiler merged two descriptor fields into an
// 8-byte store at a 4-byte boundary, and that faulted). Descriptors are written word by word, buffers copied in words.
static void DescWrite(SDesc* aDesc, TUint32 aAddr, TUint16 aNbytes, TUint16 aFlags)
	{
	volatile TUint32* w = (volatile TUint32*)aDesc;
	w[0] = aAddr;
	w[1] = 0;												// address bits 36:32 (our RAM is below 4 GB), no gather
	w[2] = aNbytes | (TUint32)aFlags << 16;
	w[3] = 0;
	}
static TUint16 DescNbytes(const SDesc* aDesc) { return (TUint16)(((volatile const TUint32*)aDesc)[2] & 0xffff); }
static TUint32 DescAddr(const SDesc* aDesc) { return ((volatile const TUint32*)aDesc)[0]; }
static void DescClearNbytes(SDesc* aDesc) { volatile TUint32* w = (volatile TUint32*)aDesc; w[2] &= 0xffff0000u; }
static void ToDev(TUint8* aDst, const TUint8* aSrc, TInt aLen)	// aSrc 4-byte aligned, aLen rounded up
	{
	volatile TUint32* d = (volatile TUint32*)aDst;
	const TUint32* s = (const TUint32*)aSrc;
	for (TInt i = 0; i < (aLen + 3) / 4; ++i)
		d[i] = s[i];
	}
static void FromDev(TUint8* aDst, const TUint8* aSrc, TInt aLen)
	{
	volatile const TUint32* s = (volatile const TUint32*)aSrc;
	TUint32* d = (TUint32*)aDst;
	for (TInt i = 0; i < (aLen + 3) / 4; ++i)
		d[i] = s[i];
	}
static volatile TUint32* Rri;	// 12 words the hardware mirrors its read indices into (bits 15:0 SRRI, 31:16 DRRI)
static TUint8* Pool;
static TPhysAddr PoolPhys;
static TUint32 PoolUsed, PoolSize;

// ---- state, for the device registry and the log
static TInt Step;				// 0 idle, 1 rings, 2 HTC ready, 3 connected, 4 service ready, 5 init sent, 6 ready,
								// 7 scanning, 8 scanned
static TInt Error;
static TUint16 CreditSize, Credits;
static TUint8 HttEid = 0xff, WmiEid = 0xff, Seq[8];
static TUint32 AbiVer0, AbiVer1, RfChains, MemReqs, BoardId, PhyCap, EepromRd, Low5g, High5g;
static TUint8 Mac[6];
static TUint32 Svc[32];			// the service bitmap: word id / 4, bit id % 4 (ath10k WMI_SERVICE_IS_ENABLED)
static TBool Svc174, Svc179;

static TInt Fail(TInt aErr, const char* aWhat)
	{
	Error = aErr;
	Kern::Printf("[wifi] step %d: %s failed (%d)", Step, aWhat, aErr);
	return aErr;
	}

// one uncached, physically contiguous pool for rings, rri and buffers (the WCN3990 is not coherent: no dma-coherent)
static TAny* Carve(TUint32 aSize, TPhysAddr& aPhys)
	{
	TUint32 at = (PoolUsed + 63) & ~63u;
	if (at + aSize > PoolSize)
		return NULL;
	PoolUsed = at + aSize;
	aPhys = PoolPhys + at;
	return Pool + at;
	}

static TUint32 Pow2(TUint32 n) { TUint32 p = 1; while (p < n) p <<= 1; return p; }

// ---- copy engines (ath10k ce.c, w2-spec.md 2)
static TInt InitRings()
	{
	TPhysAddr rriPhys;
	Rri = (volatile TUint32*)Carve(12 * 4, rriPhys);
	if (!Rri)
		return KErrNoMemory;
	R(KRriLow) = rriPhys;
	R(KRriHigh) = 0;
	for (TInt c = 0; c < 12; ++c)
		R(CeReg(c, ECtrl1)) |= KCtrl1Rri;
	for (TInt c = 0; c < 12; ++c)
		{
		const SCeCfg& cfg = KCe[c];
		if (cfg.iSrcN)
			{
			SRing& s = Src[c];
			TUint32 n = Pow2(cfg.iSrcN);
			s.iDesc = (SDesc*)Carve(n * KDesc, s.iPhys);
			// tx buffers only where W2 sends: HTC control (CE0) and WMI (CE3); CE4 alone would take 512 KB
			s.iBuf = c == 0 || c == 3 ? (TUint8*)Carve(n * cfg.iMax, s.iBufPhys) : NULL;
			if (!s.iDesc || ((c == 0 || c == 3) && !s.iBuf))
				return KErrNoMemory;
			s.iMask = n - 1;
			s.iSw = R(CeReg(c, ESrri)) & s.iMask;		// adopt the hardware's indices (they survive sessions)
			s.iWr = R(CeReg(c, ESrWrIdx)) & s.iMask;
			R(CeReg(c, ESrBaseLo)) = s.iPhys;
			R(CeReg(c, ESrBaseHi)) = 0;
			R(CeReg(c, ESrSize)) = n;
			R(CeReg(c, ECtrl1)) = (R(CeReg(c, ECtrl1)) & ~(0xffffu | KCtrl1SrcSwap)) | cfg.iMax;
			R(CeReg(c, ESrcWm)) = n;						// low 0, high n
			}
		if (cfg.iDstN)
			{
			SRing& d = Dst[c];
			TUint32 n = Pow2(cfg.iDstN);
			TInt post = Min((TInt)n - 1, KRxPost);
			d.iDesc = (SDesc*)Carve(n * KDesc, d.iPhys);
			d.iBuf = (TUint8*)Carve(post * cfg.iMax, d.iBufPhys);
			if (!d.iDesc || !d.iBuf)
				return KErrNoMemory;
			d.iMask = n - 1;
			d.iSw = R(CeReg(c, EDrri)) & d.iMask;
			d.iWr = R(CeReg(c, EDstWrIdx)) & d.iMask;
			R(CeReg(c, EDrBaseLo)) = d.iPhys;
			R(CeReg(c, EDrBaseHi)) &= ~0x1fu;
			R(CeReg(c, EDrSize)) = n;
			R(CeReg(c, ECtrl1)) &= ~KCtrl1DstSwap;
			R(CeReg(c, EDstWm)) = n;
			}
		R(CeReg(c, EHostIe)) &= ~0x1fu;					// polled: no copy-complete or watermark interrupts
		}
	return KErrNone;
	}

// hand rx buffer aPhys to destination ring aCe (ath10k __ath10k_ce_rx_post_buf_64)
static void PostRx(TInt aCe, TPhysAddr aPhys)
	{
	SRing& d = Dst[aCe];
	DescWrite(&d.iDesc[d.iWr], aPhys, 0, 0);
	__e32_memory_barrier();
	d.iWr = (d.iWr + 1) & d.iMask;
	R(CeReg(aCe, EDstWrIdx)) = d.iWr;
	}

static void PostAllRx()
	{
	for (TInt c = 0; c < 12; ++c)
		if (KCe[c].iDstN)
			for (TInt i = 0; i < Min((TInt)Dst[c].iMask, KRxPost); ++i)
				PostRx(c, Dst[c].iBufPhys + i * KCe[c].iMax);
	}

// send aLen bytes on source ring aCe, tagged with transfer id aEid (ath10k _ath10k_ce_send_nolock_64)
static TInt Send(TInt aCe, TUint8 aEid, const TUint8* aData, TInt aLen)
	{
	SRing& s = Src[aCe];
	if (!s.iDesc || !s.iBuf || aLen > KCe[aCe].iMax)
		return KErrArgument;
	TUint32 sw = Rri[aCe] & 0xffff & s.iMask;
	if (((s.iWr + 1) & s.iMask) == sw)
		return KErrOverflow;									// full: one slot always stays empty
	ToDev(s.iBuf + s.iWr * KCe[aCe].iMax, aData, aLen);
	DescWrite(&s.iDesc[s.iWr], s.iBufPhys + s.iWr * KCe[aCe].iMax, (TUint16)aLen, (TUint16)((aEid << 4) & 0xfff0));
	__e32_memory_barrier();
	s.iWr = (s.iWr + 1) & s.iMask;
	R(Shadow(aCe)) = s.iWr;
	return KErrNone;
	}

// ---- HTC (ath10k htc.c, w2-spec.md 4)
static TInt HtcSend(TUint8 aEid, const TUint8* aPayload, TInt aLen)
	{
	static TUint32 fw[512];										// the DFC thread's stack is small; word aligned
	TUint8* f = (TUint8*)fw;
	if (aLen + 8 > (TInt)sizeof(fw))
		return KErrArgument;
	TBool wmi = aEid == WmiEid;
	f[0] = aEid;
	f[1] = wmi ? 0x01 : 0;										// NEED_CREDIT_UPDATE on the credit-flow endpoint
	f[2] = (TUint8)aLen;
	f[3] = (TUint8)(aLen >> 8);
	f[4] = 0;
	f[5] = Seq[aEid & 7]++;
	f[6] = f[7] = 0;
	memcpy(f + 8, aPayload, aLen);
	TInt ce = aEid == 0 ? 0 : wmi ? 3 : 4;
	if (wmi)
		{
		if (!Credits)
			return KErrNotReady;
		--Credits;
		}
	return Send(ce, aEid, f, (aLen + 8 + 3) & ~3);
	}

static void Le16(TUint8* p, TUint16 v) { p[0] = (TUint8)v; p[1] = (TUint8)(v >> 8); }
static void Le32(TUint8* p, TUint32 v) { Le16(p, (TUint16)v); Le16(p + 2, (TUint16)(v >> 16)); }
static TUint16 G16(const TUint8* p) { return (TUint16)(p[0] | p[1] << 8); }
static TUint32 G32(const TUint8* p) { return G16(p) | (TUint32)G16(p + 2) << 16; }

static TInt Connect(TUint16 aService, TUint16 aFlags)
	{
	TUint8 m[8];
	Le16(m, 2);													// CONNECT_SERVICE
	Le16(m + 2, aService);
	Le16(m + 4, aFlags);
	m[6] = m[7] = 0;
	return HtcSend(0, m, sizeof(m));
	}

static void HtcControl(const TUint8* p, TInt n)
	{
	if (n < 2)
		return;
	TUint16 id = G16(p);
	if (id == 1 && n >= 8 && Step == 1)							// READY
		{
		TUint16 count = G16(p + 2);
		CreditSize = G16(p + 4);
		Credits = 1;											// use_fw_tx_credits = false: one credit in total
		Kern::Printf("[wifi] HTC ready: %d credits of %d bytes, %d endpoints%s", count, CreditSize, p[6],
			n >= 12 ? (p[8] ? ", HTC 2.1" : ", HTC 2.0") : "");
		if (!count || !CreditSize)
			{
			Fail(KErrCorrupt, "HTC ready");
			return;
			}
		Step = 2;
		}
	else if (id == 3 && n >= 8)									// CONNECT_SERVICE_RESP
		{
		TUint16 svc = G16(p + 2);
		Kern::Printf("[wifi] HTC service %04x -> status %d, endpoint %d, max %d", svc, p[4], p[5], G16(p + 6));
		if (p[4])
			Fail(KErrNotReady, "HTC connect");
		else if (svc == 0x0300)
			HttEid = p[5];
		else if (svc == 0x0100)
			WmiEid = p[5];
		}
	}

// ---- WMI (TLV ABI, ath10k wmi-tlv.c, w2-spec.md 5)
const TUint32 KAbi[6] = { 0x01000000, 53, 0x5f414351, 0x00004c4d, 0, 0 };
struct SChunk { TUint32 iReq, iPhys, iSize; };
static SChunk Chunks[16];
static TInt NumChunks;

// the next top-level TLV of aP[0..aN) from aAt: its tag, value and length; EFalse at the end
static TBool NextTlv(const TUint8* aP, TInt aN, TInt& aAt, TInt& aTag, const TUint8*& aV, TInt& aLen)
	{
	if (aAt + 4 > aN)
		return EFalse;
	TUint32 h = G32(aP + aAt);
	aLen = h & 0xffff;
	aTag = h >> 16;
	if (aAt + 4 + aLen > aN)
		return EFalse;
	aV = aP + aAt + 4;
	aAt += 4 + aLen;
	return ETrue;
	}

// a host memory request (tag 34: req_id, unit_size, num_unit_info, num_units): one chunk of the pool
static void MemReq(const TUint8* r)
	{
	if (NumChunks >= 16)
		return;
	TUint32 req = G32(r), unit = G32(r + 4), info = G32(r + 8), units = G32(r + 12);
	if (info & 1)
		units = 4 + 1;											// vdevs
	else if (info & 6)
		units = 33 + 1;											// peers
	TUint32 size = units * ((unit + 3) & ~3u);
	TPhysAddr phys;
	if (!size || !Carve(size, phys))
		{
		Kern::Printf("[wifi] host memory request %d: %d x %d bytes not allocated", req, units, unit);
		return;
		}
	Chunks[NumChunks].iReq = req;
	Chunks[NumChunks].iPhys = phys;
	Chunks[NumChunks].iSize = size;
	++NumChunks;
	}

static void ServiceReady(const TUint8* p, TInt n)
	{
	TBool first = ETrue;
	TInt at = 0, tag, len;
	const TUint8* v;
	while (NextTlv(p, n, at, tag, v, len))
		{
		if (tag == 32 && len >= 84)								// SERVICE_READY_EVENT
			{
			AbiVer0 = G32(v + 4);
			AbiVer1 = G32(v + 8);
			PhyCap = G32(v + 28);
			RfChains = G32(v + 36);
			MemReqs = G32(v + 72);
			BoardId = G32(v + 80);
			}
		else if (tag == 33 && len >= 36)						// HAL_REG_CAPABILITIES
			{
			EepromRd = G32(v);
			Low5g = G32(v + 28);
			High5g = G32(v + 32);
			}
		else if (tag == 16 && first)							// the service bitmap (ARRAY_UINT32)
			{
			first = EFalse;
			for (TInt i = 0; i < 32 && i * 4 + 4 <= len; ++i)
				Svc[i] = G32(v + 4 * i);
			}
		else if (tag == 18)										// mem_reqs: nested tag-34 TLVs
			{
			TInt in = 0, t, l;
			const TUint8* r;
			while (NextTlv(v, len, in, t, r, l))
				if (t == 34 && l >= 16)
					MemReq(r);
			}
		}
	Kern::Printf("[wifi] WMI service ready: ABI %d.%d.%d.%d, %d RF chains, %d memory requests (%d allocated), board %x",
		AbiVer0 >> 24, AbiVer0 & 0xffffff, AbiVer1 >> 16, AbiVer1 & 0xffff, RfChains, MemReqs, NumChunks, BoardId);
	Kern::Printf("[wifi] reg caps: eeprom_rd %x, phy caps %x (2.4 GHz %d, 5 GHz %d), 5 GHz channels %d-%d", EepromRd,
		PhyCap, (PhyCap >> 1) & 1, PhyCap & 1, Low5g, High5g);
	Step = 4;
	}

static TBool Svc65() { return (Svc[65 / 4] >> (65 % 4)) & 1; }	// RX_FULL_REORDER

static TInt SendInit()
	{
	TUint8 m[4 + 32 + 180 + 4 + 16 * 20];
	TInt at = 0;
	Le32(m, 1);													// WMI header: INIT_CMDID
	at = 4;
	Le32(m + at, 74 << 16 | 28);								// TLV INIT_CMD: abi + chunk count
	for (TInt i = 0; i < 6; ++i)
		Le32(m + at + 4 + 4 * i, KAbi[i]);
	Le32(m + at + 28, NumChunks);
	at += 32;
	TBool reorder = Svc65();
	const TUint32 cfg[44] = {
		4, 33, reorder ? 4u : 0u, reorder ? 4u : 0u, 2, 66, 16, 7, 7, 100, 100, 100, 40, 1, 4, 4, 4, 8, 0, 0, 0,
		0x400, 2, 0, 0, 0, 0, 2, 1056, 2, 1, 0x20, 2, 5, 22, 6, 0, 1, 1, 0, 0, 0, 0,
		(1u << 9) | (Svc174 ? 1u << 18 : 0u) };
	Le32(m + at, 75 << 16 | 176);								// TLV RESOURCE_CONFIG
	for (TInt i = 0; i < 44; ++i)
		Le32(m + at + 4 + 4 * i, cfg[i]);
	at += 180;
	Le32(m + at, 18 << 16 | (NumChunks * 20));					// ARRAY_STRUCT of WLAN_HOST_MEMORY_CHUNK
	at += 4;
	for (TInt i = 0; i < NumChunks; ++i, at += 20)
		{
		Le32(m + at, 76 << 16 | 16);
		Le32(m + at + 4, Chunks[i].iReq);
		Le32(m + at + 8, Chunks[i].iPhys);
		Le32(m + at + 12, Chunks[i].iSize);
		Le32(m + at + 16, 0);
		}
	if (at + 8 > CreditSize)
		return Fail(KErrOverflow, "WMI init bigger than one credit");
	return HtcSend(WmiEid, m, at);
	}

// ---- W3a: the first scan (private/wifi/w3a-scan-spec.md)
struct SBss { TUint8 iBssid[6]; TUint8 iSsidLen; TUint8 iSsid[32]; TUint8 iChannel; TInt8 iDbm; };
static SBss Bss[32];
static TInt NumBss, Beacons, ChanInfos, OtherFrames;
static TBool ScanDone;

static void ScanEvent(const TUint8* p, TInt n)
	{
	TInt at = 0, tag, len;
	const TUint8* v;
	while (NextTlv(p, n, at, tag, v, len))
		if (tag == 36 && len >= 24)
			{
			TUint32 type = G32(v), reason = G32(v + 4);
			if (type & 0x1)
				Kern::Printf("[wifi] scan started (tick %d)", NKern::TickCount());
			if (type & 0x40)
				Kern::Printf("[wifi] scan start failed (reason %d)", reason);
			if (type & 0x42)
				ScanDone = ETrue;
			if (type & 0x2)
				Kern::Printf("[wifi] scan completed (reason %d, tick %d): %d networks from %d frames; %d channel reports, "
					"%d frames on other endpoints", reason, NKern::TickCount(), NumBss, Beacons, ChanInfos, OtherFrames);
			}
	}

static void MgmtRx(const TUint8* p, TInt n)
	{
	TInt at = 0, tag, len;
	const TUint8* v;
	const TUint8* hdr = NULL;
	const TUint8* frame = NULL;
	TInt flen = 0;
	while (NextTlv(p, n, at, tag, v, len))
		{
		if (tag == 44 && len >= 40)
			hdr = v;
		else if (tag == 17)
			frame = v, flen = len;
		}
	if (!hdr || !frame)
		return;
	TUint32 channel = G32(hdr), snr = G32(hdr + 4), buflen = G32(hdr + 16), status = G32(hdr + 20);
	if (status & 0x29 || (TInt)buflen > flen || buflen < 36)
		return;
	if (frame[0] != 0x80 && frame[0] != 0x50)					// beacons and probe responses only
		return;
	++Beacons;
	const TUint8* bssid = frame + 16;
	for (TInt i = 0; i < NumBss; ++i)
		if (!memcompare(Bss[i].iBssid, 6, bssid, 6))
			return;												// seen already
	if (NumBss >= 32)
		return;
	SBss& b = Bss[NumBss++];
	memcpy(b.iBssid, bssid, 6);
	b.iChannel = (TUint8)channel;
	b.iDbm = (TInt8)((TInt)snr - 95);
	b.iSsidLen = 0;
	for (TUint32 ie = 36; ie + 2 <= buflen; ie += 2 + frame[ie + 1])
		if (frame[ie] == 0 && frame[ie + 1] <= 32 && ie + 2 + frame[ie + 1] <= buflen)
			{
			b.iSsidLen = frame[ie + 1];
			memcpy(b.iSsid, frame + ie + 2, b.iSsidLen);
			break;
			}
	TBuf8<33> ssid;
	for (TInt i = 0; i < b.iSsidLen; ++i)
		ssid.Append(b.iSsid[i] >= 0x20 && b.iSsid[i] < 0x7f ? b.iSsid[i] : '?');
	if (!b.iSsidLen)
		ssid.Copy((const TUint8*)"(hidden)", 8);
	Kern::Printf("[wifi] scan: %S  %02x:%02x:%02x:%02x:%02x:%02x  ch %d  %d dBm%s", &ssid, bssid[0], bssid[1], bssid[2],
		bssid[3], bssid[4], bssid[5], channel, b.iDbm, frame[34] & 0x10 ? "  secured" : "");
	}

static void WmiEvent(const TUint8* p, TInt n)
	{
	if (n < 4)
		return;
	TUint32 id = G32(p) & 0xffffff;
	p += 4;
	n -= 4;
	TInt at = 0, tag, len;
	const TUint8* v;
	if (id == 1)
		ServiceReady(p, n);
	else if (id == 3)											// SERVICE_AVAILABLE: ext services (id - 128)
		{
		while (NextTlv(p, n, at, tag, v, len))
			if (tag == 559 && len >= 12)
				{
				Svc174 = (G32(v + 4 + 4 * ((174 - 128) / 32)) >> ((174 - 128) % 32)) & 1;
				Svc179 = (G32(v + 4 + 4 * ((179 - 128) / 32)) >> ((179 - 128) % 32)) & 1;
				}
		}
	else if (id == 2)											// READY
		{
		while (NextTlv(p, n, at, tag, v, len))
			if (tag == 35 && len >= 36)
				{
				memcpy(Mac, v + 24, 6);
				Kern::Printf("[wifi] WMI ready: MAC %02x:%02x:%02x:%02x:%02x:%02x, status %d (W2 done)", Mac[0], Mac[1],
					Mac[2], Mac[3], Mac[4], Mac[5], G32(v + 32));
				Step = 6;
				}
		}
	else if (id == 0x3001)										// SCAN_EVENT
		ScanEvent(p, n);
	else if (id == 0x7001)										// MGMT_RX: beacons and probe responses
		MgmtRx(p, n);
	else if (id == 0x4002)										// CHAN_INFO: one per channel, counted
		++ChanInfos;
	else
		Kern::Printf("[wifi] WMI event %x, %d bytes", id, n);
	}

// one HTC frame from a destination ring: route by endpoint, take the credits from its trailer
static void HtcRx(const TUint8* f, TInt n)
	{
	if (n < 8)
		return;
	TUint8 eid = f[0], flags = f[1];
	TInt len = Min((TInt)G16(f + 2), n - 8);
	const TUint8* p = f + 8;
	if (flags & 0x02)											// TRAILER_PRESENT: records at the end
		{
		TInt tl = Min((TInt)f[4], len);
		const TUint8* t = p + len - tl;
		for (TInt at = 0; at + 4 <= tl; )						// records: {id, len, pad, pad} + len bytes
			{
			TInt id = t[at], rl = t[at + 1];
			if (at + 4 + rl > tl)
				break;
			if (id == 1)										// CREDITS: {eid, credits, pad, pad} each
				for (TInt i = 0; i + 4 <= rl; i += 4)
					if (t[at + 4 + i] == WmiEid)
						Credits += t[at + 4 + i + 1];
			at += 4 + rl;
			}
		len -= tl;
		}
	if (!len)
		return;
	if (eid == 0)
		HtcControl(p, len);
	else if (eid == WmiEid)
		WmiEvent(p, len);
	else if (++OtherFrames <= 3)								// HTT: not set up; show the first few
		Kern::Printf("[wifi] endpoint %d frame, %d bytes: %02x %02x %02x %02x", eid, len, p[0], p[1], p[2], p[3]);
	}

// reap every destination ring once: frames whose length the hardware filled in
static void Poll()
	{
	for (TInt c = 0; c < 12; ++c)
		{
		SRing& d = Dst[c];
		if (!d.iDesc || c == 8)									// CE8's buffers are the target's (never reaped)
			continue;
		for (TInt guard = 0; guard < 64; ++guard)
			{
			SDesc* e = &d.iDesc[d.iSw];
			TUint16 nbytes = DescNbytes(e);
			if (!nbytes)
				break;
			TPhysAddr phys = DescAddr(e);
			DescClearNbytes(e);
			d.iSw = (d.iSw + 1) & d.iMask;
			static TUint32 rx[512];								// a cached copy to parse (byte access is fine there)
			if (nbytes <= KCe[c].iMax && nbytes <= (TInt)sizeof(rx) && phys >= d.iBufPhys)
				{
				FromDev((TUint8*)rx, d.iBuf + (phys - d.iBufPhys), nbytes);
				HtcRx((const TUint8*)rx, nbytes);
				}
			PostRx(c, phys);									// the same buffer goes back
			}
		}
	}

static TBool WaitFor(TInt aStep, TInt aMs)
	{
	for (TInt t = 0; t < aMs && Step < aStep && !Error; ++t)
		{
		Poll();
		NKern::Sleep(NKern::TimerTicks(1));
		}
	return Step >= aStep;
	}

// one WMI command: wait for the single credit (a CREDITS trailer returns it), then send
static TUint32 CmdBuf[400];
static TUint8* const Cmd = (TUint8*)CmdBuf;
static TInt CmdLen;
static void CmdStart(TUint32 aId) { Le32(Cmd, aId); CmdLen = 4; }
static void CmdTlv(TInt aTag, TInt aLen) { Le32(Cmd + CmdLen, (TUint32)aTag << 16 | aLen); CmdLen += 4; }
static void Cmd32(TUint32 aValue) { Le32(Cmd + CmdLen, aValue); CmdLen += 4; }
static TInt CmdSend(const char* aWhat)
	{
	for (TInt t = 0; t < 1000 && !Credits && !Error; ++t)
		Poll(), NKern::Sleep(NKern::TimerTicks(1));
	if (!Credits)
		return Fail(KErrTimedOut, aWhat);
	TInt r = HtcSend(WmiEid, Cmd, CmdLen);
	return r == KErrNone ? r : Fail(r, aWhat);
	}

// channels: 2.4 GHz 1-11 active, 12-13 passive; 5 GHz UNII-1 and UNII-3 passive (no DFS yet). Word 3 = mode | flags
// (11G = 1, 11A = 0; ALLOW_HT 0x800, ALLOW_VHT 0x1000, ADHOC 0x100, PASSIVE 0x80); powers in 0.5 dBm (20 / 30 dBm)
struct SChan { TUint16 iMhz; TUint16 iWord; };
static const SChan KChans[] = {
	{ 2412, 0x1901 }, { 2417, 0x1901 }, { 2422, 0x1901 }, { 2427, 0x1901 }, { 2432, 0x1901 }, { 2437, 0x1901 },
	{ 2442, 0x1901 }, { 2447, 0x1901 }, { 2452, 0x1901 }, { 2457, 0x1901 }, { 2462, 0x1901 }, { 2467, 0x1881 },
	{ 2472, 0x1881 },
	{ 5180, 0x1880 }, { 5200, 0x1880 }, { 5220, 0x1880 }, { 5240, 0x1880 },
	{ 5745, 0x1880 }, { 5765, 0x1880 }, { 5785, 0x1880 }, { 5805, 0x1880 }, { 5825, 0x1880 },
	};
const TInt KNumChans = sizeof(KChans) / sizeof(KChans[0]);
// shortcut: one fixed locally administered MAC (the WCN3990 stores none); derive it from the phone before release
static const TUint8 KMac[6] = { 0x02, 0x4e, 0x53, 0x57, 0x46, 0x01 };

static TInt Scan()
	{
	TUint32 chains = (1u << Min(RfChains, 4u)) - 1;
	for (TUint32 param = 1; param <= 2; ++param)				// PDEV_SET_PARAM TX / RX_CHAIN_MASK
		{
		CmdStart(0x4003);
		CmdTlv(82, 8);
		Cmd32(param);
		Cmd32(chains);
		if (CmdSend("chain mask") != KErrNone)
			return Error;
		}
	CmdStart(0x3003);											// SCAN_CHAN_LIST
	CmdTlv(79, 4);
	Cmd32(KNumChans);
	CmdTlv(18, KNumChans * 28);
	for (TInt i = 0; i < KNumChans; ++i)
		{
		TBool g = KChans[i].iMhz < 3000;
		CmdTlv(80, 24);
		Cmd32(KChans[i].iMhz);
		Cmd32(KChans[i].iMhz);
		Cmd32(0);
		Cmd32(KChans[i].iWord);
		Cmd32(g ? 0x00282800 : 0x003c3c00);
		Cmd32(g ? 0x00002800 : 0x00003c00);
		}
	if (CmdSend("channel list") != KErrNone)
		return Error;
	// PDEV_SET_REGDOMAIN as ath10k (mac.c ath10k_regd_update): eeprom_rd 0 = US FCC3 0x3a, ctl 0x10; else the value as is
	// with no CTL (world 0x60-0x64: 0xff)
	TUint32 regd = EepromRd ? EepromRd & 0x7fff : 0x3a, ctl = EepromRd ? 0xff : 0x10;
	if (regd == 0x8000 || !regd)
		regd = 0x64;
	CmdStart(0x4001);
	CmdTlv(81, 24);
	Cmd32(0);													// pdev_id
	Cmd32(regd);
	Cmd32(regd);
	Cmd32(regd);
	Cmd32(ctl);
	Cmd32(ctl);
	if (CmdSend("regdomain") != KErrNone)
		return Error;
	Kern::Printf("[wifi] regdomain %x, ctl %x", regd, ctl);
	CmdStart(0x5001);											// VDEV_CREATE: vdev 0, station
	CmdTlv(86, 20);
	Cmd32(0);
	Cmd32(2);
	Cmd32(0);
	Cmd32(KMac[0] | KMac[1] << 8 | KMac[2] << 16 | (TUint32)KMac[3] << 24);
	Cmd32(KMac[4] | KMac[5] << 8);
	if (CmdSend("vdev create") != KErrNone)
		return Error;
	CmdStart(0x3001);											// START_SCAN, passive (nothing transmitted)
	CmdTlv(77, 100);
	static const TUint32 KScan[25] = { 0xa000, 0xa001, 0, 1, 0x11f, 50, 150, 50, 500, 0, 0, 0, 20000, 5,
		0x31,			// PASSIVE | CHAN_STAT_EVENT | FILTER_PROBE_REQ (inverted on TLV firmware)
		0, (TUint32)KNumChans, 1, 0, 0, 3, 0, 0, 0, 0 };
	for (TInt i = 0; i < 25; ++i)
		Cmd32(KScan[i]);
	CmdTlv(16, KNumChans * 4);									// the channels, MHz
	for (TInt i = 0; i < KNumChans; ++i)
		Cmd32(KChans[i].iMhz);
	CmdTlv(19, 0);												// no SSIDs: passive
	CmdTlv(19, 8);												// BSSIDs: the broadcast address
	Cmd32(0xffffffff);
	Cmd32(0x0000ffff);
	CmdTlv(17, 0);												// no probe request IEs
	NumBss = Beacons = ChanInfos = OtherFrames = 0;
	ScanDone = EFalse;
	if (CmdSend("start scan") != KErrNone)
		return Error;
	Step = 7;
	for (TInt t = 0; t < 25000 && !ScanDone && !Error; ++t)	// 22 channels x 150 ms dwell + rest, max 20 s
		Poll(), NKern::Sleep(NKern::TimerTicks(1));
	if (!ScanDone)
		return Fail(KErrTimedOut, "scan");
	Step = 8;
	return KErrNone;
	}

// ---- the bring-up, on its own thread
static TInt Run()
	{
	Step = 0;
	Error = KErrNone;
	if (!Pool)
		{
		PoolSize = 0x80000;										// rings ~90 KB, buffers ~200 KB, host memory requests
		NKern::ThreadEnterCS();
		TInt r = Epoc::AllocPhysicalRam(PoolSize, PoolPhys, 12);
		DPlatChunkHw* c = NULL;
		if (r == KErrNone)
			r = DPlatChunkHw::New(c, PoolPhys, PoolSize, EMapAttrSupRw | EMapAttrBufferedNC);
		NKern::ThreadLeaveCS();
		if (r != KErrNone)
			return Fail(r, "DMA memory");
		Pool = (TUint8*)c->LinearAddress();
		memclr(Pool, PoolSize);
		}
	PoolUsed = 0;
	NumChunks = 0;
	HttEid = WmiEid = 0xff;
	Credits = 0;
	memclr(Seq, sizeof(Seq));
	TInt r = Smmu_IdentityMap(0x40, 0x1);						// DTB iommus <&apps_smmu 0x40 0x1>
	Kern::Printf("[wifi] SMMU streams 40/1 -> %d; DMA pool %08x + %x", r, PoolPhys, PoolSize);
	if (r != KErrNone && r != KErrInUse)
		return Fail(r, "SMMU");
	Step = 1;
	if ((r = InitRings()) != KErrNone)
		return Fail(r, "rings");
	PostAllRx();
	Kern::Printf("[wifi] copy engines up (%d bytes of the pool); waiting for HTC ready", PoolUsed);
	if (!WaitFor(2, 3000))
		return Fail(KErrTimedOut, "HTC ready");
	if ((r = Connect(0x0300, 0x0008)) != KErrNone)				// HTT first, as ath10k, no credit flow
		return Fail(r, "HTT connect");
	for (TInt t = 0; t < 1000 && HttEid == 0xff && !Error; ++t)
		Poll(), NKern::Sleep(NKern::TimerTicks(1));
	if ((r = Connect(0x0100, 0x0100)) != KErrNone)				// WMI control: credit flow, RECV_ALLOC 1
		return Fail(r, "WMI connect");
	for (TInt t = 0; t < 1000 && WmiEid == 0xff && !Error; ++t)
		Poll(), NKern::Sleep(NKern::TimerTicks(1));
	if (HttEid == 0xff || WmiEid == 0xff)
		return Fail(KErrTimedOut, "HTC connect");
	Step = 3;
	TUint8 done[12];
	memclr(done, sizeof(done));
	Le16(done, 5);												// SETUP_COMPLETE_EX, all zero on SNOC
	if ((r = HtcSend(0, done, sizeof(done))) != KErrNone)
		return Fail(r, "setup complete");
	if (!WaitFor(4, 5000))
		return Fail(KErrTimedOut, "WMI service ready");
	for (TInt t = 0; t < 200; ++t)								// SERVICE_AVAILABLE follows within ms
		Poll(), NKern::Sleep(NKern::TimerTicks(1));
	if ((r = SendInit()) != KErrNone)
		return Error ? Error : Fail(r, "WMI init");
	Step = 5;
	if (!WaitFor(6, 5000))
		return Fail(KErrTimedOut, "WMI ready");
	return Scan();
	}

static TDynamicDfcQue* Queue;
static void RunDfcFn(TAny*) { Run(); }
static TDfc RunDfc(RunDfcFn, NULL, 1);

TInt Wcn3990_Start()
	{
	if (Step && Step < 6 && Step != 8 && !Error)
		return KErrInUse;
	if (!Queue)
		{
		TInt r = Kern::DynamicDfcQCreate(Queue, 24, _L8("WlanCe"));
		if (r != KErrNone)
			return r;
		RunDfc.SetDfcQ(Queue);
		}
	RunDfc.Enque();
	return KErrNone;
	}

static TA64DevState WifiStatus(TDes8& aDetail)
	{
	static const char* const KSteps[] = { "idle", "rings", "HTC ready", "connected", "service ready", "init sent",
		"ready", "scanning", "scanned" };
	A64Device_Format(aDetail, "WCN3990 %s", KSteps[Step]);
	if (Error)
		A64Device_Format(aDetail, ", error %d", Error);
	if (Step >= 8)
		A64Device_Format(aDetail, ", %d networks", NumBss);
	if (Step == 6)
		A64Device_Format(aDetail, ", MAC %02x:%02x:%02x:%02x:%02x:%02x", Mac[0], Mac[1], Mac[2], Mac[3], Mac[4],
			Mac[5]);
	return Error ? EA64DevError : Step >= 6 ? EA64DevPartial : Step ? EA64DevStarting : EA64DevOff;
	}
A64_DEVICE("wifi", "WCN3990 Wi-Fi", "qcom_wcn3990", WifiStatus);
