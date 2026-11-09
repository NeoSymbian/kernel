// SPDX-License-Identifier: Apache-2.0
//
// a64_audio.cpp -- the Pixel 3a XL's call-audio parts (telephony C6). The ADSP plays and records through two TDM
// ports (MODEMD drives them over APR); this board driver powers what sits on them:
//   - the earpiece and the loudspeaker: two Cirrus Logic CS35L36 amps on I2C10 (QUP wrapper 1 SE2 0xa88000) at 0x40
//     ("Top", the earpiece, reset gpio14) and 0x41 ("Bottom", the loudspeaker, reset gpio16), TDM slaves on the
//     ADSP's SECONDARY_TDM_RX (LPASS LPI pins gpio8-11), their PLLs locked to its 6.144 MHz bit clock;
//   - the microphone: a DMIC through a Realtek RT5514 (I2C10 0x57) onto PRIMARY_TDM_TX (TLMM gpio65-68), its PLL
//     from the bit clock too; the DMIC's supply is the PM660L analog codec's MIC BIAS 1 (SPMI SID 3, 0xf000).
// Routes: earpiece = Top only, speaker = both amps, both at C1's live gain. Volume = the amps'
// digital volume, mute = the RT5514's ADC mute (no APR controls exist for these).
// Register sequences from Linux cs35l36.c, rt5514.c, msm8916-wcd-analog.c and the sdm670 machine driver as the phone
// ran them under C1 (docs/telephony.md C6); pins from pinctrl-sdm670(-lpass-lpi).c; clocks from gcc-sdm845.c.
// Thread context (HAL Audio from MODEMD, the monitor's "audio").
// shortcut: no amp interrupt handling (protection errors are only logged on start); no ACDB calibration.
#include <kernel/kern_priv.h>
#include "board.h"
#include "qcom_spmi.h"
#include "qcom_rpmh.h"
#include "qcom_geni_i2c.h"
#include "qcom_telhal.h"

static volatile TUint32& Reg(TUint32 aPhys) { return *(volatile TUint32*)A64_Io(aPhys); }

// GCC: QUP wrapper 1 SE2 (I2C10): vote bit 24, its branch and RCG (19.2 MHz from XO, as SE1 for touch)
const TUint32 KGccVote1 = 0x0015200c, KSe2Cbcr = 0x00118274, KSe2RcgCmd = 0x00118278, KSe2RcgCfg = 0x0011827c;
const TUint32 KSe2 = 0x00a88000;
const TInt KBus = 1;										// qcom_geni_i2c engine 1
// TLMM CTL (pull [1:0], function [5:2], drive [8:6], OE [9]) and IO (out [1]) at tile + 0x1000 * pin
const TUint32 KTlmmNorth = 0x03900000, KTlmmWest = 0x03500000, KLpi = 0x62b40000;
static volatile TUint32& Ctl(TUint32 aTile, TInt aPin) { return Reg(aTile + 0x1000 * aPin); }
static volatile TUint32& Io(TUint32 aTile, TInt aPin) { return Reg(aTile + 0x1000 * aPin + 4); }

const TUint KTop = 0x40, KBottom = 0x41, KRt = 0x57;
const TUint KPm660l = 3;									// the PM660L's second SPMI SID: its audio codec
enum { ERoute = 0, ESpeaker = 1 };

static TBool Probed, Running, MicMuted;
static TInt Route = ERoute, Volume = 10;
static TInt ProbeErr = KErrNotReady;
static TUint32 AmpId[2], AmpRev[2], Faults;
static TBool AmpL36[2];
static TUint32 RtId;

static void Ms(TInt aMs) { NKern::Sleep(Max(1, NKern::TimerTicks(aMs))); }

// ---- I2C: 32-bit register, 32-bit value, both big-endian (the CS35L36 and the RT5514's 0x18000000 window)
static TInt W32(TUint aDev, TUint32 aReg, TUint32 aVal)
	{
	TUint8 b[8] = { (TUint8)(aReg >> 24), (TUint8)(aReg >> 16), (TUint8)(aReg >> 8), (TUint8)aReg,
		(TUint8)(aVal >> 24), (TUint8)(aVal >> 16), (TUint8)(aVal >> 8), (TUint8)aVal };
	TInt r = I2c_Xfer(KBus, aDev, b, 8, NULL, 0);
	if (r != KErrNone)
		++Faults;
	return r;
	}

static TInt R32(TUint aDev, TUint32 aReg, TUint32& aVal)
	{
	TUint8 a[4] = { (TUint8)(aReg >> 24), (TUint8)(aReg >> 16), (TUint8)(aReg >> 8), (TUint8)aReg }, v[4];
	TInt r = I2c_Xfer(KBus, aDev, a, 4, v, 4);
	aVal = r == KErrNone ? (TUint32)v[0] << 24 | v[1] << 16 | v[2] << 8 | v[3] : 0;
	if (r != KErrNone)
		++Faults;
	return r;
	}

static void Upd(TUint aDev, TUint32 aReg, TUint32 aMask, TUint32 aVal)
	{
	TUint32 v;
	if (R32(aDev, aReg, v) == KErrNone)
		W32(aDev, aReg, (v & ~aMask) | (aVal & aMask));
	}

// the RT5514's codec registers are 0x18000000 | reg on the wire
static TInt RtW(TUint32 aReg, TUint32 aVal) { return W32(KRt, 0x18000000 | aReg, aVal); }
static void RtUpd(TUint32 aReg, TUint32 aMask, TUint32 aVal) { Upd(KRt, 0x18000000 | aReg, aMask, aVal); }

static void SpmiUpd(TUint32 aAddr, TUint32 aMask, TUint32 aVal)
	{
	TInt v = Spmi_Read(KPm660l, aAddr);
	if (v >= 0)
		Spmi_Write(KPm660l, aAddr, (v & ~aMask) | (aVal & aMask));
	}

// ---- CS35L36
static void Unlock(TUint aDev) { W32(aDev, 0x0020, 0x5555); W32(aDev, 0x0020, 0xaaaa); }
static void Lock(TUint aDev) { W32(aDev, 0x0020, 0xcccc); W32(aDev, 0x0020, 0x3333); }

static const TUint32 KErrataA0[][2] = {
	{ 0x0500, 0x00002060 }, { 0x0504, 0x00000001 }, { 0x0500, 0x00002460 }, { 0x0504, 0x00000001 },
	{ 0x2088, 0x012a1838 }, { 0x3014, 0x0100ee0e }, { 0x3008, 0x0008184a }, { 0x7418, 0x509001c8 },
	{ 0x7064, 0x0929a800 }, { 0x2d10, 0x0002c01c }, { 0x410c, 0x00000a11 }, { 0x6e08, 0x8b19140c },
	{ 0x6454, 0x0300000a }, { 0x7e14, 0x000020ef }, { 0x7e34, 0x0000000e }, { 0x410c, 0x00000a11 },
	{ 0x7410, 0x20514b00 }, { 0x2020, 0x00000000 }, { 0xd00220, 0x00860001 }, { 0xd00224, 0x00860001 },
	{ 0xd00228, 0x00860001 }, { 0xd0022c, 0x00860001 }, { 0xd00230, 0x00860001 }, { 0xd00234, 0x00860001 },
	{ 0xd00238, 0x00860001 }, { 0xd0023c, 0x00860001 }, { 0xd00218, 0x000000ff } };
static const TUint32 KErrataB0[][2] = {
	{ 0x7064, 0x0929a800 }, { 0x7850, 0x00002fa9 }, { 0x7854, 0x0003f1d5 }, { 0x7858, 0x0003f5e3 },
	{ 0x785c, 0x00001137 }, { 0x7860, 0x0001a7a5 }, { 0x7864, 0x0002f16a }, { 0x7868, 0x00003e21 },
	{ 0x7848, 0x00000001 }, { 0x3854, 0x05180240 }, { 0x7418, 0x509001c8 }, { 0x394c, 0x028764bd } };

static TInt AmpProbe(TInt aI)
	{
	TUint dev = aI ? KBottom : KTop;
	TUint32 v = 0;
	if (R32(dev, 0x0000, v) != KErrNone || v != 0x00035a36)
		return KErrNotFound;
	AmpId[aI] = v;
	R32(dev, 0x200c, v);
	AmpRev[aI] = v >> 8;
	R32(dev, 0x0478, v);
	AmpL36[aI] = (v & 0x00ff0000) != 0x00cc0000;			// else the 12 V CS35L37
	if (AmpRev[aI] == 0xa0)
		{
		Unlock(dev);
		for (TUint i = 0; i < sizeof(KErrataA0) / sizeof(KErrataA0[0]); ++i)
			W32(dev, KErrataA0[i][0], KErrataA0[i][1]);
		Lock(dev);
		}
	else if (AmpRev[aI] == 0xb0)
		{
		Unlock(dev);											// the PAC's OTP trim, then the B0 table
		Ms(10);
		W32(dev, 0x0c00, 0);
		W32(dev, 0x0c08, 1);
		W32(dev, 0xe02800, 0x00dd0102);
		W32(dev, 0x0c08, 0);
		W32(dev, 0x0c00, 1);
		Ms(10);
		TInt i;
		for (i = 0; i < 100 && R32(dev, 0xd0000c, v) == KErrNone && !(v & 0x00010000); ++i)
			Kern::NanoWait(200000);
		W32(dev, 0xd0000c, 0x00010000);
		W32(dev, 0x0c00, 0);
		Lock(dev);
		if (i >= 100)
			return KErrTimedOut;
		Unlock(dev);
		for (TUint k = 0; k < sizeof(KErrataB0) / sizeof(KErrataB0[0]); ++k)
			W32(dev, KErrataB0[k][0], KErrataB0[k][1]);
		Lock(dev);
		}
	Upd(dev, 0x2400, 0x04, 0x04);							// IRQ active-high (DT level-high), output on
	Upd(dev, 0x2400, 0x01, 0x01);
	W32(dev, 0xd00040, 0xf9ba7fff);							// INT1/INT3 masks as Linux
	W32(dev, 0xd00048, 0xffffefff);
	// the DT's config: multi-amp TX Hi-Z, boost 10 V, peak 2.6 A, 1 uH, temperature warning 1, IRQ push-pull
	Upd(dev, 0x4800, 0x00200000, 0x00200000);
	Upd(dev, 0x3800, 0xff, 0x96);
	Upd(dev, 0x3804, 0x03, 0x1);
	Upd(dev, 0x3808, 0x7f, 0x14);
	Upd(dev, 0x3810, 0xffff, 0x3c3c);
	Upd(dev, 0x3818, 0x0f, 0);
	Upd(dev, 0x3814, 0xff03, 0x7500);
	Upd(dev, 0x4220, 0x03, 0x1);
	Upd(dev, 0x2400, 0x22, 0x20);
	if (AmpL36[aI])											// the 10 V part: its over-voltage clamp
		{
		Upd(dev, 0x3830, 0x3f, 0x10);
		Unlock(dev);
		Upd(dev, 0x394c, 0x00078000, 0x00060000);
		Upd(dev, 0x3804, 0x04, 0x04);
		Upd(dev, 0x3800, 0xff, 0x96);
		Lock(dev);
		}
	Upd(dev, 0x2020, 0x02, 0);
	// idle: monitors off, amp off and muted, class H off
	Upd(dev, 0x2018, 0x3300, 0);
	Upd(dev, 0x4c00, 0x7f, 0);
	Upd(dev, 0x2024, 0x10, 0x10);
	Upd(dev, 0x2018, 0x01, 0);
	Upd(dev, 0x2014, 0x01, 0);
	Upd(dev, 0x201c, 0x10, 0);
	// TDM slave: DSP_A, bit clock inverted (IB_NF), gated clocks; 16-bit RX; both amps take slot 0. Voice is mono, and
	// the ADSP's channel map {0, 4} (byte offsets) puts channel 2 in slot 2, so Bottom on slot 1 (C1's mixer) was
	// silent: the loudspeaker stayed quiet in the Dev tools test (C6 2026-11-09)
	Upd(dev, 0x4800, 0x1c0, 0x100);
	Upd(dev, 0x4804, 0x007, 0);
	Upd(dev, 0x4808, 0x007, 0);
	Upd(dev, 0x4818, 0x00ff0000, 0x00100000);
	W32(dev, 0x482c, 0);
	// C1's mixer: zero-cross, LDM ramps, LDM select, no soft ramp
	Upd(dev, 0x6c04, 0x400, 0x400);
	Upd(dev, 0x7400, 0x18, 0x18);
	Upd(dev, 0x6808, 0x3f00, 0x3f00);
	return KErrNone;
	}

static void AmpGain(TUint aDev, TInt aAnalog)
	{
	Upd(aDev, 0x6c04, 0x3e0, (TUint32)aAnalog << 5);
	TInt db8 = -3 * 8 * (10 - Volume);						// volume 0..10 = -30..0 dB in 3 dB steps
	Upd(aDev, 0x6000, 0x3ff8, ((TUint32)db8 & 0x7ff) << 3);
	}

static void AmpOn(TUint aDev, TInt aAnalog)
	{
	// the PLL from SCLK at 6.144 MHz (re-kicked at every start, as Linux), the FS monitor window, PCM not PDM
	TUint32 seq[] = { 0x810, 0xd10, 0xd00, 0x500, 0x510 };
	for (TUint i = 0; i < sizeof(seq) / sizeof(seq[0]); ++i)
		W32(aDev, 0x2c04, seq[i]);
	Unlock(aDev);
	Upd(aDev, 0x2d10, 0x00ffffff, 0x00024010);
	Lock(aDev);
	Upd(aDev, 0x7400, 0x01, 0);
	AmpGain(aDev, aAnalog);
	Upd(aDev, 0x483c, 0x00010000, 0x00010000);				// ASP RX1 on
	Upd(aDev, 0x2018, 0x01, 0x01);							// amp on
	Upd(aDev, 0x2014, 0x01, 0x01);							// global enable
	Ms(2);
	TUint32 st = 0;
	R32(aDev, 0xd0002c, st);
	if (st & 0x2000)
		Kern::Printf("[audio] amp %x: PLL not locked (INT4_RAW %08x): is the TDM bit clock running?", aDev, st);
	Upd(aDev, 0x4c00, 0x7f, 0x08);							// PCM from the ASP
	Upd(aDev, 0x2024, 0x10, 0);								// unmute
	}

static void AmpOff(TUint aDev)
	{
	Upd(aDev, 0x2024, 0x10, 0x10);
	Upd(aDev, 0x4c00, 0x7f, 0);
	Upd(aDev, 0x2018, 0x01, 0);
	Upd(aDev, 0x2014, 0x01, 0);
	Ms(2);
	Upd(aDev, 0x483c, 0x00010000, 0);
	}

// ---- RT5514
static const TUint32 KRtPatch[][2] = {
	{ 0x1800101c, 0 }, { 0x18001100, 0x31f }, { 0x18001104, 7 }, { 0x18001108, 0 }, { 0x1800110c, 0 },
	{ 0x18001110, 0 }, { 0x18001114, 1 }, { 0x18001118, 0 }, { 0x18002f08, 6 }, { 0x18002f00, 0x55149 },
	{ 0x18002f00, 0x5514b }, { 0x18002f00, 0x55149 }, { 0xfafafafa, 1 }, { 0x18002f10, 1 }, { 0x18002f10, 0 },
	{ 0x18002f10, 1 }, { 0xfafafafa, 1 }, { 0x18002000, 0x10ec }, { 0xfafafafa, 0 } };
static const TUint32 KRtRegs[][2] = {
	{ 0x2070, 0x00000040 }, { 0x2104, 0x38020041 }, { 0x20ac, 0x44000eee }, { 0x2200, 0x00028604 },
	{ 0x2224, 0x00000800 }, { 0x2180, 0x00000003 }, { 0x2198, 0x10000342 }, { 0x21a8, 0x10000342 } };

static TInt RtProbe()
	{
	TUint32 v = 0;
	if ((R32(KRt, 0x18002ff4, v) != KErrNone || v != 0x10ec5514) && (R32(KRt, 0x18002ff4, v) != KErrNone
		|| v != 0x10ec5514))
		return KErrNotFound;
	RtId = v;
	for (TUint i = 0; i < sizeof(KRtPatch) / sizeof(KRtPatch[0]); ++i)
		W32(KRt, KRtPatch[i][0], KRtPatch[i][1]);
	for (TUint i = 0; i < sizeof(KRtRegs) / sizeof(KRtRegs[0]); ++i)
		RtW(KRtRegs[i][0], KRtRegs[i][1]);
	// TDM slave: DSP_B, LR and BCLK inverted (IB_IF); 8 slots x 16 bits; no docking
	RtUpd(0x2010, 0x06070000, 0x06030000);
	RtUpd(0x2010, 0x50000ff0, 0x10000cc0);
	RtUpd(0x2014, 0xb0000000, 0);
	// PLL1 from BCLK 6.144 -> 12.288 MHz (N 10, K 4, M bypassed: C1's bytes), system clock from it; 16-bit, 48 kHz
	RtUpd(0x2100, 0x7000, 0x3000);
	RtW(0x2260, 0x000400a0);
	RtUpd(0x2264, 0x04, 0x04);
	RtUpd(0x2108, 0x3, 0x3);
	RtUpd(0x2010, 0x3, 0);
	RtUpd(0x2108, 0x770, 0);
	// C1's mixer: both stereo ADCs from DMIC1 (left), +16.5 dB; right mixers off; ADCs muted until a call
	RtUpd(0x20a4, 0x3, 0);
	RtW(0x2190, 0x000204db);
	RtW(0x2194, 0x00020cdb);
	RtW(0x21a0, 0x000204db);
	RtW(0x21a4, 0x00020cdb);
	return KErrNone;
	}

static void RtMute()
	{
	RtUpd(0x2190, 0x80, MicMuted ? 0x80 : 0);
	RtUpd(0x21a0, 0x80, MicMuted ? 0x80 : 0);
	}

// ---- the PM660L analog codec: only its MIC BIAS 1, the DMIC's supply (pm8953 family register defaults)
static const TUint32 KCdcInit[][2] = {
	{ 0xf191, 0xd1 }, { 0xf192, 0xff }, { 0xf0d0, 0xa5 }, { 0xf0da, 0x0f }, { 0xf165, 0x4c }, { 0xf183, 0xa8 },
	{ 0xf185, 0xa4 }, { 0xf1b2, 0x69 }, { 0xf1b7, 0x01 }, { 0xf1d0, 0xa5 }, { 0xf1da, 0x0f }, { 0xf1c0, 0xa2 },
	{ 0xf1b3, 0x41 }, { 0xf1b0, 0x03 }, { 0xf1b4, 0xe1 }, { 0xf194, 0xfa }, { 0xf146, 0x30 }, { 0xf143, 0x00 } };

static void CdcProbe()
	{
	Vrm_Vote("ldoa10", 1800);								// vdd-cdc-io, vdd-cdc-tx-rx-cx, vdd-micbias (DT)
	Vrm_Vote("ldoa1", 880);
	Vrm_Vote("ldoa7", 3088);
	Spmi_Write(KPm660l, 0xf0db, 0x01);
	Spmi_Write(KPm660l, 0xf1db, 0x01);
	for (TUint i = 0; i < sizeof(KCdcInit) / sizeof(KCdcInit[0]); ++i)
		Spmi_Write(KPm660l, KCdcInit[i][0], KCdcInit[i][1]);
	SpmiUpd(0xf046, 0x80, 0x80);							// digital core out of reset
	}

static void MicBias(TBool aOn)
	{
	if (!aOn)
		{
		SpmiUpd(0xf140, 0x80, 0);
		return;
		}
	SpmiUpd(0xf140, 0x0e, 0x04);							// OPA stage-2 tail current
	SpmiUpd(0xf140, 0x80, 0x80);							// MIC BIAS 1 on
	SpmiUpd(0xf142, 0x60, 0x60);							// precharge, then normal
	SpmiUpd(0xf142, 0x60, 0);
	SpmiUpd(0xf140, 0x40, 0x40);							// no external bypass capacitor
	}

// ---- bring-up
static TInt Probe()
	{
	if (Probed)
		return ProbeErr;
	Probed = ETrue;
	Reg(KGccVote1) |= 1u << 24;
	Reg(KSe2RcgCfg) = 1;									// XO 19.2 MHz
	Reg(KSe2RcgCmd) |= 1;
	for (TInt i = 0; i < 100000 && (Reg(KSe2RcgCmd) & 1); ++i)
		{}
	for (TInt i = 0; i < 100000 && (Reg(KSe2Cbcr) & (1u << 31)); ++i)
		{}
	Ctl(KTlmmNorth, 55) = 0x004;							// qup10: I2C10 SDA/SCL
	Ctl(KTlmmNorth, 56) = 0x004;
	for (TInt p = 65; p <= 68; ++p)
		Ctl(KTlmmNorth, p) = 0x0c4;							// pri_mi2s (the mic's TDM), 8 mA
	Ctl(KTlmmNorth, 79) = 0x001;							// amp IRQs: inputs, pulled down
	Ctl(KTlmmNorth, 80) = 0x001;
	Io(KTlmmWest, 14) = 0;									// amp resets: a low pulse, then out of reset
	Io(KTlmmWest, 16) = 0;
	Ctl(KTlmmWest, 14) = 0x200;
	Ctl(KTlmmWest, 16) = 0x200;
	Ms(2);
	Io(KTlmmWest, 14) = 2;
	Io(KTlmmWest, 16) = 2;
	Ms(3);
	TInt proto = I2c_Open(KBus, KSe2, 400);
	if (proto != 3)
		{
		Kern::Printf("[audio] I2C10: protocol %d (clock %08x)", proto, Reg(KSe2Cbcr));
		return ProbeErr = KErrNotReady;
		}
	TInt top = AmpProbe(0), bottom = AmpProbe(1), rt = RtProbe();
	CdcProbe();
	Kern::Printf("[audio] amps: top %d (rev %x%s), bottom %d (rev %x%s); mic %d (%08x); i2c faults %d", top,
		AmpRev[0], AmpL36[0] ? " L36" : " L37", bottom, AmpRev[1], AmpL36[1] ? " L36" : " L37", rt, RtId, Faults);
	return ProbeErr = (top == KErrNone && bottom == KErrNone && rt == KErrNone) ? KErrNone : KErrNotFound;
	}

// the amps' analog gain in every route: Linux's live value in a working call (0x6c04 = 0x633, C6 capture); at 0 the
// earpiece was barely audible
static const TInt KAnalog = 17;

static void Amps()
	{
	if (Route == ESpeaker)
		{
		AmpOn(KTop, KAnalog);
		AmpOn(KBottom, KAnalog);
		}
	else
		{
		AmpOff(KBottom);
		AmpOn(KTop, KAnalog);
		}
	}

static TInt Start(TInt aRoute)
	{
	TInt r = Probe();
	if (r != KErrNone && !AmpId[0] && !RtId)
		return r;
	// the amps' TDM pins on the LPASS LPI TLMM (the ADSP is up: MODEMD started its ports): i2s1 clk, ws, data x2
	Reg(KLpi + 0x1000 * 8) = 0xcc;
	Reg(KLpi + 0x1000 * 9) = 0xcc;
	Reg(KLpi + 0x1000 * 10) = 0xd0;
	Reg(KLpi + 0x1000 * 11) = 0xc8;
	Route = aRoute == ESpeaker ? ESpeaker : ERoute;
	Amps();
	// the mic: PLL1 LDO + PLL1, the ADC clocks, PLL1's LDO, DMIC clock = 12.288 MHz / 4, the bias, then unmute
	RtUpd(0x2008, 0x00050000, 0x00050000);
	RtUpd(0x2104, 0x01800000, 0x01800000);
	RtUpd(0x2264, 0x1, 0x1);
	RtUpd(0x2104, 0x700, 0x200);
	MicBias(ETrue);
	RtMute();
	Running = ETrue;
	Kern::Printf("[audio] on: %s, volume %d%s; i2c faults %d", Route == ESpeaker ? "speaker" : "earpiece", Volume,
		MicMuted ? ", mic muted" : "", Faults);
	return KErrNone;
	}

static void Stop()
	{
	if (!Running)
		return;
	AmpOff(KTop);
	AmpOff(KBottom);
	RtUpd(0x2190, 0x80, 0x80);
	RtUpd(0x21a0, 0x80, 0x80);
	MicBias(EFalse);
	RtUpd(0x2008, 0x00050000, 0);
	RtUpd(0x2104, 0x01800000, 0);
	RtUpd(0x2264, 0x1, 0);
	Running = EFalse;
	Kern::Printf("[audio] off");
	}

TInt A64Audio_Control(TInt aOp, TInt aArg)
	{
	switch (aOp)
		{
	case 0:
		return Probe();
	case 1:
		return Start(aArg);
	case 2:
		Stop();
		return KErrNone;
	case 3:
		Route = aArg == ESpeaker ? ESpeaker : ERoute;
		if (Running)
			Amps();
		return KErrNone;
	case 4:
		Volume = Max(0, Min(10, aArg));
		if (Running)
			{
			AmpGain(KTop, KAnalog);
			if (Route == ESpeaker)
				AmpGain(KBottom, KAnalog);
			}
		return KErrNone;
	case 5:
		MicMuted = aArg != 0;
		if (Running)
			RtMute();
		return KErrNone;
		}
	return KErrArgument;
	}

void A64Audio_Report(void (*aSay)(const char*, ...))
	{
	TUint32 v[4] = { 0, 0, 0, 0 }, b[3] = { 0, 0, 0 };
	if (Probed && ProbeErr == KErrNone)
		{
		R32(KTop, 0x2018, v[0]);
		R32(KTop, 0x201c, v[1]);
		R32(KTop, 0xd00000, v[2]);
		R32(KRt, 0x18002190, v[3]);
		R32(KBottom, 0x2018, b[0]);
		R32(KBottom, 0xd00000, b[1]);
		R32(KBottom, 0x482c, b[2]);
		}
	aSay("audio: probe %d, %s, %s, volume %d%s; top %x rev %x, bottom rev %x, mic %08x; i2c faults %d",
		Probed ? ProbeErr : 1, Running ? "on" : "off", Route == ESpeaker ? "speaker" : "earpiece", Volume,
		MicMuted ? ", mic muted" : "", AmpId[0], AmpRev[0], AmpRev[1], RtId, Faults);
	aSay("audio: top PWR_CTRL2 %08x PWR_CTRL3 %08x INT1 %08x; mic DOWNFILTER0 %08x", v[0], v[1], v[2], v[3]);
	aSay("audio: bottom PWR_CTRL2 %08x INT1 %08x RX slot %x", b[0], b[1], b[2]);
	}
