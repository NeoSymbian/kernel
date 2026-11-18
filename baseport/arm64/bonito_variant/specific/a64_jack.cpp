// SPDX-License-Identifier: Apache-2.0
// a64_jack.cpp -- bonito A8: PM660L headset jack, polled by MODEMD (thread context).
//
// Original implementation from register/sequence facts, not Linux driver code:
//   Linux sound/soc/codecs/msm8916-wcd-{analog,digital}.c;
//   android.googlesource.com/kernel/msm/+/748b6dc2a170e75f816e5734e94c74f236a02aa2/
//     sound/soc/codecs/sdm660_cdc/msm-analog-cdc.c (MBHC register fields);
//   Linux drivers/pinctrl/qcom/pinctrl-{sdm670-lpass-lpi,lpass-lpi}.c;
//   bonito.dts: codec f000, codec 62ec0000, cdc-pdm-default-state.
//
// Call audio op 0 first (CdcProbe owns the rails and PM8953-family defaults).
// MODEMD must serialize this with op 0 and call-audio transitions. For ops 9/10,
// first enable ADSP INT MCLK0 (9.6 MHz), the appropriate 1.536 MHz IBIT clock and
// 48 kHz / S16_LE / stereo / SD0 internal MI2S port. Stop this path BEFORE the
// port/clocks. DT clock selector 0x33 translates to APR clock ID 0x305, not 0x33.
// No APR, GPIO IRQs, RT5514, amps or MIC BIAS1 supply programming here.
// HAL op 8: negative error or bit0 plugged, bit1 headset, bit2 held button,
// bits8..11 button ordinal 0..4 (15 unknown, meaningful only with bit2).
// Two matching samples at least 250 ms apart publish a change; shorter calls
// return cached state. Ops 9/10: zero off, nonzero on; KErrNone or negative error.
// Enabling requires a confirmed plug (9) or headset (10). Op 2 does NOT stop us.
// MICBIAS voltage/cap/precharge fields are SHARED with BIAS1: deliberately leave
// them alone. Consequently the DT's 2.5 V headset bias is not enforced here.
#include <kernel/kern_priv.h>
#include <platform.h>
#include "board.h"
#include "a64_mutex.h"
#include "qcom_spmi.h"
#include "a64_device.h"

namespace {
const TUint KSid = 3;
enum { EPlug = 1, EHeadset = 2, EButton = 4 };
static DMutex* JackLock;
static NFastMutex JackInit;
static TBool Ready, Playing, Recording, RxDirty, TxDirty, PinsReady;
static TBool ObservedPlug, AccessoryMic;
static TBool InsertionPress, InsertionRelease;
static TInt CodecVersion;
static TInt LastError = KErrNotReady;
static TUint Faults;
static TInt Stable, Candidate, HeldButton = -1;
static TBool HaveSample, ValidState;
static TUint32 SampleTick;
static TInt RawElectrical, RawButton, RawIrq;
static DPlatChunkHw* DigitalChunk;
static DPlatChunkHw* PinsChunk;
static TLinAddr Digital, Pins;

static void Delay(TInt aMs) { NKern::Sleep(Max(1, NKern::TimerTicks(aMs))); }

static TInt Lock()
	{
	TInt r = A64Mutex_Init(JackLock, JackInit, _L("HeadsetJack"), KMutexOrdGeneral1);
	if (r == KErrNone)
		{
		NKern::ThreadEnterCS();
		Kern::MutexWait(*JackLock);
		}
	return r;
	}

static TInt Unlock(TInt aResult)
	{
	LastError = aResult < 0 ? aResult : KErrNone;
	Kern::MutexSignal(*JackLock);
	NKern::ThreadLeaveCS();
	return aResult;
	}

static TInt Read(TUint aReg)
	{
	TInt r = Spmi_Read(KSid, aReg);
	if (r < 0)
		++Faults;
	return r;
	}

static TInt Write(TUint aReg, TUint aValue)
	{
	TInt r = Spmi_Write(KSid, aReg, aValue);
	if (r < 0)
		++Faults;
	return r;
	}

static TInt Update(TUint aReg, TUint aMask, TUint aValue)
	{
	TInt v = Read(aReg);
	return v < 0 ? v : Write(aReg, (v & ~aMask) | (aValue & aMask));
	}

// On enable stop at the first bus error. Teardown attempts every step, retaining
// the first error; dirty state is cleared only after a complete successful off.
class TProgram
	{
public:
	TInt iError;
	TBool iCleanup;
	TProgram(TBool aCleanup = EFalse) : iError(KErrNone), iCleanup(aCleanup) {}
	void A(TUint aReg, TUint aMask, TUint aValue)
		{
		if (iError == KErrNone || iCleanup)
			{
			TInt r = Update(aReg, aMask, aValue);
			if (iError == KErrNone)
				iError = r;
			}
		}
	void Wait(TInt aMs) { if (iError == KErrNone || iCleanup) Delay(aMs); }
	};

// The fixed board I/O map does not include these pages. Keep private device
// mappings for boot lifetime; never dereference A64_Io()==0. No pin 8..11 or
// 26..29 mapping/programming (the call amps/DMIC) is needed.
static TInt Map(TUint32 aPhys, TUint aSize, DPlatChunkHw*& aChunk, TLinAddr& aLin)
	{
	if (aLin)
		return KErrNone;
	aLin = A64_Io(aPhys);
	if (aLin)
		return KErrNone;
	TInt r = DPlatChunkHw::New(aChunk, aPhys, aSize, EMapAttrSupRw | EMapAttrFullyBlocking);
	if (r == KErrNone)
		aLin = aChunk->LinearAddress();
	return r;
	}

static void Mmio(TLinAddr aBase, TUint aOffset, TUint aMask, TUint aValue)
	{
	volatile TUint32& v = *(volatile TUint32*)(aBase + aOffset);
	v = (v & ~aMask) | (aValue & aMask);
	__asm__ __volatile__("dsb sy" ::: "memory");
	(void)v; // complete the device write before the next sequenced operation
	}

static void D(TUint aOffset, TUint aMask, TUint aValue) { Mmio(Digital, aOffset, aMask, aValue); }

static TInt DigitalReady()
	{
	TInt r = Map(0x62ec0000, 0x1000, DigitalChunk, Digital);
	if (r != KErrNone)
		return r;
	r = Map(0x62b52000, 0x8000, PinsChunk, Pins);
	if (r != KErrNone)
		return r;
	// DT pin functions / 4 mA (8 mA on TX). CLK and SYNC and RX are
	// output-low; TX/comp are inputs. Preserve slew/reserved bits.
	const TUint cfg[8] = { 0x248, 0x24c, 0x0c8, 0x248, 0x048, 0x244, 0x044, 0x244 };
	for (TUint p = 0; !PinsReady && p < 8; ++p)
		{
		if (cfg[p] & 0x200)
			Mmio(Pins, 0x1000 * p + 4, 2, 0);
		Mmio(Pins, 0x1000 * p, 0x3ff, cfg[p]);
		}
	PinsReady = ETrue;
	D(0x0a4, 1, 1); // 9.6 MHz MCLK
	D(0x024, 1, 1); // MCLK gate
	D(0x028, 3, 3); // PDM clock, feedback source
	D(0x01c, 4, 4); // codec connectivity clock (shared RX/TX)
	return KErrNone;
	}

static void DigitalIdle()
	{
	if (!Digital || RxDirty || TxDirty)
		return;
	D(0x01c, 4, 0);
	D(0x028, 3, 0);
	D(0x024, 1, 0);
	// Keep pins muxed: no GPIO glitches during the next port start.
	}

static TInt PlayOff()
	{
	if (!RxDirty)
		return KErrNone;
	D(0x054, 1, 1); // RX1/2 digital mute before removing PA power
	D(0x074, 1, 1);
	TProgram s(ETrue);
	// Analog power-down is completed while the ADSP clocks still run.
	s.A(0xf197, 0x30, 0); // HPHL/R PA
	if (Digital)
		{
		D(0x020, 3, 0);
		D(0x000, 3, 3);
		D(0x000, 3, 0);
		}
	s.Wait(20); // PM8953-family PA post-power-down settling
	if (CodecVersion >= 4)
		{
		s.A(0xf19a, 4, 0);
		s.A(0xf19c, 4, 0);
		}
	s.A(0xf19b, 0x08, 0); // DACs
	s.A(0xf19d, 0x08, 0);
	s.A(0xf180, 1, 0); // charge pump
	s.Wait(1);
	s.A(0xf193, 0x81, 0); // RX bias / DAC reference
	s.A(0xf049, 3, 0);
	s.A(0xf04a, 0xc3, 0); // RX1/2, NCP and RX PDM, preserve MBHC/TX
	D(0x00c, 0x10, 0);
	Playing = EFalse;
	if (s.iError == KErrNone)
		RxDirty = EFalse;
	DigitalIdle();
	return s.iError;
	}

static TInt PlayOn()
	{
	if (Playing)
		return KErrNone;
	if (RxDirty)
		{
		TInt r = PlayOff();
		if (r != KErrNone)
			return r;
		}
	TInt r = DigitalReady();
	if (r != KErrNone)
		return r;
	RxDirty = ETrue;
	D(0x054, 1, 1);
	D(0x074, 1, 1);
	D(0x00c, 0x37, 0x33); // 48 kHz, 16 bits, RX I2S clock
	D(0x180, 0x3f, 3); // RX1 MIX1 INP1 = I2S RX1; INP2 = ZERO
	D(0x184, 7, 0); // INP3 = ZERO
	D(0x188, 3, 0); // sidetone = ZERO
	D(0x18c, 0x3f, 4); // RX2 MIX1 INP1 = I2S RX2
	D(0x190, 7, 0);
	D(0x194, 3, 0);
	D(0x05c, 0xff, 0xf4); // signed -12 dB
	D(0x07c, 0xff, 0xf4);
	D(0x020, 3, 3); // interpolators
	Delay(10);
	D(0x05c, 0xff, 0xf4); // latch gain after enabling interpolator
	D(0x07c, 0xff, 0xf4);
	TProgram s;
	s.A(0xf048, 0x0c, 0x0c); // A_MCLK/MCLK2; kept on for MBHC
	s.A(0xf04a, 0xc3, 0xc3);
	s.A(0xf049, 3, 3);
	s.A(0xf052, 1, 1); // HPHR uses RX2
	s.A(0xf193, 0x81, 0x81); // DAC reference / RX bias
	s.A(0xf19b, 2, 2); // DAC data reset pulse
	s.A(0xf19d, 2, 2);
	s.A(0xf19b, 8, 8);
	s.A(0xf19d, 8, 8);
	s.A(0xf19b, 2, 0);
	s.A(0xf19d, 2, 0);
	s.A(0xf183, 0x20, 0x20); // NCP feedback clock inversion
	s.A(0xf180, 1, 1);
	s.Wait(1);
	s.A(0xf197, 0x30, 0x30);
	s.Wait(7);
	if (CodecVersion >= 4)
		{
		s.A(0xf19a, 4, 4); // PM8953-family PA settling quirk
		s.A(0xf19c, 4, 4);
		}
	if (s.iError != KErrNone)
		{
		PlayOff();
		return s.iError;
		}
	D(0x054, 1, 0);
	D(0x074, 1, 0);
	Playing = ETrue;
	return KErrNone;
	}

static TInt MicOff()
	{
	if (!TxDirty)
		return KErrNone;
	D(0x288, 1, 1);
	D(0x2a8, 1, 1);
	TProgram s(ETrue);
	s.A(0xf161, 0x80, 0); // AMIC2 ADC off, BIAS2 stays on for MBHC
	s.A(0xf051, 3, 2); // serial TX2 = ZERO
	s.A(0xf163, 0x10, 0); // also recover a failed enable's TXFE init
	s.A(0xf142, 2, 0); // only ADC2 CFILT reference; not BIAS1 precharge
	s.A(0xf049, 0x20, 0);
	s.A(0xf04a, 0x10, 0);
	D(0x018, 3, 0); // both decimators (stereo transport of one AMIC)
	D(0x004, 3, 3);
	D(0x004, 3, 0);
	D(0x010, 0x10, 0);
	D(0x1a0, 0x3f, 0);
	Recording = EFalse;
	if (s.iError == KErrNone)
		TxDirty = EFalse;
	DigitalIdle();
	return s.iError;
	}

static TInt MicOn()
	{
	if (Recording)
		return KErrNone;
	if (TxDirty)
		{
		TInt r = MicOff();
		if (r != KErrNone)
			return r;
		}
	TInt r = DigitalReady();
	if (r != KErrNone)
		return r;
	TxDirty = ETrue;
	D(0x288, 1, 1);
	D(0x2a8, 1, 1);
	D(0x010, 0x37, 0x33); // 48 kHz / 16-bit TX I2S
	D(0x1a0, 0x3f, 0x12); // DEC1 and DEC2 from ADC2 (dual mono)
	D(0x28c, 0x39, 0x28); // AMIC, 150 Hz HPF, bypass while settling
	D(0x2ac, 0x39, 0x28);
	D(0x284, 0xff, 0);
	D(0x2a4, 0xff, 0); // 0 dB digital
	TProgram s;
	s.A(0xf048, 0x0c, 0x0c);
	s.A(0xf049, 0x20, 0x20);
	s.A(0xf04a, 0x10, 0x10);
	s.A(0xf142, 2, 2); // ADC2 CFILT HPF reference, independent mask
	s.Wait(10);
	s.A(0xf163, 0x10, 0x10); // TX2 front-end initialization
	s.A(0xf051, 3, 0); // ADC2 -> serial TX2
	s.A(0xf161, 0xf8, 0x80); // ADC2 on, analog gain 0 dB
	s.Wait(12);
	s.A(0xf163, 0x10, 0);
	if (s.iError != KErrNone)
		{
		MicOff();
		return s.iError;
		}
	D(0x018, 3, 3);
	D(0x28c, 8, 0); // HPF on
	D(0x2ac, 8, 0);
	D(0x284, 0xff, 0); // latch gains after decimators enabled
	D(0x2a4, 0xff, 0);
	D(0x288, 1, 0);
	D(0x2a8, 1, 0);
	Recording = ETrue;
	return KErrNone;
	}

static TInt SetupFailed(TInt aError)
	{
	// Outputs have not been enabled; undo our detector and BIAS2 only. The
	// original bus error remains visible even if this best-effort cleanup fails.
	Update(0xf151, 0x80, 0);
	Update(0xf144, 0x80, 0);
	return aError;
	}

static TInt Setup()
	{
	if (Ready)
		return KErrNone;
	TInt v = Read(0xf046);
	if (v < 0)
		return v;
	if (!(v & 0x80))
		return KErrNotReady; // require existing CdcProbe, never reset a live DMIC
	v = Read(0xf005);
	if (v < 0)
		return v;
	CodecVersion = v;
	TProgram s;
	// Configure rising-edge PMIC latches before enabling the detector. Delivery
	// stays masked: MODEMD consumes f018/f014 instead of installing IRQs.
	s.A(0xf011, 0x30, 0x30);
	s.A(0xf012, 0x30, 0x30);
	s.A(0xf013, 0x30, 0);
	if (s.iError != KErrNone)
		return s.iError;
	TInt r = Write(0xf016, 0x30);
	if (r == KErrNone)
		r = Write(0xf014, 0x30); // discard bootloader latches, not new insertion events
	if (r != KErrNone)
		return r;
	s.A(0xf144, 0xa0, 0x80); // MIC BIAS2 enabled, pull-down removed
	// Leave f140/f141 and f142[6:5] untouched: these also affect the RT5514 supply.
	s.Wait(50); // MICBIAS2 / accessory comparator settling
	s.A(0xf147, 0xff, 0xb4); // L detector, insertion, auto clamp, MBHC bias
	s.A(0xf150, 0xff, 0xf9); // HPHL and GND normally open
	s.A(0xf152, 0xff, 0x98); // 256 ms insertion / 16 ms button HW debounce
	s.A(0xf04a, 8, 8); // MBHC clock, preserve audio clocks
	const TUint threshold[5] = { 0x18, 0x30, 0x4c, 0x8c, 0x8c };
	for (TUint b = 0; b < 5; ++b)
		s.A(0xf153 + b, 0xfc, threshold[b]); // both DT tables are identical
	s.A(0xf151, 0xf0, 0x80); // FSM on, internal button current source off
	if (s.iError != KErrNone)
		return SetupFailed(s.iError);
	// Retain insertion-time button events: they also identify accessory type.
	Ready = ETrue;
	return KErrNone;
	}

static TInt ButtonNumber(TInt aCode)
	{
	switch (aCode & 0x1f)
		{
	case 0: return 0;
	case 1: return 1;
	case 3: return 2;
	case 7: return 3;
	case 15: return 4;
		}
	return 15; // press event known, button number unavailable
	}

static TInt Poll()
	{
	TInt r = Setup();
	if (r != KErrNone)
		return r;
	TUint32 now = NKern::TickCount();
	if (HaveSample && now - SampleTick < (TUint32)Max(1, NKern::TimerTicks(250)))
		return Stable; // rapid callers cannot satisfy the two-poll debounce
	TInt elect = Read(0xf159), irq = Read(0xf018), button = Read(0xf158);
	if (elect < 0 || irq < 0 || button < 0)
		{
		HaveSample = ValidState = EFalse; // an I/O gap breaks stability
		return elect < 0 ? elect : irq < 0 ? irq : button;
		}
	RawElectrical = elect;
	RawButton = button;
	RawIrq = irq;
	r = Write(0xf014, irq & 0x30); // acknowledge only MBHC button events
	if (r != KErrNone)
		{
		HaveSample = ValidState = EFalse;
		return r;
		}
	TBool plugged = !(elect & 0x10); // SWCH_LEVEL_REMOVE, NOT IRQ RT_STS
	if (plugged != ObservedPlug)
		{
		r = Update(0xf147, 0x20, plugged ? 0 : 0x20); // arm the opposite transition
		if (r != KErrNone)
			{
			HaveSample = ValidState = EFalse;
			return r;
			}
		}
	if (!plugged)
		{
		if (ObservedPlug)
			InsertionPress = InsertionRelease = EFalse; // begin a new accessory epoch
		ObservedPlug = AccessoryMic = EFalse;
		HeldButton = -1;
		// MBHC button edges can precede the debounced mechanical indication.
		// Preserve those insertion clues instead of consuming and losing them.
		if (!(Stable & EPlug))
			{
			InsertionPress |= (irq & 0x20) != 0;
			InsertionRelease |= (irq & 0x10) && !(button & 0x1f);
			}
		}
	else
		{
		if (!ObservedPlug)
			{
			// Downstream insertion test with BIAS2 on: no press + button0 +
			// comparator low is a headset. Press-only suggests headphones.
			// BTN0 release is the mainline headset evidence. A button held
			// during insertion can delay correct classification until release.
			AccessoryMic = !(elect & 1) && !(button & 0x1f) &&
				(InsertionRelease || (!InsertionPress && !(irq & 0x20)));
			InsertionPress = InsertionRelease = EFalse;
			ObservedPlug = ETrue;
			HeldButton = -1;
			}
		if ((irq & 0x10) && !(button & 0x1f) && !(elect & 1))
			AccessoryMic = ETrue;
		if (irq & 0x10)
			HeldButton = -1; // release wins when both edges accumulated
		else if ((irq & 0x20) && AccessoryMic && (Stable & EHeadset))
			HeldButton = ButtonNumber(button);
		}
	TInt raw = plugged ? EPlug | (AccessoryMic ? EHeadset : 0) : 0;
	if ((raw & EHeadset) && HeldButton >= 0)
		raw |= EButton | (HeldButton << 8); // ordinal 0..4, 15 unknown; valid with bit2
	if (HaveSample && Candidate == raw)
		{
		Stable = raw;
		ValidState = ETrue;
		}
	Candidate = raw;
	SampleTick = now;
	HaveSample = ETrue;
	// Local protection only: MODEMD still owns stopping the ADSP ports/clocks.
	// Stop on confirmed removal; retry any incomplete cleanup on later polls.
	if (ValidState && !(Stable & EPlug))
		{
		TInt rx = PlayOff(), tx = MicOff();
		if (rx != KErrNone || tx != KErrNone)
			return rx != KErrNone ? rx : tx;
		}
	return Stable;
	}

static const char* StateName()
	{
	return !Ready ? "not polled" : !ValidState ? "debouncing/unknown" :
		!(Stable & EPlug) ? "unplugged" : (Stable & EHeadset) ? "plugged headset" : "plugged headphones";
	}

} // namespace

TInt A64Jack_Poll()
	{
	TInt r = Lock();
	if (r != KErrNone)
		return r;
	return Unlock(Poll());
	}

TInt A64Jack_Play(TBool aOn)
	{
	TInt r = Lock();
	if (r != KErrNone)
		return r;
	if (!aOn)
		return Unlock(PlayOff());
	r = Setup();
	if (r == KErrNone)
		r = ValidState && (Stable & EPlug) ? PlayOn() : KErrNotReady;
	return Unlock(r);
	}

TInt A64Jack_Mic(TBool aOn)
	{
	TInt r = Lock();
	if (r != KErrNone)
		return r;
	if (!aOn)
		return Unlock(MicOff());
	r = Setup();
	if (r == KErrNone)
		r = ValidState && (Stable & EHeadset) ? MicOn() : KErrNotReady;
	return Unlock(r);
	}

void A64Jack_Report(void (*aSay)(const char*, ...))
	{
	TInt r = Lock();
	if (r != KErrNone)
		{
		aSay("jack: lock error %d", r);
		return;
		}
	aSay("jack: %s, status %03x; play %d mic %d; error %d faults %d; MBHC %02x/%02x IRQ %02x",
		StateName(), Stable, Playing, Recording, LastError, Faults, RawElectrical, RawButton, RawIrq);
	Kern::MutexSignal(*JackLock);
	NKern::ThreadLeaveCS();
	}

static TA64DevState JackStatus(TDes8& aDetail)
	{
	TInt r = Lock();
	if (r != KErrNone)
		{
		A64Device_Format(aDetail, "lock error %d", r);
		return EA64DevError;
		}
	A64Device_Format(aDetail, "%s; play %d mic %d; status %03x error %d faults %d",
		StateName(), Playing, Recording, Stable, LastError, Faults);
	TA64DevState state = !Ready ? EA64DevOff : LastError < 0 ? EA64DevError : !ValidState ? EA64DevStarting :
		Faults || RxDirty != Playing || TxDirty != Recording ? EA64DevPartial : EA64DevOk;
	Kern::MutexSignal(*JackLock);
	NKern::ThreadLeaveCS();
	return state;
	}
A64_DEVICE("jack", "Headset jack", "a64_jack", JackStatus);
