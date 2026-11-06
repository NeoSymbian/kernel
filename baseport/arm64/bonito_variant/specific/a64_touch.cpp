// SPDX-License-Identifier: Apache-2.0
//
// a64_touch.cpp -- the Pixel 3a XL touchscreen: Synaptics RMI4 at I2C 0x20 on the GENI serial engine i2c@a84000
// (QUP wrapper 1, SE1; SDA/SCL = TLMM gpio6/7 "qup9"), reset gpio99, attention gpio125, ts-switch gpio135, supply
// ts_1p8 = PM660 GPIO12 (DTB). ABL leaves all of it off (pins at reset defaults, clocks off, supply low), so:
//   clocks on -> pins -> supply on, reset released -> the SE's I2C firmware -> the RMI4 function table.
// Register offsets: Linux gcc-sdm845.c (covers sdm670), pinctrl-sdm670.c, pinctrl-spmi-gpio.c, rmi_driver.c.
// Bring-up step: it reports (also on each Volume Up press: boot-time lines scroll out of the console strip) but
// doesn't post touches yet.
#include <kernel/kern_priv.h>
#include "board.h"
#include "qcom_spmi.h"
#include "qcom_geni_i2c.h"

static volatile TUint32& Reg(TUint32 aPhys) { return *(volatile TUint32*)A64_Io(aPhys); }

// GCC: the QUP wrapper 1 clocks. Vote bits 20/21 = wrapper 1 m/s AHB, 23 = SE1; 18/19 = the wrapper's core and
// core-2x clocks in the downstream kernel (upstream left them to an interconnect vote; unused bits otherwise).
const TUint32 KGcc = 0x00100000;
enum { EGccVote1 = 0x5200c, EWrap1MAhbCbcr = 0x1800c, EWrap1SAhbCbcr = 0x18010, ESe1Cbcr = 0x18144,
	ESe1RcgCmd = 0x18148, ESe1RcgCfg = 0x1814c };
const TUint32 KVoteBits = (1u << 18) | (1u << 19) | (1u << 20) | (1u << 21) | (1u << 23);
const TUint32 KCbcrOff = 1u << 31;
// TLMM: per pin CTL (pull [1:0], function [5:2], drive [8:6], OE [9]) and IO (in [0], out [1])
const TUint32 KTlmm = 0x03400000, KNorth = 0x00500000, KWest = 0x00100000;
const TUint32 KPullNone = 0, KPullUp = 3, KOe = 1u << 9;
static volatile TUint32& Ctl(TUint32 aTile, TInt aPin) { return Reg(KTlmm + aTile + 0x1000 * aPin); }
static volatile TUint32& Io(TUint32 aTile, TInt aPin) { return Reg(KTlmm + aTile + 0x1000 * aPin + 4); }
const TUint32 KSe1 = 0x00a84000;
const TUint KRmiAddr = 0x20;

// the report, kept for the Volume Up re-print
static TBuf8<120> Lines[20];
static TInt NLines;
static void Log(const char* aFmt, ...)
	{
	VA_LIST list;
	VA_START(list, aFmt);
	TBuf8<120> b;
	Kern::AppendFormat(b, aFmt, list);
	VA_END(list);
	Kern::Printf("[touch] %S", &b);
	if (NLines < (TInt)(sizeof(Lines) / sizeof(Lines[0])))
		Lines[NLines++] = b;
	}

static const char* volatile CurStep = "not started";
static void Step(const char* aStep)
	{
	CurStep = aStep;
	Kern::Printf("[touch] step: %s", aStep);
	}

void A64TouchDump()
	{
	Kern::Printf("[touch] last step: %s", CurStep);
	for (TInt i = 0; i < NLines; ++i)
		Kern::Printf("[touch] %S", &Lines[i]);
	}

static TBool ClocksOn()
	{
	Step("clock vote");
	Reg(KGcc + EGccVote1) |= KVoteBits;
	Step("clock rcg");
	Reg(KGcc + ESe1RcgCfg) = 1;							// source 0 = XO 19.2 MHz, pre-divider 1 (encoded 2*1-1)
	Reg(KGcc + ESe1RcgCmd) |= 1;							// UPDATE
	TInt i;
	for (i = 0; i < 100000 && (Reg(KGcc + ESe1RcgCmd) & 1); ++i)
		{}
	Step("clock wait");
	TBool on = EFalse;
	for (TInt j = 0; j < 100000 && !on; ++j)
		on = !((Reg(KGcc + EWrap1MAhbCbcr) | Reg(KGcc + EWrap1SAhbCbcr) | Reg(KGcc + ESe1Cbcr)) & KCbcrOff);
	Log("clocks %s: vote %08x m_ahb %08x s_ahb %08x se1 %08x rcg %08x/%08x", on ? "on" : "STILL OFF",
		Reg(KGcc + EGccVote1), Reg(KGcc + EWrap1MAhbCbcr), Reg(KGcc + EWrap1SAhbCbcr), Reg(KGcc + ESe1Cbcr),
		Reg(KGcc + ESe1RcgCmd), Reg(KGcc + ESe1RcgCfg));
	return on;
	}

// PM660 GPIO12 (the ts_1p8 regulator's enable) to a digital output driving high
static TInt SupplyOn()
	{
	TInt sub = Spmi_Read(0, 0xcb05), r;
	if (sub == 0x10 || sub == 0x11)						// LV/MV: mode in MODE_CTL, level in DIG_OUT_SOURCE_CTL bit 7
		{
		r = Spmi_Write(0, 0xcb40, 1);
		if (r == KErrNone)
			r = Spmi_Write(0, 0xcb44, 0x80);
		}
	else													// older GPIO: MODE_CTL = dir << 4 | function << 1 | value
		r = Spmi_Write(0, 0xcb40, (1 << 4) | 1);
	if (r == KErrNone)
		r = Spmi_Write(0, 0xcb46, 0x80);					// enabled
	Log("supply: subtype %02x -> %d; mode %02x out %02x en %02x status1 %02x", sub, r, Spmi_Read(0, 0xcb40),
		Spmi_Read(0, 0xcb44), Spmi_Read(0, 0xcb46), Spmi_Read(0, 0xcb08));
	return r;
	}

static TInt RmiRead(TUint aAddr, TUint8* aBuf, TInt aLen)
	{
	TUint8 page[2] = { 0xff, (TUint8)(aAddr >> 8) };
	TInt r = I2c_WriteRead(KRmiAddr, page, 2, NULL, 0);
	if (r != KErrNone)
		return r;
	TUint8 a = (TUint8)aAddr;
	return I2c_WriteRead(KRmiAddr, &a, 1, aBuf, aLen);
	}

static void BringUpDfcFn(TAny*)
	{
	NKern::ThreadEnterCS();
	Step("pins");
	Io(KNorth, 99) = 0;									// hold the controller in reset while its supply comes up
	Ctl(KNorth, 99) = KPullUp | KOe;
	Io(KWest, 135) = 0;									// ts-switch low (DTB)
	Ctl(KWest, 135) = KPullNone | KOe;
	Ctl(KNorth, 125) = KPullNone;							// attention: input
	Ctl(KNorth, 6) = KPullNone | (1 << 2);					// function 1 = qup9: SDA, SCL
	Ctl(KNorth, 7) = KPullNone | (1 << 2);
	Step("clocks");
	TBool clocks = ClocksOn();
	Step("supply");
	TInt supply = SupplyOn();
	Step("reset release");
	NKern::Sleep(NKern::TimerTicks(10));
	Io(KNorth, 99) = 2;									// out of reset
	NKern::Sleep(NKern::TimerTicks(200));					// syna,reset-delay-ms
	Log("pins: gpio6 %08x gpio7 %08x gpio99 %08x/%x gpio125 %08x/%x gpio135 %08x", Ctl(KNorth, 6), Ctl(KNorth, 7),
		Ctl(KNorth, 99), Io(KNorth, 99), Ctl(KNorth, 125), Io(KNorth, 125), Ctl(KWest, 135));
	if (!clocks || supply != KErrNone)
		{
		Log("stopped: clocks or supply");
		NKern::ThreadLeaveCS();
		return;
		}
	Step("SE1");
	TInt proto = I2c_Init(KSe1);
	Log("SE1 firmware protocol %d (I2C = 3), status %08x", proto, Reg(KSe1 + 0x40));
	if (proto != 3)
		{
		NKern::ThreadLeaveCS();
		return;
		}
	Step("rmi");
	// the RMI4 page description table, page 0 and 1: function number, query/command/control/data bases
	for (TInt page = 0; page < 2; ++page)
		for (TInt addr = 0xe9; addr >= 0x05; addr -= 6)
			{
			TUint8 e[6];
			Step("rmi read");
			TInt r = RmiRead((page << 8) | addr, e, 6);
			if (r != KErrNone)
				{
				Log("rmi page %d @%02x: error %d", page, addr, r);
				page = 2;
				break;
				}
			if (e[5] == 0 || e[5] == 0xff)
				break;
			Log("rmi page %d F%02x: query %02x cmd %02x ctrl %02x data %02x ints %d v%d", page, e[5], e[0], e[1],
				e[2], e[3], e[4] & 7, (e[4] >> 5) & 3);
			}
	Step("done");
	NKern::ThreadLeaveCS();
	}

static TDfc BringUpDfc(BringUpDfcFn, NULL, 1);
static TDynamicDfcQue* TouchQ;

void A64StartTouch()
	{
	_LIT(KName, "TouchQ");
	TInt r = Kern::DynamicDfcQCreate(TouchQ, 24, KName);	// its own thread: a stuck bring-up must not stop the keys
	if (r != KErrNone)
		{
		Kern::Printf("[touch] no DFC queue: %d", r);
		return;
		}
	BringUpDfc.SetDfcQ(TouchQ);
	BringUpDfc.Enque();
	}
