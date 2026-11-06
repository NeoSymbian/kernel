// SPDX-License-Identifier: Apache-2.0
//
// a64_touch.cpp -- the Pixel 3a XL touchscreen: Synaptics RMI4 at I2C 0x20 on the GENI serial engine i2c@a84000
// (QUP wrapper 1, SE1; SDA/SCL = TLMM gpio6/7 "qup9"), reset gpio99, attention gpio125, ts-switch gpio135, supply
// ts_1p8 = PM660 GPIO12 (DTB). ABL leaves all of it off (pins at reset defaults, clocks off, supply low), so:
//   clocks on -> pins -> supply on, reset released -> the SE's I2C firmware -> the RMI4 function table.
// Register offsets: Linux gcc-sdm845.c (covers sdm670), pinctrl-sdm670.c, pinctrl-spmi-gpio.c, rmi_driver.c, rmi_f12.c.
// Then the Synaptics F12 2-D sensor's first object is polled and posted as EButton1Down/EPointerMove/EButton1Up in
// screen pixels (Kern::AddEvent). Volume Up re-prints the bring-up report (boot-time lines scroll out of the strip).
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

static TDynamicDfcQue* TouchQ;								// the bring-up and the poll run here
static TInt RmiPage = -1;									// the controller's page register, as last written

static TInt RmiSetPage(TUint aAddr)
	{
	TInt page = (aAddr >> 8) & 0xff;
	if (page == RmiPage)
		return KErrNone;
	TUint8 w[2] = { 0xff, (TUint8)page };
	TInt r = I2c_WriteRead(KRmiAddr, w, 2, NULL, 0);
	RmiPage = r == KErrNone ? page : -1;
	return r;
	}

static TInt RmiRead(TUint aAddr, TUint8* aBuf, TInt aLen)
	{
	TInt r = RmiSetPage(aAddr);
	if (r != KErrNone)
		return r;
	TUint8 a = (TUint8)aAddr;
	r = I2c_WriteRead(KRmiAddr, &a, 1, aBuf, aLen);
	if (r != KErrNone)
		RmiPage = -1;										// unknown after a failed transfer: select it again
	return r;
	}

static TInt RmiWrite(TUint aAddr, const TUint8* aData, TInt aLen)
	{
	TInt r = RmiSetPage(aAddr);
	if (r != KErrNone)
		return r;
	TUint8 w[8];
	if (aLen > 7)
		return KErrArgument;
	w[0] = (TUint8)aAddr;
	memcpy(w + 1, aData, aLen);
	r = I2c_WriteRead(KRmiAddr, w, aLen + 1, NULL, 0);
	if (r != KErrNone)
		RmiPage = -1;
	return r;
	}

// ---- RMI4 register descriptors (Linux rmi_driver.c rmi_read_register_desc): which packet registers a function
// has, each one's size and subpackets. A control/query packet register has one address: base + its index among the
// present ones; the data packet is read as one block from the data base, a register at the byte sum below it.
struct SDesc
	{
	TUint64 iPresent;										// registers 0..63
	TUint32 iSize[64];
	TUint8 iSubpackets[64], iSub0[64];
	TBool Has(TInt aReg) const { return (iPresent >> aReg) & 1; }
	TUint32 Index(TInt aReg) const
		{
		TUint32 i = 0;
		for (TInt k = 0; k < aReg; ++k)
			i += Has(k);
		return i;
		}
	TUint32 Offset(TInt aReg) const
		{
		TUint32 o = 0;
		for (TInt k = 0; k < aReg; ++k)
			if (Has(k))
				o += iSize[k];
		return o;
		}
	};

static TInt ReadDesc(TUint aAddr, SDesc& aDesc)
	{
	memclr(&aDesc, sizeof(aDesc));
	TUint8 n, p[35], st[64];
	TInt r = RmiRead(aAddr, &n, 1);
	if (r == KErrNone && (n == 0 || n > sizeof(p)))
		r = KErrCorrupt;
	if (r == KErrNone)
		r = RmiRead(aAddr + 1, p, n);
	if (r != KErrNone)
		return r;
	TInt first = 1;
	TUint size = p[0];
	if (p[0] == 0)
		{
		first = 3;
		size = p[1] | (p[2] << 8);
		}
	if (size > sizeof(st))
		return KErrTooBig;
	r = RmiRead(aAddr + 2, st, size);
	if (r != KErrNone)
		return r;
	TUint off = 0;
	for (TInt i = first, reg = 0; i < n; ++i)
		for (TInt b = 0; b < 8; ++b, ++reg)
			{
			if (!(p[i] & (1 << b)))
				continue;
			if (off >= size)
				return KErrCorrupt;
			TUint32 rs = st[off++];
			if (rs == 0)
				{
				rs = st[off] | (st[off + 1] << 8);
				off += 2;
				if (rs == 0)
					{
					rs = st[off] | (st[off + 1] << 8) | (st[off + 2] << 16) | ((TUint32)st[off + 3] << 24);
					off += 4;
					}
				}
			TInt subs = 0;
			TUint8 sub0 = st[off];
			do
				for (TInt k = 0; k < 7; ++k)
					subs += (st[off] >> k) & 1;
			while ((st[off++] & 0x80) && off < size);
			if (reg < 64)
				{
				aDesc.iPresent |= 1ull << reg;
				aDesc.iSize[reg] = rs;
				aDesc.iSubpackets[reg] = (TUint8)subs;
				aDesc.iSub0[reg] = sub0;
				}
			}
	return KErrNone;
	}

// ---- the function table, the sensor, the poll
struct SFn { TUint8 iQuery, iCtrl, iData; TBool iFound; };
static SFn F01, F12;
static TUint32 MaxX = 1080, MaxY = 2160;				// the sensor's range (F12 control 8)
static TUint32 Data1Off;								// F12 data register 1 (the finger objects, 8 bytes each) in the packet

static TInt RmiSetup()
	{
	TUint8 q0;
	TInt r = RmiRead(F12.iQuery, &q0, 1);
	if (r != KErrNone || !(q0 & 1))
		{
		Log("F12 query0 %02x -> %d: no register descriptors", q0, r);
		return KErrNotSupported;
		}
	static SDesc ctrl, data;
	r = ReadDesc(F12.iQuery + 4, ctrl);
	if (r == KErrNone)
		r = ReadDesc(F12.iQuery + 7, data);
	if (r != KErrNone || !data.Has(1))
		{
		Log("F12 descriptors -> %d, data1 %d", r, data.Has(1));
		return r != KErrNone ? r : KErrNotSupported;
		}
	Data1Off = data.Offset(1);
	if (Data1Off + 8 > 64)
		{
		Log("F12 data1 at byte %d: too far into the packet", Data1Off);
		return KErrTooBig;
		}
	if (ctrl.Has(8) && (ctrl.iSub0[8] & 1))
		{
		TUint8 c8[4];
		if (RmiRead(F12.iCtrl + ctrl.Index(8), c8, 4) == KErrNone && (c8[0] | c8[1]) && (c8[2] | c8[3]))
			{
			MaxX = c8[0] | (c8[1] << 8);
			MaxY = c8[2] | (c8[3] << 8);
			}
		}
	// F01 device control: no sleep, configured; interrupt enable for F12 (source 1, after F01's 0)
	TUint8 c[2];
	r = RmiRead(F01.iCtrl, c, 2);
	if (r == KErrNone)
		{
		c[0] = (TUint8)((c[0] & ~3) | (1 << 2) | (1 << 7));
		c[1] |= 0x02;
		r = RmiWrite(F01.iCtrl, c, 2);
		}
	Log("F12 max %d x %d, data1 at byte %d (%d objects); F01 ctrl %02x %02x -> %d", MaxX, MaxY, Data1Off,
		data.iSubpackets[1], c[0], c[1], r);
	return KErrNone;
	}

// shortcut: one finger, polled at 10 ms off the attention pin (no TLMM interrupt); upgrade to the TLMM summary
// interrupt + multi-touch for neoui gestures
const TInt KPollMs = 10, KFallbackTicks = 10;			// read anyway every 100 ms in case attention never asserts
const TBool KFlipX = EFalse, KFlipY = EFalse, KSwapXY = EFalse;	// set from the corner test if an axis is off
const TInt KScreenW = 1080, KScreenH = 2160, KUserH = 1800;	// the user side owns the top 1800 rows

static void PollDfcFn(TAny*);
static TDfc PollDfc(PollDfcFn, NULL, 1);
static void PollTimerFn(TAny*) { PollDfc.Add(); }
static NTimer PollTimer(PollTimerFn, NULL);
static TBool Down;
static TInt LastX, LastY, Quiet, Errors, Logged, AttnSeen;

static void Post(TRawEvent::TType aType, TInt aX, TInt aY)
	{
	TRawEvent e;
	e.Set(aType, aX, aY);
	Kern::AddEvent(e);
	}

static void PollDfcFn(TAny*)
	{
	TInt next = KPollMs;
	TBool attn = !(Io(KNorth, 125) & 1);
	AttnSeen += attn;
	if (attn || ++Quiet >= KFallbackTicks)
		{
		Quiet = 0;
		TUint8 st, pkt[64];
		TInt r = RmiRead(F01.iData + 1, &st, 1);			// interrupt status: reading it releases attention
		if (r == KErrNone)
			r = RmiRead(F12.iData, pkt, Data1Off + 8);		// the data packet up to the first object
		const TUint8* o = pkt + Data1Off;
		if (r != KErrNone)
			{
			if (++Errors == 50)
				Log("poll: 50 I2C errors in a row (last %d), slowing to 1 s", r);
			next = Errors >= 50 ? 1000 : KPollMs;
			}
		else
			{
			Errors = 0;
			TInt rx = o[1] | (o[2] << 8), ry = o[3] | (o[4] << 8);
			TInt x = rx, y = ry;
			TInt mx = MaxX, my = MaxY;
			if (KSwapXY)
				{
				TInt t = x; x = y; y = t;
				t = mx; mx = my; my = t;
				}
			if (KFlipX)
				x = mx - x;
			if (KFlipY)
				y = my - y;
			x = x * KScreenW / mx;
			y = y * KScreenH / my;
			x = x < 0 ? 0 : x >= KScreenW ? KScreenW - 1 : x;
			y = y < 0 ? 0 : y >= KScreenH ? KScreenH - 1 : y;
			TBool down = (o[0] == 1 || o[0] == 2 || o[0] == 4) && y < KUserH;	// finger, stylus, unclassified
			if (down && !Down)
				Post(TRawEvent::EButton1Down, x, y);
			else if (down && (x != LastX || y != LastY))
				Post(TRawEvent::EPointerMove, x, y);
			else if (!down && Down)
				Post(TRawEvent::EButton1Up, LastX, LastY);
			if (down != Down && Logged < 30)
				{
				++Logged;
				Kern::Printf("[touch] %s %d %d (raw %d %d type %d, attn %d)", down ? "down" : "up",
					down ? x : LastX, down ? y : LastY, rx, ry, o[0], AttnSeen);
				}
			Down = down;
			if (down)
				{
				LastX = x;
				LastY = y;
				}
			}
		}
	PollTimer.Again(NKern::TimerTicks(next));
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
			SFn* fn = page != 0 ? NULL : e[5] == 0x01 ? &F01 : e[5] == 0x12 ? &F12 : NULL;
			if (fn)
				{
				fn->iQuery = e[0];
				fn->iCtrl = e[2];
				fn->iData = e[3];
				fn->iFound = ETrue;
				}
			}
	if (!F01.iFound || !F12.iFound)
		{
		Log("stopped: no F01/F12 on page 0");
		NKern::ThreadLeaveCS();
		return;
		}
	Step("rmi setup");
	if (RmiSetup() != KErrNone)
		{
		NKern::ThreadLeaveCS();
		return;
		}
	PollDfc.SetDfcQ(TouchQ);
	PollTimer.OneShot(NKern::TimerTicks(KPollMs));
	Step("polling");
	NKern::ThreadLeaveCS();
	}

static TDfc BringUpDfc(BringUpDfcFn, NULL, 1);

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
