// SPDX-License-Identifier: Apache-2.0
//
// a64_keys.cpp -- the Pixel 3a XL's three keys as key presses for the user side (Kern::AddEvent EKeyDown/EKeyUp):
//   - power:       PM660 (SID 0) PON 0x800, INT_RT_STS 0x810 bit 0 (KPDPWR_N), 1 = pressed;
//   - volume down: the same register, bit 1 (RESIN_N), 1 = pressed;
//   - volume up:   PM660L (SID 2) GPIO7 (peripheral 0xc600; spmi-gpio numbers pins from 1), RT_STS 0xc610 bit 0,
//                  active low (DTB gpio-keys; ABL already set it to an input with a pull-up).
// shortcut: polled every 20 ms, not the SPMI arbiter's interrupt (SPI 481); upgrade at H3 (wake on the power key).
#include <kernel/kern_priv.h>
#include <e32keys.h>
#include "board.h"
#include "qcom_spmi.h"
#include "a64_device.h"

struct SKey { const char* iName; TInt iScanCode; TUint8 iSid; TUint16 iAddr; TUint8 iBit; TBool iActiveLow; };
static const SKey Keys[] = {
	{ "Power",   EStdKeyDevice0,   0, 0x0810, 0, EFalse },
	{ "VolDown", EStdKeyDecVolume, 0, 0x0810, 1, EFalse },
	{ "VolUp",   EStdKeyIncVolume, 2, 0xc610, 0, ETrue },
	};
const TInt KKeys = sizeof(Keys) / sizeof(Keys[0]);
const TInt KPollMs = 20;

static TBool Down[KKeys];
static TInt SpmiVersion = KErrNotReady;					// < 0: the SPMI bus did not come up
static void PollDfcFn(TAny*);
static TDfc PollDfc(PollDfcFn, NULL, 1);
static void PollTimerFn(TAny*) { PollDfc.Add(); }
static NTimer PollTimer(PollTimerFn, NULL);

static void Post(TRawEvent::TType aType, TInt aScanCode)
	{
	TRawEvent e;
	e.Set(aType, aScanCode);
	Kern::AddEvent(e);
	}

void A64_RebootToFastboot();								// a64_reboot.cpp
static TInt ComboPolls;								// Volume Up + Volume Down held: back to fastboot after 2 s
__attribute__((weak)) void A64TouchDump() {}				// a64_touch.cpp: its probe, again on Volume Up
__attribute__((weak)) void A64BlkDump() {}					// qcom_sdhci.cpp: the eMMC report, again on Volume Up

static void PollDfcFn(TAny*)
	{
	for (TInt i = 0; i < KKeys; ++i)
		{
		TInt v = Spmi_Read(Keys[i].iSid, Keys[i].iAddr);
		if (v < 0)
			continue;									// a failed read leaves the key as it was
		TBool down = ((v >> Keys[i].iBit) & 1) != (Keys[i].iActiveLow ? 1 : 0);
		if (down == Down[i])
			continue;
		Down[i] = down;
		Kern::Printf("[keys] %s %s", Keys[i].iName, down ? "down" : "up");
		Post(down ? TRawEvent::EKeyDown : TRawEvent::EKeyUp, Keys[i].iScanCode);
		if (down && Keys[i].iScanCode == EStdKeyIncVolume)
			{
			A64TouchDump();
			A64BlkDump();								// last: the strip shows only ~10 lines
			}
		}
	ComboPolls = Down[1] && Down[2] ? ComboPolls + 1 : 0;	// VolDown and VolUp (Keys[] order)
	if (ComboPolls == 2000 / KPollMs)
		A64_RebootToFastboot();
	PollTimer.OneShot(NKern::TimerTicks(KPollMs));		// not Again(): it stops for good after a slow poll
	}

void A64StartKeys()
	{
	TInt ver = Spmi_Init();
	SpmiVersion = ver;
	Kern::Printf("[spmi] arbiter version %08x; apid PON %d, PM660L GPIO7 %d, PM660 GPIO12 %d", ver,
		Spmi_Apid(0, 0x08), Spmi_Apid(2, 0xc6), Spmi_Apid(0, 0xcb));
	if (ver < 0)
		return;
	// what ABL left: the volume-up pin's type/subtype, mode, status, and the PON real-time status
	Kern::Printf("[keys] GPIO7 type %02x/%02x mode %02x status1 %02x rt %02x; PON rt %02x",
		Spmi_Read(2, 0xc604), Spmi_Read(2, 0xc605), Spmi_Read(2, 0xc640), Spmi_Read(2, 0xc608),
		Spmi_Read(2, 0xc610), Spmi_Read(0, 0x0810));
	PollDfc.SetDfcQ(Kern::DfcQue0());					// a static TDfc was constructed before the queue existed
	PollTimer.OneShot(NKern::TimerTicks(KPollMs));
	}

static TA64DevState KeysStatus(TDes8& aDetail)
	{
	if (SpmiVersion < 0)
		{
		A64Device_Format(aDetail, "SPMI bus: %d", SpmiVersion);
		return EA64DevError;
		}
	A64Device_Format(aDetail, "Power, VolDown, VolUp, polled %d ms; held:", KPollMs);
	TBool any = EFalse;
	for (TInt i = 0; i < KKeys; ++i)
		if (Down[i])
			A64Device_Format(aDetail, " %s", Keys[i].iName), any = ETrue;
	if (!any)
		A64Device_Format(aDetail, " none");
	return EA64DevOk;
	}
A64_DEVICE("keys", "Buttons (PM660 PON, GPIO)", "a64_keys", KeysStatus);
