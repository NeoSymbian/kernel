// SPDX-License-Identifier: Apache-2.0
//
// fajita_keys.cpp -- the OnePlus 6T's three keys as key presses for the user side (Kern::AddEvent EKeyDown/EKeyUp);
// a copy of bonito's a64_keys.cpp with the 6T's wiring (docs fajita.md OP4; live DT gpio_keys):
//   - power:       PM8998 (SID 0) PON 0x800, INT_RT_STS 0x810 bit 0 (KPDPWR_N), 1 = pressed;
//   - volume down: PM8998 GPIO5 (peripheral 0xc400; spmi-gpio numbers pins from 1), RT_STS 0xc410 bit 0, active low;
//   - volume up:   PM8998 GPIO6 (peripheral 0xc500), RT_STS 0xc510 bit 0, active low.
// shortcut: a copy (owner, 2026-11-26: the Pixel's files stay untouched); merge with a board key table later.
// shortcut: polled every 20 ms, not the SPMI arbiter's interrupt (SPI 481); upgrade at H3 (wake on the power key).
#include <kernel/kern_priv.h>
#include <e32keys.h>
#include "board.h"
#include "qcom_spmi.h"
#include "a64_device.h"

struct SKey { const char* iName; TInt iScanCode; TUint8 iSid; TUint16 iAddr; TUint8 iBit; TBool iActiveLow; };
static const SKey Keys[] = {
	{ "Power",   EStdKeyDevice0,   0, 0x0810, 0, EFalse },
	{ "VolDown", EStdKeyDecVolume, 0, 0xc410, 0, ETrue },
	{ "VolUp",   EStdKeyIncVolume, 0, 0xc510, 0, ETrue },
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
	TInt ver = Spmi_Version();
	SpmiVersion = ver;
	if (ver < 0)
		return;
	// what ABL left: the volume pins' type/subtype, mode, real-time status, and the PON real-time status
	Kern::Printf("[keys] GPIO5 %02x/%02x mode %02x rt %02x; GPIO6 %02x/%02x mode %02x rt %02x; PON rt %02x",
		Spmi_Read(0, 0xc404), Spmi_Read(0, 0xc405), Spmi_Read(0, 0xc440), Spmi_Read(0, 0xc410),
		Spmi_Read(0, 0xc504), Spmi_Read(0, 0xc505), Spmi_Read(0, 0xc540), Spmi_Read(0, 0xc510), Spmi_Read(0, 0x0810));
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
A64_DEVICE("keys", "Buttons (PM8998 PON, GPIO5/6)", "fajita_keys", KeysStatus);
