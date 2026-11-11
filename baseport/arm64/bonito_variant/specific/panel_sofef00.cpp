// SPDX-License-Identifier: Apache-2.0
// panel_sofef00.cpp -- bonito's Samsung SOFEF00 OLED panel (DTB samsung,sofef00-bonito on DSI0): brightness, sleep.
// ABL powers and initialises the panel; we never cut its supplies or reset it, so only DCS sleep in/out, display
// off/on and the brightness are sent. Sequence and timing facts: Linux panel-samsung-sofef00.c (read, not copied):
// brightness = DCS 0x51 with a 16-bit big-endian level 0..1023; off = 0x28, 40 ms, 0x10, 160 ms; on = 0x11, 10 ms, 0x29.
#include <kernel/kern_priv.h>
#include "qcom_dsi.h"

static const TInt KMaxBrightness = 1023;

static void Wait(TInt aMs) { NKern::Sleep(NKern::TimerTicks(aMs)); }
static TInt Cmd(TUint8 aCmd) { return Dsi_DcsWrite(&aCmd, 1); }

TInt A64Panel_MaxBrightness() { return KMaxBrightness; }

TInt A64Panel_Brightness(TInt aLevel)
	{
	if (aLevel < 0 || aLevel > KMaxBrightness)
		return KErrArgument;
	const TUint8 c[3] = { 0x51, (TUint8)(aLevel >> 8), (TUint8)aLevel };
	return Dsi_DcsWrite(c, 3);
	}

TInt A64Panel_Power(TBool aOn)
	{
	TInt r;
	if (aOn)
		{
		if ((r = Cmd(0x11)) != KErrNone)				// exit sleep
			return r;
		Wait(10);
		return Cmd(0x29);								// display on
		}
	if ((r = Cmd(0x28)) != KErrNone)					// display off
		return r;
	Wait(40);
	r = Cmd(0x10);										// enter sleep
	Wait(160);
	return r;
	}
