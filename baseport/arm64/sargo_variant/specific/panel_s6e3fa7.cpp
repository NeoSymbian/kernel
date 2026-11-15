// SPDX-License-Identifier: Apache-2.0
// panel_s6e3fa7.cpp -- sargo's Samsung S6E3FA7 OLED panel (dtbo "s6e3fa7 cmd mode dsi sdc panel" on DSI0): brightness,
// sleep. ABL powers and initialises the panel; we never cut its supplies or reset it, so only DCS sleep in/out, display
// off/on and the brightness are sent. Sequence and timing facts: the stock dtbo's on/off commands
// (qcom,mdss-dsi-on-command / -off-command, S4 PVT): on = 0x11, 120 ms, ..., 0x29; off = 0x28, 0x10, 120 ms;
// brightness bl_ctrl_dcs, max level 0x3ff = DCS 0x51 with a 16-bit big-endian level.
// shortcut: power-on skips the dtbo's TE and 0xf0-unlocked 0xb4/0x53 setup ABL already sent; send it if a sleep cycle
// comes back with the wrong brightness mode or tearing.
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
		if ((r = Cmd(0x11)) != KErrNone)
			return r;
		Wait(120);
		return Cmd(0x29);
		}
	if ((r = Cmd(0x28)) != KErrNone)
		return r;
	r = Cmd(0x10);
	Wait(120);
	return r;
	}
