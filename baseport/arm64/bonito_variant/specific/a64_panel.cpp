// SPDX-License-Identifier: Apache-2.0
// Bonito Samsung SOFEF00 OLED on DSI0 (DTB samsung,sofef00-bonito). Keep ABL's scan-out/PHY and regulators.
// Original bounded command-FIFO implementation from register/protocol facts in Qualcomm's reference:
// https://android.googlesource.com/kernel/msm-extra/display-drivers/+/e7ab2aa8bd2f6f27b311ccada3722d20dee4bcef/msm/dsi/dsi_ctrl_hw_cmn.c
// https://android.googlesource.com/kernel/msm-extra/display-drivers/+/e7ab2aa8bd2f6f27b311ccada3722d20dee4bcef/msm/dsi/dsi_ctrl_reg.h
// MSM packet byte order: https://github.com/torvalds/linux/blob/master/drivers/gpu/drm/msm/dsi/dsi_host.c
// SOFEF00 uses DCS display-off (0x28), 40ms wait and display-on (0x29):
// https://github.com/torvalds/linux/blob/master/drivers/gpu/drm/panel/panel-samsung-sofef00.c
// No DMA or interrupt ownership: command bytes go directly to the controller FIFO, completion is polled.
// This blanks the OLED physically, without sleep-in/reset (which would require full panel reinitialization).
#include <kernel/kern_priv.h>
#include "board.h"
#include "a64_mutex.h"
#include "a64_device.h"

static volatile TUint32& R(TUint32 aOffset) { return *(volatile TUint32*)A64_Io(0x0ae94000 + aOffset); }
static DMutex* Lock;
static NFastMutex InitLock;
static TInt LastError;
static TBool On = ETrue;
static TUint32 ElapsedMs(TUint32 aStart)
	{
	return (TUint32)((TUint64)(NKern::TickCount() - aStart) * NKern::TickPeriod() / 1000);
	}

static TInt Send(TUint8 aCommand)
	{
	TUint32 started = NKern::TickCount();
	// The 6G host adds four bytes to the old register offsets. Refuse other revisions rather than guess.
	TUint32 hw = R(0), major = R(0x1f4) >> 24;
	Kern::Printf("[panel] DSI command %02x version %08x/%d", aCommand, hw, major);
	if (major != 3 || (hw != 0x20020001 && hw != 0x20030000))
		{
		Kern::Printf("[panel] unsupported DSI version %08x/%d", hw, major);
		return KErrNotSupported;
		}
	TUint32 ctrl = R(4);
	Kern::Printf("[panel] DSI ready ctrl %08x busy %08x irq %08x", ctrl, R(8), R(0x110));
	if (!(ctrl & 1))
		{
		Kern::Printf("[panel] DSI command engine unavailable");
		return KErrNotReady;
		}
	for (TInt i = 0; R(8) & 6; ++i)
		{
		if (i == 200)
			{
			Kern::Printf("[panel] DSI busy timeout status %08x after %d ms", R(8), ElapsedMs(started));
			return KErrTimedOut;
			}
		NKern::Sleep(NKern::TimerTicks(1));
		}
	TUint32 dma = R(0x3c), trigger = R(0x84), pattern = R(0x15c), length = R(0x4c);
	if (pattern)
		{
		Kern::Printf("[panel] DSI FIFO owned pattern %08x", pattern);
		return KErrInUse; // another owner is using the command/test FIFO
		}
	const TUint32 masks = 0x2222aa02; // writable interrupt enable bits; status bits are W1C
	R(0x110) = (R(0x110) & masks) | 1;
	R(4) = ctrl | 4; // enable command engine, retain video and lane configuration
	R(0x84) = (trigger & ~7u) | 4; // software DMA trigger
	R(0x1ec) = 1;
	Kern::NanoWait(1000);
	R(0x1ec) = 0;
	R(0x15c) = (3u << 16) | 6; // custom TPG DMA FIFO, not framebuffer DMA
	R(0x17c) = 0x80050000u | aCommand; // DCS short write, VC0, last packet
	R(0x17c) = 0; // hardware requires an even count of DWORDs, command length remains four bytes
	R(0x3c) = (dma & ~((3u << 30) | (1u << 29) | (1u << 24))) | (1u << 28) | (1u << 26);
	R(0x4c) = 4;
	Kern::Printf("[panel] DSI trigger command %02x after %d ms", aCommand, ElapsedMs(started));
	__e32_memory_barrier(); // all FIFO/config writes before trigger
	R(0x90) = 1;
	TInt r = KErrTimedOut;
	for (TInt i = 0; i < 200; ++i)
		{
		TUint32 status = R(0x110);
		if (status & (1u << 24))
			{
			r = KErrGeneral;
			break;
			}
		if (status & 1)
			{
			r = KErrNone;
			break;
			}
		NKern::Sleep(NKern::TimerTicks(1));
		}
	Kern::Printf("[panel] DSI completion %d irq %08x busy %08x after %d ms", r, R(0x110), R(8), ElapsedMs(started));
	R(0x110) = (R(0x110) & masks) | 1; // acknowledge our completion only
	R(0x15c) = 0;
	R(0x1ec) = 1;
	Kern::NanoWait(1000);
	R(0x1ec) = 0;
	R(0x3c) = dma;
	R(0x4c) = length;
	R(0x84) = trigger;
	R(4) = ctrl;
	__e32_memory_barrier();
	Kern::Printf("[panel] DSI restored ctrl %08x result %d after %d ms", R(4), r, ElapsedMs(started));
	return r;
	}

TInt A64Panel_SetState(TBool aOn)
	{
	TUint32 started = NKern::TickCount();
	Kern::Printf("[panel] enter tick %u display %s", started, aOn ? "on" : "off");
	TInt r = A64Mutex_Init(Lock, InitLock, _L("Panel"), KMutexOrdGeneral0);
	if (r != KErrNone)
		{
		Kern::Printf("[panel] lock init failed %d after %d ms", r, ElapsedMs(started));
		return r;
		}
	NKern::ThreadEnterCS();
	Kern::MutexWait(*Lock);
	Kern::Printf("[panel] locked after %d ms, display %s", ElapsedMs(started), On ? "on" : "off");
	if (aOn != On)
		{
		r = Send(aOn ? 0x29 : 0x28);
		if (r == KErrNone)
			{
			On = aOn;
			if (!aOn)
				NKern::Sleep(Max(1, NKern::TimerTicks(40)));
			}
		}
	LastError = r;
	Kern::MutexSignal(*Lock);
	NKern::ThreadLeaveCS();
	Kern::Printf("[panel] exit display %s result %d after %d ms", aOn ? "on" : "off", r, ElapsedMs(started));
	return r;
	}
static TA64DevState Status(TDes8& aDetail)
	{
	A64Device_Format(aDetail, "SOFEF00 %s, error %d", On ? "on" : "off", LastError);
	return LastError ? EA64DevError : On ? EA64DevOk : EA64DevOff;
	}
A64_DEVICE("panel", "OLED panel", "sofef00", Status);
