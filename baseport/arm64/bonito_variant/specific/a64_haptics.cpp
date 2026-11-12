// SPDX-License-Identifier: Apache-2.0
// Bonito DRV2624, I2C10 0x5a, reset TLMM gpio24. Facts from the C1 DTB and TI SLOS893D register map:
// https://www.ti.com/lit/ds/symlink/drv2624.pdf (revision D, August 2025, sections 7.6 and 8).
// DT property encoding verified against Google's dev_init_platform_data (reference only):
// https://android.googlesource.com/kernel/google-modules/amplifiers/+/aba0cca145df5e55d1c5c4294a90bc382b0d2ec1/drv2624/drv2624.c
// RTP mode, polled I2C status, no interrupt source enabled. The timer queues thread work, never I2C in an ISR.
#include <kernel/kern_priv.h>
#include "board.h"
#include "a64_mutex.h"
#include "qcom_geni_i2c.h"
#include "a64_device.h"

TInt A64Board_I2c10();
static DMutex* Lock; // f2 -> board I2C10 f1 -> GENI fast mutex (strictly descending DMutex order)
static NFastMutex InitLock;
static TBool Ready, Running;
static TInt LastError;
static TUint32 Deadline;
static TDynamicDfcQue* Queue;
static void StopFn(TAny*);
static TDfc StopDfc(StopFn, NULL, 1);
static void TimerFn(TAny*) { StopDfc.Add(); }
static NTimer Timer(TimerFn, NULL);
static TInt Write(TUint8 aReg, TUint8 aValue)
	{
	TUint8 b[2] = { aReg, aValue };
	return I2c_Xfer(1, 0x5a, b, 2, NULL, 0);
	}
static TInt Read(TUint8 aReg, TUint8& aValue) { return I2c_Xfer(1, 0x5a, &aReg, 1, &aValue, 1); }
static TInt Probe()
	{
	if (Ready)
		return KErrNone;
	TInt r = A64Board_I2c10();
	if (r != KErrNone)
		return r;
	volatile TUint32& ctl = *(volatile TUint32*)A64_Io(0x03918000);
	volatile TUint32& io = *(volatile TUint32*)A64_Io(0x03918004);
	io = 0;
	ctl = 0x200; // GPIO output, no pull, reset active low
	Kern::NanoWait(1000000);
	io = 2;
	Kern::NanoWait(1000000);
	TUint8 id = 0;
	if ((r = Read(0, id)) != KErrNone)
		return r;
	// SLOS893D table 8-2: CHIPID[7:4]=0 for DRV2624, REV[3:0]=2 or 3.
	if ((id & 0xf0) != 0 || ((id & 0x0f) != 2 && (id & 0x0f) != 3))
		return KErrNotSupported;
	// DT: open loop LRA, 172Hz nominal/155Hz open-loop, rated0x4b/clamp0x6a,
	// compensation0x12/BEMF0x96, blanking2/idiss2, no feedback braking, 5ms waveform interval.
	static const TUint8 init[][2] = {
		{0x0c,0}, {0x07,0x48}, {0x08,0xd8}, {0x0d,0x20}, {0x1f,0x4b}, {0x20,0x6a},
		{0x21,0x12}, {0x22,0x96}, {0x23,0x04}, {0x27,24}, {0x28,0x22}, {0x29,0x0c},
		{0x2c,0}, {0x2e,1}, {0x2f,6}, {0x0e,0}
		};
	for (TUint i = 0; i < sizeof(init) / sizeof(init[0]); ++i)
		if ((r = Write(init[i][0], init[i][1])) != KErrNone)
			{
			Write(0x0c, 0);
			io = 0; // reset guarantees stop even when I2C fails
			return r;
			}
	Ready = ETrue;
	return KErrNone;
	}
static TInt Stop()
	{
	if (!Ready)
		return KErrNone;
	TInt r = Write(0x0c, 0);
	if (r != KErrNone)
		{
		*(volatile TUint32*)A64_Io(0x03918004) = 0;
		Ready = EFalse;
		}
	Running = EFalse;
	return r;
	}
static void StopFn(TAny*)
	{
	NKern::ThreadEnterCS();
	Kern::MutexWait(*Lock);
	TInt left = (TInt)(Deadline - NKern::TickCount());
	if (Running && left > 0)
		Timer.OneShot(left); // an old queued stop must not truncate a newer pulse
	else if (Running)
		LastError = Stop();
	Kern::MutexSignal(*Lock);
	NKern::ThreadLeaveCS();
	}
TInt A64Haptic_Control(TInt aMs, TInt aAmplitude)
	{
	if (aMs < 0 || aMs > 2000 || aAmplitude < 0 || aAmplitude > 127)
		return KErrArgument;
	TInt r = A64Mutex_Init(Lock, InitLock, _L("Haptic"), KMutexOrdGeneral2);
	if (r != KErrNone)
		return r;
	NKern::ThreadEnterCS();
	Kern::MutexWait(*Lock);
	Timer.Cancel();
	if (!aMs || !aAmplitude)
		r = Stop();
	else
		{
		if (!Queue)
			{
			r = Kern::DynamicDfcQCreate(Queue, 24, _L8("HapticStop"));
			if (r == KErrNone)
				StopDfc.SetDfcQ(Queue);
			}
		if (r == KErrNone)
			r = Probe();
		if (r == KErrNone)
			r = Write(0x0e, aAmplitude);
		if (r == KErrNone)
			r = Write(0x0c, 1);
		if (r == KErrNone)
			{
			Running = ETrue;
			TInt ticks = Max(1, NKern::TimerTicks(aMs));
			Deadline = NKern::TickCount() + ticks;
			Timer.OneShot(ticks);
			}
		else
			Stop();
		}
	LastError = r;
	Kern::MutexSignal(*Lock);
	NKern::ThreadLeaveCS();
	return r;
	}
static TA64DevState Status(TDes8& aDetail)
	{
	A64Device_Format(aDetail, "DRV2624 %s, error %d", Running ? "running" : Ready ? "ready" : "not probed", LastError);
	return LastError ? EA64DevError : Running || Ready ? EA64DevOk : EA64DevOff;
	}
A64_DEVICE("haptic", "Vibration", "drv2624", Status);
