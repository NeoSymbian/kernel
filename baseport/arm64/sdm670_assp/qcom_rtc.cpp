// SPDX-License-Identifier: Apache-2.0
//
// qcom_rtc.cpp -- the PM660 real-time clock, read only: SID 0, RTC_RW peripheral at 0x6000, the layout of Linux
// drivers/rtc/rtc-pm8xxx.c "qcom,pm8941-rtc" (CTRL 0x46 bit 7 = running; READ 0x48..0x4b = seconds, little-endian).
// It counts on through power-offs and restarts. Phones don't let the OS set it, so its zero means nothing: the user
// side keeps "real time - RTC" (neoui's clock.dat) and reads the raw count through HAL ENeoHalRtcRead.
#include <kernel/kern_priv.h>
#include "qcom_spmi.h"

const TUint32 KRtcCtrl = 0x6046, KRtcRead = 0x6048, KRtcEnabled = 0x80;

// thread or DFC context (Spmi_Read)
TInt A64_RtcSeconds(TUint32& aSecs)
	{
	TInt c = Spmi_Read(0, KRtcCtrl);
	if (c < 0)
		return c;
	if (!(c & KRtcEnabled))
		return KErrNotReady;
	for (TInt tries = 0; tries < 3; ++tries)
		{
		TUint32 v = 0;
		for (TInt i = 0; i < 4; ++i)
			{
			TInt b = Spmi_Read(0, KRtcRead + i);
			if (b < 0)
				return b;
			v |= TUint32(b) << (8 * i);
			}
		TInt b0 = Spmi_Read(0, KRtcRead);			// a tick between the byte reads may have carried into the high bytes
		if (b0 < 0)
			return b0;
		if (TUint32(b0) == (v & 0xff))
			{
			aSecs = v;
			return KErrNone;
			}
		}
	return KErrTimedOut;
	}
