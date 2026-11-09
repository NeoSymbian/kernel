// SPDX-License-Identifier: Apache-2.0
// qcom_telhal.h -- the telephony HAL functions for the user side (qcom_telhal.cpp) and the QRTR message queue.
#ifndef QCOM_TELHAL_H
#define QCOM_TELHAL_H
// GLINK receive callback: queue a message for the user side in ring aPtr (Tel_Ring; NULL = the modem's IPCRTR)
void Qrtr_Rx(TAny* aPtr, const TUint8* aData, TInt aLen);
void Qrtr_SetChannel(TInt aChannel);						// the GLINK channel id of the modem's IPCRTR once open
const TInt KTelRings = 3;									// 0 = modem IPCRTR, 1 = ADSP IPCRTR, 2 = APR
TAny* Tel_Ring(TInt aRing);									// a ring, to pass to Glink_Open as aPtr with Qrtr_Rx
void Tel_SetChannel(TInt aRing, TInt aChannel);				// the channel ring aRing's sends go to
void Tel_Dump(void (*aSay)(const char*, ...));				// the monitor's "glink": rings, drops
// HAL Audio, the board's codecs (weak: none): op 0 probe, 1 start(route 0 earpiece / 1 speaker), 2 stop,
// 3 route, 4 volume 0..10, 5 mic mute
TInt A64Audio_Control(TInt aOp, TInt aArg);
void A64Audio_Report(void (*aSay)(const char*, ...));		// the monitor's "audio" (weak: none)
#endif
