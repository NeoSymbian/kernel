// SPDX-License-Identifier: Apache-2.0
// qcom_glink.h -- GLINK channels to a remote processor over SMEM FIFOs (qcom_glink.cpp).
#ifndef QCOM_GLINK_H
#define QCOM_GLINK_H

typedef void (*TGlinkRx)(TAny* aPtr, const TUint8* aData, TInt aLen);	// a whole message, on the GLINK thread

TInt Glink_Start(TInt aRemote);								// the edge's FIFOs + version handshake + polling
// a channel by name ("IPCRTR"): opened from our side too, aRx gets its messages; returns a channel id > 0
// (edge << 8 | lcid: the modem's are 1..8)
TInt Glink_Open(TInt aRemote, const char* aName, TGlinkRx aRx, TAny* aPtr);
TInt Glink_Send(TInt aChannel, const TAny* aData, TInt aLen);	// KErrNotReady until the remote gave an intent
void Glink_Dump(void (*aSay)(const char*, ...));
#endif
