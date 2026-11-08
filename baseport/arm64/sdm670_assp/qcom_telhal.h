// SPDX-License-Identifier: Apache-2.0
// qcom_telhal.h -- the telephony HAL functions for the user side (qcom_telhal.cpp) and the QRTR message queue.
#ifndef QCOM_TELHAL_H
#define QCOM_TELHAL_H
void Qrtr_Rx(TAny* aPtr, const TUint8* aData, TInt aLen);	// IPCRTR's GLINK callback: queue for the user side
void Qrtr_SetChannel(TInt aChannel);						// the GLINK channel id of IPCRTR once open
#endif
