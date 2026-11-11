// SPDX-License-Identifier: Apache-2.0
// qcom_dsi.h -- DCS commands to the panel on DSI0, through the command DMA ABL left running (qcom_dsi.cpp)
#ifndef QCOM_DSI_H
#define QCOM_DSI_H
#include <e32def.h>

// one DCS write: aData[0] is the command, the rest its parameters (1..15 bytes in all); thread context.
// KErrNone, KErrNotReady (DSI0 not left in command mode), KErrTimedOut (no CMD_DMA_DONE in 200 ms)
TInt Dsi_DcsWrite(const TUint8* aData, TInt aLen);

#endif
