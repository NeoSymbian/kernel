// SPDX-License-Identifier: Apache-2.0
#ifndef QCOM_TSENS_H
#define QCOM_TSENS_H
// Thread context only; controller 0: sensors 0..12, controller 1: 0..7.
// Reads firmware-configured hardware without modifying it. Leaves output unchanged on error.
// KErrNotReady: disabled/no valid sample; KErrNotSupported: unmapped/unsupported version.
TInt A64ThermalRead(TInt aController, TInt aSensor, TInt& aMilliC);
void A64ThermalDump();
#endif
