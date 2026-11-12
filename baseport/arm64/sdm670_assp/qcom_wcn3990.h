// SPDX-License-Identifier: Apache-2.0
// qcom_wcn3990.h -- the WCN3990 Wi-Fi data path (qcom_wcn3990.cpp), bring-up stage W2 (docs wifi.md).
#ifndef QCOM_WCN3990_H
#define QCOM_WCN3990_H
// after the QMI WLFW handshake, WLAN cfg and mission mode (MODEMD): the copy engines, HTC and WMI up to READY, on
// the driver's own thread. KErrNone = started; the device registry ("drivers": wifi) shows how far it got.
TInt Wcn3990_Start();
// HAL WlanScan (0x4e1e): the latest scan results into the TDes8* a1; a2 != 0 asks for a new scan (qcom_wcn3990.cpp)
TInt Wcn3990_ScanHal(TAny* a1, TAny* a2);
#endif
