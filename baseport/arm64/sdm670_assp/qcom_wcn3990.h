// SPDX-License-Identifier: Apache-2.0
// qcom_wcn3990.h -- the WCN3990 Wi-Fi data path (qcom_wcn3990.cpp), bring-up stage W2 (docs wifi.md).
#ifndef QCOM_WCN3990_H
#define QCOM_WCN3990_H
// after the QMI WLFW handshake, WLAN cfg and mission mode (MODEMD): the copy engines, HTC and WMI up to READY, on
// the driver's own thread. KErrNone = started; the device registry ("drivers": wifi) shows how far it got.
TInt Wcn3990_Start();
// HAL WlanScan (0x4e1e): the latest scan results into the TDes8* a1; a2 != 0 asks for a new scan (qcom_wcn3990.cpp)
TInt Wcn3990_ScanHal(TAny* a1, TAny* a2);
// HAL WlanLink (0x4e1f): a1 = op, a2 = TDes8* (join, leave, status, EAPOL in/out, keys, authorize; W4)
TInt Wcn3990_LinkHal(TAny* a1, TAny* a2);
// the modem was stopped (C7 recovery, qcom_telhal.cpp): the WLAN firmware ran inside it, so the driver stops using
// the chip and reports the link lost
void Wcn3990_ModemDown();
// the debug monitor's "wifi" (0), "wifi join N" (1, N), "wifi leave" (2)
void Wcn3990_Monitor(TInt aOp, TInt aArg);
#endif
