// SPDX-License-Identifier: Apache-2.0
// qcom_qmp.h -- text messages to the always-on processor (AOP) over QMP (qcom_qmp.cpp).
#ifndef QCOM_QMP_H
#define QCOM_QMP_H
TInt Qmp_Open();											// the link and channel handshake; idempotent
TInt Qmp_Send(const char* aMsg);							// < 64 bytes; waits for the AOP to take it
#endif
