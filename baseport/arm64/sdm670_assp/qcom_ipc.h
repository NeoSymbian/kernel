// SPDX-License-Identifier: Apache-2.0
// qcom_ipc.h -- the APCS IPC doorbell (DTB mailbox@17990000, "qcom,sdm670-apss-shared"): writing bit n of +0xc
// interrupts a remote processor. Bits (DTB mboxes): AOP QMP 0, modem GLINK 12, modem SMP2P 14, ADSP GLINK 24,
// ADSP SMP2P 26. Written from Linux drivers/mailbox/qcom-apcs-ipc-mailbox.c.
#ifndef QCOM_IPC_H
#define QCOM_IPC_H
#include "board.h"

const TInt KIpcAop = 0, KIpcModemGlink = 12, KIpcModemSmp2p = 14, KIpcAdspGlink = 24, KIpcAdspSmp2p = 26;

inline void Ipc_Kick(TInt aBit)
	{
	__e32_memory_barrier();									// the data the remote reads lands first
	*(volatile TUint32*)(A64_Io(0x17990000) + 0xc) = 1u << aBit;
	}
#endif
