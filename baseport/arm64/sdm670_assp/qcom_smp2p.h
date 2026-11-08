// SPDX-License-Identifier: Apache-2.0
// qcom_smp2p.h -- SMP2P, 32-bit state words shared with a remote processor through SMEM (qcom_smp2p.cpp).
#ifndef QCOM_SMP2P_H
#define QCOM_SMP2P_H

// the modem's inbound "slave-kernel" bits (DTB remoteproc@4080000 interrupts)
const TUint32 KMssFatal = 1u << 0, KMssReady = 1u << 1, KMssHandover = 1u << 2, KMssStopAck = 1u << 3,
	KMssShutdownAck = 1u << 7;
const TUint32 KMssStop = 1u << 0;							// our outbound "master-kernel" bit 0

TInt Smp2p_Start(TInt aRemote);								// our outbound item + entries; idempotent
// an inbound entry's value; KErrNotReady until the remote created it
TInt Smp2p_In(TInt aRemote, const char* aEntry, TUint32& aValue);
TInt Smp2p_Set(TInt aRemote, const char* aEntry, TUint32 aMask, TUint32 aValue);	// outbound bits, then a kick
void Smp2p_Poll(TInt aRemote);								// version negotiation and the restart ack
void Smp2p_Dump(TInt aRemote, void (*aSay)(const char*, ...));
#endif
