// SPDX-License-Identifier: Apache-2.0
// qcom_smem.h -- the SDM670's shared memory with the remote processors (qcom_smem.cpp).
#ifndef QCOM_SMEM_H
#define QCOM_SMEM_H

const TInt KSmemHostGlobal = -1;							// the global partition (or heap)
const TInt KSmemHostModem = 1, KSmemHostAdsp = 2;			// remote host ids (DTB qcom,remote-pid)

TInt Smem_Init();											// KErrNone once the header and partitions are read
TAny* Smem_Get(TInt aHost, TUint aItem, TUint32* aSize = NULL);	// NULL if not allocated
TInt Smem_Alloc(TInt aHost, TUint aItem, TUint32 aSize);	// KErrNone, KErrAlreadyExists, or an error
TUint32 Smem_Phys(const TAny* aPtr);						// the physical address of a pointer into SMEM
void Smem_Dump(void (*aSay)(const char*, ...));				// the monitor's "smem"
#endif
