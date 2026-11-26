// SPDX-License-Identifier: Apache-2.0
// qcom_smmu.h -- the apps SMMU (MMU-500 at 0x15000000): an identity-mapped context for one group of stream ids
// (qcom_smmu.cpp, telephony C5b: IPA's 0x720/0x722).
#ifndef QCOM_SMMU_H
#define QCOM_SMMU_H

// streams matching aId under aMask translate through a stage-1 context mapping IOVA == PA for the low 4 GB
// (normal non-cacheable). KErrNone, or KErrInUse if a stream match already covers them, KErrOverflow if no free slot.
TInt Smmu_IdentityMap(TUint32 aId, TUint32 aMask);
// the context's fault status/address for the monitor (KErrNotReady before Smmu_IdentityMap)
TInt Smmu_Fault(TUint32& aFsr, TUint64& aFar, TUint32& aFsynr);
void Smmu_GlobalFault(TUint32& aGfsr, TUint64& aGfar, TUint32& aGfsynr0);	// unmatched/global faults (sGFSR)
#endif
