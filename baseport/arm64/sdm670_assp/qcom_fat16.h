// SPDX-License-Identifier: Apache-2.0
// qcom_fat16.h -- read-only files from a FAT16 GPT partition of the eMMC (qcom_fat16.cpp): the modem's firmware
// on modem_a. Thread context; one volume mounted at a time.
#ifndef QCOM_FAT16_H
#define QCOM_FAT16_H

struct SFatFile { TUint32 iCluster, iSize; };

TInt Gpt_Find(const char* aName, TUint64& aFirst, TUint64& aCount);	// a partition by GPT name, read-only
TInt Fat_Mount(const char* aPartition);					// e.g. "modem_a"
TInt Fat_Find(const char* aPath, SFatFile& aFile);		// "IMAGE/MODEM.MDT": 8.3 names, any case
TInt Fat_Read(const SFatFile& aFile, TUint32 aPos, TAny* aBuf, TUint32 aLen);	// KErrNone, all of it or nothing
// the aIndex-th file of a directory ("IMAGE"): KErrNone, KErrEof past the end
TInt Fat_DirEntry(const char* aDir, TInt aIndex, TDes8& aName, TUint32& aSize);
#endif
