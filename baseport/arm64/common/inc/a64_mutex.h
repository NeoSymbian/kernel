// SPDX-License-Identifier: Apache-2.0
// a64_mutex.h -- lazy boot-lifetime driver mutexes. Creation/closing may sleep: never hold a fast mutex there.
#ifndef A64_MUTEX_H
#define A64_MUTEX_H
#include <kernel/kern_priv.h>

inline TInt A64Mutex_Init(DMutex*& aMutex, NFastMutex& aPublicationLock, const TDesC& aName, TUint aOrder)
	{
	NKern::ThreadEnterCS(); // required by Kern::MutexCreate and DObject::Close
	NKern::FMWait(&aPublicationLock);
	TBool exists = aMutex != NULL;
	NKern::FMSignal(&aPublicationLock);
	TInt r = KErrNone;
	if (!exists)
		{
		DMutex* candidate = NULL;
		r = Kern::MutexCreate(candidate, aName, aOrder); // no fast mutex held
		NKern::FMWait(&aPublicationLock);
		if (!aMutex && r == KErrNone)
			{
			aMutex = candidate;
			candidate = NULL;
			}
		if (aMutex)
			r = KErrNone; // another initializer may have won even if this allocation failed
		NKern::FMSignal(&aPublicationLock);
		if (candidate)
			candidate->Close(NULL); // losing allocation, outside the publication lock
		}
	NKern::ThreadLeaveCS();
	return r;
	}
#endif
