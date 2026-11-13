// Copyright (c) 1994-2009 Nokia Corporation and/or its subsidiary(-ies).
// All rights reserved.
// This component and the accompanying materials are made available
// under the terms of the License "Eclipse Public License v1.0"
// which accompanies this distribution, and is available
// at the URL "http://www.eclipse.org/legal/epl-v10.html".
// SPDX-License-Identifier: EPL-2.0 (changed by the NeoSymbian Authors)
// Initial Contributors:
// Nokia Corporation - initial contribution.
//
// Contributors:
// The NeoSymbian Authors (2026): TRAPs of RVCT-built Symbian 9 binaries (C++ try/catch) beside ours (setjmp).
// Description:
// e32\euser\us_trp.cpp
// 
//

#include "us_std.h"
#include "us_data.h"

#ifndef __LEAVE_EQUALS_THROW__
// NeoSymbian: the TRAP of an RVCT-built Symbian 9 binary (an old .sis app) is a C++ `try ... catch
// (XLeaveException&)` between User::MarkCleanupStack and UnMarkCleanupStack; ours (-fno-exceptions) is
// TTrap::Trap's setjmp. Both kinds share the thread's trap-frame chain: MarkCleanupStack pushes a marker frame (from
// a per-thread pool), and User::Leave pops the innermost frame -- a marker throws an XLeaveException through
// drtaeabi.dll (the binary's C++ run time; our images carry unwind tables for it), any other frame returns into its
// TTrap::Trap as before.
const TInt KNeoCppTrapTls = 0x4E435054 | 1;	// a TLS key no module handle can be (odd)
const TInt KNeoCppTrapDepth = 32;
// drtaeabi.dll's ordinals (kernelhwsrv compsupp/eabi/drtaeabiu.def)
const TInt KOrdAllocateException = 177, KOrdThrow = 191, KOrdLeaveTypeInfo = 206, KOrdLeaveVtable = 207;
_LIT(KNeoCppTrap, "NeoCppTrap");

struct TNeoCppTraps
	{
	TInt iDepth;
	TTrap iMark[KNeoCppTrapDepth];
	RLibrary iRt;
	TAny* (*iAlloc)(TUint);
	void (*iThrow)(TAny*, TAny*, void (*)(TAny*));
	TAny* iTypeInfo;
	TUint8* iVtable;
	};

static TNeoCppTraps* NeoCppTraps()
	{
	return static_cast<TNeoCppTraps*>(UserSvr::DllTls(KNeoCppTrapTls));
	}

extern void PanicNoTrapFrame();
extern void NeoTrapLongjmp(TTrap* aFrame, TInt aReason) __attribute__((noreturn));	// uc_trp.cia

EXPORT_C TTrapHandler* User::MarkCleanupStack()
	{
	TNeoCppTraps* t = NeoCppTraps();
	if (!t)
		{
		t = static_cast<TNeoCppTraps*>(User::AllocZ(sizeof(TNeoCppTraps)));
		if (!t || UserSvr::DllSetTls(KNeoCppTrapTls, t) != KErrNone)
			User::Panic(KNeoCppTrap, KErrNoMemory);
		}
	if (t->iDepth == KNeoCppTrapDepth)
		User::Panic(KNeoCppTrap, KErrOverflow);
	TTrapHandler* pH = Exec::PushTrapFrame(&t->iMark[t->iDepth++]);
	if (pH)
		pH->Trap();
	return pH;
	}

EXPORT_C void User::UnMarkCleanupStack(TTrapHandler* aHandler)
	{
	TTrap* frame = Exec::PopTrapFrame();
	TNeoCppTraps* t = NeoCppTraps();
	if (!t || !t->iDepth || frame != &t->iMark[t->iDepth - 1])
		User::Panic(KNeoCppTrap, KErrCorrupt);
	--t->iDepth;
	if (aHandler)
		aHandler->UnTrap();
	}

static void NeoCppTrapThrow(TNeoCppTraps& aTraps, TTrap* aFrame, TInt aReason) __attribute__((noreturn));
static void NeoCppTrapThrow(TNeoCppTraps& aTraps, TTrap* aFrame, TInt aReason)
	{
	aTraps.iDepth = aFrame - aTraps.iMark;		// this TRAP and every one inside it are left
	if (aFrame->iHandler)
		aFrame->iHandler->Leave(aReason);		// the cleanup stack back to the TRAP's mark
	if (!aTraps.iThrow)
		{
		if (aTraps.iRt.Load(_L("drtaeabi.dll")) != KErrNone)
			User::Panic(KNeoCppTrap, aReason);
		aTraps.iAlloc = (TAny* (*)(TUint))aTraps.iRt.Lookup(KOrdAllocateException);
		aTraps.iTypeInfo = (TAny*)aTraps.iRt.Lookup(KOrdLeaveTypeInfo);
		aTraps.iVtable = (TUint8*)aTraps.iRt.Lookup(KOrdLeaveVtable);
		aTraps.iThrow = (void (*)(TAny*, TAny*, void (*)(TAny*)))aTraps.iRt.Lookup(KOrdThrow);
		if (!aTraps.iAlloc || !aTraps.iTypeInfo || !aTraps.iVtable || !aTraps.iThrow)
			User::Panic(KNeoCppTrap, aReason);
		}
	// an XLeaveException as RVCT lays it out: {vptr (past the vtable's offset-to-top and typeinfo words), reason}
	TInt* x = static_cast<TInt*>(aTraps.iAlloc(2 * sizeof(TInt)));
	x[0] = reinterpret_cast<TInt>(aTraps.iVtable + 2 * sizeof(TAny*));
	x[1] = aReason;
	aTraps.iThrow(x, aTraps.iTypeInfo, NULL);
	for (;;) {}
	}

EXPORT_C void User::Leave(TInt aReason)
/**
Leaves the currently executing function, unwinds the call stack, and returns
from the most recently entered trap harness.

@param aReason The value returned from the most recent call to TRAP or TRAPD.
               This is known as the reason code and, typically, it gives the
               reason for the environment or user error causing this leave
               to occur.

@see TRAP
@see TRAPD
*/
	{
	TTrap* frame = Exec::PopTrapFrame();
	if (!frame)
		PanicNoTrapFrame();
	TNeoCppTraps* t = NeoCppTraps();
	if (t && frame >= t->iMark && frame < t->iMark + KNeoCppTrapDepth)
		NeoCppTrapThrow(*t, frame, aReason);
	NeoTrapLongjmp(frame, aReason);
	}
#endif




EXPORT_C TTrapHandler::TTrapHandler()
/**
Default constructor.
*/
	{}


#ifndef __LEAVE_EQUALS_THROW__

EXPORT_C void TTrap::UnTrap()
//
// Pop the current trap frame
//
	{

	TTrapHandler *pH=Exec::PopTrapFrame()->iHandler;
	if (pH!=NULL)
		pH->UnTrap();
	}


#else //__LEAVE_EQUALS_THROW__

EXPORT_C void User::Leave(TInt aReason)
/**
Leaves the currently executing function, unwinds the call stack, and returns
from the most recently entered trap harness.

@param aReason The value returned from the most recent call to TRAP or TRAPD.
               This is known as the reason code and, typically, it gives the
               reason for the environment or user error causing this leave
               to occur.
              
@see TRAP
@see TRAPD              
*/
	{
#ifdef __USERSIDE_THREAD_DATA__
	Exec::LeaveStart();
	TTrapHandler* pH = GetTrapHandler();
#else
	TTrapHandler* pH = Exec::LeaveStart();
#endif
	if (pH)
		pH->Leave(aReason);	// causes things on the cleanup stack to be cleaned up
	throw XLeaveException(aReason);
	}

#endif // !__LEAVE_EQUALS_THROW__


// Private declaration to prevent def file branching
#ifndef __SUPPORT_CPP_EXCEPTIONS__
class XLeaveException
	{
public:
	IMPORT_C TInt GetReason() const;
	};
#endif //__SUPPORT_CPP_EXCEPTIONS__

#if !defined(__LEAVE_EQUALS_THROW__) || !defined(__WINS__)
EXPORT_C TInt XLeaveException::GetReason() const
	{
#ifdef __SUPPORT_CPP_EXCEPTIONS__
	Exec::LeaveEnd();
	return iR;
#else // !__SUPPORT_CPP_EXCEPTIONS__
	return reinterpret_cast<const TInt*>(this)[1];	// NeoSymbian: {vptr, iR}, as User::Leave throws it (above)
#endif //__SUPPORT_CPP_EXCEPTIONS__
	}
#endif	// !defined(__LEAVE_EQUALS_THROW__) || !defined(__WINS__)

EXPORT_C void User::LeaveNoMemory()
/**
Leaves with the specific reason code KErrNoMemory.

@see KErrNoMemory
*/
	{

	User::Leave(KErrNoMemory);
	}




EXPORT_C TInt User::LeaveIfError(TInt aReason)
/**
Leaves or returns with a specified reason code.

If the reason code is negative the function leaves, and the reason code is 
returned through the trap harness.

If the reason code is zero or positive, the function simply returns with the 
reason value.

@param aReason The reason code.

@return If the function returns, the reason code which is either zero or positive.
*/
	{

	if (aReason<0)
		User::Leave(aReason);
	return(aReason);
	}




EXPORT_C TAny * User::LeaveIfNull(TAny *aPtr)
/**
Leaves with the reason code KErrNoMemory, if the specified pointer is NULL. 

If the pointer is not NULL, the function simply returns with the value of 
the pointer.

@param aPtr The pointer to be tested.

@return If the function returns, the value of aPtr.
*/
	{

	if (aPtr==NULL)
		User::LeaveNoMemory();
	return(aPtr);
	}

