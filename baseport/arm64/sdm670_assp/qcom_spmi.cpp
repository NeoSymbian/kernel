// SPDX-License-Identifier: Apache-2.0
//
// qcom_spmi.cpp -- reads PMIC registers through the SDM670's SPMI PMIC arbiter v5 (DTB spmi@c440000, ee 0), after
// Linux drivers/spmi/spmi-pmic-arb.c (the v5 ops): each peripheral (ppid = sid << 8 | periph) has an arbiter channel
// (apid) listed in the core's map at 0x900 + 4 * apid; a read goes through the observer channel obsrvr + 0x80 * apid
// with command EXT_READL, polled until STATUS says done.
// shortcut: reads only; writes (the chnls channel + the ownership table) come with the touchscreen's supply (M4.7b).
#include <kernel/kern_priv.h>
#include "board.h"
#include "qcom_spmi.h"

const TUint32 KArbCore = 0x0c440000, KArbObsrvr = 0x0e600000;	// board IO regions (pixel_bonito.cpp)
const TInt KMaxApid = 512;										// v5: the map fills core 0x900..0x10ff
enum { ECmd = 0x00, EStatus = 0x08, ERData0 = 0x18 };
enum { EOpExtReadL = 1 };
enum { EStDone = 1, EStFailure = 2, EStDenied = 4, EStDropped = 8 };

static TInt16 ApidOf[0x1000];									// ppid -> apid, -1 = none
static TBool Ready;

static volatile TUint32* Reg(TUint32 aBase, TUint32 aOff) { return (volatile TUint32*)(A64_Io(aBase) + aOff); }

int Spmi_Init()
	{
	TUint32 ver = *Reg(KArbCore, 0);
	if (ver < 0x50000000u)
		return KErrNotSupported;
	for (TInt i = 0; i < 0x1000; ++i)
		ApidOf[i] = -1;
	for (TInt apid = 0; apid < KMaxApid; ++apid)
		{
		TUint32 m = *Reg(KArbCore, 0x900 + 4 * apid);
		TUint32 ppid = (m >> 8) & 0xfff;
		if (m && ApidOf[ppid] < 0)
			ApidOf[ppid] = (TInt16)apid;
		}
	Ready = ETrue;
	return (int)ver;
	}

int Spmi_Apid(unsigned aSid, unsigned aPeriph)
	{
	return Ready ? ApidOf[((aSid & 0xf) << 8) | (aPeriph & 0xff)] : -1;
	}

int Spmi_Read(unsigned aSid, unsigned aAddr)
	{
	TInt apid = Spmi_Apid(aSid, aAddr >> 8);
	if (apid < 0)
		return KErrNotFound;
	TUint32 ch = 0x80 * apid;										// ee 0's observer channels
	*Reg(KArbObsrvr, ch + ECmd) = (EOpExtReadL << 27) | ((aAddr & 0xff) << 4) | 0;	// byte count - 1 = 0
	for (TInt i = 0; i < 100000; ++i)								// a few us normally; never hang the caller
		{
		TUint32 st = *Reg(KArbObsrvr, ch + EStatus);
		if (st & (EStFailure | EStDenied | EStDropped))
			return KErrGeneral;
		if (st & EStDone)
			return (int)(*Reg(KArbObsrvr, ch + ERData0) & 0xff);
		}
	return KErrTimedOut;
	}
