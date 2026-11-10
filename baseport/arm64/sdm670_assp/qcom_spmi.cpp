// SPDX-License-Identifier: Apache-2.0
//
// qcom_spmi.cpp -- reads PMIC registers through the SDM670's SPMI PMIC arbiter v5 (DTB spmi@c440000, ee 0), after
// Linux drivers/spmi/spmi-pmic-arb.c (the v5 ops): each peripheral (ppid = sid << 8 | periph) has an arbiter channel
// (apid) listed in the core's map at 0x900 + 4 * apid; a read goes through the observer channel obsrvr + 0x80 * apid
// with command EXT_READL, polled until STATUS says done.
// A write goes through the peripheral's own channel chnls + 0x10000 * apid (mapped on first use), if the ownership
// table (cnfg + 0x700 + 4 * apid) gives the peripheral to ee 0.
#include <kernel/kern_priv.h>
#include <mmboot.h>
#include "board.h"
#include "qcom_spmi.h"
#include "a64_device.h"

const TUint32 KArbCore = 0x0c440000, KArbObsrvr = 0x0e600000, KArbCnfg = 0x0c40a000;	// board IO regions
const TUint32 KArbChnls = 0x0c600000;							// 32 MB: pages mapped as needed
const TInt KMaxApid = 512;										// v5: the map fills core 0x900..0x10ff
enum { ECmd = 0x00, EStatus = 0x08, EWData0 = 0x10, ERData0 = 0x18 };
enum { EOpExtWriteL = 0, EOpExtReadL = 1 };
enum { EStDone = 1, EStFailure = 2, EStDenied = 4, EStDropped = 8 };

static TInt16 ApidOf[0x1000];									// ppid -> apid, -1 = none
static TBool Ready;
static TInt Version = KErrNotReady;

static volatile TUint32* Reg(TUint32 aBase, TUint32 aOff) { return (volatile TUint32*)(A64_Io(aBase) + aOff); }

// poll a channel's STATUS: the byte read (aRData) or KErrNone, or an error; never hangs the caller
static int Wait(volatile TUint32* aCh, TBool aRead)
	{
	for (TInt i = 0; i < 100000; ++i)
		{
		TUint32 st = aCh[EStatus / 4];
		if (st & (EStFailure | EStDenied | EStDropped))
			return KErrGeneral;
		if (st & EStDone)
			return aRead ? (int)(aCh[ERData0 / 4] & 0xff) : KErrNone;
		}
	return KErrTimedOut;
	}

int Spmi_Init()
	{
	if (Ready)
		return Version;
	TUint32 ver = *Reg(KArbCore, 0);
	if (ver < 0x50000000u)
		return Version = KErrNotSupported;
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
	return Version = (int)ver;
	}

int Spmi_Version() { return Version; }

// The bus belongs to the ASSP, not the keys driver. RTC, audio and future PMIC clients may be built without keys.
void A64StartBuses()
	{
	TInt ver = Spmi_Init();
	Kern::Printf("[spmi] arbiter version %08x; apid PON %d, PM660L GPIO7 %d, PM660 GPIO12 %d", ver,
		Spmi_Apid(0, 0x08), Spmi_Apid(2, 0xc6), Spmi_Apid(0, 0xcb));
	}

static TA64DevState SpmiStatus(TDes8& aDetail)
	{
	A64Device_Format(aDetail, "arbiter version %08x; polled", Version);
	return Ready ? EA64DevOk : (Version == KErrNotReady ? EA64DevOff : EA64DevError);
	}
A64_DEVICE("spmi", "PMIC register bus", "qcom_spmi", SpmiStatus);

int Spmi_Apid(unsigned aSid, unsigned aPeriph)
	{
	return Ready && aSid <= 0xf && aPeriph <= 0xff ? ApidOf[(aSid << 8) | aPeriph] : -1;
	}

int Spmi_Read(unsigned aSid, unsigned aAddr)
	{
	if (aSid > 0xf || aAddr > 0xffff)
		return KErrArgument;
	TInt apid = Spmi_Apid(aSid, aAddr >> 8);
	if (apid < 0)
		return KErrNotFound;
	TUint32 ch = 0x80 * apid;										// ee 0's observer channels
	*Reg(KArbObsrvr, ch + ECmd) = (EOpExtReadL << 27) | ((aAddr & 0xff) << 4) | 0;	// byte count - 1 = 0
	return Wait(Reg(KArbObsrvr, ch), ETrue);
	}

static struct { TInt iApid; DPlatChunkHw* iChunk; } WriteChannels[4];

int Spmi_Write(unsigned aSid, unsigned aAddr, unsigned aValue)
	{
	if (aSid > 0xf || aAddr > 0xffff || aValue > 0xff)
		return KErrArgument;
	TInt apid = Spmi_Apid(aSid, aAddr >> 8);
	if (apid < 0)
		return KErrNotFound;
	if ((*Reg(KArbCnfg, 0x700 + 4 * apid) & 7) != 0)			// another execution environment owns it
		return KErrAccessDenied;
	DPlatChunkHw* chunk = NULL;
	TUint i;
	for (i = 0; i < sizeof(WriteChannels) / sizeof(WriteChannels[0]) && WriteChannels[i].iChunk; ++i)
		if (WriteChannels[i].iApid == apid)
			chunk = WriteChannels[i].iChunk;
	if (!chunk)
		{
		if (i == sizeof(WriteChannels) / sizeof(WriteChannels[0]))
			return KErrNoMemory;
		NKern::ThreadEnterCS();
		TInt r = DPlatChunkHw::New(chunk, KArbChnls + 0x10000 * apid, KPageSize, EMapAttrSupRw | EMapAttrFullyBlocking);
		NKern::ThreadLeaveCS();
		if (r != KErrNone)
			return r;
		WriteChannels[i].iApid = apid;
		WriteChannels[i].iChunk = chunk;
		}
	volatile TUint32* ch = (volatile TUint32*)chunk->LinearAddress();
	ch[EWData0 / 4] = aValue & 0xff;
	ch[ECmd / 4] = (EOpExtWriteL << 27) | ((aAddr & 0xff) << 4) | 0;
	return Wait(ch, EFalse);
	}
