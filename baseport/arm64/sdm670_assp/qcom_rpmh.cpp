// SPDX-License-Identifier: Apache-2.0
//
// qcom_rpmh.cpp -- a minimal RPMh bus vote for the SDM670: the apps RSC's DRV2 sends one active-only command,
// polled, to a Bus Clock Manager (BCM) or a power rail (ARC) found by name in the AOP's command DB. ABL drops its
// votes for the devices it shuts down at handoff; Linux re-votes them at probe from the DT (sdhci-msm's opp table:
// CX low_svs, and the eMMC's path xm_emmc -> aggre1_noc -> snoc = BCM "SN8"), and so do we.
// Written from the register layout and sequences in Linux drivers/soc/qcom/cmd-db.c, rpmh-rsc.c (layout v2.7),
// drivers/interconnect/qcom/bcm-voter.c and include/soc/qcom/tcs.h; facts from the phone's DTB (rsc@179c0000
// drv-id 2, tcs-offset 0xd00, tcs-config ACTIVE 2 first; aop-cmd-db-mem@85fe0000).
// shortcut: active-only votes that are never dropped, no sleep/wake sets; proper voting comes with H3 power.
#include <kernel/kern_priv.h>
#include "board.h"
#include "qcom_rpmh.h"

const TUint32 KCmdDb = 0x85fe0000, KDrv2 = 0x179e0000, KTcsBase = KDrv2 + 0xd00;
const TInt KTcs = 0;										// the first ACTIVE TCS (tcs-config lists ACTIVE first)
// layout v2.7: per TCS (stride 672) and per command (stride 20)
enum { EIrqEnable = 0x00, EIrqStatus = 0x04, EIrqClear = 0x08 };	// at the TCS base, one bit per TCS
enum { EControl = 0x14, EStatus = 0x18, ECmdEnable = 0x1c, ECmdMsgId = 0x30, ECmdAddr = 0x34, ECmdData = 0x38,
	ECmdStatus = 0x3c };
const TUint32 KAmcEnable = 1u << 16, KAmcTrigger = 1u << 24;
const TUint32 KMsgIdWriteRr = (1u << 16) | (1u << 8) | 4;	// write, response required, 4 bytes
const TUint32 KCmdCompl = 1u << 16;
const TInt KSpin = 2000000;

static volatile TUint32& Tcs(TUint32 aReg) { return *(volatile TUint32*)(A64_Io(KTcsBase) + 672 * KTcs + aReg); }
static volatile TUint32& TcsGlobal(TUint32 aReg) { return *(volatile TUint32*)(A64_Io(KTcsBase) + aReg); }

// ---- the command DB (Linux cmd-db.c): a header {version, magic, 8 x rsc_hdr, checksum, reserved}, then data
struct SRscHdr { TUint16 iSlvId, iHeaderOffset, iDataOffset, iCnt, iVersion, iReserved[3]; };
struct SEntry { TUint8 iId[8]; TUint32 iPriority[2]; TUint32 iAddr; TUint16 iLen, iOffset; };
const TInt KDbData = 4 + 4 + 8 * sizeof(SRscHdr) + 4 + 4;

static TInt Find(const char* aName, TUint32& aAddr, const TUint8*& aAux, TInt& aAuxLen)
	{
	const TUint8* db = (const TUint8*)A64_Io(KCmdDb);
	if (!db || db[4] != 0xdb || db[5] != 0x30 || db[6] != 0x03 || db[7] != 0x0c)
		return KErrNotFound;
	TUint8 key[8] = { 0 };
	for (TInt i = 0; i < 8 && aName[i]; ++i)
		key[i] = (TUint8)aName[i];
	const SRscHdr* rh = (const SRscHdr*)(db + 8);
	for (TInt s = 0; s < 8 && rh[s].iSlvId; ++s)
		{
		const SEntry* e = (const SEntry*)(db + KDbData + rh[s].iHeaderOffset);
		for (TInt j = 0; j < rh[s].iCnt; ++j, ++e)
			if (memcompare(e->iId, 8, key, 8) == 0)
				{
				aAddr = e->iAddr;
				aAux = db + KDbData + rh[s].iDataOffset + e->iOffset;
				aAuxLen = e->iLen;
				return KErrNone;
				}
		}
	return KErrNotFound;
	}

TInt CmdDb_Find(const char* aName, TUint32& aAddr, TUint32 aAux[2])
	{
	const TUint8* aux = NULL;
	TInt len = 0;
	aAux[0] = aAux[1] = 0;
	TInt r = Find(aName, aAddr, aux, len);
	if (r == KErrNone && len >= 8)							// a BCM's bcm_db: unit (u32), width (u16), vcd, reserved
		memcpy(aAux, aux, 8);
	return r;
	}

// ---- one active-only command on TCS 0 (Linux rpmh-rsc.c __tcs_buffer_write + __tcs_set_trigger), polled
static TInt Rpmh_Write(TUint32 aAddr, TUint32 aData)
	{
	if ((*(volatile TUint32*)A64_Io(KDrv2) >> 16 & 0xff) >= 3)	// RSC_DRV_ID major: v3 moves every register
		return KErrNotSupported;
	TInt i = 0;
	while (!Tcs(EStatus) && ++i < KSpin)					// nonzero = idle
		{}
	if (i >= KSpin)
		return KErrInUse;
	TcsGlobal(EIrqClear) = 1u << KTcs;
	TcsGlobal(EIrqEnable) |= 1u << KTcs;					// latch the completion (the GIC line stays off)
	Tcs(ECmdEnable) = 0;
	Tcs(ECmdMsgId) = KMsgIdWriteRr;
	Tcs(ECmdAddr) = aAddr;
	Tcs(ECmdData) = aData;
	Tcs(ECmdEnable) = 1;
	TUint32 c = Tcs(EControl) & ~KAmcTrigger;				// HW req: clear the trigger, then the mode, then re-arm
	Tcs(EControl) = c;
	Tcs(EControl) = c & ~KAmcEnable;
	Tcs(EControl) = KAmcEnable;
	Tcs(EControl) = KAmcEnable | KAmcTrigger;
	for (i = 0; i < KSpin && !(TcsGlobal(EIrqStatus) & (1u << KTcs)); ++i)
		{}
	TUint32 st = Tcs(ECmdStatus);
	TcsGlobal(EIrqClear) = 1u << KTcs;
	Tcs(ECmdEnable) = 0;
	return i < KSpin && (st & KCmdCompl) ? KErrNone : KErrTimedOut;
	}

static void Fmt(TDes8& aBuf, const char* aFmt, ...)
	{
	VA_LIST list;
	VA_START(list, aFmt);
	Kern::AppendFormat(aBuf, aFmt, list);
	VA_END(list);
	}

// vote a BCM up for aAvgKBps / aPeakKBps through a node aBusWidth bytes wide (Linux bcm-voter.c bcm_aggregate,
// vote_scale 1000; at least 1, the keepalive vote)
TInt Bcm_Vote(const char* aName, TUint32 aAvgKBps, TUint32 aPeakKBps, TUint32 aBusWidth, TDes8* aOut)
	{
	TUint32 addr = 0, aux[2];
	TInt r = CmdDb_Find(aName, addr, aux);
	TUint32 unit = aux[0], width = aux[1] & 0xffff;
	TUint64 x = 1, y = 1;
	if (r == KErrNone && unit && width && aBusWidth)
		{
		x = (TUint64)aAvgKBps * 1000 * width / aBusWidth / unit;
		y = (TUint64)aPeakKBps * 1000 * width / aBusWidth / unit;
		x = x < 1 ? 1 : x > 0x3fff ? 0x3fff : x;
		y = y < 1 ? 1 : y > 0x3fff ? 0x3fff : y;
		}
	TUint32 cmd = (1u << 30) | (1u << 29) | ((TUint32)x << 14) | (TUint32)y;	// BCM_TCS_CMD(commit, valid, x, y)
	if (r == KErrNone)
		r = Rpmh_Write(addr, cmd);
	TBuf8<120> b;
	TPtrC8 name((const TUint8*)aName);
	Fmt(b, "[rpmh] %S: addr %05x unit %d width %d vcd %d, vote %08x -> %d", &name, addr, unit, width,
		(aux[1] >> 16) & 0xff, cmd, r);
	Kern::Printf("%S", &b);
	if (aOut)
		aOut->Copy(b);
	return r;
	}

// the debug monitor's "rpmh <name>": a BCM at the eMMC's DT bandwidth through a 16-byte NoC port, or a rail
void A64RpmhVote(const TDesC8& aName, TDes8& aOut)
	{
	char n[9] = { 0 };
	TBool arc = EFalse;
	for (TInt i = 0; i < aName.Length() && i < 8; ++i)
		{
		n[i] = (char)aName[i];
		arc = arc || n[i] == '.';
		}
	if (n[0] == 'l' && n[1] == 'd' && n[2] == 'o')			// "ldob7": a regulator, looked up only (no vote)
		{
		TUint32 addr = 0, aux[2];
		TInt r = CmdDb_Find(n, addr, aux);
		TPtrC8 name((const TUint8*)n);
		Fmt(aOut, "%S: cmd-db addr %05x -> %d (lookup only)", &name, addr, r);
		}
	else if (arc)
		Arc_Vote(n, 64, &aOut);								// "cx.lvl": low_svs, the eMMC's DT opp at 20 MHz
	else
		Bcm_Vote(n, 52286, 80000, 16, &aOut);
	}

// an ARC (power rail) corner (Linux pmdomain/qcom/rpmhpd.c): the aux data lists the rail's levels (u16 vlvl, e.g.
// 64 = low_svs, 128 = svs, 256 = nominal); the vote is the index of the first level >= aVlvl
TInt Arc_Vote(const char* aName, TUint32 aVlvl, TDes8* aOut)
	{
	TUint32 addr = 0;
	const TUint8* aux = NULL;
	TInt len = 0, hlvl = -1;
	TInt r = Find(aName, addr, aux, len);
	TUint32 got = 0;
	for (TInt i = 0; r == KErrNone && i < len / 2 && i < 32; ++i)
		{
		TUint32 v = aux[2 * i] | (aux[2 * i + 1] << 8);
		if (i && !v)
			break;											// past the rail's last level
		hlvl = i;											// above every level: the highest one (Linux INT_MAX)
		got = v;
		if (v >= aVlvl)
			break;
		}
	if (r == KErrNone && hlvl < 0)
		r = KErrNotFound;
	if (r == KErrNone)
		r = Rpmh_Write(addr, hlvl);
	TBuf8<120> b;
	TPtrC8 name((const TUint8*)aName);
	Fmt(b, "[rpmh] %S: addr %05x, %d levels, vlvl %d -> corner %d (vlvl %d) -> %d", &name, addr, len / 2, aVlvl,
		hlvl, got, r);
	Kern::Printf("%S", &b);
	if (aOut)
		aOut->Copy(b);
	return r;
	}

// a VRM regulator (an LDO or SMPS by its cmd-db name, e.g. "ldob7" = pm660l ldo7): voltage (mV) at +0, mode at +8
// (7 = an LDO's high-power mode), enable at +4 -- the layout of the RPMh VRM resource
TInt Vrm_Vote(const char* aName, TUint32 aMilliVolts, TDes8* aOut)
	{
	TUint32 addr = 0;
	const TUint8* aux = NULL;
	TInt len = 0;
	TInt r = Find(aName, addr, aux, len);
	if (r == KErrNone)
		r = Rpmh_Write(addr + 0, aMilliVolts);
	if (r == KErrNone)
		r = Rpmh_Write(addr + 8, 7);
	if (r == KErrNone)
		r = Rpmh_Write(addr + 4, 1);
	TBuf8<120> b;
	TPtrC8 name((const TUint8*)aName);
	Fmt(b, "[rpmh] %S: addr %05x, %d mV, HPM, on -> %d", &name, addr, aMilliVolts, r);
	Kern::Printf("%S", &b);
	if (aOut)
		aOut->Copy(b);
	return r;
	}

// one raw active-only write to a resource by name (+ aOffset): the XO clock is the ARC "xo.lvl", on = 3 (Linux
// clk-rpmh bi_tcxo), which has no level table for Arc_Vote
TInt Rpmh_Set(const char* aName, TUint32 aOffset, TUint32 aValue)
	{
	TUint32 addr = 0;
	const TUint8* aux = NULL;
	TInt len = 0;
	TInt r = Find(aName, addr, aux, len);
	if (r == KErrNone)
		r = Rpmh_Write(addr + aOffset, aValue);
	Kern::Printf("[rpmh] %s+%d = %d -> %d", aName, aOffset, aValue, r);
	return r;
	}

TInt Vrm_Off(const char* aName)
	{
	TUint32 addr = 0;
	const TUint8* aux = NULL;
	TInt len = 0;
	TInt r = Find(aName, addr, aux, len);
	if (r == KErrNone)
		r = Rpmh_Write(addr + 4, 0);
	Kern::Printf("[rpmh] %s: off -> %d", aName, r);
	return r;
	}
