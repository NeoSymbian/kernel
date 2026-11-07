// SPDX-License-Identifier: Apache-2.0
//
// qcom_bootctrl.cpp -- docs/filesystem.md FS5b: the A/B slot state ABL keeps in the GPT, read and (gate G3, approved
// 2026-11-07) the one write NeoSymbian needs: "this slot booted successfully". Without it ABL counts the slot's
// retries down on every boot -- fastboot RAM boots included -- and at 0 switches to the other slot
// (docs/known-issues.md).
// ABL's layout, read off this phone's GPT and checked against `fastboot getvar slot-*` (2026-11-07): byte 6 of the
// boot_a / boot_b entries' attribute field (attribute bits 48-55) = priority (bits 0-1), active (2), retry count
// (3-5), successful (6), unbootable (7). The same layout as Qualcomm's BSD-licensed bootctrl HAL and ABL.
// This phone's ABL counts a slot's tries only on fastboot RAM boots, never on normal ones (seen 2026-11-08: a crash
// loop left slot b at 3 retries), so an install image counts them itself at boot (BootCount): a slot not yet marked
// successful loses a try; with none left it is marked unbootable, the other slot (if marked successful) made active
// as `fastboot set_active` does, and the phone restarted into it. Test images (AUTOFASTBOOT) never count.
// Both GPT headers on this phone point at ONE entry array (LBA 2; the array at the disk's end is stale): every write
// is that entry sector, then the backup header, then the primary, each copy read back with its CRCs.
// Every write: both GPT copies, the backup first, one sector at a time through A64BlkDev_WriteGpt (the GPT's own
// sectors only), each copy read back and its CRCs checked before the next is touched.
#include <kernel/kern_priv.h>
#include "a64_blk.h"

const TInt KHdrLba = 1, KMaxEntryBytes = 128 * 128;
const TUint8 KActive = 1 << 2, KOk = 1 << 6, KUnbootable = 1 << 7;

static TUint32 Crc32(const TUint8* aP, TInt aLen)			// IEEE 802.3, as the GPT uses
	{
	TUint32 c = ~0u;
	while (aLen--)
		{
		c ^= *aP++;
		for (TInt k = 0; k < 8; ++k)
			c = c & 1 ? (c >> 1) ^ 0xedb88320 : c >> 1;
		}
	return ~c;
	}
static TUint32 Get32(const TUint8* p) { return p[0] | p[1] << 8 | p[2] << 16 | (TUint32)p[3] << 24; }
static TUint64 Get64(const TUint8* p) { return Get32(p) | (TUint64)Get32(p + 4) << 32; }
static void Put32(TUint8* p, TUint32 v) { p[0] = (TUint8)v; p[1] = (TUint8)(v >> 8); p[2] = (TUint8)(v >> 16); p[3] = (TUint8)(v >> 24); }

struct TGpt
	{
	TUint64 iHdrLba, iEntLba;
	TInt iEntries, iEntrySize, iSlot[2];				// iSlot: the entry index of boot_a, boot_b (-1: none)
	TUint8 iHdr[KA64BlkSector];
	TUint8 iEnt[KMaxEntryBytes];
	TUint8 Attr(TInt aSlot) const { return iEnt[iSlot[aSlot] * iEntrySize + 54]; }
	TInt Load(TUint64 aHdrLba);
	};
static TGpt Primary, Backup;
static DMutex* Lock;
static TInt Current;										// 'a' / 'b', once read

static TBool NameIs(const TUint8* aE, const char* aName)	// the UTF-16 name at entry offset 56
	{
	for (TInt c = 0; ; ++c)
		{
		if (aE[56 + 2 * c] != (TUint8)aName[c] || aE[57 + 2 * c] != 0)
			return EFalse;
		if (!aName[c])
			return ETrue;
		}
	}

// the header at aHdrLba and its entry array, both CRCs checked, boot_a / boot_b found exactly once
TInt TGpt::Load(TUint64 aHdrLba)
	{
	iHdrLba = aHdrLba;
	TInt r = A64BlkDev_Read(aHdrLba, 1, iHdr);
	if (r != KErrNone)
		return r;
	const TUint32 hsize = Get32(iHdr + 12);
	if (memcompare(iHdr, 8, (const TUint8*)"EFI PART", 8) != 0 || hsize < 92 || hsize > KA64BlkSector)
		return KErrCorrupt;
	const TUint32 hcrc = Get32(iHdr + 16);
	Put32(iHdr + 16, 0);
	const TBool hok = Crc32(iHdr, hsize) == hcrc;
	Put32(iHdr + 16, hcrc);
	iEntLba = Get64(iHdr + 72);
	iEntries = Get32(iHdr + 80);
	iEntrySize = Get32(iHdr + 84);
	const TInt bytes = iEntries * iEntrySize;
	if (!hok || iEntrySize < 128 || bytes <= 0 || bytes > KMaxEntryBytes || bytes % KA64BlkSector)
		return KErrCorrupt;
	r = A64BlkDev_Read(iEntLba, bytes / KA64BlkSector, iEnt);
	if (r != KErrNone)
		return r;
	if (Crc32(iEnt, bytes) != Get32(iHdr + 88))
		return KErrCorrupt;
	TInt found[2] = { 0, 0 };
	iSlot[0] = iSlot[1] = -1;
	for (TInt i = 0; i < iEntries; ++i)
		for (TInt s = 0; s < 2; ++s)
			if (NameIs(iEnt + i * iEntrySize, s ? "boot_b" : "boot_a") && ++found[s] == 1)
				iSlot[s] = i;
	return found[0] == 1 && found[1] == 1 ? KErrNone : KErrNotFound;
	}

// both copies, which must agree on the slot bytes
static TInt LoadBoth()
	{
	TInt r = Primary.Load(KHdrLba);
	if (r == KErrNone)
		r = Backup.Load(Get64(Primary.iHdr + 32));			// the primary's AlternateLBA
	if (r == KErrNone && (Primary.Attr(0) != Backup.Attr(0) || Primary.Attr(1) != Backup.Attr(1)))
		r = KErrCorrupt;
	return r;
	}

static void Fmt(TDes8& aOut, const char* aFmt, ...)
	{
	VA_LIST list;
	VA_START(list, aFmt);
	Kern::AppendFormat(aOut, aFmt, list);
	VA_END(list);
	}

// Kern::Printf formats into 256 bytes and faults past them: long results go out in pieces
static void PrintLong(const TDesC8& aText)
	{
	for (TInt i = 0; i < aText.Length(); i += 200)
		{
		TPtrC8 piece = aText.Mid(i, Min(200, aText.Length() - i));
		Kern::Printf("[slot] %S", &piece);
		}
	}

static void Describe(TDes8& aOut, TInt aSlot, TUint8 aAttr)
	{
	Fmt(aOut, "%c %02x: priority %d%s, retries %d%s%s", 'a' + aSlot, aAttr, aAttr & 3, aAttr & KActive ? " active" : "",
		(aAttr >> 3) & 7, aAttr & KOk ? ", successful" : "", aAttr & KUnbootable ? ", UNBOOTABLE" : "");
	}

static TBool BootCount(TBool aDryRun, TDes8& aOut);
void A64_RebootNormal();								// a64_reboot.cpp

TInt A64Slot_Current()
	{
	if (Current)
		return Current;
	if (!Lock)
		{
		_LIT(KLockName, "BootCtrl");
		NKern::ThreadEnterCS();
		Kern::MutexCreate(Lock, KLockName, KMutexOrdGeneral0);
		NKern::ThreadLeaveCS();
		}
	TInt r = Primary.Load(KHdrLba);
	const TBool a = r == KErrNone && (Primary.Attr(0) & KActive), b = r == KErrNone && (Primary.Attr(1) & KActive);
	Current = b && !a ? 'b' : 'a';						// no or an unreadable GPT: slot a, as before FS5b
	TBuf8<160> d;
	if (r == KErrNone)
		{
		Describe(d, 0, Primary.Attr(0));
		d.Append((const TUint8*)"; ", 2);
		Describe(d, 1, Primary.Attr(1));
		}
	Kern::Printf("[slot] booted from slot %c (GPT -> %d) %S", Current, r, &d);
#ifndef A64_AUTOFASTBOOT
	if (r == KErrNone)										// an install image: count this try (see the top)
		{
		TBuf8<800> out;
		NKern::ThreadEnterCS();
		Kern::MutexWait(*Lock);
		const TBool restart = LoadBoth() == KErrNone && BootCount(EFalse, out);
		Kern::MutexSignal(*Lock);
		NKern::ThreadLeaveCS();
		PrintLong(out);
		if (restart)
			A64_RebootNormal();
		}
#endif
	return Current;
	}

void A64Slot_Show(TDes8& aOut)
	{
	if (!Lock)
		A64Slot_Current();
	NKern::ThreadEnterCS();
	Kern::MutexWait(*Lock);
	TInt r = LoadBoth();
	if (r == KErrNone)
		{
		Fmt(aOut, "current %c; ", A64Slot_Current());
		Describe(aOut, 0, Primary.Attr(0));
		aOut.Append((const TUint8*)"; ", 2);
		Describe(aOut, 1, Primary.Attr(1));
		Fmt(aOut, "; GPT copies at %d and %d agree", (TInt)Primary.iHdrLba, (TInt)Backup.iHdrLba);
		}
	else
		Fmt(aOut, "GPT -> %d (primary at 1, backup at %d)", r, (TInt)Get64(Primary.iHdr + 32));
	Kern::MutexSignal(*Lock);
	NKern::ThreadLeaveCS();
	}

// set the slot byte's "successful" bit in one copy: the entry sector, then the header, then read back
static TInt Update(TGpt& aG, TInt aSlot, TUint8 aNew, TBool aDryRun, TDes8& aOut)
	{
	const TInt off = aG.iSlot[aSlot] * aG.iEntrySize + 54, bytes = aG.iEntries * aG.iEntrySize;
	const TUint32 oldEnt = Get32(aG.iHdr + 88), oldHdr = Get32(aG.iHdr + 16);
	const TUint8 old = aG.iEnt[off];
	aG.iEnt[off] = aNew;
	Put32(aG.iHdr + 88, Crc32(aG.iEnt, bytes));
	Put32(aG.iHdr + 16, 0);
	Put32(aG.iHdr + 16, Crc32(aG.iHdr, Get32(aG.iHdr + 12)));
	const TUint64 entSector = aG.iEntLba + off / KA64BlkSector;
	Fmt(aOut, "[hdr %d: sector %d+%d %02x>%02x crc %08x>%08x hcrc %08x>%08x] ", (TInt)aG.iHdrLba, (TInt)entSector,
		off % KA64BlkSector, old, aNew, oldEnt, Get32(aG.iHdr + 88), oldHdr, Get32(aG.iHdr + 16));
	if (aDryRun)
		return KErrNone;
	TInt r = A64BlkDev_WriteGpt(entSector, aG.iEnt + (off / KA64BlkSector) * KA64BlkSector);
	if (r == KErrNone)
		r = A64BlkDev_WriteGpt(aG.iHdrLba, aG.iHdr);
	if (r == KErrNone)
		r = aG.Load(aG.iHdrLba);						// read back: both CRCs and the byte
	if (r == KErrNone && aG.iEnt[off] != aNew)
		r = KErrCorrupt;
	return r;
	}

// one slot byte in both copies (backup first), then both read back and agreeing
static TInt SetSlotByte(TInt aSlot, TUint8 aNew, TBool aDryRun, TDes8& aOut)
	{
	TInt r = Update(Backup, aSlot, aNew, aDryRun, aOut);
	if (r == KErrNone)
		r = Update(Primary, aSlot, aNew, aDryRun, aOut);
	if (!aDryRun && r == KErrNone)
		r = LoadBoth();
	return r;
	}

// under Lock, both copies loaded: ABL's try counting, done by us (see the top). ETrue if the phone must restart
static TBool BootCount(TBool aDryRun, TDes8& aOut)
	{
	const TInt cur = Current - 'a', other = 1 - cur;
	const TUint8 v = Primary.Attr(cur), o = Primary.Attr(other);
	const TInt tries = (v >> 3) & 7;
	if (v & KOk)
		{
		Fmt(aOut, "slot %c is marked successful: no tries counted", 'a' + cur);
		return EFalse;
		}
	if (tries > 0)
		{
		Fmt(aOut, "slot %c not yet successful: try %d left after this boot: ", 'a' + cur, tries - 1);
		Fmt(aOut, "-> %d", SetSlotByte(cur, (TUint8)((v & ~0x38) | (tries - 1) << 3), aDryRun, aOut));
		return EFalse;
		}
	if (!(o & KOk) || (o & KUnbootable))
		{
		Fmt(aOut, "slot %c has no tries left, but slot %c (%02x) is no better: carrying on", 'a' + cur, 'a' + other, o);
		return EFalse;
		}
	Fmt(aOut, "slot %c has no tries left: back to slot %c: ", 'a' + cur, 'a' + other);
	TInt r = SetSlotByte(other, (TUint8)(o | KActive | 3), aDryRun, aOut);	// the other slot active, top priority
	if (r == KErrNone)
		r = SetSlotByte(cur, (TUint8)((v & ~(KActive | 3)) | KUnbootable), aDryRun, aOut);
	Fmt(aOut, "-> %d", r);
	return r == KErrNone && !aDryRun;
	}

// the monitor's "slot boot": what the boot-time count would do now, never written
void A64Slot_BootDry(TDes8& aOut)
	{
	A64Slot_Current();
	NKern::ThreadEnterCS();
	Kern::MutexWait(*Lock);
	if (LoadBoth() == KErrNone)
		BootCount(ETrue, aOut);
	else
		aOut.Append((const TUint8*)"GPT unreadable", 14);
	Kern::MutexSignal(*Lock);
	NKern::ThreadLeaveCS();
	}

TInt A64Slot_MarkOk(TBool aDryRun, TDes8& aOut)
	{
	const TInt slot = A64Slot_Current() - 'a';
	NKern::ThreadEnterCS();
	Kern::MutexWait(*Lock);
	TInt r = LoadBoth();
	if (r == KErrNone)
		{
		const TUint8 old = Primary.Attr(slot);
		if (old & KOk)
			Fmt(aOut, "slot %c already successful (%02x)", 'a' + slot, old);
		else
			{
			Fmt(aOut, "%s slot %c successful: ", aDryRun ? "would mark" : "marking", 'a' + slot);
			r = Update(Backup, slot, old | KOk, aDryRun, aOut);			// the backup first
			if (r == KErrNone)
				r = Update(Primary, slot, old | KOk, aDryRun, aOut);
			if (!aDryRun && r == KErrNone)
				r = LoadBoth();										// both copies again, agreeing
			Fmt(aOut, "-> %d", r);
			}
		}
	else
		Fmt(aOut, "GPT -> %d: nothing written", r);
	Kern::MutexSignal(*Lock);
	NKern::ThreadLeaveCS();
	PrintLong(aOut);
	return r;
	}
