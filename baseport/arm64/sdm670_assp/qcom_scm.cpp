// SPDX-License-Identifier: Apache-2.0
//
// qcom_scm.cpp -- calls into the SDM670's TrustZone (Qualcomm SCM over SMCCC 1.x): yielding SMC64, owner SIP, the
// function id 0x42000000 | svc << 8 | cmd, x1 = the arg info, x2..x5 = the first 4 args; with more, x5 points at a
// buffer holding args 3.. (TZ reads it by physical address, so it lives in an uncached page). A call TZ interrupts
// (x0 = 1) is resumed by calling again with x0 = 1; a busy one (x0 = -12) is retried after a sleep.
// Written from the calling convention in Linux drivers/firmware/qcom/qcom_scm-smc.c and qcom_scm.c (assign_mem,
// is_call_available). Telephony stage C2 (docs/telephony.md): the modem's memory goes to its VMID before it boots.
// shortcut: one caller at a time (the monitor, then the modem loader); a mutex comes with a second caller.
#include <kernel/kern_priv.h>
#include <platform.h>
#include "board.h"
#include "qcom_scm.h"

const TUint64 KFnBase = 0x42000000;						// yielding call, SMC64, owner SIP (2 << 24)
const TInt64 KInterrupted = 1, KBusy = -12;
const TInt KRegArgs = 4;
const TInt KPage = 0x1000;

struct SSmcRes { TUint64 iA0, iA1, iA2, iA3; };

static void Smc(TUint64 aX0, TUint64 aX1, TUint64 aX2, TUint64 aX3, TUint64 aX4, TUint64 aX5, SSmcRes& aRes)
	{
	register TUint64 x0 __asm__("x0") = aX0;
	register TUint64 x1 __asm__("x1") = aX1;
	register TUint64 x2 __asm__("x2") = aX2;
	register TUint64 x3 __asm__("x3") = aX3;
	register TUint64 x4 __asm__("x4") = aX4;
	register TUint64 x5 __asm__("x5") = aX5;
	register TUint64 x6 __asm__("x6") = 0;					// the "quirk" state some TZs return in x6: start at 0
	__asm__ __volatile__("smc #0"
		: "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3), "+r"(x4), "+r"(x5), "+r"(x6)
		:
		: "x7", "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	aRes.iA0 = x0;
	aRes.iA1 = x1;
	aRes.iA2 = x2;
	aRes.iA3 = x3;
	}

// one uncached page shared with TZ: the extra args at +0, assign_mem's lists at +0x100 / +0x200 / +0x300
static TUint8* Page;
static TUint32 PagePhys;

static TInt PageInit()
	{
	if (Page)
		return KErrNone;
	TPhysAddr phys;
	NKern::ThreadEnterCS();
	TInt r = Epoc::AllocPhysicalRam(KPage, phys);
	DPlatChunkHw* chunk = NULL;
	if (r == KErrNone)
		r = DPlatChunkHw::New(chunk, phys, KPage, EMapAttrSupRw | EMapAttrFullyBlocking);
	NKern::ThreadLeaveCS();
	if (r != KErrNone)
		return r;
	Page = (TUint8*)chunk->LinearAddress();
	PagePhys = phys;
	memclr(Page, KPage);
	return KErrNone;
	}

static TInt MapError(TInt64 aErr)							// Linux qcom_scm_remap_error
	{
	switch (aErr)
		{
	case -2: return KErrNotSupported;					// QCOM_SCM_EOPNOTSUPP
	case -3: case -4: return KErrArgument;				// EINVAL_ADDR, EINVAL_ARG
	case -12: return KErrInUse;							// V2_EBUSY
	case -13: return KErrNoMemory;						// ENOMEM
	default: return KErrGeneral;
		}
	}

TInt Scm_Call(TUint aSvc, TUint aCmd, TUint32 aArgInfo, const TUint64* aArgs, TInt aCount, TUint64 aRes[3])
	{
	if (aCount < 0 || aCount > 10)
		return KErrArgument;
	TUint64 a[KRegArgs] = { 0, 0, 0, 0 };
	for (TInt i = 0; i < aCount && i < KRegArgs; ++i)
		a[i] = aArgs[i];
	if (aCount > KRegArgs)									// args 3.. go in the page; x5 = its address
		{
		TInt r = PageInit();
		if (r != KErrNone)
			return r;
		volatile TUint64* ext = (volatile TUint64*)Page;
		for (TInt i = KRegArgs - 1; i < aCount; ++i)
			ext[i - (KRegArgs - 1)] = aArgs[i];
		a[KRegArgs - 1] = PagePhys;
		}
	SSmcRes res;
	TUint64 fn = KFnBase | (aSvc & 0xff) << 8 | (aCmd & 0xff);
	for (TInt busy = 0; ; ++busy)
		{
		Smc(fn, aArgInfo, a[0], a[1], a[2], a[3], res);
		while ((TInt64)res.iA0 == KInterrupted)			// TZ paused the call: resume it
			Smc(KInterrupted, aArgInfo, a[0], a[1], a[2], a[3], res);
		if ((TInt64)res.iA0 != KBusy || busy >= 20)
			break;
		NKern::Sleep(NKern::TimerTicks(30));
		}
	if (aRes)
		{
		aRes[0] = res.iA1;
		aRes[1] = res.iA2;
		aRes[2] = res.iA3;
		}
	return res.iA0 ? MapError((TInt64)res.iA0) : KErrNone;
	}

TInt Scm_IsAvailable(TUint aSvc, TUint aCmd)
	{
	TUint64 arg = (2u << 24) | (aSvc & 0xff) << 8 | (aCmd & 0xff);	// the asked function's id, owner SIP
	TUint64 res[3];
	TInt r = Scm_Call(6, 1, 1, &arg, 1, res);				// QCOM_SCM_SVC_INFO, IS_CALL_AVAIL, 1 value arg
	return r == KErrNone ? (TInt)res[0] : r;
	}

TInt Scm_AssignMem(TUint32 aPhys, TUint32 aSize, TUint64& aSrc, const SScmVmPerm* aDst, TInt aDstCount)
	{
	if (aDstCount < 1 || aDstCount > 8)
		return KErrArgument;
	TInt r = PageInit();
	if (r != KErrNone)
		return r;
	volatile TUint32* src = (volatile TUint32*)(Page + 0x100);	// the current owners, one u32 VMID each
	TInt nsrc = 0;
	for (TInt b = 0; b < 64; ++b)
		if (aSrc >> b & 1)
			src[nsrc++] = b;
	volatile TUint64* map = (volatile TUint64*)(Page + 0x200);	// {address, size}
	map[0] = aPhys;
	map[1] = aSize;
	volatile TUint32* dst = (volatile TUint32*)(Page + 0x300);	// {vmid, perm, ctx (u64), ctx_size, unused}
	TUint64 next = 0;
	for (TInt i = 0; i < aDstCount; ++i)
		{
		dst[i * 6 + 0] = aDst[i].iVmid;
		dst[i * 6 + 1] = aDst[i].iPerm;
		dst[i * 6 + 2] = dst[i * 6 + 3] = dst[i * 6 + 4] = dst[i * 6 + 5] = 0;
		next |= (TUint64)1 << aDst[i].iVmid;
		}
	// QCOM_SCM_SVC_MP / MP_ASSIGN: args RO, VAL, RO, VAL, RO, VAL, VAL (Linux __qcom_scm_assign_mem)
	const TUint32 info = 7 | 1 << 4 | 0 << 6 | 1 << 8 | 0 << 10 | 1 << 12 | 0 << 14 | 0 << 16;
	TUint64 args[7] = { PagePhys + 0x200, 16, PagePhys + 0x100, (TUint64)nsrc * 4, PagePhys + 0x300,
		(TUint64)aDstCount * 24, 0 };
	TUint64 res[3];
	r = Scm_Call(0x0c, 0x16, info, args, 7, res);
	if (r == KErrNone && res[0])
		r = KErrGeneral;									// TZ's own result (x1)
	if (r == KErrNone)
		aSrc = next;
	return r;
	}
