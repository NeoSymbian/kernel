// SPDX-License-Identifier: Apache-2.0
//
// qcom_telmon.cpp -- the debug monitor's telephony commands (docs/telephony.md C2/C3), so each piece of the modem
// path can be tried on the phone before the user side exists:
//   scm [pas <id>]        which TrustZone calls exist (assign_mem, PAS); PAS support for a peripheral
//   smem                  SMEM's version, partitions and their items
//   fw ls [dir]           the modem_a files (default IMAGE)
//   fw sha <path>         SHA-256 of a modem_a file, read by the kernel's own FAT16 reader
//   smp2p [start]         the modem pair's SMP2P items (start: create ours)
//   qmp [modem on|off]    open the AOP link; send the modem's load_state
//   glink [hello]         the GLINK edge to the modem: FIFO pointers, channels; hello: a QRTR HELLO from node 1
//   mss [boot]            boot the modem (on its own thread; progress in the log); the boot's step and RMB registers
#include <kernel/kern_priv.h>
#include <sha256.h>
#include "board.h"
#include "a64_dbgmon.h"
#include "qcom_scm.h"
#include "qcom_smem.h"
#include "qcom_fat16.h"
#include "qcom_smp2p.h"
#include "qcom_qmp.h"
#include "qcom_mss.h"
#include "qcom_glink.h"
#include "qcom_rpmh.h"

static TBool Eq(const TPtrC8& aWord, const char* aLit)
	{
	TInt n = 0;
	while (aLit[n])
		++n;
	return aWord.Length() == n && memcompare(aWord.Ptr(), n, (const TUint8*)aLit, n) == 0;
	}

static void Scm(TInt aArgc, const TPtrC8* aArg, const TUint64* aNum)
	{
	if (aArgc >= 3 && Eq(aArg[1], "pas"))					// QCOM_SCM_PIL_PAS_IS_SUPPORTED(peripheral)
		{
		TUint64 res[3] = { 0, 0, 0 };
		TInt r = Scm_Call(2, 7, 1, &aNum[2], 1, res);
		A64Mon_Say("scm pas %d supported -> %d, result %d", (TInt)aNum[2], r, (TInt)res[0]);
		return;
		}
	A64Mon_Say("scm: assign_mem %d, pas init %d, pas mem_setup %d, pas auth_reset %d, pas shutdown %d, pas supported %d",
		Scm_IsAvailable(0x0c, 0x16), Scm_IsAvailable(2, 1), Scm_IsAvailable(2, 2), Scm_IsAvailable(2, 5),
		Scm_IsAvailable(2, 6), Scm_IsAvailable(2, 7));
	}

static void Fw(TInt aArgc, const TPtrC8* aArg)
	{
	TInt r = Fat_Mount("modem_a");
	if (r != KErrNone)
		{
		A64Mon_Say("fw: mount modem_a -> %d", r);
		return;
		}
	TBuf8<64> path;
	path.Copy(aArgc >= 3 ? aArg[2] : TPtrC8((const TUint8*)"IMAGE", 5));
	path.Append(0);										// a C string (the kernel's descriptors have no PtrZ)
	const char* p = (const char*)path.Ptr();
	if (aArgc >= 2 && Eq(aArg[1], "sha") && aArgc >= 3)
		{
		SFatFile f;
		if ((r = Fat_Find(p, f)) != KErrNone)
			{
			A64Mon_Say("fw: %s -> %d", p, r);
			return;
			}
		NKern::ThreadEnterCS();
		TUint8* buf = (TUint8*)Kern::Alloc(0x10000);
		SHA256* sha = new SHA256;
		NKern::ThreadLeaveCS();
		if (!buf || !sha)
			r = KErrNoMemory;
		TUint32 t0 = NKern::TickCount();
		for (TUint32 pos = 0; r == KErrNone && pos < f.iSize; pos += 0x10000)
			{
			TUint32 n = Min(f.iSize - pos, (TUint32)0x10000);
			if ((r = Fat_Read(f, pos, buf, n)) == KErrNone)
				sha->Update(buf, n);
			}
		TBuf8<80> hex;
		if (r == KErrNone)
			{
			const TUint8* h = sha->Final().Ptr();				// Ptr(): the hash descriptor's length stays 0
			for (TInt i = 0; i < KSHA256HashSize; ++i)
				{
				hex.Append("0123456789abcdef"[h[i] >> 4]);
				hex.Append("0123456789abcdef"[h[i] & 15]);
				}
			}
		NKern::ThreadEnterCS();
		delete sha;
		Kern::Free(buf);
		NKern::ThreadLeaveCS();
		A64Mon_Say("fw sha %s: %d bytes, %S (%d, %d ms)", p, f.iSize, &hex, r,
			(TInt)(NKern::TickCount() - t0) * NKern::TickPeriod() / 1000);
		return;
		}
	TBuf8<13> name;
	TUint32 size, total = 0;
	TInt i = 0;
	for (; (r = Fat_DirEntry(p, i, name, size)) == KErrNone; ++i)
		{
		A64Mon_Say("%S %u", &name, size);
		total += size;
		}
	A64Mon_Say("fw ls %s: %d entries, %u bytes (%d)", p, i, total, r == KErrEof ? KErrNone : r);
	}

void A64Tel_Monitor(TInt aArgc, const TPtrC8* aArg, const TUint64* aNum)
	{
	if (Eq(aArg[0], "scm"))
		Scm(aArgc, aArg, aNum);
	else if (Eq(aArg[0], "smem"))
		Smem_Dump(A64Mon_Say);
	else if (Eq(aArg[0], "fw"))
		Fw(aArgc, aArg);
	else if (Eq(aArg[0], "smp2p"))
		{
		if (aArgc >= 2 && Eq(aArg[1], "start"))
			A64Mon_Say("smp2p start -> %d", Smp2p_Start(KSmemHostModem));
		Smp2p_Poll(KSmemHostModem);
		Smp2p_Dump(KSmemHostModem, A64Mon_Say);
		}
	else if (Eq(aArg[0], "glink"))
		{
		if (aArgc >= 2 && Eq(aArg[1], "hello"))			// QRTR v1 control packet: header + {cmd, 16 bytes}
			{
			const TUint32 KCtrlPort = 0xfffffffe, KBcast = 0xffffffff, KHello = 2;
			TUint32 pkt[13] = { 1, KHello, 1, KCtrlPort, 0, 20, KBcast, KCtrlPort, KHello, 0, 0, 0, 0 };
			A64Mon_Say("glink hello -> %d", Glink_Send(1, pkt, sizeof(pkt)));	// channel 1 = IPCRTR
			}
		Glink_Dump(A64Mon_Say);
		}
	else if (Eq(aArg[0], "mss"))
		{
		if (aArgc >= 2 && Eq(aArg[1], "boot"))
			A64Mon_Say("mss boot -> %d (see log)", Mss_StartBoot());
		TInt step, err;
		TUint32 rmb[4];
		Mss_State(step, err, rmb);
		TUint32 sk = 0;
		TInt r = Smp2p_In(KSmemHostModem, "slave-kernel", sk);
		A64Mon_Say("mss: step %d error %d; RMB pbl %d status %d mss %x code %x; slave-kernel %08x (%d)", step, err,
			rmb[0], (TInt)rmb[1], rmb[2], rmb[3], sk, r);
		TUint32 len = 0;
		const TUint8* why = (const TUint8*)Smem_Get(KSmemHostGlobal, 421, &len);	// SMEM_SSR_REASON_MSS0
		if (!why)
			why = (const TUint8*)Smem_Get(KSmemHostModem, 421, &len);
		if (sk & KMssFatal && why && len)
			{
			TInt n = 0;
			while (n < (TInt)len && n < 200 && why[n])
				++n;
			TPtrC8 s(why, n);
			A64Mon_Say("mss crash reason: %S", &s);
			}
		}
	else if (Eq(aArg[0], "qmp"))
		{
		if (aArgc >= 3 && Eq(aArg[1], "modem"))
			{
			TBool on = Eq(aArg[2], "on");
			A64Mon_Say("qmp modem %s -> %d", on ? "on" : "off", Qmp_Send(on
				? "{class: image, res: load_state, name: modem, val: on}"
				: "{class: image, res: load_state, name: modem, val: off}"));
			}
		else
			A64Mon_Say("qmp open -> %d", Qmp_Open());
		}
	}
