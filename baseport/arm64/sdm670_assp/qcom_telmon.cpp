// SPDX-License-Identifier: Apache-2.0
//
// qcom_telmon.cpp -- the debug monitor's telephony commands (docs/telephony.md C2/C3), so each piece of the modem
// path can be tried on the phone before the user side exists:
//   scm [pas <id>]        which TrustZone calls exist (assign_mem, PAS); PAS support for a peripheral
//   smem [item]           SMEM's version, partitions and their items; an item as text (421 = the modem's crash reason)
//   fw ls [dir]           the modem_a files (default IMAGE)
//   fw sha <path>         SHA-256 of a modem_a file, read by the kernel's own FAT16 reader
//   smp2p [start]         the modem pair's SMP2P items (start: create ours)
//   qmp [modem on|off]    open the AOP link; send the modem's load_state
//   glink [hello]         the GLINK edges (modem, lpass): FIFO pointers, channels; hello: a QRTR HELLO from node 1
//   adsp                  the ADSP (C6): boot step, error, SMP2P bits (MODEMD loads it through HAL AdspLoad)
//   audio [probe|start [speaker]|stop|vol <0-10>|mute|unmute]   the board's call-audio codecs (a64_audio.cpp)
//   mss [boot]            boot the modem (on its own thread; progress in the log); the boot's step and RMB registers
//   ipa [clock|start]     GSI_STATUS once powered; clock: the RPMh IPA clock only; start: clock + the GSI firmware
//                         MODEMD handed over (HAL 0x4e13) through PAS 15
//   smmu [probe]          the apps SMMU (MMU-500 at 0x15000000), read only: global config, and the stream matches for
//                         IPA's stream ids 0x720/0x722 with their context banks (telephony C5b I0); probe: can we
//                         write it under the hypervisor? a pattern into the last SMR with VALID clear (no effect on
//                         matching), read back, then cleared again
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
#include "qcom_adsp.h"
#include "qcom_telhal.h"
#include "qcom_rpmh.h"
#include "qcom_ipa.h"

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
		A64Mon_Say("scm pas %d supported -> %d (tz %d), result %d", (TInt)aNum[2], r, r ? (TInt)Scm_LastError : 0,
			(TInt)res[0]);
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

// ---- smmu: read only (Arm SMMUv2 / MMU-500 register layout: GR0 at 0, GR1 one page up, context banks in the
// upper half). Which of IPA's streams match, and whether unmatched streams fault (sCR0.USFCFG).
static void Smmu(TBool aProbe)
	{
	const TUint32 KSmmu = 0x15000000;
	volatile TUint32* g = (volatile TUint32*)A64_Io(KSmmu);
	TUint32 cr0 = g[0], idr0 = g[0x20 / 4], idr1 = g[0x24 / 4], gfsr = g[0x48 / 4];
	TUint32 page = (idr1 >> 31) ? 0x10000 : 0x1000, numpage = 1u << (((idr1 >> 28) & 7) + 1);
	TInt smrs = idr0 & 0xff, cbs = idr1 & 0xff;
	A64Mon_Say("smmu: cr0 %08x (clientpd %d usfcfg %d) idr0 %08x idr1 %08x gfsr %08x; %d SMRs, %d CBs, page %x x %d",
		cr0, cr0 & 1, (cr0 >> 10) & 1, idr0, idr1, gfsr, smrs, cbs, page, numpage);
	TInt valid = 0;
	for (TInt n = 0; n < smrs && n < 128; ++n)
		{
		TUint32 smr = g[(0x800 + 4 * n) / 4];
		if (!(smr >> 31))
			continue;
		++valid;
		TUint32 id = smr & 0x7fff, mask = (smr >> 16) & 0x7fff, s2cr = g[(0xc00 + 4 * n) / 4];
		TBool ipa = !((id ^ 0x720) & ~mask & 0x7fff) || !((id ^ 0x722) & ~mask & 0x7fff);
		if (!ipa)
			continue;
		TInt cb = s2cr & 0xff;
		volatile TUint32* c = (volatile TUint32*)A64_Io(KSmmu + numpage * page + cb * page);
		A64Mon_Say("smmu: SMR %d id %x mask %x -> S2CR %08x (type %d cb %d): CBAR %08x, CB sctlr %08x ttbr0 %08x%08x tcr %08x",
			n, id, mask, s2cr, (s2cr >> 16) & 3, cb, ((volatile TUint32*)A64_Io(KSmmu + page))[cb], c[0], c[0x24 / 4],
			c[0x20 / 4], c[0x30 / 4]);
		}
	A64Mon_Say("smmu: %d valid SMRs; none above means IPA's streams are unmatched", valid);
	if (!aProbe)
		return;
	TInt last = smrs - 1;
	volatile TUint32& smr = g[(0x800 + 4 * last) / 4];
	TUint32 before = smr;
	if (before >> 31)
		{
		A64Mon_Say("smmu probe: SMR %d is in use (%08x): not touched", last, before);
		return;
		}
	smr = 0x07ff0123;										// VALID (bit 31) clear: matches nothing
	__e32_memory_barrier();
	TUint32 seen = smr;
	smr = before;
	__e32_memory_barrier();
	A64Mon_Say("smmu probe: SMR %d %08x -> wrote 07ff0123, read %08x, restored %08x: %s", last, before, seen, smr,
		seen == 0x07ff0123 ? "writable" : "NOT writable");
	}

__attribute__((weak)) void A64Audio_Report(void (*aSay)(const char*, ...)) { aSay("audio: no codecs on this board"); }

void A64Tel_Monitor(TInt aArgc, const TPtrC8* aArg, const TUint64* aNum)
	{
	if (Eq(aArg[0], "adsp"))
		Adsp_Report(A64Mon_Say);
	else if (Eq(aArg[0], "audio"))
		{
		TInt r = KErrNone;
		if (aArgc >= 2 && Eq(aArg[1], "probe"))
			r = A64Audio_Control(0, 0);
		else if (aArgc >= 2 && Eq(aArg[1], "start"))
			r = A64Audio_Control(1, aArgc >= 3 && Eq(aArg[2], "speaker"));
		else if (aArgc >= 2 && Eq(aArg[1], "stop"))
			r = A64Audio_Control(2, 0);
		else if (aArgc >= 3 && Eq(aArg[1], "vol"))
			r = A64Audio_Control(4, (TInt)aNum[2]);
		else if (aArgc >= 2 && (Eq(aArg[1], "mute") || Eq(aArg[1], "unmute")))
			r = A64Audio_Control(5, Eq(aArg[1], "mute"));
		if (aArgc >= 2)
			A64Mon_Say("audio %S -> %d", &aArg[1], r);
		A64Audio_Report(A64Mon_Say);
		}
	else if (Eq(aArg[0], "ipa"))
		{
		if (aArgc >= 2 && Eq(aArg[1], "clock"))
			{
			A64Mon_Say("ipa clock -> %d", Ipa_PowerOn());
			return;										// the registers stay unread: "ipa" alone reads them
			}
		else if (aArgc >= 3 && Eq(aArg[1], "reg"))
			{
			TUint32 v = 0;
			TInt r = Ipa_Reg((TUint32)aNum[2], v);
			A64Mon_Say("ipa reg %x -> %d: %08x", (TUint32)aNum[2], r, v);
			return;
			}
		else if (aArgc >= 2 && Eq(aArg[1], "start"))
			A64Mon_Say("ipa start -> %d", Ipa_Start());
		TUint32 gsi = 0;
		TInt r = Ipa_Status(gsi);
		A64Mon_Say("ipa: %d, GSI_STATUS %08x (bit 0 enabled)", r, gsi);
		Ipa_Report(A64Mon_Say);
		return;
		}
	if (Eq(aArg[0], "smmu"))
		{
		Smmu(aArgc >= 2 && Eq(aArg[1], "probe"));
		return;
		}
	if (Eq(aArg[0], "scm"))
		Scm(aArgc, aArg, aNum);
	else if (Eq(aArg[0], "smem") && aArgc >= 2)
		{
		TUint32 len = 0;										// an item as text: 421 = the modem's crash reason
		const TUint8* p = (const TUint8*)Smem_Get(KSmemHostGlobal, (TUint)aNum[1], &len);
		TBuf8<200> t;
		for (TUint32 i = 0; p && i < len && t.Length() < t.MaxLength() && p[i]; ++i)
			t.Append(p[i] >= 0x20 && p[i] < 0x7f ? p[i] : '.');
		A64Mon_Say("smem %d: %d bytes: %S", (TInt)aNum[1], p ? (TInt)len : -1, &t);
		}
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
		Tel_Dump(A64Mon_Say);
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
