// SPDX-License-Identifier: Apache-2.0
//
// a64_dbgmon.cpp -- the kernel's debug monitor and log ring (board-agnostic). Every kernel debug character goes into a
// 64 KB ring (A64Log_Putc, from DebugOutputChar), so a transport that connects late can still replay the whole boot
// log. Commands come as text lines from a transport -- the PL011 on QEMU (Ctrl-B, then a line), USB CDC-ACM on the
// Pixel -- and answer through that transport's blocking writer:
//   help | log | lastlog | drivers (the device registry, a64_device.h) | peek <phys> [n] | poke <phys> <val> | spmi r <sid> <addr> | spmi w <sid> <addr> <val>
//   rpmh <bcm> (vote a bus clock manager, Pixel) | scm | smem | fw ls|sha | smp2p | qmp | mss | glink (telephony, Pixel: qcom_telmon.cpp)
//   blk info | blk gpt | blk read <lba> [n] | blk dump <lba> <n> | blk put <lba> <n> (vendor_a/_b, EMMC_C=2) | slot | slot ok (dry run) | slot ok now | slot boot (dry run) | touch | keys | threads | cpus | cpufreq | tap <x> <y> | swipe <x1> <y1> <x2> <y2> [moves [ms]]
//   key <scancode> | type <text> | shot | reboot | poweroff | fastboot | rtc | date <unix UTC s> [UTC offset s]
// "shot" sends "SHOT <w> <h>\n" and then w*h*4 bytes of BGRX pixels; "blk dump" sends "DUMP <n>\n", n*512 raw bytes,
// then a text line with their SHA-256; every other answer is text, ending "\n.\n".
// Debug builds only, on a development phone: peek/poke/spmi w reach any register.
#include <kernel/kern_priv.h>
#include <mmboot.h>
#include <e32keys.h>
#include <sha256.h>
#include "board.h"
#include "a64_blk.h"
#include "a64_dbgmon.h"
#include "a64_device.h"
#include "a64_platform.h"

// ---- the log ring
// A board with RAM that survives a reset (A64_LastLogPhys: an IO region, Normal NC, 0x20000 bytes) keeps the ring
// there: a header {magic, head} at +0 and the ring at +0x10000. At boot the previous boot's ring is copied out for
// "lastlog", so a crash that halts or resets the SoC still leaves its log readable on the next boot.
const TUint32 KLogSize = 64 * 1024, KLogMagic = 0x4e4c4f47;	// 'NLOG'
static char BssRing[KLogSize];
static char* LogRing = BssRing;
static volatile TUint32* LogSaved;							// the persistent header's head, once switched
static TBool LogInited;
static volatile TUint32 LogHead;							// characters ever written (the ring keeps the last 64 KB)
static char LastLog[KLogSize];
static TUint32 LastLen;
extern "C" __attribute__((weak)) unsigned long A64_LastLogPhys() { return 0; }

static void LogInit()										// first character with the IO window up (boot CPU only)
	{
	LogInited = ETrue;
	volatile TUint32* h = A64_LastLogPhys() ? (volatile TUint32*)A64_Io(A64_LastLogPhys()) : NULL;
	if (!h)
		return;
	char* ring = (char*)h + 0x10000;
	if (h[0] == KLogMagic)
		{
		TUint32 head = h[1];
		LastLen = head < KLogSize ? head : KLogSize;
		for (TUint32 i = 0; i < LastLen; ++i)
			LastLog[i] = ring[(head - LastLen + i) % KLogSize];
		}
	for (TUint32 i = 0; i < LogHead && i < KLogSize; ++i)	// what this boot logged before the switch
		ring[i] = BssRing[i];
	h[1] = LogHead;
	h[0] = KLogMagic;
	LogRing = ring;
	LogSaved = h + 1;
	}

void A64Log_Putc(char aChar)
	{
	if (!LogInited && A64_IoWindow)
		LogInit();
	TUint32 i = __e32_atomic_add_ord32(&LogHead, 1);
	LogRing[i % KLogSize] = aChar;
	if (LogSaved)
		*LogSaved = i + 1;		// shortcut: CPUs racing here may store heads out of order; lastlog may lose a few chars
	}

void A64Log_Report()										// once at boot (the IO window is up by then)
	{
	if (!LogInited && A64_IoWindow)
		LogInit();
	if (LastLen)
		Kern::Printf("[log] previous boot: %d bytes kept (lastlog)", LastLen);
	}

TInt A64Log_LastRead(TUint32& aPos, TUint8* aBuf, TInt aMax)	// the previous boot's log, from a cursor
	{
	TInt n = 0;
	while (n < aMax && aPos < LastLen)
		aBuf[n++] = LastLog[aPos++];
	return n;
	}

TInt A64Log_Read(TUint32& aPos, TUint8* aBuf, TInt aMax)
	{
	TUint32 head = LogHead;
	if (head - aPos > KLogSize)								// the reader fell behind: skip to the oldest kept byte
		aPos = head - KLogSize;
	TInt n = 0;
	while (n < aMax && aPos != head)
		aBuf[n++] = LogRing[aPos++ % KLogSize];
	return n;
	}

// ---- keys: ASCII -> EPOC standard scan codes (EKDATA translates them, as a real keyboard's)
// ASCII -> (standard scan code, shift); 0 = no key
static TInt ScanCode(TUint8 aChar, TBool& aShift)
	{
	aShift = EFalse;
	if (aChar >= 'a' && aChar <= 'z')
		return aChar - 'a' + 'A';
	if (aChar >= 'A' && aChar <= 'Z')
		{
		aShift = ETrue;
		return aChar;
		}
	if (aChar >= '0' && aChar <= '9')
		return aChar;
	switch (aChar)
		{
	case '\r': case '\n':	return EStdKeyEnter;
	case 0x08: case 0x7f:	return EStdKeyBackspace;
	case 0x1b:				return EStdKeyEscape;
	case ' ':				return EStdKeySpace;
	case '.':				return EStdKeyFullStop;
	case ',':				return EStdKeyComma;
	case '/':				return EStdKeyForwardSlash;
	case '\\':				return EStdKeyBackSlash;
	case ';':				return EStdKeySemiColon;
	case '-':				return EStdKeyMinus;
	case '=':				return EStdKeyEquals;
	case ':':	aShift = ETrue;	return EStdKeySemiColon;
	case '_':	aShift = ETrue;	return EStdKeyMinus;
	case '?':	aShift = ETrue;	return EStdKeyForwardSlash;
	case '*':	aShift = ETrue;	return '8';
	default:				return 0;
		}
	}

static void PostKey(TRawEvent::TType aType, TInt aScanCode)
	{
	TRawEvent e;
	e.Set(aType, aScanCode);
	while (Kern::AddEvent(e) == KErrOverflow)			// the window server will drain the queue
		NKern::Sleep(1);
	}

// one character as a key press, paced like typing (a64_serialkbd.cpp on QEMU, the monitor's "type")
void A64Key_Type(TUint8 aChar)
	{
	TBool shift;
	TInt code = ScanCode(aChar, shift);
	if (!code)
		return;
	if (shift)
		PostKey(TRawEvent::EKeyDown, EStdKeyLeftShift);
	PostKey(TRawEvent::EKeyDown, code);
	NKern::Sleep(10);								// a key is held a little, and keys come apart, as typed:
	PostKey(TRawEvent::EKeyUp, code);				// a burst of a whole pasted line overruns the window
	if (shift)										// server's queue for the console (eshell then stalls)
		PostKey(TRawEvent::EKeyUp, EStdKeyLeftShift);
	NKern::Sleep(20);
	}

// ---- output to the current transport
static TMonWrite Out;

static void Write(const TAny* aData, TInt aLen)
	{
	if (Out)
		Out((const TUint8*)aData, aLen);
	}

static void Fmt(TDes8& aBuf, const char* aFmt, ...)		// append, Kern::Printf's format (no field widths)
	{
	VA_LIST list;
	VA_START(list, aFmt);
	Kern::AppendFormat(aBuf, aFmt, list);
	VA_END(list);
	}

static TBool Eq(const TDesC8& aWord, const char* aLit)
	{
	TInt n = 0;
	while (aLit[n])
		++n;
	return aWord.Length() == n && memcompare(aWord.Ptr(), n, (const TUint8*)aLit, n) == 0;
	}

static void Say(const char* aFmt, ...)
	{
	VA_LIST list;
	VA_START(list, aFmt);
	TBuf8<256> b;
	Kern::AppendFormat(b, aFmt, list);
	VA_END(list);
	b.Append('\n');
	Write(b.Ptr(), b.Length());
	}

void A64Mon_Say(const char* aFmt, ...)						// Say for board files' commands (a64_dbgmon.h)
	{
	VA_LIST list;
	VA_START(list, aFmt);
	TBuf8<256> b;
	Kern::AppendFormat(b, aFmt, list);
	VA_END(list);
	b.Append('\n');
	Write(b.Ptr(), b.Length());
	}

// ---- hooks other board files provide (weak: absent on some boards)
__attribute__((weak)) void A64TouchDump() {}
__attribute__((weak)) void A64BlkDump() {}
__attribute__((weak)) void A64KeysDump() {}
__attribute__((weak)) void A64_RebootToFastboot() { Say("fastboot: not on this board"); }
// QEMU virt: PSCI resets or stops the guest (a64_reboot.cpp on the Pixel, with the PMIC's reboot reason)
__attribute__((weak)) void A64_RebootNormal() { A64_Psci(0x84000009, 0, 0, 0); }
__attribute__((weak)) TInt A64_PowerOff() { A64_Psci(0x84000008, 0, 0, 0); return KErrGeneral; }
__attribute__((weak)) TInt A64_RtcSeconds(TUint32&) { return KErrNotSupported; }	// qcom_rtc.cpp
__attribute__((weak)) TLinAddr A64_FbLinear() { return 0; }
__attribute__((weak)) void A64_WatchdogBite(TInt) { Say("wdt: not on this board"); }
__attribute__((weak)) TInt A64BlkDev_Probe() { return KErrNotSupported; }
__attribute__((weak)) TInt A64BlkDev_SelfTest() { return KErrNotSupported; }
__attribute__((weak)) TInt A64BlkDev_Monitor(TInt, const TPtrC8*, const TUint64*, TDes8& aOut)
	{
	aOut.Copy((const TUint8*)"blk: not on this board", 22);
	return KErrNotSupported;
	}
__attribute__((weak)) void A64UsbStatus(TDes8& aBuf) { aBuf.Append((const TUint8*)"no USB console", 14); }
__attribute__((weak)) void A64UsbCold(TDes8& aOut, TInt) { aOut.Copy((const TUint8*)"usb cold: not on this board", 27); }
__attribute__((weak)) void A64RpmhVote(const TDesC8&, TDes8& aOut) { aOut.Copy((const TUint8*)"rpmh: not on this board", 23); }
// qcom_telmon.cpp on the Pixel: the modem path (docs/telephony.md C2/C3)
__attribute__((weak)) void A64Tel_Monitor(TInt, const TPtrC8*, const TUint64*) { Say("telephony: not on this board"); }
__attribute__((weak)) int Spmi_Read(unsigned, unsigned) { return KErrNotSupported; }
__attribute__((weak)) int Spmi_Write(unsigned, unsigned, unsigned) { return KErrNotSupported; }
void A64DumpThreads();										// below, needs scheduler internals
void A64CpufreqDump();										// qcom_cpufreq.cpp (a weak default in a64_variant.cpp)

// ---- physical access: the board's IO regions, else pages mapped on first use
static TLinAddr Map(TUint32 aPhys)
	{
	TLinAddr io = A64_Io(aPhys);
	if (io)
		return io;
	static struct { TUint32 iPage; DPlatChunkHw* iChunk; } cache[8];
	static TInt next;
	TUint32 page = aPhys & ~KPageMask;
	for (TUint i = 0; i < sizeof(cache) / sizeof(cache[0]); ++i)
		if (cache[i].iChunk && cache[i].iPage == page)
			return cache[i].iChunk->LinearAddress() + (aPhys & KPageMask);
	DPlatChunkHw* chunk = NULL;
	if (DPlatChunkHw::New(chunk, page, KPageSize, EMapAttrSupRw | EMapAttrFullyBlocking) != KErrNone)
		return 0;
	TInt slot = next++ % (TInt)(sizeof(cache) / sizeof(cache[0]));
	if (cache[slot].iChunk)
		cache[slot].iChunk->Close(NULL);
	cache[slot].iPage = page;
	cache[slot].iChunk = chunk;
	return chunk->LinearAddress() + (aPhys & KPageMask);
	}

// ---- parsing
static TUint64 Num(const TDesC8& aWord)					// 0x... hex, else decimal; stops at the first bad digit
	{
	TInt i = 0, base = 10;
	if (aWord.Length() > 2 && aWord[0] == '0' && (aWord[1] | 0x20) == 'x')
		{
		i = 2;
		base = 16;
		}
	TUint64 v = 0;
	for (; i < aWord.Length(); ++i)
		{
		TUint c = aWord[i] | 0x20, d = c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : 99;
		if (d >= (TUint)base)
			break;
		v = v * base + d;
		}
	return v;
	}

const TInt KMaxArgs = 7;							// swipe x1 y1 x2 y2 moves ms

static void BlkPut(TUint64 aLba, TUint32 aCount);

static void Blk(TInt aArgc, const TPtrC8* aArg, const TUint64* aNum)
	{
	static TUint8 s[KA64BlkSector];
	if (aArgc >= 2 && Eq(aArg[1], "init"))					// bring the device up now (Pixel: the eMMC)
		{
		TInt r = KErrNone, x;
		XTRAP(x, XT_DEFAULT, r = A64BlkDev_Probe());		// an aborting register shows as -38, not a crash
		Say("blk init -> %d (trap %d; see 'log' for the steps)", r, x);
		}
	else if (aArgc >= 2 && Eq(aArg[1], "wtest"))			// writable builds: write/compare at the window's end, then
		{													// clear its start and end for a fresh format
		TInt r = KErrNone, x;
		XTRAP(x, XT_DEFAULT, r = A64BlkDev_SelfTest());
		Say("blk wtest -> %d (trap %d; see 'log')", r, x);
		}
	else if (aArgc >= 2 && Eq(aArg[1], "info"))
		{
		A64BlkDump();
		Say("window: %d sectors", (TInt)A64Blk_Sectors());
		}
	else if (aArgc >= 3 && Eq(aArg[1], "read"))		// the raw device, read-only
		{
		TInt n = aArgc >= 4 ? (TInt)aNum[3] : 1;
		for (TInt k = 0; k < n && k < 16; ++k)
			{
			TInt r = A64BlkDev_Read(aNum[2] + k, 1, s);
			if (r != KErrNone)
				{
				Say("read %d -> %d", (TInt)(aNum[2] + k), r);
				return;
				}
			for (TInt i = 0; i < KA64BlkSector; i += 32)
				{
				TBuf8<120> b;
				Fmt(b, "%08x+%03x:", (TUint32)(aNum[2] + k), i);
				for (TInt j = 0; j < 32; ++j)
					Fmt(b, j % 4 ? "%02x" : " %02x", s[i + j]);
				Say("%S", &b);
				}
			}
		}
	else if (aArgc >= 4 && Eq(aArg[1], "dump"))			// raw sectors, read-only (telephony C0: radio partition backups)
		{
		static TUint8 d[128 * KA64BlkSector];
		SHA256* sha = new SHA256;							// heap: a static would need a C++ constructor run
		if (!sha)
			{
			Say("dump: no memory");
			return;
			}
		TUint32 n = (TUint32)aNum[3];
		TBuf8<40> h;
		Fmt(h, "DUMP %u\n", n);
		Write(h.Ptr(), h.Length());
		TInt err = KErrNone;
		for (TUint32 done = 0; done < n; )					// always n*512 bytes, so the host never waits forever
			{
			TInt k = (TInt)Min(n - done, (TUint32)128);
			TInt r = A64BlkDev_Read(aNum[2] + done, k, d);
			if (r != KErrNone)
				{
				memclr(d, k * KA64BlkSector);
				if (err == KErrNone)
					err = r;
				}
			sha->Update(d, k * KA64BlkSector);
			Write(d, k * KA64BlkSector);
			done += k;
			}
		const TUint8* hash = sha->Final().Ptr();			// Ptr(): the descriptor's length stays 0 (as a64_medverity.cpp)
		TBuf8<80> hex;
		for (TInt i = 0; i < KSHA256HashSize; ++i)
			Fmt(hex, "%02x", hash[i]);
		delete sha;
		Say("\ndump sha256 %S err %d", &hex, err);
		}
	else if (aArgc >= 4 && Eq(aArg[1], "put"))			// raw sectors from the host (A64Mon_PutChunk)
		BlkPut(aNum[2], (TUint32)aNum[3]);
	else if (aArgc >= 2 && Eq(aArg[1], "gpt"))			// every partition: name, first, last sector
		{
		TInt r = A64BlkDev_Read(1, 1, s);
		if (r != KErrNone || memcompare(s, 8, (const TUint8*)"EFI PART", 8) != 0)
			{
			Say("no GPT (%d)", r);
			return;
			}
		TUint64 lba = *(TUint64*)(s + 72);
		TUint32 n = *(TUint32*)(s + 80), size = *(TUint32*)(s + 84);
		Say("GPT: %d entries of %d bytes at sector %d", n, size, (TInt)lba);
		static TUint8 e[KA64BlkSector];
		for (TUint32 i = 0; i < n && size >= 128 && size <= KA64BlkSector; ++i)
			{
			TUint32 off = i * size;
			if (off % KA64BlkSector == 0 && A64BlkDev_Read(lba + off / KA64BlkSector, 1, e) != KErrNone)
				return;
			const TUint8* p = e + off % KA64BlkSector;
			if (!(*(const TUint64*)(p + 32)))
				continue;
			TBuf8<40> name;
			for (TInt c = 0; c < 36 && p[56 + 2 * c]; ++c)
				name.Append(p[56 + 2 * c]);
			TUint64 first = *(const TUint64*)(p + 32), last = *(const TUint64*)(p + 40);
			Say("%d %S %d..%d %d MB", i, &name, (TInt)first, (TInt)last, (TInt)((last - first + 1) >> 11));
			}
		}
	else if (aArgc >= 2 && (Eq(aArg[1], "speed") || Eq(aArg[1], "guard") || Eq(aArg[1], "mode")))	// the board's
		{
		TBuf8<200> b;
		TInt r = KErrNone, x;
		XTRAP(x, XT_DEFAULT, r = A64BlkDev_Monitor(aArgc, aArg, aNum, b));
		Say("%S (%d, trap %d)", &b, r, x);
		}
	else
		Say("blk init | blk info | blk gpt | blk read <lba> [n] | blk dump <lba> <n> | blk put <lba> <n> | blk wtest | blk speed | blk guard <lba> | blk mode <0-2>");
	}

// smooth-ui S3b: per CPU, the core type (MIDR part) and the real clock: PMU cycles over 5 ms of the generic timer,
// counted on that CPU (the monitor thread pins itself to each in turn). XTRAP in case the hypervisor traps the PMU.
static void Cpus()
	{
	NThread* me = NKern::CurrentThread();
	TUint64 hz;
	__asm__ __volatile__("mrs %0, cntfrq_el0" : "=r"(hz));
	for (TInt cpu = 0; cpu < NKern::NumberOfCpus(); ++cpu)
		{
		TUint32 old = NKern::ThreadSetCpuAffinity(me, cpu);
		TInt on = NKern::CurrentCpu();
		TUint64 midr, t0 = 0, t1 = 0, c0 = 0, c1 = 0;
		__asm__ __volatile__("mrs %0, midr_el1" : "=r"(midr));
		TInt r = KErrNone;
		XTRAP(r, XT_DEFAULT, {
			__asm__ __volatile__("msr pmcr_el0, %0; msr pmccfiltr_el0, xzr; msr pmcntenset_el0, %1; isb"
				:: "r"((TUint64)5), "r"((TUint64)1 << 31));		// E + reset the cycle counter; count EL0 and EL1
			__asm__ __volatile__("isb; mrs %0, cntvct_el0; mrs %1, pmccntr_el0" : "=r"(t0), "=r"(c0));
			do
				__asm__ __volatile__("isb; mrs %0, cntvct_el0" : "=r"(t1));
			while (t1 - t0 < hz / 200);
			__asm__ __volatile__("isb; mrs %0, pmccntr_el0" : "=r"(c1));
			});
		NKern::ThreadSetCpuAffinity(me, old);
		TUint part = (TUint)(midr >> 4) & 0xfff;
		const char* type = part == 0x803 || part == 0xd05 ? "silver (A55)" : part == 0x802 || part == 0xd0a ? "gold (A75)" : "?";	// Kryo 385 / Arm
		if (r != KErrNone)
			Say("cpu %d (ran on %d): MIDR %08x part %03x %s, PMU trapped (%d)", cpu, on, (TUint)midr, part, type, r);
		else
			Say("cpu %d (ran on %d): MIDR %08x part %03x %s, %d MHz", cpu, on, (TUint)midr, part, type,
				(TInt)((c1 - c0) * hz / (t1 - t0) / 1000000));
		}
	}

static void Post(TRawEvent::TType aType, TInt aX, TInt aY)
	{
	TRawEvent e;
	e.Set(aType, aX, aY);
	Kern::AddEvent(e);
	}

static void Shot()
	{
	TLinAddr fb = A64_FbLinear();
	if (!fb)
		{
		Say("shot: no framebuffer");
		return;
		}
	TBuf8<40> h;
	Fmt(h, "SHOT %d %d\n", A64_FbWidth, A64_FbHeight);
	Write(h.Ptr(), h.Length());
	for (TInt y = 0; y < A64_FbHeight; ++y)
		Write((const TAny*)(fb + y * A64_FbStride), A64_FbWidth * 4);
	}

// ---- "blk put <lba> <n>" (telephony C5b): n sectors from the host, written from lba on. Only into vendor_a or vendor_b
// (the firmware ext4), and only where the eMMC driver's allow-list lets a write through (an EMMC_C=2 build). The host
// waits for "PUT <n>", sends n*512 raw bytes, and gets "put sha256 <hex> err <e> written <bytes>": SHA-256 of what
// arrived; every 64 KB stage is written, read back and compared.
__attribute__((weak)) TInt A64Usb_RawIn(TInt) { return KErrNotSupported; }
static TUint64 PutLba;
static TInt PutErr, PutWritten;
static SHA256* PutSha;
static TUint8* PutCheck;

static void BlkPut(TUint64 aLba, TUint32 aCount)
	{
	TUint64 first = 0, count = 0;
	TBool inside = EFalse;
	const char* const KTargets[] = { "vendor_a", "vendor_b" };
	for (TInt i = 0; i < 2; ++i)
		if (A64BlkDev_Partition(KTargets[i], first, count) == KErrNone && aLba >= first && aLba + aCount <= first + count)
			inside = ETrue;
	if (!inside || !aCount || aCount > 0x400000)			// up to 2 GB, inside one vendor partition
		{
		Say("put: %u sectors at %u: only inside vendor_a or vendor_b", aCount, (TUint32)aLba);
		return;
		}
	if (!PutSha)
		{
		NKern::ThreadEnterCS();
		PutSha = new SHA256;
		PutCheck = (TUint8*)Kern::Alloc(64 * 1024);
		NKern::ThreadLeaveCS();
		}
	else
		PutSha->Reset();
	TInt r = PutSha && PutCheck ? A64Usb_RawIn(aCount * KA64BlkSector) : KErrNoMemory;
	if (r != KErrNone)
		{
		Say("put: %d", r);
		return;
		}
	PutLba = aLba;
	PutErr = KErrNone;
	PutWritten = 0;
	TBuf8<40> h;
	Fmt(h, "PUT %u\n", aCount);
	Write(h.Ptr(), h.Length());
	}

void A64Mon_PutChunk(const TUint8* aData, TInt aLen, TBool aLast, TMonWrite aWrite)
	{
	Out = aWrite;
	PutSha->Update(aData, aLen);
	TInt sectors = aLen / KA64BlkSector;
	if (PutErr == KErrNone)								// after a failure the rest only drains (the host never waits)
		{
		PutErr = A64BlkDev_Write(PutLba, sectors, aData);
		if (PutErr == KErrNone)
			PutErr = A64BlkDev_Read(PutLba, sectors, PutCheck);
		if (PutErr == KErrNone && memcompare(PutCheck, aLen, aData, aLen) != 0)
			PutErr = KErrCorrupt;
		if (PutErr == KErrNone)
			PutWritten += aLen;
		}
	PutLba += sectors;
	if (!aLast)
		return;
	const TUint8* hash = PutSha->Final().Ptr();
	TBuf8<80> hex;
	for (TInt i = 0; i < KSHA256HashSize; ++i)
		Fmt(hex, "%02x", hash[i]);
	Say("put sha256 %S err %d written %d", &hex, PutErr, PutWritten);
	}

void A64Mon_Line(const TDesC8& aLine, TMonWrite aWrite)
	{
	Out = aWrite;
	TPtrC8 arg[KMaxArgs];
	TUint64 num[KMaxArgs];
	TInt argc = 0;
	for (TInt i = 0; i < aLine.Length() && argc < KMaxArgs; )		// words separated by spaces
		{
		while (i < aLine.Length() && aLine[i] <= ' ')
			++i;
		TInt start = i;
		while (i < aLine.Length() && aLine[i] > ' ')
			++i;
		if (i > start)
			{
			arg[argc].Set(aLine.Mid(start, i - start));
			num[argc] = Num(arg[argc]);
			++argc;
			}
		}
	if (!argc)
		return;
	NKern::ThreadEnterCS();
	const TPtrC8& c = arg[0];
	if (Eq(c, "help"))
		Say("help log lastlog console drivers peek poke spmi rpmh scm smem fw smp2p qmp mss glink blk slot touch keys threads cpus cpufreq usb tap swipe key type shot reboot poweroff fastboot wdt rtc date");
	else if (Eq(c, "console"))							// dev mode: the kernel log on screen (Pixel)
		{
		if (argc >= 2)
			A64_ConsoleShow(Eq(arg[1], "on"));
		Say("console %s", A64_ConsoleShown() ? "on" : "off");
		}
	else if (Eq(c, "lastlog"))
		{
		if (LastLen)
			Write(LastLog, LastLen);
		else
			Say("no log kept from a previous boot");
		}
	else if (Eq(c, "log"))
		{
		TUint32 pos = 0;
		TUint8 b[256];
		for (TInt n; (n = A64Log_Read(pos, b, sizeof(b))) > 0; )
			Write(b, n);
		}
	else if (Eq(c, "peek") && argc >= 2)
		{
		TInt n = argc >= 3 ? (TInt)num[2] : 1;
		for (TInt i = 0; i < n && i < 256; ++i)
			{
			TUint32 a = (TUint32)num[1] + 4 * i;
			TLinAddr l = Map(a);
			TUint32 v = 0;
			TInt x = KErrNone;
			if (l)
				XTRAP(x, XT_DEFAULT, v = *(volatile TUint32*)l);	// a register can answer with an abort
			if (l && x)
				Say("%08x: abort (%d)", a, x);
			else if (l)
				Say("%08x: %08x", a, v);
			else
				Say("%08x: not mapped", a);
			}
		}
	else if (Eq(c, "poke") && argc >= 3)
		{
		TLinAddr l = Map((TUint32)num[1]);
		TUint32 v = 0;
		TInt x = KErrNone;
		if (l)
			XTRAP(x, XT_DEFAULT, *(volatile TUint32*)l = (TUint32)num[2]; v = *(volatile TUint32*)l);
		if (l)
			Say("%08x <- %08x, reads %08x (trap %d)", (TUint32)num[1], (TUint32)num[2], v, x);
		}
	else if (Eq(c, "spmi") && argc >= 4 && Eq(arg[1], "r"))
		Say("spmi %d:%04x = %d (0x%02x)", (TInt)num[2], (TInt)num[3], Spmi_Read((TUint)num[2], (TUint)num[3]),
			Spmi_Read((TUint)num[2], (TUint)num[3]) & 0xff);
	else if (Eq(c, "spmi") && argc >= 5 && Eq(arg[1], "w"))
		Say("spmi %d:%04x <- %02x -> %d", (TInt)num[2], (TInt)num[3], (TInt)num[4],
			Spmi_Write((TUint)num[2], (TUint)num[3], (TUint)num[4]));
	else if (Eq(c, "scm") || Eq(c, "smem") || Eq(c, "fw") || Eq(c, "smp2p") || Eq(c, "qmp") || Eq(c, "mss") || Eq(c, "glink") || Eq(c, "smmu") || Eq(c, "ipa") || Eq(c, "adsp") || Eq(c, "audio"))	// telephony (qcom_telmon.cpp)
		{
		TInt x;
		XTRAP(x, XT_DEFAULT, A64Tel_Monitor(argc, arg, num));
		if (x)
			Say("%S: trap %d", &c, x);
		}
	else if (Eq(c, "rpmh") && argc >= 2)
		{
		TBuf8<120> b;
		A64RpmhVote(arg[1], b);
		Say("%S", &b);
		}
	else if (Eq(c, "blk"))
		Blk(argc, arg, num);
	else if (Eq(c, "slot"))								// FS5b: slot | slot ok dry | slot ok
		{
		TBuf8<800> b;									// past Say's 256 bytes: written as it is
		if (argc >= 2 && Eq(arg[1], "ok"))
			A64Slot_MarkOk(!(argc >= 3 && Eq(arg[2], "now")), b);	// "slot ok" is the dry run; "slot ok now" writes
		else if (argc >= 2 && Eq(arg[1], "boot"))
			A64Slot_BootDry(b);							// what an install image's boot-time try count would write
		else
			A64Slot_Show(b);
		b.Append('\n');
		Write(b.Ptr(), b.Length());
		}
	else if (Eq(c, "drivers"))							// the device registry (a64_devices.cpp), as HAL 0x4e19 lists it
		{
		TBuf8<200> b;
		for (TInt i = 0; A64Device_Line(i, b) == KErrNone; ++i)
			Say("%S", &b);
		Say("%d devices", A64Device_Count());
		}
	else if (Eq(c, "touch"))
		A64TouchDump();
	else if (Eq(c, "keys"))
		A64KeysDump();
	else if (Eq(c, "threads"))
		A64DumpThreads();
	else if (Eq(c, "cpus"))
		Cpus();
	else if (Eq(c, "cpufreq"))
		A64CpufreqDump();
	else if (Eq(c, "tap") && argc >= 3)
		{
		Post(TRawEvent::EButton1Down, (TInt)num[1], (TInt)num[2]);
		NKern::Sleep(NKern::TimerTicks(80));
		Post(TRawEvent::EButton1Up, (TInt)num[1], (TInt)num[2]);
		}
	else if (Eq(c, "swipe") && argc >= 5)
		{
		// moves default 10, `ms` apart (default 16): "swipe 900 900 180 900 600" is a 10 s drag (smooth-ui S1)
		TInt n = argc >= 6 ? (TInt)num[5] : 10, ms = argc >= 7 ? (TInt)num[6] : 16;
		n = n < 1 ? 1 : n > 2000 ? 2000 : n;
		ms = ms < 1 ? 1 : ms > 1000 ? 1000 : ms;
		Post(TRawEvent::EButton1Down, (TInt)num[1], (TInt)num[2]);
		for (TInt k = 1; k <= n; ++k)
			{
			NKern::Sleep(NKern::TimerTicks(ms));
			Post(TRawEvent::EPointerMove, (TInt)(num[1] + ((TInt)num[3] - (TInt)num[1]) * k / n),
				(TInt)(num[2] + ((TInt)num[4] - (TInt)num[2]) * k / n));
			}
		Post(TRawEvent::EButton1Up, (TInt)num[3], (TInt)num[4]);
		}
	else if (Eq(c, "key") && argc >= 2)
		{
		TRawEvent e;
		e.Set(TRawEvent::EKeyDown, (TInt)num[1]);
		Kern::AddEvent(e);
		NKern::Sleep(NKern::TimerTicks(30));
		e.Set(TRawEvent::EKeyUp, (TInt)num[1]);
		Kern::AddEvent(e);
		}
	else if (Eq(c, "type") && argc >= 2)				// the rest of the line as typed keys, then Enter
		{
		TPtrC8 text = aLine.Mid(aLine.Find(arg[1]));
		for (TInt i = 0; i < text.Length(); ++i)
			A64Key_Type(text[i]);
		A64Key_Type('\r');
		Say("typed %d chars", text.Length());
		}
	else if (Eq(c, "wdt"))
		{
		Say("watchdog bite in 1 s");
		A64_WatchdogBite(1000);
		}
	else if (Eq(c, "usb"))
		{
		TBuf8<200> b;
		if (argc >= 2 && Eq(arg[1], "cold"))				// Pixel: power USB off and back up ourselves (debug)
			A64UsbCold(b, argc >= 3 ? (TInt)num[2] : 7);	// rails voted: 1 ldob1, 2 ldoa10, 4 ldob7
		else
			A64UsbStatus(b);
		Say("%S", &b);
		}
	else if (Eq(c, "shot"))
		Shot();
	else if (Eq(c, "fastboot"))
		{
		Say("rebooting to fastboot");
		A64_RebootToFastboot();
		}
	else if (Eq(c, "reboot"))
		{
		Say("rebooting");
		A64_RebootNormal();
		}
	else if (Eq(c, "poweroff"))
		{
		Say("powering off");
		Say("poweroff failed (%d)", A64_PowerOff());
		}
	else if (Eq(c, "rtc"))								// the PMIC RTC's raw seconds (Pixel)
		{
		TUint32 secs = 0;
		TInt r = A64_RtcSeconds(secs);
		Say("rtc %u (%d)", secs, r);
		}
	else if (Eq(c, "date") && argc >= 2)				// date <unix UTC s> [UTC offset s]: tools/phone.py sends the Mac's
		{
		TInt off = argc < 3 ? 0 : arg[2][0] == '-' ? -(TInt)Num(arg[2].Mid(1)) : (TInt)num[2];
		const TInt64 KSecs0To1970 = TInt64(719540) * 86400;	// Symbian's day count (e32-rs time.rs DAYS_0_TO_1970)
		TTimeK t = ((TInt64)num[1] + KSecs0To1970) * 1000000;
		NKern::LockSystem();								// the locale offset too, as ExecHandler::SetUTCTimeAndOffset
		TheMachineConfig().iLocale.iUniversalTimeOffset = off;
		TheMachineConfig().iLocale.iDaylightSaving = EDstNone;
		NKern::UnlockSystem();
		TUint changes = 0;
		TInt r = K::SetSystemTimeAndOffset(t, off, 0, changes, ETimeSetTime | ETimeSetOffset);
		Kern::NotifyChanges(changes | EChangesLocale);
		Say("date set: %u UTC, offset %d s (%d)", (TUint32)num[1], off, r);
		}
	else
		Say("unknown: %S (try help)", &c);
	NKern::ThreadLeaveCS();
	if (!Eq(c, "shot"))
		Write("\n.\n", 3);									// end of answer, for tools/phone.py
	}

// ---- the thread dump (was a64_serialkbd.cpp's Ctrl-T; this file is built with private/protected opened up)
void A64DumpThreads()
	{
	DObjectCon& c = *K::Containers[EThread];
	c.Wait();
	Kern::Printf("[dump] ===== %d threads at tick %d =====", c.Count(), NKern::TickCount());
	Kern::Printf("[dump] event queue: %d queued, request %08x status %08x ready %d, event thread %O",
		(TInt)((K::EventHeadPtr - K::EventTailPtr + (K::EventBufferEnd - K::EventBufferStart)) % (K::EventBufferEnd - K::EventBufferStart)),
		(TUint32)(TLinAddr)K::EventRequest, (TUint32)K::EventRequest->iStatus, K::EventRequest->IsReady(), K::EventThread);
	for (TInt i = 0; i < c.Count(); ++i)
		{
		DThread* t = (DThread*)c[i];
		NThreadBase& n = t->iNThread;
		const NThreadWaitState::S& w = n.iWaitState.iWtC;
		Kern::Printf("[dump] %O wait %02x type %d obj %08x waitobj %08x pri %d cs %d umcb %08x", t, w.iWtStFlags,
			w.iWtObjType, (TUint32)(TLinAddr)w.iWtObj, (TUint32)(TLinAddr)t->iWaitObj, n.iPriority, n.iCsCount,
			(TUint32)(TLinAddr)n.iUserModeCallbacks);
		if (t->iThreadType == EThreadUser && n.iStackBase)
			{
			const SThreadExcStack* f = (const SThreadExcStack*)((TLinAddr)n.iStackBase + n.iStackSize) - 1;
			if ((f->iSpsr & 0x1F) == 0x10)
				{
				TInt word = 0x0BADBAD0;						// the word r0 points at (a TRequestStatus, for WaitForRequest)
				Kern::ThreadRawRead(t, (const TAny*)(TLinAddr)(TUint32)f->iX[0], &word, sizeof(word));
				Kern::Printf("[dump]   user pc %08x lr %08x sp %08x r0 %08x [r0] %08x reqsem %d", (TUint32)f->iElr,
					(TUint32)f->iX[14], (TUint32)f->iX[13], (TUint32)f->iX[0], word, n.iRequestSemaphore.iCount);
				}
			}
		}
	c.Signal();
	Kern::Printf("[dump] ===== end =====");
	}
