// SPDX-License-Identifier: EPL-2.0
// a64_main_select.cpp -- nkernsa Main that runs only the test groups chosen at build time, in the order of the
// suite's own main.cpp, so a fix can be retried without the tests before it:
//   NKSA_MAIN=select NKSA_TESTS="bench pipe sem ipi basic dfc|dfcquick tied rwspin fmutex skm|skmpar" build-kernel.sh
// Phone regression set: "bench pipe sem ipi basic dfcquick tied rwspin fmutex skmpar" (skm = the same cases serially).
// dfcquick = TestDFCs without its two DoStressTest runs (~50 min of fixed-length phases). It needs TTestDfc, which
// testdfc.cpp keeps to itself, so with SEL_DFCQUICK this file includes testdfc.cpp (unchanged) instead of linking it.
#include <nktest/nkutils.h>
#ifdef SEL_DFCQUICK
#include <nktest/testdfc.cpp>
#endif

extern void TestFastSemaphore();
extern void TestFastMutex();
extern void TestSuspendKillMigrate();
extern void BasicThreadTests();
extern void TestIPI();
extern void TestDFCs();
extern void TestWaitFreePipe();
extern void BenchmarkTests();
extern void TestRWSpinLock();
extern void TestTiedEvents();
extern void DoRWTest(TInt aTestType, TUint32 aReadLimit, TUint32 aInSeq, TUint32 aOutSeq, TInt aRWCpu, TInt aICpu);	// fastmutex.cpp

#if defined(SEL_DFC) || defined(SEL_DFCQUICK)
/* The DFC tests assume Main stays on the CPU it starts on: DFCTest2 pins its priority-1 DfcQ2 thread there and
 * expects queued DFCs to wait while Main spins. ThreadGroupTest (basic) leaves Main's affinity "any", and with the
 * load balancer running (Hw_Init3; the 2010 nkernsa never started it) Main could be moved mid-test, so the DFCs ran
 * at once (phone: testdfc.cpp:441-444 "Cancel" failures). Pin Main for the duration. */
static void RunPinned(void (*aTest)())
	{
	NThread* t = NKern::CurrentThread();
	TUint32 aff = NKern::ThreadSetCpuAffinity(t, NKern::CurrentCpu());
	(*aTest)();
	NKern::ThreadSetCpuAffinity(t, aff);
	}
#endif

#ifdef SEL_SKMPAR
/* skmpar: TestSuspendKillMigrate's 138 cases (skm_cases.inc, generated from fastmutex.cpp by build-kernel.sh) run
 * several at a time, each on its own CPU pair (reader + writer on 2k, intervention on 2k+1), instead of one after
 * another on CPUs 0/1 with the other six idle. DoRWTest keeps all its state per call. Workers take the next case from
 * a shared index (the Pixel's pairs 6/7 are the fast cores). Workers run at Main's priority 16: DoRWTest creates the
 * case's intervention thread from its caller after the reader/writer (priority 11, never blocking in timeslice mode)
 * already own the CPU, so a lower-priority worker starved and every case hung (QEMU, first try at priority 9). */
enum { ETimeslice, ESuspend, EKill, EMigrate, EInterlockedSuspend, EInterlockedKill, EInterlockedMigrate };
static const TUint32 KSkmCases[][3] = {
#include "skm_cases.inc"
	};
static const TUint32 KSkmCount = sizeof(KSkmCases) / sizeof(KSkmCases[0]);
static volatile TUint32 SkmNext;
static void SkmWorker(TAny* aPair)
	{
	TInt pair = (TInt)(TLinAddr)aPair;
	for (TUint32 i; (i = __e32_atomic_add_ord32(&SkmNext, 1)) < KSkmCount; )
		DoRWTest(KSkmCases[i][0], SKM_READS, KSkmCases[i][1], KSkmCases[i][2], 2 * pair, 2 * pair + 1);
	}
static void TestSkmParallel()
	{
	TInt pairs = NKern::NumberOfCpus() / 2;
	if (pairs > 4)
		pairs = 4;
	TEST_PRINT2("Testing Suspend/Kill/Migrate: %d cases on %d CPU pairs...", KSkmCount, pairs);
	NFastSemaphore exitSem(0);
	for (TInt p = 0; p < pairs; ++p)
		CreateThreadSignalOnExit("SkmWorker", &SkmWorker, 16, (TAny*)(TLinAddr)p, 0, KSmallTimeslice, &exitSem, 2 * p);
	for (TInt p = 0; p < pairs; ++p)
		NKern::FSWait(&exitSem);
	}
#endif

#ifdef SEL_SKMGRP
/* skmgrp: TestSuspendKillMigrate's "join a group with a mutex-holder" section (in-op 8), which exercised the
 * JoinGroup fix, plus two controls (frozen group, no group). Types: SReaderInfo::TTestType, private to fastmutex.cpp. */
static void TestSkmGroup()
	{
	TEST_PRINT("Testing Suspend/Kill/Migrate, mutex-holder group...");
	enum { ETimeslice, ESuspend, EKill, EMigrate, EInterlockedSuspend, EInterlockedKill, EInterlockedMigrate };
	const TUint32 KReads = 10000;
	static const TUint32 KCases[][3] = {
		{ ETimeslice, 0x138, 0x831 }, { ETimeslice, 0x438, 0x834 },
		{ ESuspend, 0x138, 0x831 }, { ESuspend, 0x438, 0x834 },
		{ EKill, 0x138, 0x831 }, { EKill, 0x438, 0x834 },
		{ EMigrate, 0x138, 0x831 }, { EMigrate, 0x438, 0x834 }, { EMigrate, 0x080, 0x800 },
		{ EInterlockedSuspend, 0x438, 0x834 }, { EInterlockedSuspend, 0x138, 0x831 },
		{ EInterlockedKill, 0x438, 0x834 }, { EInterlockedKill, 0x138, 0x831 },
		{ EInterlockedMigrate, 0x138, 0x831 }, { EInterlockedMigrate, 0x438, 0x834 },
		{ EInterlockedMigrate, 0x180, 0x810 }, { EInterlockedMigrate, 0x480, 0x840 },
		{ ETimeslice, 0x137, 0x731 }, { ETimeslice, 0x310, 0x310 },		// controls
		};
	for (TUint i = 0; i < sizeof(KCases) / sizeof(KCases[0]); ++i)
		DoRWTest(KCases[i][0], KReads, KCases[i][1], KCases[i][2], 0, 1);
	}
#endif

#ifdef SEL_DFCQUICK
static void TestDFCsQuick()
	{
	TEST_PRINT("Testing DFCs (quick: no stress tests)...");
	TTestDfc::Buffer = CircBuf::New(TTestDfc::EBufferSlots);
	TInt cpu;
	DoDFCTest1();
	DoDFCTest2();
	for_each_cpu(cpu)
		DoDFCTest3(cpu);
	DoIDFCTest1();
	for_each_cpu(cpu)
		DoIdleDFCTest1(cpu);
	DoIdleDFCTest2();
	delete TTestDfc::Buffer;
	TTestDfc::Buffer = 0;
	}
#endif

void Main(TAny*)
	{
	TEST_PRINT("Selected tests (a64_main_select)");
#ifdef SEL_BENCH
	BenchmarkTests();
#endif
#ifdef SEL_PIPE
	TestWaitFreePipe();
#endif
#ifdef SEL_SEM
	TestFastSemaphore();
#endif
#ifdef SEL_IPI
	TestIPI();
#endif
#ifdef SEL_BASIC
	BasicThreadTests();
#endif
#ifdef SEL_DFC
	RunPinned(&TestDFCs);
#endif
#ifdef SEL_DFCQUICK
	RunPinned(&TestDFCsQuick);
#endif
#ifdef SEL_TIED
	TestTiedEvents();
#endif
#ifdef SEL_RWSPIN
	TestRWSpinLock();
#endif
#ifdef SEL_FMUTEX
	TestFastMutex();
#endif
#ifdef SEL_SKM
	TestSuspendKillMigrate();
#endif
#ifdef SEL_SKMPAR
	TestSkmParallel();
#endif
#ifdef SEL_SKMGRP
	TestSkmGroup();
#endif
#ifdef SEL_SKMIS
	// skmis: the interlocked-suspend mutex-holder-group cases, repeated (QEMU saw F2 = 2 once in 438/834)
	TEST_PRINT("Testing interlocked suspend, mutex-holder group, x4...");
	for (TInt i = 0; i < 4; ++i)
		{
		DoRWTest(4, 10000, 0x438, 0x834, 0, 1);		// 4 = SReaderInfo::EInterlockedSuspend
		DoRWTest(4, 10000, 0x138, 0x831, 0, 1);
		}
#endif
	TEST_PRINT("Tests Completed");
	__finish();
	}
