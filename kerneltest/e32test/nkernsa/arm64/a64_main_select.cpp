// SPDX-License-Identifier: EPL-1.0
// a64_main_select.cpp -- nkernsa Main that runs only the test groups chosen at build time, in the order of the
// suite's own main.cpp, so a fix can be retried without the tests before it:
//   NKSA_MAIN=select NKSA_TESTS="bench pipe sem ipi basic dfc|dfcquick tied rwspin fmutex skm" build-kernel.sh
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
	TestDFCs();
#endif
#ifdef SEL_DFCQUICK
	TestDFCsQuick();
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
#ifdef SEL_SKMGRP
	TestSkmGroup();
#endif
	TEST_PRINT("Tests Completed");
	__finish();
	}
