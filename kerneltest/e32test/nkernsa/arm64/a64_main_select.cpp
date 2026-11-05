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
	TEST_PRINT("Tests Completed");
	__finish();
	}
