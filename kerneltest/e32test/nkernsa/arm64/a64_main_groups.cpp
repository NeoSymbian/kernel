// SPDX-License-Identifier: EPL-1.0
// a64_main_groups.cpp -- alternative nkernsa Main (build with NKSA_MAIN=groups instead of the suite's main.cpp):
// only the thread-group tests from threadbasic.cpp, which hung on the phone in the migrate case, so the
// failure reproduces in seconds instead of after 10 minutes of the full suite.
#include <nktest/nkutils.h>

extern void ThreadGroupTest1(TInt aCount, TBool aJoin, TBool aMigrate, TBool aReJoin);

void Main(TAny*)
	{
	TEST_PRINT("Testing thread groups only (a64_main_groups)");
	ThreadGroupTest1(2,0,FALSE,FALSE);
	ThreadGroupTest1(3,1,FALSE,FALSE);
	ThreadGroupTest1(3,1,TRUE,FALSE);
	ThreadGroupTest1(3,1,TRUE,TRUE);
	TEST_PRINT("Tests Completed");
	__finish();
	}
