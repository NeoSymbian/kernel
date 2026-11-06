// SPDX-License-Identifier: Apache-2.0
// console_io.cpp -- board-independent console helpers and device addressing, shared by both images:
// A64_DebugPuts/Hex over the board's A64_DebugPutc, and physical device address -> the address board code uses
// (board.h A64_IoRegions).
#include "board.h"

extern "C" {
unsigned long A64_IoWindow;			// 0 until the full kernel's _E32Startup moves device access into the IO window

unsigned long A64_Io(unsigned long aPhys)
	{
	if (!A64_IoWindow)
		return aPhys;
	unsigned long lin = A64_IoWindow;
	for (int i = 0; i < A64_IoRegionCount; ++i)
		{
		const SA64IoRegion& r = A64_IoRegions[i];
		if (aPhys - r.iPhys < r.iSize)
			return lin + (aPhys - r.iPhys);
		lin += r.iSize;
		}
	return 0;						// not a board region: the access faults at 0, loudly
	}

void A64_DebugPuts(const char* s) { while (*s) A64_DebugPutc(*s++); }
void A64_DebugHex(unsigned long long v)
	{
	A64_DebugPuts("0x");
	for (int i = 60; i >= 0; i -= 4) A64_DebugPutc("0123456789abcdef"[(v >> i) & 0xF]);
	}

// boards without a screen have no status bar or pinned lines
__attribute__((weak)) void A64_ConsoleStatus(const char*, unsigned, unsigned, unsigned) {}
__attribute__((weak)) void A64_ConsolePin(int, const char*) {}
__attribute__((weak)) void A64_ConsoleReserveTop(int) {}		// serial consoles share nothing
}
