// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// Portions Copyright The Phorepoc Authors
// SPDX-License-Identifier: EPL-2.0
//
// a64e_entry.cpp -- the kernel image's entry point, _E32Startup(ROM header, super page): kernel/arm/k_entry.cia in
// C++, with what Phorepoc's x86-64 entry (ekern/xk_entry.cpp) adds for a kernel whose bootstrap is linked in.
// boot/a64boot.cpp calls it with the MMU on, at the kernel's linear addresses, on the initial thread stack.
#include <kernel/kern_priv.h>
#include <plat_priv.h>
#include <mmboot.h>
#include "../board/board.h"

extern "C" void KernelMain();
extern "C" char _start[];									// the boot stub (ekern.lds), identity-mapped
extern "C" char A64_Vectors[];								// boot/vectors.S
extern "C" void (*const __init_array_start[])();			// ekern.lds
extern "C" void (*const __init_array_end[])();

extern "C" __attribute__((section(".text.entry"))) void _E32Startup(TLinAddr aRomHeader, TLinAddr aSuperPage)
	{
	A64_IoWindow = KPrimaryIOBase;							// the bootstrap mapped the board's devices there
	__asm__ __volatile__("msr vbar_el1, %0\n\tisb" :: "r"((TUint64)(TLinAddr)A64_Vectors) : "memory");
	A64_DebugPuts("[ekern] _E32Startup: MMU on, kernel linear addresses, initial thread stack\n");

	// Nothing low is needed any more: drop the bootstrap's identity window (one 2 MB block over the boot stub, in
	// the local half of OS ASID 0's page directory) before the memory model inventories the address space.
	volatile TUint64* pd = (volatile TUint64*)KPageDirectoryBase;		// PDEs are 8 bytes (mm/mmboot.h)
	pd[(TLinAddr)_start >> KChunkShift] = KPdeUnallocatedEntry;
	__asm__ __volatile__("dsb ishst\n\ttlbi vmalle1\n\tdsb ish\n\tisb" ::: "memory");

	PP::InitSuperPageFromRom(aRomHeader, aSuperPage);
	TInt n = 0;
	for (void (*const* f)() = __init_array_start; f < __init_array_end; ++f, ++n)
		(*f)();
	A64_DebugPuts("[ekern] static constructors: "); A64_DebugHex(n);
	A64_DebugPuts("\n[ekern] -> KernelMain()\n");
	KernelMain();
	A64_DebugPuts("[ekern] KernelMain returned?!\n");
	for (;;)
		__asm__ __volatile__("wfi");
	}
