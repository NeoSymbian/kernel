/* SPDX-License-Identifier: EPL-2.0
 * Portions Copyright The Phorepoc Authors
 *
 * ekern.lds -- the AArch64 full kernel as ONE arm64 Image (after Phorepoc's adaptation-x86-64 ekern/ekern.lds):
 *   - the throw-away boot stub (boot.S + a64boot.cpp), at its load address (MMU off, then an identity window);
 *   - the EKA2 kernel at the flexible memory model's native linear addresses (port/arm64/mm/mmboot.h): "ROM" from
 *     KRomLinearBase 0x80000000, kernel .data/.bss at KKernelDataBase 0xC8000000, whose load image stays inside the
 *     ROM, as in a real XIP ROM (the bootstrap copies it out).
 * The loader places the file by physical (load) address; a64boot.cpp builds the tables that make the linear
 * addresses true. build-ekern.sh substitutes KERNEL_BASE (QEMU virt 0x40080000, Pixel ABL 0x80080000) and ROM_LIN:
 * 0x80000000, or 0x80200000 on the Pixel, whose load address is inside the first ROM chunk (the kernel finds the ROM
 * through TRomHeader::iRomBase, so it need not start at KRomLinearBase).
 */
OUTPUT_ARCH(aarch64:ilp32)
ENTRY(_start)

MEMORY
	{
	LOW   (rwx) : ORIGIN = KERNEL_BASE, LENGTH = 1M				/* boot stub: linear = physical */
	ROMP  (rwx) : ORIGIN = KERNEL_BASE + 1M, LENGTH = 30M		/* where the ROM sits physically */
	ROMV  (rx)  : ORIGIN = ROM_LIN, LENGTH = 30M				/* ... and linearly */
	DATV  (rw)  : ORIGIN = 0xC8000000, LENGTH = 64M			/* kernel .data/.bss, run address */
	}

SECTIONS
	{
	.boot :
		{
		KEEP(*(.text.head))
		*/boot.o(.text .text.* .rodata .rodata.* .data .data.*)		/* exact names: "*boot.o" also caught a64_reboot.o */
		*/a64boot.o(.text .text.* .rodata .rodata.* .data .data.*)
		} >LOW
	.boot.bss (NOLOAD) : ALIGN(16)
		{
		__bss_start = .;				/* boot.S zeroes this */
		*/boot.o(.bss .bss.* COMMON)
		*/a64boot.o(.bss .bss.* COMMON)
		. = ALIGN(16);
		__bss_end = .;
		} >LOW
	__boot_end = .;
	ASSERT((__boot_end - 1) / 0x200000 == _start / 0x200000, "the boot stub crosses its 2 MB identity window")

	/* ROM header area: TRomHeader + the image records, filled in at boot by a64boot.cpp (the "ROM" is RAM) */
	.romhdr : ALIGN(0x1000)
		{
		__rom_lin = .;
		BYTE(0)
		. = __rom_lin + 0x1000;
		} >ROMV AT>ROMP
	__rom_phys = LOADADDR(.romhdr);
	/* every ROM section's load address is pinned to __rom_phys + (linear - __rom_lin): left to ">ROMV AT>ROMP", ld
	 * aligns the two independently, and one slip breaks the ROM's linear <-> physical correspondence */

	.text ALIGN(0x1000) : AT(__rom_phys + (ADDR(.text) - __rom_lin))
		{
		__code_base = .;
		*(.text.entry)				/* _E32Startup first */
		*(.text .text.*)
		. = ALIGN(16);
		*(.utext)					/* AArch32 EL0 code, XIP (the ROM is user-readable and -executable) */
		} >ROMV
	.rodata ALIGN(32) : AT(__rom_phys + (ADDR(.rodata) - __rom_lin))
		{
		*(.rodata .rodata.*)
		*(.data.rel.ro .data.rel.ro.*)
		. = ALIGN(4);
		__init_array_start = .;
		KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*)))
		KEEP(*(.init_array))
		__init_array_end = .;
		. = ALIGN(0x1000);
		} >ROMV
	__code_end = .;
	__data_load = .;				/* ROM-linear address of the .data image */

	.data 0xC8000000 : AT(__rom_phys + (__data_load - __rom_lin))
		{
		__data_run = .;
		*(.data .data.*)
		. = ALIGN(8);
		__data_run_end = .;
		} >DATV
	__data_size = __data_run_end - __data_run;

	.bss (NOLOAD) : ALIGN(8)
		{
		*(.bss .bss.*)
		*(COMMON)
		. = ALIGN(0x1000);
		__svbss_end = .;
		} >DATV
	__svdata_size = __svbss_end - __data_run;					/* TRomHeader::iTotalSvDataSize */

	/* The user side: the ROM section built by tools/build-userrom.sh (prelinked for exactly this address), 2 MB into
	 * the ROM. Empty in a kernel-only build: then the bootstrap starts boot/a32test.S as the secondary. */
	.uimage (__rom_lin + 0x200000) : AT(__rom_phys + 0x200000)
		{
		__uimage_base = .;
		KEEP(*(.uimage))
		__uimage_end = .;
		}
	__uimage_size = __uimage_end - __uimage_base;
	ASSERT(__data_load + SIZEOF(.data) <= __uimage_base, "the kernel ROM reaches the user-side section")
	__rom_size = (__uimage_size ? __uimage_end : __data_load + SIZEOF(.data)) - __rom_lin;	/* bytes of ROM loaded */
	__image_size = ALIGN(__rom_phys + __rom_size - _start, 0x1000);	/* boot.S Image header */

	/DISCARD/ : { *(.note*) *(.comment) *(.eh_frame*) }
	}
