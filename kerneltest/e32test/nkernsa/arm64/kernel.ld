/* SPDX-License-Identifier: Apache-2.0
 * Runs at its load address (MMU off, then an identity map): KERNEL_BASE = RAM base + text_offset 0x80000,
 * given by build-kernel.sh (QEMU virt 0x40080000, Pixel ABL 0x80080000). */
OUTPUT_ARCH(aarch64:ilp32)
ENTRY(_start)
SECTIONS
{
	. = KERNEL_BASE;
	.text : { KEEP(*(.text.head)) *(.text .text.*) }
	.rodata : { *(.rodata .rodata.*) }
	.init_array : { __init_array_start = .; KEEP(*(SORT(.init_array.*) .init_array)) __init_array_end = .; }
	.data : { *(.data .data.*) }
	. = ALIGN(16);
	__image_end = .;
	.bss (NOLOAD) : { __bss_start = .; *(.bss .bss.* COMMON) . = ALIGN(16); __bss_end = .; }
	__image_size = 0x400000;		/* boot.S Image header */
	ASSERT(__bss_end - _start <= __image_size, "kernel image + .bss exceed the 4 MB image_size")
	/DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
