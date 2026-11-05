/* SPDX-License-Identifier: Apache-2.0 -- step 1: runs at its load address on QEMU virt (RAM 0x40000000 + 0x80000) */
OUTPUT_ARCH(aarch64:ilp32)
ENTRY(_start)
SECTIONS
{
	. = 0x40080000;
	.text : { KEEP(*(.text.head)) *(.text .text.*) }
	.rodata : { *(.rodata .rodata.*) }
	.init_array : { __init_array_start = .; KEEP(*(SORT(.init_array.*) .init_array)) __init_array_end = .; }
	.data : { *(.data .data.*) }
	. = ALIGN(16);
	__image_end = .;
	.bss (NOLOAD) : { __bss_start = .; *(.bss .bss.* COMMON) . = ALIGN(16); __bss_end = .; }
	ASSERT(__bss_end - _start <= 0x400000, "kernel image + .bss exceed the 4 MB image_size in boot.S")
	/DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
