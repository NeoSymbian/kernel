#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# build-kernel.sh -- build the AArch64 EKA2 kernel image (arm64 "Image") for QEMU virt. Runs in symbian-a64dev.
set -eu
HERE=$(cd "$(dirname "$0")/.." && pwd)
OUT=${OUT:-$HERE/../../out/kernel}
X=${CROSS:-aarch64-linux-gnu-}
mkdir -p "$OUT"; rm -f "$OUT"/*.o
ARCH="-mabi=ilp32 -march=armv8.2-a -mno-outline-atomics"
CXX="${X}g++-14 $ARCH -ffreestanding -fno-pic -fno-stack-protector -nostdinc -mgeneral-regs-only -fno-exceptions -fno-rtti -O2 -Wall"
AS="${X}gcc-14 $ARCH -c"
$AS "$HERE/boot/boot.S" -o "$OUT/boot.o"
$AS "$HERE/boot/vectors.S" -o "$OUT/vectors.o"
$CXX -std=gnu++17 -c "$HERE/board/qemu_virt.cpp" -o "$OUT/qemu_virt.o"
${X}ld -m aarch64linux32 -nostdlib --no-warn-rwx-segments -T "$HERE/boot/kernel.ld" -o "$OUT/kernel.elf" "$OUT"/*.o
${X}objcopy -O binary "$OUT/kernel.elf" "$OUT/Image"
echo "built $OUT/Image ($(wc -c < "$OUT/Image") bytes)"
