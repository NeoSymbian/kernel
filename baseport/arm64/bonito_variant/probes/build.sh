#!/bin/sh
# Build probe-a64.img (green) and probe-a32.img (blue) for `fastboot boot` on the Pixel 3a XL.
# Usage: probes/build.sh path/to/bonito.dtb
set -eu
DTB=${1:?usage: build.sh path/to/bonito.dtb}
cd "$(dirname "$0")"
LLVM=/opt/homebrew/opt/llvm/bin
LLD=/opt/homebrew/opt/lld/bin
mkdir -p out

# 64-bit: arm64 Image, gzipped like the working ztOS kernel (ABL decompresses and finds the DTB after it).
$LLVM/clang --target=aarch64-none-elf -c a64.S -o out/a64.o
$LLD/ld.lld -Ttext=0 --image-base=0 --oformat binary out/a64.o -o out/a64.Image
gzip -9nc out/a64.Image > out/a64.Image.gz
python3 -I mkboot.py out/a64.Image.gz "$DTB" out/probe-a64.img

# 32-bit: uncompressed zImage-style header; ABL treats a non-gzip, non-arm64 kernel as 32-bit.
$LLVM/clang --target=armv7a-none-eabi -c a32.S -o out/a32.o
$LLD/ld.lld -Ttext=0 --image-base=0 --oformat binary out/a32.o -o out/a32.zImage
python3 -I mkboot.py out/a32.zImage "$DTB" out/probe-a32.img

# Hybrid: AArch64 EL1 stub erets into AArch32 EL0 code (blue), exceptions paint red.
$LLVM/clang --target=armv7a-none-eabi -c el0_a32.S -o out/el0_a32.o
$LLD/ld.lld -Ttext=0 --image-base=0 --oformat binary out/el0_a32.o -o out/el0_a32.bin
$LLVM/clang --target=aarch64-none-elf -c el0.S -o out/el0.o
$LLD/ld.lld -Ttext=0 --image-base=0 --oformat binary out/el0.o -o out/el0.Image
gzip -9nc out/el0.Image > out/el0.Image.gz
python3 -I mkboot.py out/el0.Image.gz "$DTB" out/probe-el0.img
