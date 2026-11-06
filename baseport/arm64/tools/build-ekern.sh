#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# build-ekern.sh -- build the AArch64 FULL kernel image (arm64 "Image"): the EKA2 kernel, flexible memory model and
# SMP nanokernel from unmodified 2010 source (compiled by ekern-census.sh, which also builds our backend and the
# platform layer), plus the boot stub + bootstrap, the static variant and the board. Runs in the symbian-a64dev image.
# Env: BOARD = qemu (default) | bonito, KHS = kernelhwsrv checkout, OUT = output dir,
#      USERSIDE = 1 (default: the 2010 user side from the symbian-2010 build volume mounted at S2010, see
#      build-userrom.sh) | 0 (kernel only: boot/a32test.S is the secondary),
#      BONITO_DTB = the Pixel's DTB (bonito: appended to boot.img; never commit it).
set -eu
HERE=$(cd "$(dirname "$0")/.." && pwd)
KHS=${KHS:-/DATA/AppData/symbian-build/src/core/symbian/kernelhwsrv}
BOARD=${BOARD:-qemu}
OUT=${OUT:-$HERE/../../out/ekern-$BOARD}
case $BOARD in
	qemu)   BASE=0x40080000; ROM_LIN=0x80000000; BOARD_SRC=qemu_virt.cpp ;;
	bonito) BASE=0x80080000; ROM_LIN=0x80200000; BOARD_SRC=pixel_bonito.cpp ;;	# the load window owns 0x80000000-0x801FFFFF
	*) echo "unknown BOARD=$BOARD" >&2; exit 2 ;;
esac
EKA=$KHS/kernel/eka
X=${CROSS:-aarch64-linux-gnu-}
BDEF=-DBOARD_$(echo "$BOARD" | tr a-z A-Z)
mkdir -p "$OUT/img"; rm -f "$OUT"/img/*

# 1. kernel + memory model + nanokernel + our backend + platform layer: the census build, which must be complete
OUT="$OUT/census" KHS="$KHS" LINK=1 sh "$HERE/tools/ekern-census.sh" > "$OUT/census.log" 2>&1
grep -E "^=== |^FAIL|^PATCH NOT APPLIED|^ld -r FAILED" "$OUT/census.log"
grep -q "^=== ekern census .*: [0-9]* OK, 0 FAIL ===" "$OUT/census.log" || { echo "census: kernel TUs failed"; exit 1; }
! grep -qE "^FAIL|^PATCH NOT APPLIED|^ld -r FAILED" "$OUT/census.log" || exit 1

# 2. boot stub, bootstrap, variant, board: kernel-side TUs take the census flags
ARCH="-mabi=ilp32 -march=armv8.2-a -mno-outline-atomics"
CFLAGS="$ARCH -ffreestanding -fno-pic -fno-stack-protector -nostdinc -mgeneral-regs-only -fno-exceptions -fno-rtti -O2"
EKXX="${X}g++-14 $CFLAGS -std=gnu++03 -fno-lifetime-dse -fno-strict-aliasing -fno-delete-null-pointer-checks -fpermissive -w"
DEFS="-D__GCC32__ -D__EPOC32__ -D__SYMBIAN32__ -D__KERNEL_MODE__ -D__IN_KERNEL__ -D_UNICODE -D__SMP__ \
      -D__MEMMODEL_FLEXIBLE__ -DMM_FLEXIBLE -D__CPU_ARMV7"
C=$OUT/census
FLX=$EKA/memmodel/epoc/flexible
INCS="-I $HERE/a64smp -I $HERE/ekern -I $HERE/mm \
      -I $EKA/include/nkernsmp -I $EKA/include -I $EKA/include/kernel -I $EKA/include/kernel/arm \
      -I $EKA/include/memmodel/epoc -I $EKA/include/memmodel/epoc/flexible -I $EKA/include/memmodel/epoc/flexible/arm \
      -I $FLX -I $FLX/mmu -I $FLX/mmu/arm -I $EKA/include/memmodel/epoc/mmubase -I $EKA/memmodel/epoc/mmubase \
      -I $EKA/memmodel/epoc -I $EKA/klib -I $EKA/common -I $KHS/halservices/hal/inc -I $C/kinc -I $C/kinc/kernel"
PRE="-include $HERE/prelude/port_prelude.h -include $HERE/prelude/arm64_prelude.h -include $EKA/include/e32def.h"
PLAIN="${X}g++-14 $CFLAGS -std=gnu++17 -Wall -Wno-unused"
AS="${X}gcc-14 $ARCH -c"
I=$OUT/img
# the bootstrap runs with the MMU off: no unaligned accesses, and no calls a loop could turn into memset/memcpy
$EKXX $DEFS $INCS $PRE $BDEF -mstrict-align -fno-tree-loop-distribute-patterns -c "$HERE/boot/a64boot.cpp" -o "$I/a64boot.o"
$EKXX $DEFS $INCS $PRE -c "$HERE/board/a64_variant.cpp" -o "$I/a64_variant.o"
# the local-media stack as built-in extensions (a64_media.cpp): the unmodified ELOCD and internal RAM drive, entry
# points renamed (each is its own DLL in a real ROM)
MED="-I $EKA/drivers/locmedia -I $EKA/drivers/locmedia/traces -I $EKA/include/drivers -I $HERE/board"
$EKXX $DEFS $INCS $MED $PRE -DInitExtension=InitExtension_locmedia -DKernelModuleEntry=KernelModuleEntry_locmedia \
	-DCreateLogicalDevice=CreateLogicalDevice_locmedia -c "$EKA/drivers/locmedia/locmedia.cpp" -o "$I/locmedia.o"
$EKXX $DEFS $INCS $MED $PRE -c "$EKA/drivers/locmedia/dmasupport.cpp" -o "$I/dmasupport.o"
$EKXX $DEFS $INCS $MED $PRE -DInitExtension=InitExtension_medint -DKernelModuleEntry=KernelModuleEntry_medint \
	-DCreatePhysicalDevice=CreatePhysicalDevice_medint -c "$EKA/drivers/medint/iram.cpp" -o "$I/iram.o"
$EKXX $DEFS $INCS $MED $PRE -c "$HERE/board/a64_media.cpp" -o "$I/a64_media.o"
# the real exstart (starts the ROM secondary), statically linked: only its entry points are renamed
$EKXX $DEFS $INCS $PRE -DKernelModuleEntry=A64ExstartEntry -DInitExtension=A64ExstartInit \
	-c "$EKA/kernel/exstart.cpp" -o "$I/exstart.o"
# the AArch32 EL0 test program (the secondary for now): exec numbers from the generated exec_enum.h, assembled for
# the 2010 user side's ISA, flattened, and carried in the ROM's .utext
sed -n 's/^[[:space:]]*\(E[A-Za-z0-9_]*\) = \([0-9]*\),*$/	.equ \1, \2/p' "$C/kinc/kernel/exec_enum.h" > "$I/exec_enum.inc"
grep -q "EExecDebugPrint" "$I/exec_enum.inc" || { echo "exec_enum.inc: no exec numbers"; exit 1; }
arm-none-eabi-as -march=armv5te -I "$I" "$HERE/boot/a32test.S" -o "$I/a32test.a32"
arm-none-eabi-objcopy -O binary "$I/a32test.a32" "$I/a32test.bin"
printf '\t.section .utext, "ax"\n\t.balign 16\n\t.global A32Test, A32TestEnd\nA32Test:\n\t.incbin "%s"\nA32TestEnd:\n' \
	"$I/a32test.bin" > "$I/a32test_rom.S"
$AS $BDEF -DA64_EKERN "$HERE/boot/boot.S" -o "$I/boot.o"
$AS "$HERE/boot/vectors.S" -o "$I/vectors.o"
$AS "$I/a32test_rom.S" -o "$I/a32test_rom.o"
# the user-side ROM section, prelinked for its place 2 MB into the ROM (ekern.lds .uimage)
if [ "${USERSIDE:-1}" != 0 ]; then
	OUT="$OUT" BASE=$(printf '0x%08X' $((ROM_LIN + 0x200000))) sh "$HERE/tools/build-userrom.sh"
	printf '\t.section .uimage, "a"\n\t.incbin "%s"\n' "$OUT/romsect.bin" > "$I/uimage.S"
	$AS "$I/uimage.S" -o "$I/uimage.o"
fi
[ "$BOARD" = bonito ] && python3 "$HERE/tools/mkfont.py" "$I/font.h" >/dev/null
$PLAIN -I "$I" -c "$HERE/board/$BOARD_SRC" -o "$I/board.o"
$PLAIN -c "$HERE/board/gicv3.cpp" -o "$I/gicv3.o"
$PLAIN -c "$HERE/board/console_io.cpp" -o "$I/console_io.o"
# the boot stub may reach the ROM only by address, never by a call
bad=$(${X}nm -u "$I/a64boot.o" | awk '{print $2}' | grep -vE '^(_start|_E32Startup|A64VariantEntry|A64VariantInitialise|A64ExtensionEntry|A64ExstartShim|A32Test|A32TestEnd|A64_IoRegions|A64_IoRegionCount|__(rom|code|data|svdata|uimage)_[a-z]*)$' || true)
[ -z "$bad" ] || { echo "a64boot.o calls outside the boot stub: $bad"; exit 1; }

# 3. link at the native addresses
sed -e "s/KERNEL_BASE/$BASE/g" -e "s/ROM_LIN/$ROM_LIN/g" "$HERE/boot/ekern.lds" > "$I/ekern.lds"
if ! ${X}ld -m aarch64linux32 -nostdlib --no-warn-rwx-segments --orphan-handling=warn -T "$I/ekern.lds" -o "$OUT/ekern.elf" \
		"$I"/*.o "$C"/obj/*.o "$C"/nk/*.o 2>"$OUT/link.log"; then
	echo "LINK FAILED:"; grep -o "undefined reference to .*" "$OUT/link.log" | sort -u | ${X}c++filt | head -40
	grep -v "undefined reference" "$OUT/link.log" | grep -v "orphan section" | head -10
	exit 1
fi
grep -i "warning" "$OUT/link.log" | grep -vE "orphan section .(\.rela\.|\.iplt|\.igot)" | sort -u | head -10 || true	# static link: those stay empty
${X}objcopy -O binary "$OUT/ekern.elf" "$OUT/Image"
${X}size -A "$OUT/ekern.elf" | grep -E "^\.(boot|text|rodata|data|bss) "
echo "built $OUT/Image ($(wc -c < "$OUT/Image") bytes, $BOARD at $BASE)"
if [ "$BOARD" = bonito ]; then	# fastboot boot image: gzip(Image) + DTB, as build-kernel.sh
	gzip -9nc "$OUT/Image" > "$OUT/Image.gz"
	python3 -I "$HERE/../../probes/mkboot.py" "$OUT/Image.gz" "${BONITO_DTB:?set BONITO_DTB to the Pixel DTB}" "$OUT/boot.img"
fi
