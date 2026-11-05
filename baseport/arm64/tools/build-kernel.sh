#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# build-kernel.sh -- build the AArch64 EKA2 nanokernel test image (arm64 "Image") for QEMU virt:
#   the arch-neutral 2010 nkernsmp core + the standalone nanokernel tests (kerneltest/e32test/nkernsa), both
#   UNMODIFIED, + our arm64 backend, boot code, GICv3 driver and board glue. Runs in the symbian-a64dev image.
# Env: BOARD = qemu (default) | bonito, KHS = kernelhwsrv checkout, OUT = output dir,
#      BONITO_DTB = the Pixel's DTB (bonito: the boot.img needs it appended; ABL rejects an image without one).
set -eu
HERE=$(cd "$(dirname "$0")/.." && pwd)
KHS=${KHS:-/DATA/AppData/symbian-build/src/core/symbian/kernelhwsrv}
BOARD=${BOARD:-qemu}
OUT=${OUT:-$HERE/../../out/kernel-$BOARD}
case $BOARD in
	qemu)   BASE=0x40080000; BOARD_SRC=qemu_virt.cpp ;;
	bonito) BASE=0x80080000; BOARD_SRC=pixel_bonito.cpp ;;
	*) echo "unknown BOARD=$BOARD" >&2; exit 2 ;;
esac
EKA=$KHS/kernel/eka
NKSA=$KHS/kerneltest/e32test/nkernsa
X=${CROSS:-aarch64-linux-gnu-}
mkdir -p "$OUT/inc"; rm -f "$OUT"/*.o "$OUT"/*.log
ln -sfn "$NKSA" "$OUT/inc/nktest"			# the tests include <nktest/...>

ARCH="-mabi=ilp32 -march=armv8.2-a -mno-outline-atomics"
CFLAGS="$ARCH -ffreestanding -fno-pic -fno-stack-protector -nostdinc -mgeneral-regs-only -fno-exceptions -fno-rtti -O2 -Wall -Wno-unused"
# EKA2 TUs: the census flags (-fpermissive: operator new(TUint) vs ILP32 size_t unsigned long)
EKXX="${X}g++-14 $CFLAGS -std=gnu++03 -fno-lifetime-dse -fno-strict-aliasing -fno-delete-null-pointer-checks -fpermissive -Wno-invalid-offsetof"
DEFS="-D__GCC32__ -D__EPOC32__ -D__SYMBIAN32__ -D__KERNEL_MODE__ -D__SMP__ -D__STANDALONE_NANOKERNEL__"
INCS="-I $HERE/a64smp -I $EKA/include -I $EKA/include/nkernsmp -I $EKA/include/kernel -I $OUT/inc"
PRE="-include $HERE/prelude/port_prelude.h -include $HERE/prelude/arm64_prelude.h -include $EKA/include/e32def.h"
PLAIN="${X}g++-14 $CFLAGS -std=gnu++17"
AS="${X}gcc-14 $ARCH -c"

fail=0
ek() {	# ek <src> <obj> [extra flags]
	if ! $EKXX ${3:-} $DEFS $INCS $PRE -c "$1" -o "$OUT/$2.o" 2>"$OUT/$2.log"; then
		echo "FAIL $1"; grep -m8 'error' "$OUT/$2.log" | sed 's/^/    /'; fail=1
	fi
}

for spec in nklib.cpp:nkern nkern.cpp:nkernsmp nkerns.cpp:nkernsmp sched.cpp:nkernsmp dfcs.cpp:nkernsmp \
            nk_timer.cpp:nkernsmp nk_irq.cpp:nkernsmp nk_bal.cpp:nkernsmp; do
	f=${spec%%:*}; ek "$EKA/${spec##*:}/$f" "nk_${f%.cpp}"
done
for f in "$HERE"/a64smp/*.cpp; do b=$(basename "$f" .cpp); ek "$f" "be_$b"; done
# NKSA_MAIN=groups: our a64_main_groups.cpp replaces the suite's main.cpp (focused reproduction)
# NKSA_MAIN=select NKSA_TESTS="...": a64_main_select.cpp runs only the named groups (see that file)
MAIN=main; TESTDFC=testdfc; SELDEFS=""
case "${NKSA_MAIN:-}" in
	groups) MAIN="" ;;
	select) MAIN=""
	        for t in ${NKSA_TESTS:?NKSA_TESTS lists the tests to run}; do SELDEFS="$SELDEFS -DSEL_$(echo $t | tr a-z A-Z)"; done
	        case " $NKSA_TESTS " in *" dfcquick "*) TESTDFC="" ;; esac ;;	# dfcquick compiles testdfc.cpp into the selector
esac
for f in init kprintf tlsf nkutils diag $MAIN testutils threadbasic fastsem fastmutex debug testipi $TESTDFC fastbuf \
         benchmark rwspinlock tiedevents; do
	# benchmark.cpp times empty functions through a pointer: GCC -O2 inlines them and deletes the timed loop,
	# so Measure() never reaches its threshold (RVCT did not). Keep the calls real.
	extra=""; [ $f = benchmark ] && extra="-fno-inline -fno-ipa-cp"
	ek "$NKSA/$f.cpp" "sa_$f" "$extra"
done
# -Dprivate=public: the stall dump reads scheduler internals (NThreadWaitState, SIterDQ) of the 2010 headers
ek "$HERE/nksa/a64_nksa.cpp" "a64_nksa" "-Dprivate=public"
[ "${NKSA_MAIN:-}" = groups ] && ek "$HERE/nksa/a64_main_groups.cpp" "sa_main_groups"
[ "${NKSA_MAIN:-}" = select ] && ek "$HERE/nksa/a64_main_select.cpp" "sa_main_select" "$SELDEFS"
[ $fail = 0 ] || exit 1

$AS "$HERE/a64smp/a64_ctxsw.S" -o "$OUT/a64_ctxsw.o"
$AS -DBOARD_$(echo "$BOARD" | tr a-z A-Z) "$HERE/boot/boot.S" -o "$OUT/boot.o"
$AS "$HERE/boot/vectors.S" -o "$OUT/vectors.o"
[ "$BOARD" = bonito ] && python3 "$HERE/tools/mkfont.py" "$OUT/inc/font.h" >/dev/null
$PLAIN -I "$OUT/inc" -c "$HERE/board/$BOARD_SRC" -o "$OUT/board.o"
$PLAIN -c "$HERE/board/boot_common.cpp" -o "$OUT/boot_common.o"
$PLAIN -c "$HERE/board/gicv3.cpp" -o "$OUT/gicv3.o"

if ! ${X}ld -m aarch64linux32 -nostdlib --no-warn-rwx-segments --defsym=KERNEL_BASE=$BASE -T "$HERE/boot/kernel.ld" -o "$OUT/kernel.elf" \
		"$OUT"/*.o 2>"$OUT/link.log"; then
	echo "LINK FAILED -- undefined:"; grep -o "undefined reference to .*" "$OUT/link.log" | sort -u | ${X}c++filt | head -40
	grep -v "undefined reference" "$OUT/link.log" | head -5
	exit 1
fi
${X}objcopy -O binary "$OUT/kernel.elf" "$OUT/Image"
echo "built $OUT/Image ($(wc -c < "$OUT/Image") bytes, $BOARD at $BASE)"
if [ "$BOARD" = bonito ]; then	# fastboot boot image: gzip(Image) + DTB, as the ztOS/Linux boot.img ABL accepts
	gzip -9nc "$OUT/Image" > "$OUT/Image.gz"
	python3 -I "$HERE/../../probes/mkboot.py" "$OUT/Image.gz" "${BONITO_DTB:?set BONITO_DTB to the Pixel DTB}" "$OUT/boot.img"
fi
