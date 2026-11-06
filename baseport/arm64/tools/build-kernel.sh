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
	f=${spec%%:*}; src="$EKA/${spec##*:}/$f"
	# a64 port fix in the 2010 kernel (keep it in the ekern build too, see docs/port-map.md): NKern::JoinGroup moved a
	# group that was already ready to the TAIL of the joining thread's priority (ChangePriority), behind its equal-
	# priority peers, so the running thread lost its place and the next reschedule switched it out even while it held
	# a fast mutex (fastmutex.cpp DoRWTest 0x138/0x438, F2 != 0). The group takes the joiner's place at the head.
	if [ $f = nkern.cpp ] && [ -z "${NO_JOINGROUP_FIX:-}" ]; then	# NO_JOINGROUP_FIX=1: the 2010 original, to compare
		sed -e 's/^			else if (pC->iPriority > aGroup->iPriority)$/			else if (pC->iPriority >= aGroup->iPriority)	\/\/ a64 port fix: JoinGroup keeps the head/' \
		    -e 's/^				ss.SSChgEntryP(aGroup, pC->iPriority);$/				{ ss.SSRemoveEntry(aGroup); aGroup->iPriority = pC->iPriority; ss.SSAddEntryHead(aGroup); }	\/\/ a64 port fix: group first/' \
		    "$src" > "$OUT/nkern.cpp"
		[ $(grep -c 'a64 port fix:' "$OUT/nkern.cpp") -eq 2 ] || { echo "FAIL nkern.cpp JoinGroup fix did not apply"; fail=1; }
		src="$OUT/nkern.cpp"
	fi
	ek "$src" "nk_${f%.cpp}"
done
for f in "$HERE"/a64smp/*.cpp; do b=$(basename "$f" .cpp); ek "$f" "be_$b"; done
for f in "$HERE"/platform/*.cpp; do b=$(basename "$f" .cpp); ek "$f" "pf_$b"; done	# shared with the full kernel
# NKSA_MAIN=groups: our a64_main_groups.cpp replaces the suite's main.cpp (focused reproduction)
# NKSA_MAIN=select NKSA_TESTS="...": a64_main_select.cpp runs only the named groups (see that file)
MAIN=main; TESTDFC=testdfc; SELDEFS=""
case "${NKSA_MAIN:-}" in
	groups) MAIN="" ;;
	select) MAIN=""
	        for t in ${NKSA_TESTS:?NKSA_TESTS lists the tests to run}; do SELDEFS="$SELDEFS -DSEL_$(echo $t | tr a-z A-Z)"; done
	        case " $NKSA_TESTS " in *" dfcquick "*) TESTDFC="" ;; esac	# dfcquick compiles testdfc.cpp into the selector
	        # skmpar passes DoRWTest the same read count as the (scaled or original) fastmutex.cpp
	        if [ "$BOARD" = bonito ] || [ -n "${NKSA_SCALE:-}" ]; then SELDEFS="$SELDEFS -DSKM_READS=10000"; else SELDEFS="$SELDEFS -DSKM_READS=100000"; fi ;;
esac
for f in init kprintf tlsf nkutils diag $MAIN testutils threadbasic fastsem fastmutex debug testipi $TESTDFC fastbuf \
         benchmark rwspinlock tiedevents; do
	# benchmark.cpp times empty functions through a pointer: GCC -O2 inlines them and deletes the timed loop,
	# so Measure() never reaches its threshold (RVCT did not). Keep the calls real.
	# -ftrivial-auto-var-init=zero (test files only): tiedevents.cpp leaves SNThreadGroupCreateInfo::iDestructionDfc
	# uninitialised, so group destruction queued a garbage DFC and faulted (RVCT-era stacks evidently held 0 there)
	extra="-ftrivial-auto-var-init=zero"; [ $f = benchmark ] && extra="$extra -fno-inline -fno-ipa-cp"
	src="$NKSA/$f.cpp"
	# Two statistical checks were tuned on ~400 MHz ARM11 boards; on the phone, compile a copy scaled for 2 GHz cores
	# (QEMU's emulated cores are slow enough for the originals):
	# - tiedevents: an untied reader must overlap the 10 ms timer, but 100 passes over 2048 flags take ~0.2 ms per
	#   CPU ("Flag wasn't set, test broken?") -> 50x the passes.
	# - fastmutex DoRWTest: a cross-CPU suspend must catch the writer inside an unprotected 256-word block >= 10% of
	#   the time, but the block takes less than the ~1 us suspend IPI latency (F5 ~ 0 right after LeaveCS) -> 16x.
	#   Each read is a full cross-CPU suspend/kill/migrate round trip, so 78 cases x 100000 reads took hours: 10000
	#   reads keep both pass rules (0 bad reads protected, >= 10% unprotected) at a tenth of the time.
	pat=""
	if [ "$BOARD" = bonito ] || [ -n "${NKSA_SCALE:-}" ]; then	# NKSA_SCALE=1: the same on QEMU
		case $f in
		tiedevents) pat='s/^const TInt LoopCount = 100;$/const TInt LoopCount = 5000;	\/\/ a64 port: scaled for 2 GHz cores/' ;;
		fastmutex)  pat='s/^	TUint32 bufwords = 256;$/	TUint32 bufwords = 4096;	\/\/ a64 port: scaled for 2 GHz cores/;s/^const TInt KReadCount = 100000;$/const TInt KReadCount = 10000;	\/\/ a64 port: reads/' ;;
		esac
	fi
	# NKSA_RWTRACE=1: the DoRWTest reader reports the writer's state at each bad block read (race diagnostics)
	if [ $f = fastmutex ] && [ -n "${NKSA_RWTRACE:-}" ]; then
		pat="${pat:+$pat;}"'s/^				++info.iFails\[i\];$/				{ ++info.iFails[i]; extern void A64_RWFail(TInt, TInt, NThread*, TUint32, TUint32); A64_RWFail(info.iTestType, i, info.iWriter, winfo.iInSeq, winfo.iOutSeq); }	\/\/ a64 port: trace/'
	fi
	if [ -n "$pat" ]; then
		sed "$pat" "$src" > "$OUT/$f.cpp"
		[ $(grep -c 'a64 port: ' "$OUT/$f.cpp") -eq $(echo "$pat" | grep -o 'a64 port: ' | wc -l) ] || { echo "FAIL $f scaling patch did not apply"; fail=1; }
		src="$OUT/$f.cpp"
	fi
	ek "$src" "sa_$f" "$extra"
done
# -Dprivate=public: the stall dump reads scheduler internals (NThreadWaitState, SIterDQ) of the 2010 headers
ek "$HERE/nksa/a64_nksa.cpp" "a64_nksa" "-Dprivate=public"
[ "${NKSA_MAIN:-}" = groups ] && ek "$HERE/nksa/a64_main_groups.cpp" "sa_main_groups"
# skmpar: TestSuspendKillMigrate's DoRWTest cases, extracted from the suite source so the parallel runner can't drift
# from it: "DoRWTest(SReaderInfo::EKill, KReadCount, 0x010, 0x100, 0, 1);" -> "{ EKill, 0x010, 0x100 },"
awk '/^void TestSuspendKillMigrate/,/^\t}/' "$NKSA/fastmutex.cpp" \
	| sed -n 's/^[[:space:]]*DoRWTest(SReaderInfo::\(E[A-Za-z]*\),[[:space:]]*KReadCount,[[:space:]]*\(0x[0-9a-f]*\),[[:space:]]*\(0x[0-9a-f]*\),.*/	{ \1, \2, \3 },/p' \
	> "$OUT/skm_cases.inc"
[ $(wc -l < "$OUT/skm_cases.inc") -gt 100 ] || { echo "FAIL skm_cases.inc: only $(wc -l < "$OUT/skm_cases.inc") cases"; fail=1; }
[ "${NKSA_MAIN:-}" = select ] && ek "$HERE/nksa/a64_main_select.cpp" "sa_main_select" "$SELDEFS -ftrivial-auto-var-init=zero -I$OUT"
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
