#!/bin/sh
# SPDX-License-Identifier: EPL-1.0
# census.sh -- compile the arch-NEUTRAL nkernsmp core UNMODIFIED for AArch64 ILP32, plus whatever arm64 backend
# exists, and list the symbols still undefined: the backend surface left to write. Modelled on Phorepoc's
# adaptation-x86-64 port/build_nkernsmp_link.sh. Never stops on a failing TU; prints a summary.
# Runs in the build image (NeoSymbian/build docker/toolchain). Env: KHS = the kernel tree (default: this checkout), OUT = output dir.
set -u
HERE=$(cd "$(dirname "$0")/.." && pwd)
. "$HERE/tools/paths.sh"
OUT=${OUT:-$HERE/../../out/census}
X=${CROSS:-aarch64-linux-gnu-}
mkdir -p "$OUT"; rm -f "$OUT"/*.o "$OUT"/*.log

CXX="${X}g++-14 -mabi=ilp32 -march=armv8.2-a -mno-outline-atomics -ffreestanding -fno-pic -fno-stack-protector -nostdinc -mgeneral-regs-only \
     -std=gnu++03 -fno-exceptions -fno-rtti -fno-lifetime-dse -fno-strict-aliasing -fno-delete-null-pointer-checks \
     -Wall -Wno-unused -O2 -fpermissive"   # -fpermissive: EKA2 declares operator new(TUint), ILP32 size_t is unsigned long (Phorepoc ARM build uses it too)
DEFS="-D__GCC32__ -D__EPOC32__ -D__SYMBIAN32__ -D__KERNEL_MODE__ -D__SMP__ -D__STANDALONE_NANOKERNEL__"
INCS="-I $EKA/include/nkernsmp/arm64 -I $EKA/include -I $EKA/include/nkernsmp -I $EKA/include/kernel/arm64"

NEUTRAL="nklib.cpp:nkern nkern.cpp:nkernsmp nkerns.cpp:nkernsmp sched.cpp:nkernsmp \
         dfcs.cpp:nkernsmp nk_timer.cpp:nkernsmp nk_irq.cpp:nkernsmp nk_bal.cpp:nkernsmp"

ok=0; fail=0
echo "== neutral nkernsmp TUs (unmodified 2010 source)"
for spec in $NEUTRAL; do
	f=${spec%%:*}; d=${spec##*:}; o=$OUT/${f%.cpp}.o
	if $CXX $DEFS $INCS $PRE -c "$EKA/$d/$f" -o "$o" 2>"$OUT/${f%.cpp}.log"; then
		printf '  %-14s OK\n' "$f"; ok=$((ok+1))
	else
		printf '  %-14s FAIL (%s errors)\n' "$f" "$(grep -c 'error:' "$OUT/${f%.cpp}.log")"; fail=$((fail+1))
		grep -m6 'error:' "$OUT/${f%.cpp}.log" | sed "s|$EKA/||; s/^/      /"
	fi
done

echo "== arm64 backend"
for src in "$NK64"/a64s_*.cpp "$EKA"/common/arm64/*.cpp "$NK64"/a64_ctxsw.S; do
	[ -e "$src" ] || continue
	b=$(basename "$src"); o=$OUT/backend_${b%.*}.o
	case $src in *.S) cc="${X}gcc-14 -mabi=ilp32 -c" ;; *) cc="$CXX $DEFS $INCS $PRE -c" ;; esac   # asm: no C++ prelude
	if $cc "$src" -o "$o" 2>"$OUT/backend_${b%.*}.log"; then printf '  %-14s OK\n' "$b"; ok=$((ok+1))
	else printf '  %-14s FAIL\n' "$b"; fail=$((fail+1)); grep -m6 'error:' "$OUT/backend_${b%.*}.log" | sed 's/^/      /'; fi
done

echo "== undefined symbols (backend still to write)"
if ls "$OUT"/*.o >/dev/null 2>&1; then
	${X}ld -m aarch64linux32 -r -o "$OUT/all.o" "$OUT"/*.o 2>"$OUT/link.log" || cat "$OUT/link.log"
	${X}nm -u "$OUT/all.o" | awk '{print $2}' | sort -u > "$OUT/undefined.txt"
	${X}c++filt < "$OUT/undefined.txt" | sed 's/^/  /'
	echo "== summary: $ok TUs OK, $fail failed, $(wc -l < "$OUT/undefined.txt") undefined symbols"
fi
