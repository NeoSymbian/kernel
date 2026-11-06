#!/bin/sh
# SPDX-License-Identifier: EPL-1.0
# ekern-census.sh -- compile census of the FULL EKA2 kernel (ekern, flexible memory model, SMP) for AArch64 ILP32,
# from UNMODIFIED kernelhwsrv source, the way Phorepoc's adaptation-x86-64 port/ekern_census_x32.sh sized the x32
# port: the same TU set (ekern.mmp + memmodel.mmp), with the ARM arch C++ (kernel/arm, memmodel/.../arm) selected.
# Never stops on a failing TU; prints OK/FAIL counts and the top error classes. LINK=1 also links the objects with
# the nanokernel + arm64 backend (compiled here with the ekern flags) and lists the UNDEFINED symbols = the ekern
# backend still to write.
# Shadow headers win over the baseline: port/arm64/ekern (kernel arch) and port/arm64/mm (memory model) come first.
# Runs in the symbian-a64dev image. Env: KHS = kernelhwsrv checkout, OUT = output dir, LINK=0|1.
set -u
HERE=$(cd "$(dirname "$0")/.." && pwd)
KHS=${KHS:-/DATA/AppData/symbian-build/src/core/symbian/kernelhwsrv}
OUT=${OUT:-$HERE/../../out/ekern-census}
EKA=$KHS/kernel/eka
X=${CROSS:-aarch64-linux-gnu-}
LINK=${LINK:-0}
mkdir -p "$OUT/kinc/kernel" "$OUT/obj"; rm -f "$OUT"/*.log "$OUT"/obj/*.o

# kernel include shims (adaptation-x86-64/recipes/kernel.sh): headers the real build exports to a relocated path,
# and exec_{enum,user,kernel}.h GENERATED from execs.txt by the release's own genexec.pl (its `cpp -I- -lang-c++`
# call is a 2004 GCC's: a stand-in maps it to g++ -E)
KINC=$OUT/kinc
echo "#include \"$EKA/include/heap_hybrid.h\"" > "$KINC/kernel/heap_hybrid.h"
echo "#include \"$KHS/halservices/hal/inc/hal_int.h\"" > "$KINC/kernel/hal_int.h"
SHIM=$OUT/cppshim; mkdir -p "$SHIM"
printf '#!/bin/sh\nargs=""\nfor a in "$@"; do case "$a" in -I-|-lang-c++) ;; *) args="$args $a";; esac; done\nexec '"$X"'g++-14 -E -x c++ $args\n' > "$SHIM/cpp"
chmod +x "$SHIM/cpp"
( cd "$EKA/kernel" && PATH="$SHIM:$PATH" perl genexec.pl -i execs.txt \
	-e "$KINC/kernel/exec_enum.h" -u "$KINC/kernel/exec_user.h" -k "$KINC/kernel/exec_kernel.h" ) >"$OUT/genexec.log" 2>&1 \
	|| { echo "genexec.pl FAILED:"; tail -5 "$OUT/genexec.log"; }

CXX="${X}g++-14 -mabi=ilp32 -march=armv8.2-a -mno-outline-atomics -ffreestanding -fno-pic -fno-stack-protector -nostdinc \
     -mgeneral-regs-only -std=gnu++03 -fno-exceptions -fno-rtti -fno-lifetime-dse -fno-strict-aliasing \
     -fno-delete-null-pointer-checks -fpermissive -w -O2"
# __CPU_ARMV7: ARMv8's cache maintenance is the v7 model (PoU/PoC by VA, set/way); cache_maintenance.h picks its
# InternalCache interface with it. cpudefs.h's ARM sections sit under __MARM__ and nk_cpu.h's under __CPU_ARM, which
# we don't define, so this selects no AArch32 asm.
DEFS="-D__GCC32__ -D__EPOC32__ -D__SYMBIAN32__ -D__KERNEL_MODE__ -D__IN_KERNEL__ -D_UNICODE -D__SMP__ \
      -D__MEMMODEL_FLEXIBLE__ -DMM_FLEXIBLE -D__CPU_ARMV7"
FLX=$EKA/memmodel/epoc/flexible
INCS="-I $HERE/a64smp -I $HERE/ekern -I $HERE/mm \
      -I $EKA/include/nkernsmp -I $EKA/include -I $EKA/include/kernel -I $EKA/include/kernel/arm \
      -I $EKA/include/memmodel/epoc -I $EKA/include/memmodel/epoc/flexible -I $EKA/include/memmodel/epoc/flexible/arm \
      -I $FLX -I $FLX/mmu -I $FLX/mmu/arm -I $EKA/include/memmodel/epoc/mmubase -I $EKA/memmodel/epoc/mmubase \
      -I $EKA/memmodel/epoc -I $EKA/klib -I $EKA/common -I $KHS/halservices/hal/inc -I $KINC -I $KINC/kernel"
PRE="-include $HERE/prelude/port_prelude.h -include $HERE/prelude/arm64_prelude.h -include $EKA/include/e32def.h"

TUS="$(cd $EKA && ls kernel/*.cpp kernel/arm/*.cpp klib/*.cpp klib/arm/*.cpp 2>/dev/null)
common/alloc.cpp common/array.cpp common/des8.cpp common/mem.cpp common/heap_hybrid.cpp
common/debugfunction.cpp common/secure.cpp
$(cd $EKA && ls memmodel/epoc/*.cpp memmodel/epoc/flexible/*.cpp memmodel/epoc/flexible/mmu/*.cpp \
	memmodel/epoc/flexible/mmu/arm/*.cpp memmodel/epoc/flexible/arm/*.cpp \
	memmodel/epoc/mmubase/kblockmap.cpp memmodel/epoc/mmubase/ramalloc.cpp memmodel/epoc/mmubase/defragbase.cpp 2>/dev/null)"

pass=0; fail=0; : > "$OUT/failed.txt"; : > "$OUT/errors.txt"
for tu in $TUS; do
	# not in ekern.mmp: crash logger, debug monitors, EABI-only helper, other binaries' entry points, exstart.exe
	# kernel/arm/vfp.cpp is evfp.dll's (kernel/evfp.mmp): the ARM VFPv2 bounce library, not needed on ARMv8
	case "$tu" in kernel/crash*|kernel/smon*|kernel/kdebug.cpp|kernel/array_eabi.cpp|kernel/arm/cmon*|*monitor.cpp|\
		kernel/arm/*_entry*.cpp|kernel/exstart.cpp|kernel/arm/vfp.cpp) continue;; esac
	n=$(echo "$tu" | tr '/' '_'); n=${n%.cpp}
	src="$EKA/$tu"
	# a marked-delta copy in port/arm64/mm (memory model) or port/arm64/ekern (kernel) replaces the baseline TU
	[ -f "$HERE/mm/$(basename "$tu")" ] && case "$tu" in memmodel/*) src="$HERE/mm/$(basename "$tu")";; esac
	[ -f "$HERE/ekern/$(basename "$tu")" ] && case "$tu" in kernel/arm/*) src="$HERE/ekern/$(basename "$tu")";; esac
	# one-line patches, checked: a copy in $OUT with an "a64 port" marker that must appear exactly once
	pat=""
	case "$tu" in
	# the demand-paging minimum is sized by the worst user instruction; the paged code is the AArch32 user side
	memmodel/epoc/flexible/mmu/mpager.cpp)
		pat='s|^#if defined(__CPU_ARM)$|#if defined(__CPU_ARM) \|\| defined(__CPU_ARM64)	// a64 port: AArch32 user code is paged|' ;;
	# the PRRR/NMRR statics: the header declares them only for __CPU_ARM (the same text at 835/844 must stay)
	# a release kernel prints nothing when a thread panics: print the category and reason (bring-up)
	kernel/sthread.cpp)
		pat='s|^\t\t__KTRACE_OPT2(KPANIC,KSCHED,\(Kern::Printf("Thread %O [PTK].*\)));$|\t\t\1);	// a64 port: always|' ;;
	kernel/arm/cache_maintenance.cpp)
		pat='34s|^#if !defined(__CPU_MEMORY_TYPE_REMAPPING)$|#if !defined(__CPU_MEMORY_TYPE_REMAPPING) \&\& defined(__CPU_ARM)	// a64 port: header declares them for ARM only|' ;;
	esac
	# Kern::Printf straight to the board console from the kernel's first line (the variant defines DebugOutputChar)
	extra=""; [ "$tu" = kernel/arm/cutils.cpp ] && extra="-D__EARLY_DEBUG__"
	# DThread::Exit's exit report sits under #ifdef KPANIC, a debug-build trace bit: keep it (and the patch below)
	[ "$tu" = kernel/sthread.cpp ] && extra="-DKPANIC=31"
	if [ -n "$pat" ]; then
		sed "$pat" "$src" > "$OUT/patched_$(basename "$tu")"
		want=1; [ "$tu" = kernel/sthread.cpp ] && want=3			# the exit-print patch changes three lines
		[ "$(grep -c 'a64 port:' "$OUT/patched_$(basename "$tu")")" = $want ] || echo "PATCH NOT APPLIED: $tu"
		src="$OUT/patched_$(basename "$tu")"
	fi
	if $CXX $extra $DEFS $INCS $PRE -c "$src" -o "$OUT/obj/$n.o" 2>"$OUT/$n.log"; then
		pass=$((pass+1)); rm -f "$OUT/$n.log"
	else
		fail=$((fail+1)); echo "$tu" >> "$OUT/failed.txt"; rm -f "$OUT/obj/$n.o"
		printf "FAIL %-48s %3d errors | %s\n" "$tu" "$(grep -c 'error:' "$OUT/$n.log")" \
			"$(grep -m1 'error:' "$OUT/$n.log" | sed 's/.*error: //' | cut -c1-90)"
		grep 'error:' "$OUT/$n.log" | sed 's/.*error: //' >> "$OUT/errors.txt"
	fi
done
echo "=== ekern census AArch64 ILP32 (flexible, SMP): $pass OK, $fail FAIL ==="
echo "--- top error classes ---"
sed -E "s/'[^']*'/'X'/g" "$OUT/errors.txt" | sort | uniq -c | sort -rn | head -25
echo "--- .cia files not compiled (AArch32 asm; need AArch64 C++/asm siblings) ---"
(cd $EKA && ls kernel/arm/*.cia klib/arm/*.cia common/arm/*.cia memmodel/epoc/flexible/arm/*.cia \
	memmodel/epoc/flexible/mmu/arm/*.cia 2>/dev/null) | sed 's/^/  /'

if [ "$LINK" = 1 ]; then
	# the nanokernel the full kernel links: the neutral nkernsmp core (not the standalone test build) + our backend
	mkdir -p "$OUT/nk"; rm -f "$OUT"/nk/*.o
	for spec in nklib.cpp:nkern nkern.cpp:nkernsmp nkerns.cpp:nkernsmp sched.cpp:nkernsmp dfcs.cpp:nkernsmp \
	            nk_timer.cpp:nkernsmp nk_irq.cpp:nkernsmp nk_bal.cpp:nkernsmp; do
		f=${spec%%:*}
		$CXX $DEFS $INCS $PRE -c "$EKA/${spec##*:}/$f" -o "$OUT/nk/nk_${f%.cpp}.o" 2>"$OUT/nk_${f%.cpp}.log" \
			|| { echo "FAIL nanokernel $f"; grep -m3 'error:' "$OUT/nk_${f%.cpp}.log"; }
	done
	for b in "$HERE"/a64smp/*.cpp; do
		n=$(basename "$b" .cpp)
		$CXX $DEFS $INCS $PRE -c "$b" -o "$OUT/nk/be_$n.o" 2>"$OUT/be_$n.log" \
			|| { echo "FAIL backend $n"; grep -m3 'error:' "$OUT/be_$n.log"; }
	done
	${X}gcc-14 -mabi=ilp32 -c "$HERE/a64smp/a64_ctxsw.S" -o "$OUT/nk/a64_ctxsw.o"
	# the ekern backend (port/arm64/ekern/a64e_*) and the shared platform layer
	for b in "$HERE"/ekern/a64e_*.cpp "$HERE"/platform/*.cpp; do
		[ -e "$b" ] || continue
		n=$(basename "$b" .cpp)
		$CXX $DEFS $INCS $PRE -c "$b" -o "$OUT/nk/x_$n.o" 2>"$OUT/x_$n.log" \
			&& echo "  backend $n OK" || { echo "FAIL backend $n"; grep -m6 'error' "$OUT/x_$n.log" | sed 's/^/      /'; }
	done
	for b in "$HERE"/ekern/a64e_*.S; do
		[ -e "$b" ] || continue
		n=$(basename "$b" .S)
		${X}gcc-14 -mabi=ilp32 -c "$b" -o "$OUT/nk/x_$n.o" 2>"$OUT/x_$n.log" \
			&& echo "  backend $n OK" || { echo "FAIL backend $n"; grep -m6 'rror' "$OUT/x_$n.log" | sed 's/^/      /'; }
	done
	${X}ld -m aarch64linux32 -r -o "$OUT/ekern_all.o" "$OUT"/obj/*.o "$OUT"/nk/*.o \
		2>"$OUT/ld-r.log" || { echo "ld -r FAILED:"; head -20 "$OUT/ld-r.log"; }
	${X}nm -u "$OUT/ekern_all.o" | awk '{print $2}' | ${X}c++filt | sort -u > "$OUT/undefined.txt"
	echo "=== relocatable link: $(wc -l < "$OUT/undefined.txt") UNDEFINED symbols -> $OUT/undefined.txt ==="
	echo "multiple definitions: $(grep -c 'multiple definition' "$OUT/ld-r.log")"
	${X}size "$OUT/ekern_all.o" | tail -1
fi
