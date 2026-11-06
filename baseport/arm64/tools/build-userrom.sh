#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# build-userrom.sh -- the ROM SECTION of the AArch64 kernel image: the real 2010 user side (euser, F32, estart, ...)
# as prelinked XIP E32 images with a Z: directory tree, built by Phorepoc's pdk rombuild.py --section for the ARM
# flexible memory model. The kernel image .incbin's it at a fixed ROM address; boot/a64boot.cpp reads its header.
# The images come READ-ONLY from the NeoSymbian/symbian-2010 build volume (run its ./build.sh first).
# Env: S2010 = that volume's mount (/s2010), PDK = pdk checkout, OUT = output dir, BASE = the section's linear address.
set -eu
HERE=$(cd "$(dirname "$0")/.." && pwd)
S2010=${S2010:-/s2010}
PDK=${PDK:-/DATA/AppData/symbian-build/src/pdk}
OUT=${OUT:?}
BASE=${BASE:?}
[ -d "$S2010/emitter" ] || { echo "no symbian-2010 build at $S2010 (mount the phorepoc-symbian-2010 volume; run its ./build.sh)"; exit 1; }
mkdir -p "$OUT/user"; rm -f "$OUT"/user/*
ARGS=$OUT/user/rombuild.args; : > "$ARGS"
grep -v '^#' "$HERE/boot/userside.args" | grep . | while IFS= read -r line; do
	zname=${line%%=*}; img=${line#*=}
	[ -f "$S2010/emitter/$img" ] || { echo "missing $S2010/emitter/$img"; exit 1; }
	cp "$S2010/emitter/$img" "$OUT/user/$img"
	printf -- '--xip\n%s=%s\n' "$zname" "$OUT/user/$img" >> "$ARGS"
done
printf -- '--data\nSys\\Data\\estart.txt=%s\n' "$HERE/boot/estart.txt" >> "$ARGS"
# EXE data/bss run at the bottom of the local data area; ROM DLL data under the ARM flexible model's 0x7f000000
python3 -I "$PDK/tools/rombuild.py" --section "$BASE" --data-top 0x7f000000 --exe-data-base 0x00400000 \
	--secondary-name efile.exe "@$ARGS" "$OUT/romsect.bin" > "$OUT/userrom.log" 2>&1 \
	|| { echo "rombuild --section FAILED:"; tail -15 "$OUT/userrom.log"; exit 1; }
tail -1 "$OUT/userrom.log"
head -1 "$OUT/userrom.log"
