#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# build-userrom.sh -- the user side of the AArch64 kernel image, from the NeoSymbian/symbian-2010 build (READ-ONLY:
# its Docker volume, mounted at S2010; run its ./build.sh first):
#   - romsect.bin: the ROM SECTION -- the same user-side images and data files as that build's Syborg minigui ROM
#     (its own .ekern-build/rombuild.args, so the content can't drift: euser, F32, estart, the window server stack,
#     eshell, the Rust EXEs) plus the font, with our estart.txt; prelinked as XIP E32 images with a Z: tree by Phorepoc's pdk
#     rombuild.py --section for the ARM flexible memory model. The kernel image .incbin's it; a64boot.cpp reads it.
# Env: S2010 = the volume's mount, PDK = pdk checkout, OUT = output dir, BASE = the section's linear address,
#      USERROM_SKIP = comma-separated Z: names to leave out (default none: estart starts SYSSTART.EXE = neoui, the
#      Rust UI server; USERROM_SKIP='Sys\Bin\SYSSTART.EXE' gives the window server -> eshell console instead).
set -eu
HERE=$(cd "$(dirname "$0")/.." && pwd)
S2010=${S2010:-/s2010}
PDK=${PDK:-/DATA/AppData/symbian-build/src/pdk}
OUT=${OUT:?}
BASE=${BASE:?}
SRCARGS=$S2010/.ekern-build/rombuild.args
[ -f "$SRCARGS" ] || { echo "no symbian-2010 build at $S2010 (mount the phorepoc-symbian-2010 volume; run its ./build.sh)"; exit 1; }
mkdir -p "$OUT/user"; rm -f "$OUT"/user/*
ARGS=$OUT/user/rombuild.args
# --xip/--data "<Z: path>=<path in the volume>" pairs, re-rooted at the mount; our estart.txt; --xinfo dropped (the
# section takes its layout from its own options)
python3 -I - "$SRCARGS" "$S2010" "$HERE/boot/estart.txt" "$ARGS" "${USERROM_SKIP-}" <<'PY'
import sys
src, root, estart, out = sys.argv[1:5]
words = [w.rstrip("\n") for w in open(src) if w.strip()]
res, n = [], 0
# estart starts SYSSTART.EXE (neoui, the Rust UI) in preference to EWSRV.EXE (the window server -> eshell); leaving
# SYSSTART out (USERROM_SKIP) gives the eshell console
SKIP = {x.strip().lower() for x in (sys.argv[5] if len(sys.argv) > 5 else "").split(",") if x.strip()}
for opt, val in zip(words[0::2], words[1::2]):
    if opt not in ("--xip", "--data"):
        continue
    z, _, path = val.partition("=")
    if z.lower() in SKIP:
        continue
    host = estart if z.lower() == r"sys\data\estart.txt" else root + "/" + path
    # without SYSSTART (the eshell ROM) nothing serves econs_twin's !E32WindowServer, which the UI ROM maps as
    # ECONS.DLL: give eshell the window server's console instead
    if z.lower() == r"sys\bin\econs.dll" and r"sys\bin\sysstart.exe" in SKIP:
        host = root + "/emitter/econswserv.dll"
    res += [opt, z + "=" + host]
    n += 1
# the window server's font, which symbian-2010 keeps on C: (its ROM placed it so fntstore's in-place path faulted); in
# this section data files are word-aligned and fbserv uses it from Z: as is
res += ["--data", r"Resource\Fonts\ceurope.gdr=" + root + "/src/oss.FCL.sf.os.graphics/graphicstest/uibench/data/CEUROPE.GDR"]
open(out, "w").write("\n".join(res) + "\n")
print("user side: %d files from %s" % (n, src))
PY
# EXE data/bss run at the bottom of the local data area; ROM DLL data under the ARM flexible model's 0x7f000000
python3 -I "$PDK/tools/rombuild.py" --section "$BASE" --data-top 0x7f000000 --exe-data-base 0x00400000 \
	--secondary-name efile.exe "@$ARGS" "$OUT/romsect.bin" > "$OUT/userrom.log" 2>&1 \
	|| { echo "rombuild --section FAILED:"; tail -15 "$OUT/userrom.log"; exit 1; }
grep "^wrote" "$OUT/userrom.log"

