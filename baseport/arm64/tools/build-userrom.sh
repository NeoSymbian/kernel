#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# build-userrom.sh -- the user side of the AArch64 kernel image, from the NeoSymbian/build build (READ-ONLY:
# its Docker volume, mounted at S2010; run its ./build.sh first):
#   - romsect.bin: the ROM SECTION -- the same user-side images and data files as that build's Syborg minigui ROM
#     (its own .ekern-build/rombuild.args, so the content can't drift: euser, F32, estart, the window server stack,
#     eshell, the Rust EXEs) plus the font, with our estart.txt; prelinked as XIP E32 images with a Z: tree by
#     tools/rombuild.py --section (from Phorepoc's pdk) for the ARM flexible memory model. The kernel image .incbin's it; a64boot.cpp reads it.
# Env: S2010 = the volume's mount, OUT = output dir, BASE = the section's linear address,
#      USERROM_SKIP = comma-separated Z: names to leave out (default none: estart starts SYSSTART.EXE = neoui, the
#      Rust UI server; USERROM_SKIP='Sys\Bin\SYSSTART.EXE' gives the window server -> eshell console instead),
#      ROFS = 1: Z: is composite (docs/filesystem.md FS4a) -- the section keeps the core (the file server, estart, the
#      file system plug-ins, the locale, and everything they import), rofsbuild packs the rest into rofs.img (plain
#      E32 images, loaded into RAM), and estart.txt gains the ROFS line.
set -eu
HERE=$(cd "$(dirname "$0")/.." && pwd)
S2010=${S2010:-/s2010}
OUT=${OUT:?}
BASE=${BASE:?}
SRCARGS=$S2010/.ekern-build/rombuild.args
[ -f "$SRCARGS" ] || { echo "no NeoSymbian/build output at $S2010 (mount the NeoSymbian/build volume, neosymbian-build; run its ./build.sh)"; exit 1; }
mkdir -p "$OUT/user"; rm -f "$OUT"/user/*
ARGS=$OUT/user/rombuild.args
# --xip/--data "<Z: path>=<path in the volume>" pairs, re-rooted at the mount; our estart.txt; --xinfo dropped (the
# section takes its layout from its own options)
ROFS=${ROFS:-0}
ESTART=$HERE/common/estart.txt
if [ "$ROFS" != 0 ]; then	# local drive 2 = the ROFS image (a64_medrofs.cpp), added to Z: by the composite file system
	cat "$ESTART" - > "$OUT/user/estart.txt" <<'TXT'
Z:  2  EROFS   ROFS  0       FS_COMPOSITE                       # the ROFS part of Z: (docs/filesystem.md FS4a)
TXT
	ESTART=$OUT/user/estart.txt
fi
python3 -I - "$SRCARGS" "$S2010" "$ESTART" "$ARGS" "${USERROM_SKIP-}" "$ROFS" "$HERE/tools" "$OUT/user/rofs.oby" <<'PY'
import os, re, sys
src, root, estart, out = sys.argv[1:5]
rofs, tools, oby = sys.argv[6] != "0", sys.argv[7], sys.argv[8]
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
# the window server's font, which NeoSymbian/build keeps on C: (its ROM placed it so fntstore's in-place path faulted); in
# this section data files are word-aligned and fbserv uses it from Z: as is
res += ["--data", r"Resource\Fonts\ceurope.gdr=" + root + "/src/oss.FCL.sf.os.graphics/graphicstest/uibench/data/CEUROPE.GDR"]
if rofs:
    # the core: what starts before Z: is composite (EFILE, ESTART and its estart.txt, the file system plug-ins, the
    # locale and keymap DLLs), closed over their imports -- an XIP image can't import a ROFS DLL (prelinked ordinals)
    sys.path.insert(0, tools)
    from rombuild import E32Xip
    pairs = list(zip(res[0::2], res[1::2]))
    xip = {v.partition("=")[0].lower(): v.partition("=")[2] for o, v in pairs if o == "--xip"}
    by_file = {z.rsplit("\\", 1)[-1]: z for z in xip}
    core = {z for z in xip if re.search(r"\\(efile\.exe|estart\.exe|e(local|rofs|comp)\.fsy|ekdata\.dll|elocl[^\\]*)$", z)}
    todo = list(core)
    while todo:
        z = todo.pop()
        for nm, _ in E32Xip(xip[z], z).imports:
            dep = by_file.get(re.sub(r"\{[0-9a-f]+\}|\[[0-9a-f]+\]", "", nm.lower()))
            if dep and dep not in core:
                core.add(dep)
                todo.append(dep)
    # a fixed time: the image (its header) depends only on its content, so a core-only build (ROFS=flash) matches the
    # ROFS a ROFS_INSTALL build wrote to system_a (a64_medrofs.cpp compares the headers)
    keep, lines = [], ["rofsname=" + os.path.join(os.path.dirname(oby), "rofs.img"), "rofssize=0x4000000",
                       "time=01/01/2026 00:00:00"]
    for o, v in pairs:
        z, _, path = v.partition("=")
        if z.lower() in core or z.lower() == r"sys\data\estart.txt":
            keep += [o, v]
        else:
            # executables get every capability, as the XIP ROM's (rombuild.py --xinfo cap1/cap2): the images' own
            # headers say none, and servers like !Windowserver need ProtServ
            lines.append("file=%s %s capability=All" % (path, z) if o == "--xip" else "data=%s %s" % (path, z))
    res = keep
    open(oby, "w").write("\n".join(lines) + "\n")
    print("ROFS split: %d in the core ROM, %d in the ROFS" % (len(keep) // 2, len(lines) - 3))
open(out, "w").write("\n".join(res) + "\n")
print("user side: %d files from %s" % (n, src))
PY
if [ "$ROFS" != 0 ]; then
	command -v rofsbuild >/dev/null || { echo "ROFS=1 needs rofsbuild (the toolchain image's host tools)"; exit 1; }
	(cd "$OUT/user" && rofsbuild rofs.oby > rofsbuild.log 2>&1) || { echo "rofsbuild FAILED:"; tail -15 "$OUT/user/rofsbuild.log"; exit 1; }
	! grep -E "WARNING|ERROR" "$OUT/user/rofsbuild.log" || { echo "rofsbuild: warnings (an image it refused?)"; exit 1; }
	echo "rofs.img: $(wc -c < "$OUT/user/rofs.img") bytes"
	# FS5a: SHA-256 of each 4 KB block (the last one zero-padded) for a64_medverity.cpp. VERITY_BAD=N spoils block N's
	# hash, to test that a changed block is caught.
	python3 -I - "$OUT/user/rofs.img" "$OUT/user/rofs.hash" "${VERITY_BAD:-}" <<'PY'
import hashlib, sys
img = open(sys.argv[1], "rb").read()
img += bytes(-len(img) % 4096)
table = bytearray(b"".join(hashlib.sha256(img[i:i + 4096]).digest() for i in range(0, len(img), 4096)))
if sys.argv[3]:
    table[int(sys.argv[3]) * 32] ^= 0xff
open(sys.argv[2], "wb").write(table)
print("rofs.hash: %d blocks%s" % (len(table) // 32, ", block %s spoilt (VERITY_BAD)" % sys.argv[3] if sys.argv[3] else ""))
PY
fi
# EXE data/bss run at the bottom of the local data area; ROM DLL data under the ARM flexible model's 0x7f000000
python3 -I "$HERE/tools/rombuild.py" --section "$BASE" --data-top 0x7f000000 --exe-data-base 0x00400000 \
	--secondary-name efile.exe "@$ARGS" "$OUT/romsect.bin" > "$OUT/userrom.log" 2>&1 \
	|| { echo "rombuild --section FAILED:"; tail -15 "$OUT/userrom.log"; exit 1; }
grep "^wrote" "$OUT/userrom.log"

