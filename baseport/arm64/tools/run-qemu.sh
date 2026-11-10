#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# run-qemu.sh -- boot the kernel Image on QEMU virt (GICv3, Cortex-A55) with a timeout; serial to stdout and a log.
# Usage: run-qemu.sh [seconds] [expected-final-line]   exit 0 iff the expected line appeared.
# Env: SMP (default 4), OUT, MON (a monitor socket path),
#      INPUT=<text> (printf escapes, e.g. "dir\r"; each "~" waits 5 s): typed into the serial console after INPUT_DELAY
#      seconds (default 15); Ctrl-T ("\024") makes the full kernel dump its threads,
#      SERIAL1=<chardev>: the second PL011 (the debug serial port, RBusDevComm unit 1), default null; e.g.
#      unix:/tmp/uart1.sock,server=on,wait=off (tools/test-serial-qemu.py). The first stays the console on stdio.
#      DISK=<file>: a virtio-blk disk (the persistent C:, board/virtio_blk.cpp), created 256 MB if missing,
#      FBDUMP=<file.png>: a moment before the timeout, save the full kernel's RAM framebuffer (the address its display
#      driver prints: "[display] WxH 32 bpp, framebuffer phys P") through the monitor and convert it to a PNG.
set -u
HERE=$(cd "$(dirname "$0")/.." && pwd)
OUT=${OUT:-$HERE/../../out/kernel-qemu}
SECS=${1:-10}; EXPECT=${2:-}
FBDUMP=${FBDUMP:-}
[ -n "$FBDUMP" ] && MON=${MON:-$OUT/qemu-mon.sock}
[ -n "${MON:-}" ] && rm -f "$MON"
MONITOR=none; [ -n "${MON:-}" ] && MONITOR="unix:$MON,server,nowait"
DISKARGS=""
if [ -n "${DISK:-}" ]; then
	[ -f "$DISK" ] || truncate -s 256M "$DISK"
	DISKARGS="-global virtio-mmio.force-legacy=true -drive file=$DISK,if=none,format=raw,id=d0 -device virtio-blk-device,drive=d0"
fi
qemu() { timeout "$SECS" qemu-system-aarch64 -M virt,gic-version=3 -cpu cortex-a55 -smp "${SMP:-4}" -m 512 \
	-display none -serial stdio -serial "${SERIAL1:-null}" -monitor "$MONITOR" \
	-no-reboot -nic none $DISKARGS -kernel "$OUT/Image" > "$OUT/serial.log" 2>&1; }
if [ -n "${INPUT:-}" ]; then
	{ sleep "${INPUT_DELAY:-15}"
	  rest=$INPUT
	  while :; do piece=${rest%%"~"*}; printf "$piece"; [ "$piece" = "$rest" ] && break; rest=${rest#*"~"}; sleep 5; done
	  sleep "$SECS"; } | qemu &
else
	qemu < /dev/null &
fi
QPID=$!
if [ -n "$FBDUMP" ]; then
	sleep $((SECS - 5))
	python3 -I - "$MON" "$OUT/serial.log" "$FBDUMP" <<'PY'
import re, socket, sys, time
from PIL import Image
mon, log, png = sys.argv[1:]
m = re.search(r"\[display\] (\d+)x(\d+) 32 bpp, framebuffer phys ([0-9a-f]+)", open(log, errors="replace").read())
if not m:
    sys.exit("FBDUMP: no '[display]' line in the serial log")
w, h, phys = int(m.group(1)), int(m.group(2)), int(m.group(3), 16)
raw = png + ".raw"
s = socket.socket(socket.AF_UNIX); s.connect(mon); time.sleep(0.3); s.recv(65536)
s.sendall(("pmemsave 0x%x %d \"%s\"\n" % (phys, w * h * 4, raw)).encode())	# quoted: a bare /path parses as division
reply, t0 = b"", time.time()
while b"(qemu)" not in reply and time.time() - t0 < 15:		# the monitor echoes keystroke by keystroke
    reply += s.recv(65536)
print("FBDUMP: monitor said:", repr(reply.decode(errors="replace").replace("\x1b[K","").replace("\x1b[D","")[-160:]))
if b"rror" in reply:
    sys.exit("FBDUMP: monitor: " + reply.decode(errors="replace")[-200:])
Image.frombytes("RGB", (w, h), open(raw, "rb").read(), "raw", "BGRX").save(png)
print("FBDUMP: %s (%dx%d from phys %08x)" % (png, w, h, phys))
PY
fi
wait $QPID
QSTATUS=$?
tr -d '\r' < "$OUT/serial.log"
case $QSTATUS in 0|124) ;; *) echo "run-qemu: QEMU failed ($QSTATUS)" >&2; exit "$QSTATUS" ;; esac	# 124: the timeout
[ -z "$EXPECT" ] || grep -q "$EXPECT" "$OUT/serial.log"
