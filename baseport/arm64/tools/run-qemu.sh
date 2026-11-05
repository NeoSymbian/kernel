#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# run-qemu.sh -- boot the kernel Image on QEMU virt (GICv3, Cortex-A55) with a timeout; serial to stdout and a log.
# Usage: run-qemu.sh [seconds] [expected-final-line]   exit 0 iff the expected line appeared.
set -u
HERE=$(cd "$(dirname "$0")/.." && pwd)
OUT=${OUT:-$HERE/../../out/kernel-qemu}
SECS=${1:-10}; EXPECT=${2:-}
timeout "$SECS" qemu-system-aarch64 -M virt,gic-version=3 -cpu cortex-a55 -smp "${SMP:-4}" -m 512 \
	-nographic -no-reboot -nic none ${MON:+-monitor unix:$MON,server,nowait} -kernel "$OUT/Image" > "$OUT/serial.log" 2>&1
tr -d '\r' < "$OUT/serial.log"
[ -z "$EXPECT" ] || grep -q "$EXPECT" "$OUT/serial.log"
