#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""test-serial-qemu.py -- the debug serial port test on QEMU virt (inside the build image, from the kernel tree).

Boots out/ekern-qemu (neo kernel qemu, with NEOCOM.EXE in the ROM) with the second PL011 on a socket, types "neocom"
into eshell, then answers the test's "SERIAL READY" with the 256 byte values and checks their echo. Passes iff the
echo matches and the console shows "[serial] PASS". Usage: test-serial-qemu.py [SECS] (default 90).
"""

import os
import socket
import subprocess
import sys
import tempfile
import time

secs = int(sys.argv[1]) if len(sys.argv) > 1 else 90
tools = os.path.dirname(os.path.abspath(__file__))
out = os.environ.get("OUT", os.path.join(tools, "../../../out/ekern-qemu"))
# not in out/: a bind-mounted /DATA on ZimaOS refuses Unix sockets
sock_path = os.path.join(tempfile.gettempdir(), "neo-uart1.sock")
if os.path.exists(sock_path):
    os.unlink(sock_path)
env = dict(
    os.environ,
    OUT=out,
    SERIAL1=f"unix:{sock_path},server=on,wait=off",
    INPUT="neocom\\r",
    INPUT_DELAY=os.environ.get("INPUT_DELAY", "25"),
)
qemu = subprocess.Popen(
    ["sh", os.path.join(tools, "run-qemu.sh"), str(secs), r"\[serial\] PASS"],
    env=env,
    stdout=subprocess.DEVNULL,
    stderr=subprocess.STDOUT,
)


def fail(why):
    print("test-serial-qemu: FAIL:", why)
    qemu.kill()
    qemu.wait()
    sys.exit(1)


deadline = time.monotonic() + 20
while not os.path.exists(sock_path):
    if qemu.poll() is not None or time.monotonic() > deadline:
        fail("no UART1 socket")
    time.sleep(0.05)
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.connect(sock_path)
s.settimeout(secs)
got = b""
while b"SERIAL READY\n" not in got:
    piece = s.recv(4096)
    if not piece:
        fail(f"UART1 closed before READY ({got[-80:]!r})")
    got += piece
payload = bytes(range(256))
s.sendall(payload)
echo = b""
while len(echo) < len(payload):
    piece = s.recv(len(payload) - len(echo))
    if not piece:
        fail(f"UART1 closed after {len(echo)} echo bytes")
    echo += piece
if echo != payload:
    fail("echo differs")
print("test-serial-qemu: 256-byte echo matches")
r = qemu.wait()
with open(os.path.join(out, "serial.log"), errors="replace") as f:
    log = f.read()
line = [l for l in log.splitlines() if "[serial]" in l]
print("\n".join(line))
sys.exit(0 if r == 0 and "[serial] PASS" in log else 1)
