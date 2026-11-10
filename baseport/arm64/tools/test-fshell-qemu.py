#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""test-fshell-qemu.py -- fshell over the debug serial port on QEMU virt (inside the build image, from the kernel tree).

Boots out/ekern-qemu (neo kernel qemu: a ROM with fshell, NeoSymbian/build) with the second PL011 on a socket and types
into eshell the command that starts fshell on it (vt100busdevcons over ECOMM unit 1). Then, through fshell_link.py:
`ls`, a SHA-256 `hash`, `ps`, a failing command's exit status, and a YMODEM round trip (push, pull) of a file holding
every byte value, checked by SHA-256 on both ends. Prints "test-fshell-qemu: PASS" and exits 0 iff all pass.
Usage: test-fshell-qemu.py [SECS] (default 240).
"""

import hashlib
import os
import random
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import fshell_link as fl

secs = int(sys.argv[1]) if len(sys.argv) > 1 else 240
tools = os.path.dirname(os.path.abspath(__file__))
out = os.environ.get("OUT", os.path.join(tools, "../../../out/ekern-qemu"))
tmp = tempfile.mkdtemp(
    prefix="neo-fshell-"
)  # not in out/: a bind-mounted /DATA on ZimaOS refuses sockets
sock_path = os.path.join(tmp, "uart1.sock")
start = "fshell --console vt100busdevcons --console-title pdd=euart1,ldd=ecomm,port=1,rate=115200,sizedetect=0\\r"
env = dict(
    os.environ,
    OUT=out,
    SERIAL1=f"unix:{sock_path},server=on,wait=off",
    INPUT=start,
    INPUT_DELAY=os.environ.get("INPUT_DELAY", "25"),
)
qemu = subprocess.Popen(
    ["sh", os.path.join(tools, "run-qemu.sh"), str(secs)],
    env=env,
    stdout=subprocess.DEVNULL,
    stderr=subprocess.STDOUT,
)


def check(ok, what):
    print(f"test-fshell-qemu: {'ok  ' if ok else 'FAIL'} {what}", flush=True)
    if not ok:
        qemu.kill()
        qemu.wait()
        sys.exit(1)


deadline = time.monotonic() + 30
while not os.path.exists(sock_path):
    check(qemu.poll() is None and time.monotonic() < deadline, "UART1 socket")
    time.sleep(0.05)
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.connect(sock_path)
stream = fl.Stream(sock.fileno())
try:
    shell = fl.Shell(stream, timeout=60, idle=30)
    shell.prompt(time.monotonic() + 120)  # fshell's first prompt: it is up and reading
    check(True, "fshell prompt on the serial port")
    r = shell.execute("ls z:\\sys\\bin\\fshell.exe")
    check(
        r.status == 0 and b"fshell.exe" in r.output,
        f"ls ({r.status}, {r.output[:60]!r})",
    )
    r = shell.execute("hash --algorithm SHA256 --input z:\\sys\\bin\\fshell.exe --hex")
    check(
        r.status == 0 and len(r.output.split()[-1]) == 64,
        f"hash ({r.status}, {r.output[-70:]!r})",
    )
    r = shell.execute("ps")
    check(r.status == 0 and b"fshell" in r.output, f"ps ({r.status})")
    r = shell.execute("ls z:\\no\\such\\file")
    check(r.status != 0, f"a failing command reports its status ({r.status})")
    data = bytes(range(256)) * 64 + random.Random(7).randbytes(
        48 * 1024
    )  # every byte value, 64 KiB
    local, back = os.path.join(tmp, "t7.bin"), os.path.join(tmp, "t7.back")
    with open(local, "wb") as f:
        f.write(data)
    sent = shell.push(local, "C:\\t7.bin", overwrite=True)
    got = shell.pull("C:\\t7.bin", back, overwrite=True)
    want = hashlib.sha256(data).hexdigest()
    check(
        sent == want == got and Path(back).read_bytes() == data,
        f"YMODEM round trip of {len(data)} bytes, SHA-256 {want}",
    )
except fl.DebugError as error:
    check(False, f"{type(error).__name__}: {error}")
finally:
    sock.close()
qemu.kill()
qemu.wait()
print("test-fshell-qemu: PASS")
