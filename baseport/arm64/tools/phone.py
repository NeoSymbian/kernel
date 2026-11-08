#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""phone.py -- drive the Pixel running the AArch64 Symbian kernel over its USB debug console (board/qcom_dwc3_acm.cpp),
with nothing but the Python standard library and the fastboot tool.

  phone.py run IMG [--cmd C]... [--log F] [--secs N] [--stay] [--no-date]
      fastboot-boot IMG (RAM only), wait for the console, save its log, run the commands, then send the phone back
      to fastboot (unless --stay) and wait until fastboot answers again
      first it sets the phone's clock to the Mac's (monitor "date"; neoui then keeps it), unless --no-date
  phone.py cmd C...          run commands on a phone already up
  phone.py shot OUT.png      the framebuffer as a PNG
  phone.py console           interactive (Ctrl-D ends)
  phone.py selftest          checks the PNG writer

Monitor answers end with "\\n.\\n"; "shot" answers "SHOT w h\\n" and w*h*4 bytes of BGRX. Everything else on the line is
the kernel log, streamed live.
"""

import glob
import os
import select
import struct
import subprocess
import sys
import termios
import time
import zlib

END = b"\n.\n"


def fastboot(*args, timeout=30):
    try:
        return subprocess.run(
            ["fastboot", *args], capture_output=True, text=True, timeout=timeout, check=False
        )
    except subprocess.TimeoutExpired:
        return None


def wait_fastboot(secs):
    t0 = time.time()
    while time.time() - t0 < secs:
        r = fastboot("devices", timeout=5)
        if r and r.stdout.strip():
            return True
        time.sleep(2)
    return False


def find_port(secs=0):
    t0 = time.time()
    while True:
        ports = sorted(glob.glob("/dev/cu.usbmodem*"))
        if ports or time.time() - t0 >= secs:
            return ports[0] if ports else None
        time.sleep(1)


class Console:
    def __init__(self, port):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        a = termios.tcgetattr(self.fd)
        a[0] = a[1] = a[3] = 0  # raw: no input/output/local processing
        a[2] |= termios.CLOCAL | termios.CREAD
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        self.log = bytearray()

    def read(self, secs):
        out, t0 = bytearray(), time.time()
        while time.time() - t0 < secs:
            r, _, _ = select.select([self.fd], [], [], 0.2)
            if r:
                try:
                    out += os.read(self.fd, 1 << 16)
                except (BlockingIOError, OSError):
                    pass
        return bytes(out)

    def read_until(self, marker, secs):
        out, t0 = bytearray(), time.time()
        while marker not in out and time.time() - t0 < secs:
            r, _, _ = select.select([self.fd], [], [], 0.2)
            if r:
                try:
                    out += os.read(self.fd, 1 << 16)
                except (BlockingIOError, OSError):
                    pass
        return bytes(out)

    def read_exact(self, n, secs):
        out, t0 = bytearray(), time.time()
        while len(out) < n and time.time() - t0 < secs:
            r, _, _ = select.select([self.fd], [], [], 0.2)
            if r:
                try:
                    out += os.read(self.fd, n - len(out))
                except (BlockingIOError, OSError):
                    pass
        return bytes(out)

    def send(self, line):
        os.write(self.fd, line.encode() + b"\r")

    def cmd(self, line, secs=20):
        self.log += self.read(0.3)  # log text that arrived before the command
        self.send(line)
        out = self.read_until(END, secs)
        return out[: out.rfind(END)] if END in out else out

    def shot(self, png, secs=120):
        self.log += self.read(0.3)
        self.send("shot")
        head = self.read_until(b"\n", 10)
        i = head.find(b"SHOT ")
        if i < 0:
            raise SystemExit(f"shot: no header ({head[-200:]!r})")
        line, rest = head[i:].split(b"\n", 1)
        w, h = map(int, line.split()[1:3])
        data = rest + self.read_exact(w * h * 4 - len(rest), secs)
        if len(data) < w * h * 4:
            raise SystemExit(f"shot: {len(data)} of {w * h * 4} bytes")
        write_png(png, w, h, data[: w * h * 4])
        return w, h

    def close(self):
        os.close(self.fd)


def write_png(path, w, h, bgrx):
    rows = b"".join(
        b"\0"
        + bytes(
            c
            for px in range(w)
            for c in (
                bgrx[(y * w + px) * 4 + 2],
                bgrx[(y * w + px) * 4 + 1],
                bgrx[(y * w + px) * 4],
            )
        )
        for y in range(h)
    )

    def chunk(t, d):
        return (
            struct.pack(">I", len(d))
            + t
            + d
            + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
        )

    with open(path, "wb") as f:
        f.write(
            b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(rows, 6))
            + chunk(b"IEND", b"")
        )


def selftest():
    p = (
        "/tmp/phone-selftest.png"
        if not os.environ.get("TMPDIR")
        else os.path.join(os.environ["TMPDIR"], "phone-selftest.png")
    )
    write_png(p, 2, 1, bytes([0, 0, 255, 0, 255, 0, 0, 0]))  # red, blue in BGRX
    with open(p, "rb") as f:
        d = f.read()
    assert d[:8] == b"\x89PNG\r\n\x1a\n"
    idat = d[d.find(b"IDAT") + 4 : d.find(b"IEND") - 8]
    assert zlib.decompress(idat) == b"\0" + bytes([255, 0, 0, 0, 0, 255]), "pixel order"
    print("selftest ok")


def main(argv):
    if not argv or argv[0] in ("-h", "--help"):
        print(__doc__)
        return 0
    verb, args = argv[0], argv[1:]
    if verb == "selftest":
        selftest()
        return 0
    if verb == "run":
        img, cmds, log, secs, stay, date = args[0], [], None, 5.0, False, True
        i = 1
        while i < len(args):
            if args[i] == "--cmd":
                cmds.append(args[i + 1])
                i += 2
            elif args[i] == "--log":
                log = args[i + 1]
                i += 2
            elif args[i] == "--secs":
                secs = float(args[i + 1])
                i += 2
            elif args[i] == "--stay":
                stay = True
                i += 1
            elif args[i] == "--no-date":
                date = False
                i += 1
            else:
                raise SystemExit("unknown option " + args[i])
        if not wait_fastboot(60):
            raise SystemExit("no phone in fastboot")
        r = fastboot("getvar", "product")
        if not r or "bonito" not in (r.stdout + r.stderr):
            raise SystemExit(f"not the bonito: {r and r.stderr!r}")
        r = fastboot("boot", img, timeout=60)
        if not r or r.returncode:
            raise SystemExit(f"fastboot boot failed: {r and r.stderr}")
        port = find_port(60)
        if not port:
            raise SystemExit(
                "no debug console appeared (the image's AUTOFASTBOOT, if any, will return the phone)"
            )
        con = Console(port)
        con.log += con.read(secs)
        if date:
            c = f"date {int(time.time())} {time.localtime().tm_gmtoff}"
            print("== " + c)
            print(con.cmd(c).decode(errors="replace").strip())
        for c in cmds:
            if c.startswith("shot"):
                out = c.split(None, 1)[1] if " " in c else "shot.png"
                w, h = con.shot(out)
                print(f"== shot -> {out} {w}x{h}")
            else:
                print("== " + c)
                print(con.cmd(c).decode(errors="replace").strip())
        if log:
            with open(log, "wb") as f:
                f.write(con.log)
        if not stay:
            con.send("fastboot")
            time.sleep(1)
        con.close()
        if not stay and not wait_fastboot(60):
            raise SystemExit("the phone did not come back to fastboot")
        return 0
    port = find_port(5)
    if not port:
        raise SystemExit("no debug console (/dev/cu.usbmodem*)")
    con = Console(port)
    if verb == "cmd":
        for c in args:
            print("== " + c)
            print(con.cmd(c).decode(errors="replace").strip())
    elif verb == "shot":
        out = args[0] if args else "shot.png"
        w, h = con.shot(out)
        print(f"{out} {w}x{h}")
    elif verb == "console":
        while True:
            r, _, _ = select.select([sys.stdin, con.fd], [], [])
            if sys.stdin in r:
                line = sys.stdin.readline()
                if not line:
                    break
                con.send(line.rstrip("\n"))
            if con.fd in r:
                sys.stdout.write(con.read(0.05).decode(errors="replace"))
                sys.stdout.flush()
    con.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
