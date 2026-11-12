#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""phone.py -- drive the Pixel running the AArch64 Symbian kernel over its USB debug console (common/specific/a64_usb_acm_monitor.cpp),
with nothing but the Python standard library and the fastboot tool.

  phone.py run IMG [--cmd C]... [--log F] [--secs N] [--stay] [--no-date]
      fastboot-boot IMG (RAM only), wait for the console, save its log, run the commands, then send the phone back
      to fastboot (unless --stay) and wait until fastboot answers again
      first it sets the phone's clock to the Mac's (monitor "date"; neoui then keeps it), unless --no-date
  phone.py cmd C...          run commands on a phone already up
  phone.py shot OUT.png      the framebuffer as a PNG
  phone.py put vendor_a|vendor_b FILE   FILE onto the partition (kernel "blk put", EMMC_C=2 builds), verified
  phone.py dump PART OUT     a GPT partition by name (or FIRST:COUNT sectors), read-only, checked against the
                             kernel's SHA-256; prints the hash (docs telephony.md C0: radio partition backups)
  phone.py console [--log F] interactive (Ctrl-D ends), optionally capture continuously
  phone.py sh [COMMAND]      fshell (USBDEBUG builds): a terminal, or one command with its real exit status
  phone.py push LOCAL REMOTE [--overwrite]   one file to the phone, checked by length and SHA-256
  phone.py pull REMOTE LOCAL [--overwrite]   one file from the phone, checked the same way
  phone.py selftest          checks PNG output and console failure handling

Ports are found by USB interface (ioreg): the monitor is interface 0, fshell interface 2. Options, before or after
the verb: --device ID (serial or location, with several phones), --monitor-port PATH, --shell-port PATH; sh/push/pull
also take --timeout SECONDS (60) and --idle-timeout SECONDS (30). Quote COMMAND, or put -- before one starting with
a dash. One operation at a time owns the shell port (a lock); the kernel log stays on the monitor.

Monitor answers end with "\\n.\\n"; "shot" answers "SHOT w h\\n" and w*h*4 bytes of BGRX; "blk dump" answers "DUMP n\\n",
n*512 raw bytes and "dump sha256 HEX err E". Everything else on the line is the kernel log, streamed live.
"""

import errno
import hashlib
import os
import re
import select
import struct
import subprocess
import sys
import termios
import time
import zlib

import fshell_link

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


def find_port(secs=0, *, device=None, override=None):
    t0 = time.monotonic()
    while True:
        port = fshell_link.select_port("monitor", device=device, override=override)
        if port or time.monotonic() - t0 >= secs:
            return port
        time.sleep(1)


class Console:
    def __init__(self, port, log_path=None):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            a = termios.tcgetattr(self.fd)
            a[0] = a[1] = a[3] = 0
            a[2] |= termios.CLOCAL | termios.CREAD
            termios.tcsetattr(self.fd, termios.TCSANOW, a)
            self.sink = open(log_path, "ab", buffering=0) if log_path else None
        except BaseException:
            os.close(self.fd)
            raise
        self.log = bytearray()

    def capture(self, data):
        self.log += data
        if len(self.log) > 128 * 1024:
            del self.log[:-128 * 1024]
        if self.sink:
            self.sink.write(data)
        return data

    def chunk(self, timeout, size=1 << 16):
        try:
            ready, _, _ = select.select([self.fd], [], [], max(0, timeout))
            if not ready:
                return b""
            data = os.read(self.fd, size)
        except OSError as e:
            if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EINTR):
                return b""
            raise ConnectionError(f"console disconnected: {e}") from e
        if not data:
            raise ConnectionError("console disconnected (EOF)")
        return self.capture(data)

    def read(self, secs):
        out, deadline = bytearray(), time.monotonic() + secs
        while time.monotonic() < deadline:
            out += self.chunk(min(0.2, deadline - time.monotonic()))
        return bytes(out)

    def read_until(self, marker, secs):
        out, deadline = bytearray(), time.monotonic() + secs
        while marker not in out:
            if time.monotonic() >= deadline:
                raise TimeoutError(f"console incomplete reply: {len(out)} bytes, missing {marker!r}")
            out += self.chunk(min(0.2, deadline - time.monotonic()))
        return bytes(out)

    def read_exact(self, n, secs):
        out, deadline = bytearray(), time.monotonic() + secs
        while len(out) < n:
            if time.monotonic() >= deadline:
                raise TimeoutError(f"console incomplete transfer: {len(out)} of {n} bytes")
            out += self.chunk(min(0.2, deadline - time.monotonic()), n - len(out))
        return bytes(out)

    def send(self, line):
        data, deadline = memoryview(line.encode() + b"\r"), time.monotonic() + 20
        while data:
            if time.monotonic() >= deadline:
                raise TimeoutError("console write timed out")
            try:
                _, ready, _ = select.select([], [self.fd], [], 0.2)
                if ready:
                    n = os.write(self.fd, data)
                    if not n:
                        raise ConnectionError("console write returned zero")
                    data = data[n:]
            except OSError as e:
                if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EINTR):
                    raise ConnectionError(f"console write disconnected: {e}") from e

    def cmd(self, line, secs=20):
        self.read(0.3)
        # Log only the verb: command arguments may contain private user input.
        verb = line.split()[0] if line.split() else "empty"
        self.capture(f"\n# host {time.time():.3f} command {verb}\n".encode())
        self.send(line)
        out = self.read_until(END, secs)
        return out[:out.rfind(END)]

    def put(self, first, data, secs=600):
        """data (padded to whole sectors) onto the eMMC from sector first: "blk put", which the kernel allows only
        inside vendor_a/vendor_b in an EMMC_C=2 build; returns the SHA-256, checked against the kernel's"""
        data = data + bytes(-len(data) % 512)
        n = len(data) // 512
        self.read(0.3)
        self.send(f"blk put {first} {n}")
        head = self.read_until(b"\n", 10)
        if f"PUT {n}".encode() not in head:
            raise SystemExit(f"put: refused ({head[-200:]!r})")
        view, t0 = memoryview(data), time.time()
        while view:
            _, w, _ = select.select([], [self.fd], [], 5)
            if not w:
                raise SystemExit(f"put: stalled with {len(view)} bytes left")
            try:
                view = view[os.write(self.fd, view[: 1 << 16]) :]
            except BlockingIOError:
                pass
            done = len(data) - len(view)
            if done % (4 << 20) < (1 << 16):
                print(f"  {done >> 20} MB in {time.time() - t0:.0f} s", file=sys.stderr)
        # the command's own answer ended right after "PUT n": wait for the trailer line itself
        tail = self.read_until(b"put sha256", secs)
        tail += self.read_until(b"\n", 10)
        m = re.search(rb"put sha256 ([0-9a-f]{64}) err (-?\d+) written (\d+)", tail)
        if not m:
            raise SystemExit(f"put: no trailer ({tail[-200:]!r})")
        want = hashlib.sha256(data).hexdigest()
        if m[1].decode() != want:
            raise SystemExit(f"put: the phone got sha256 {m[1].decode()}, sent {want}")
        if int(m[2]) or int(m[3]) != len(data):
            raise SystemExit(f"put: error {int(m[2])}, {int(m[3])} of {len(data)} bytes written and verified")
        return want

    def shot(self, png, secs=120):
        self.read(0.3)
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

    def parts(self):
        """name -> (first, last) from the monitor's "blk gpt" """
        out = self.cmd("blk gpt").decode(errors="replace")
        return {m[1]: (int(m[2]), int(m[3])) for m in re.finditer(r"^\d+ (\S+) (\d+)\.\.(\d+)", out, re.M)}

    def dump(self, first, count, path, idle=20):
        """count sectors from first into path; returns the SHA-256 hex, after checking it against the kernel's"""
        self.read(0.3)
        self.send(f"blk dump {first} {count}")
        head = self.read_until(b"\n", 10)
        i = head.find(b"DUMP ")
        if i < 0:
            raise SystemExit(f"dump: no header ({head[-200:]!r})")
        line, rest = head[i:].split(b"\n", 1)
        if int(line.split()[1]) != count:
            raise SystemExit(f"dump: header {line!r}, asked {count}")
        total, h, got, t0, last = count * 512, hashlib.sha256(), 0, time.time(), time.time()
        with open(path, "wb") as f:
            buf = rest
            while True:
                take = buf[: total - got]
                f.write(take)
                h.update(take)
                got += len(take)
                if got == total:
                    tail = buf[len(take) :]
                    break
                if time.time() - last > idle:
                    raise SystemExit(f"dump: stalled at {got} of {total} bytes")
                buf = b""
                buf = self.chunk(0.5, 1 << 20)
                if buf:
                    last = time.time()
                if got >> 23 != (got + len(buf)) >> 23:
                    print(f"  {(got + len(buf)) >> 20} of {total >> 20} MB", file=sys.stderr)
        tail += self.read_until(END, 30)
        m = re.search(rb"dump sha256 ([0-9a-f]{64}) err (-?\d+)", tail)
        if not m:
            raise SystemExit(f"dump: no trailer ({tail[-200:]!r})")
        if int(m[2]):
            raise SystemExit(f"dump: read error {int(m[2])} on the phone (zeros in the file)")
        if m[1].decode() != h.hexdigest():
            raise SystemExit(f"dump: SHA-256 mismatch, phone {m[1].decode()} Mac {h.hexdigest()}")
        secs = time.time() - t0
        print(f"  {total >> 20} MB in {secs:.0f} s ({total / secs / 1e6:.1f} MB/s)", file=sys.stderr)
        return h.hexdigest()

    def close(self):
        try:
            os.close(self.fd)
        finally:
            if self.sink:
                self.sink.close()


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


def _read_file(path):
    with open(path, "rb") as f:
        return f.read()


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
    # Exercise actual fragmented transport, timeout and EOF, preserving partial bytes.
    import socket
    import tempfile
    import threading
    with tempfile.TemporaryDirectory() as tmp:
        def pair():
            a, b = socket.socketpair()
            a.setblocking(False)
            c = Console.__new__(Console)
            c.fd, c.log = a.detach(), bytearray()
            c.sink = open(os.path.join(tmp, "partial.log"), "wb", buffering=0)
            return c, b

        c, peer = pair()
        def fragmented():
            peer.sendall(b"reply\n.")
            time.sleep(0.01)
            peer.sendall(b"\n")
        writer = threading.Thread(target=fragmented)
        writer.start()
        assert c.read_until(END, 1) == b"reply\n.\n"
        writer.join()
        c.close()
        peer.close()

        c, peer = pair()
        peer.sendall(b"partial")
        try:
            c.read_until(END, 0.03)
            raise AssertionError("missing marker accepted")
        except TimeoutError:
            assert c.log == b"partial"
            assert _read_file(c.sink.name) == b"partial"
        c.close()
        peer.close()

        c, peer = pair()
        peer.sendall(b"before disconnect")
        peer.close()
        try:
            c.read_until(END, 1)
            raise AssertionError("disconnect accepted")
        except ConnectionError:
            assert c.log == b"before disconnect"
        c.close()
    print("selftest ok: PNG, fragmented reply, timeout, disconnect, partial capture")


def _main(argv):
    connection, argv = fshell_link.connection_options(argv)
    if not argv or argv[0] in ("-h", "--help"):
        print(__doc__)
        return 0
    verb, args = argv[0], argv[1:]
    if verb in ("sh", "push", "pull"):
        return fshell_link.run_cli(verb, args, connection)
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
        port = find_port(60, device=connection.device, override=connection.monitor_port)
        if not port:
            raise SystemExit(
                "no debug console appeared (the image's AUTOFASTBOOT, if any, will return the phone)"
            )
        con = Console(port, log)
        con.read(secs)
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
        if not stay:
            con.send("fastboot")
            time.sleep(1)
        con.close()
        if not stay and not wait_fastboot(60):
            raise SystemExit("the phone did not come back to fastboot")
        return 0
    if verb not in ("cmd", "dump", "put", "shot", "console"):
        raise fshell_link.DebugError(f"unknown command {verb!r}; use --help")
    port = find_port(5, device=connection.device, override=connection.monitor_port)
    if not port:
        raise SystemExit("no debug console (/dev/cu.usbmodem*)")
    log_path = None
    if verb == "console" and args:
        if len(args) != 2 or args[0] != "--log":
            raise SystemExit("console: expected --log FILE")
        log_path = args[1]
    con = Console(port, log_path)
    if verb == "cmd":
        for c in args:
            print("== " + c)
            print(con.cmd(c).decode(errors="replace").strip())
    elif verb == "dump":
        what, out = args
        if ":" in what:
            first, count = map(int, what.split(":"))
        else:
            parts = con.parts()
            if what not in parts:
                raise SystemExit(f"no partition {what!r} (have: {' '.join(sorted(parts))})")
            first, count = parts[what][0], parts[what][1] - parts[what][0] + 1
        print(f"{con.dump(first, count, out)}  {os.path.basename(out)}")
    elif verb == "put":
        what, path = args
        parts = con.parts()
        if what not in ("vendor_a", "vendor_b") or what not in parts:
            raise SystemExit("put: only vendor_a or vendor_b")
        data = open(path, "rb").read()
        first, last = parts[what]
        if len(data) > (last - first + 1) * 512:
            raise SystemExit(f"put: {len(data)} bytes don't fit in {what}")
        print(f"{con.put(first, data)}  {os.path.basename(path)} -> {what}")
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


def main(argv):
    try:
        return _main(argv)
    except fshell_link.RemoteError as error:
        if error.output:
            sys.stderr.buffer.write(error.output)
        print(f"phone: {error}", file=sys.stderr)
        return 1
    except (fshell_link.DebugError, OSError, UnicodeError, termios.error) as error:
        print(f"phone: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except (ConnectionError, TimeoutError, OSError) as e:
        print(f"phone: {e}", file=sys.stderr)
        sys.exit(1)
