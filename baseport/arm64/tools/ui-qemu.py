#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""ui-qemu.py [SECS] -- boot out/ekern-qemu (a UI ROM: neoui's SYSSTART.EXE) on QEMU virt and drive it by script.

The UI CI on virt (docs user-side-smp.md US6b; NeoSymbian/neoui symbian/taps.sh writes the script). run-qemu.sh starts
the machine with a QMP socket; the script runs once the serial log shows READY. Event-driven like symbian-qemu's
run-syborg (Syborg's runner, whose contract this keeps): it waits on the serial log, not on fixed sleeps.

Environment:
  MONITOR      the script, ';'-separated:
                 expect TEXT   wait until TEXT appears in the serial log after the previous expect's match
                               (EXPECT_MAX seconds, default 30)
                 wait N        sleep N seconds
                 abs X Y       move the tablet to X, Y in QEMU's absolute range 0..0x7fff (QMP input-send-event;
                               virt_assp/virtio_input.cpp maps v to v * size / 0x8000 pixels)
                 anything else: an HMP command (mouse_button 1, sendkey a)
               every abs and HMP step comes MON_GAP seconds after the previous one (default 0.1, as the Syborg job)
  READY        regex in the serial log that means "ready for input" (default: neoui: boot OK), READY_SETTLE seconds more
               (default 3)
  FB           the screen as a PNG at the end and when the run fails (default $OUT/fb.png; empty: none)
  FAULT        regex that fails the run (default: kernel faults, kernel panics, Rust panics)
  LINGER       seconds to run after the script (default 2)
  OUT, VNC, SMP, DISK   as run-qemu.sh (VNC=:0 lets a person watch, or drive it by hand without MONITOR)

Without MONITOR the machine runs for SECS (default 300) with QMP and, if set, VNC. Exit status: 0; 3 when FAULT
matched; 124 when READY, an expect or the script's time ran out. `ui-qemu.py selftest` checks the parser.
"""

import json
import os
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import time

env = os.environ.get
FAULT = r"K::Fault|Kern::Fault|\[EKERN\] panic|rust panic"


def parse(script):
    """The script's steps: (kind, argument).

    >>> parse("expect neoui: tap from Some((90, 37)); wait 1;abs 2753 18447 ; mouse_button 1")
    [('expect', 'neoui: tap from Some((90, 37))'), ('wait', 1.0), ('abs', (2753, 18447)), ('hmp', 'mouse_button 1')]
    >>> parse("abs 40000 1")
    Traceback (most recent call last):
    ValueError: abs: 0..32767, not 'abs 40000 1'
    """
    steps = []
    for line in (s.strip() for s in script.split(";")):
        if not line:
            continue
        if line.startswith("expect "):
            steps.append(("expect", line[len("expect ") :]))
        elif line.startswith("wait "):
            steps.append(("wait", float(line[len("wait ") :])))
        elif line.startswith("abs "):
            x, y = (int(v) for v in line.split()[1:3])
            if not (0 <= x <= 0x7FFF and 0 <= y <= 0x7FFF):
                raise ValueError(f"abs: 0..32767, not {line!r}")
            steps.append(("abs", (x, y)))
        else:
            steps.append(("hmp", line))
    return steps


def pixel(v, size):
    """The pixel virtio_input.cpp makes of an absolute value; taps.sh's (2p + 1) * 16384 / size lands on p.

    >>> [pixel((2 * p + 1) * 16384 // 539, 539) for p in (0, 90, 270, 538)]
    [0, 90, 270, 538]
    >>> [pixel((2 * p + 1) * 16384 // 1079, 1079) for p in (0, 73, 1037, 1078)]
    [0, 73, 1037, 1078]
    """
    return v * size // 0x8000


def ppm_rows(data):
    r"""(width, height, row length, pixels) of QEMU's screendump PPM.

    shortcut: QEMU 11.1's ppm_save writes each row at pixman's 4-byte stride, so a width whose 3 * width is not a
    multiple of 4 (539: 1617 bytes) gets padded rows; the row length comes from the file size. Drop it once QEMU
    writes width * 3 bytes.
    >>> ppm_rows(b"P6\n3 2\n255\n" + bytes(12) * 2)[:3]
    (3, 2, 12)
    >>> ppm_rows(b"P6\n4 2\n255\n" + bytes(12) * 2)[:3]
    (4, 2, 12)
    """
    m = re.match(rb"P6\s+(\d+)\s+(\d+)\s+255\s", data)
    if not m:
        raise ValueError("not a P6 PPM")
    w, h = int(m[1]), int(m[2])
    pixels = data[m.end() :]
    return w, h, len(pixels) // h, pixels


def save_ppm(ppm, png, image):
    with open(ppm, "rb") as f:
        w, h, row, pixels = ppm_rows(f.read())
    image.frombytes("RGB", (w, h), pixels, "raw", "RGB", row).save(png)


class Qmp:
    """A minimal QMP client: one command at a time, events skipped."""

    def __init__(self, path, deadline):
        while True:
            try:
                self.sock = socket.socket(socket.AF_UNIX)
                self.sock.connect(path)
                break
            except OSError:
                if time.time() > deadline:
                    raise
                time.sleep(0.05)
        self.file = self.sock.makefile("rwb")
        self._recv()  # greeting
        self.cmd("qmp_capabilities")

    def _recv(self):
        while True:
            line = self.file.readline()
            if not line:
                raise EOFError("QEMU closed QMP")
            msg = json.loads(line)
            if "event" not in msg:
                return msg

    def cmd(self, name, **args):
        self.file.write(
            json.dumps({"execute": name, "arguments": args}).encode() + b"\n"
        )
        self.file.flush()
        msg = self._recv()
        if "error" in msg:
            raise RuntimeError(f"{name}: {msg['error'].get('desc')}")
        return msg.get("return")

    def hmp(self, line):
        return self.cmd("human-monitor-command", **{"command-line": line})


class Serial:
    """Follows the serial log run-qemu.sh writes."""

    def __init__(self, path):
        self.path, self.text = path, ""

    def poll(self):
        try:
            with open(self.path, "rb") as f:
                f.seek(len(self.text.encode("utf-8", "surrogateescape")))
                self.text += f.read().decode("utf-8", "surrogateescape")
        except FileNotFoundError:
            pass

    def wait(self, pattern, start, until):
        """The end of `pattern`'s first match after offset `start`, or None at `until`."""
        rx = re.compile(pattern)
        while time.time() < until:
            self.poll()
            m = rx.search(self.text, start)
            if m:
                return m.end()
            time.sleep(0.05)
        return None


def main():
    if sys.argv[1:] == ["selftest"]:
        import doctest

        failed = doctest.testmod().failed
        print("ui-qemu selftest:", "FAILED" if failed else "ok")
        return 1 if failed else 0
    if sys.argv[1:2] in (["-h"], ["--help"]):
        print(__doc__)
        return 0
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    tools = os.path.dirname(os.path.abspath(__file__))
    out = env("OUT", os.path.join(tools, "../../../out/ekern-qemu"))
    steps = parse(env("MONITOR", ""))
    ready = env("READY", r"neoui: boot OK")
    fault = env("FAULT", FAULT)
    fb = env("FB", os.path.join(out, "fb.png"))
    tmp = tempfile.mkdtemp(
        prefix="neo-ui-"
    )  # not in out/: a bind-mounted directory may refuse sockets
    qmp_path = os.path.join(tmp, "qmp.sock")
    proc = subprocess.Popen(
        ["sh", os.path.join(tools, "run-qemu.sh"), str(int(secs) + 30)],
        env=dict(os.environ, OUT=out, QMP=qmp_path),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )
    start = time.time()
    deadline = start + secs
    serial = Serial(os.path.join(out, "serial.log"))
    qmp, status = None, 0

    def shot():
        # QEMU's PPM (this QEMU has no libpng), as a PNG through PIL (the build image has it)
        if fb and qmp:
            ppm = os.path.join(tmp, "fb.ppm")
            qmp.cmd("screendump", filename=ppm)
            try:
                from PIL import Image
            except ImportError:  # outside the build image: keep the PPM
                shutil.copyfile(ppm, os.path.splitext(fb)[0] + ".ppm")
                return
            save_ppm(ppm, fb, Image)

    try:
        qmp = Qmp(qmp_path, start + 30)
        if not steps:
            while proc.poll() is None and time.time() < deadline:
                time.sleep(0.2)
        else:
            pos = serial.wait(ready, 0, deadline)
            if pos is None:
                raise TimeoutError(f"READY ({ready}) not seen")
            time.sleep(float(env("READY_SETTLE", "3")))
            print(f"ui-qemu: ready after {time.time() - start:.1f} s", flush=True)
            gap = float(env("MON_GAP", "0.1"))
            for kind, arg in steps:
                if re.search(fault, serial.text):
                    break
                if kind == "expect":
                    end = serial.wait(
                        re.escape(arg),
                        pos,
                        min(deadline, time.time() + float(env("EXPECT_MAX", "30"))),
                    )
                    if end is None:
                        raise TimeoutError(
                            f"expect: {arg!r} did not appear in the serial log"
                        )
                    pos = end
                elif kind == "wait":
                    time.sleep(arg)
                elif kind == "abs":
                    time.sleep(gap)  # paced like Syborg's mouse_move lines: swipes keep their speed
                    qmp.cmd(
                        "input-send-event",
                        events=[
                            {"type": "abs", "data": {"axis": "x", "value": arg[0]}},
                            {"type": "abs", "data": {"axis": "y", "value": arg[1]}},
                        ],
                    )
                else:
                    time.sleep(gap)
                    qmp.hmp(arg)
            time.sleep(float(env("LINGER", "2")))
            if time.time() > deadline:
                raise TimeoutError(f"the script did not finish in {secs:.0f} s")
        serial.poll()
        m = re.search(fault, serial.text)
        if m:
            print(f"ui-qemu: fault: {m[0]}", file=sys.stderr)
            status = 3
        shot()
    except TimeoutError as e:
        print(f"ui-qemu: {e}", file=sys.stderr)
        status = 124
        try:
            shot()  # the screen of the failed run
        except (OSError, RuntimeError, EOFError) as e:
            print(f"ui-qemu: no screenshot: {e}", file=sys.stderr)
    finally:
        try:
            if qmp:
                qmp.cmd("quit")
        except (OSError, RuntimeError, EOFError):
            pass  # QEMU already gone
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(tmp, ignore_errors=True)
    print(f"ui-qemu: {time.time() - start:.1f} s, status {status}", flush=True)
    return status


if __name__ == "__main__":
    sys.exit(main())
