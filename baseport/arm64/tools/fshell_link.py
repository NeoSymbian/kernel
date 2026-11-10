# SPDX-License-Identifier: Apache-2.0
"""fshell_link -- talk to fshell over a byte stream (a serial port, a QEMU socket): commands framed by nonce
markers with their real exit status, and binary-safe YMODEM file transfer checked by SHA-256 (fshell's own
"ymodem receive --sha256" and "hash --algorithm SHA256 --hex", NeoSymbian/fshell). Standard library only.

    Shell(Stream(fd)).execute("ls")          -> Result(status, output)
    Shell(Stream(fd)).push(local, "C:\\x")   / .pull("C:\\x", local)

Users: tools/test-fshell-qemu.py (QEMU's second serial port); phone.py later (roadmap T9).
"""

import binascii
import hashlib
import io
import os
import re
import secrets
import select
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

SOH, STX, EOT, ACK, NAK, CAN, CRC = 1, 2, 4, 6, 21, 24, 67
MAX_OUTPUT = 16 * 1024 * 1024
MAX_LINE = 1024 * 1024
USB_VENDOR, USB_PRODUCT = 0x1209, 0x0001
MAX_FILE_SIZE = (
    1 << 31
) - 1  # fshell's native RFile sender/receiver use signed 32-bit sizes


class DebugError(Exception):
    """A failed local operation or serial protocol, never a successful command."""


class BusyError(DebugError):
    pass


class LinkError(DebugError):
    pass


class DeadlineExpired(DebugError):
    pass


class RemoteError(DebugError):
    def __init__(self, status, output=b""):
        super().__init__(f"native command exited {status}")
        self.status, self.output = status, output


class Stream:
    """Buffered, deadline-bounded POSIX byte stream; also usable with PTY test peers."""

    def __init__(self, fd):
        self.fd, self.buffer = fd, bytearray()

    def write(self, data, deadline):
        view = memoryview(data)
        while view:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise DeadlineExpired(
                    "serial write made no progress before its deadline"
                )
            try:
                if not select.select([], [self.fd], [], remaining)[1]:
                    raise DeadlineExpired("serial write timed out")
                n = os.write(self.fd, view)
            except BlockingIOError:
                continue
            except OSError as error:
                raise LinkError(f"serial write failed: {error}") from error
            if n == 0:
                raise LinkError("serial link closed during write")
            view = view[n:]

    def read(self, count, deadline):
        while not self.buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise DeadlineExpired(
                    "serial peer made no progress before its deadline"
                )
            try:
                if not select.select([self.fd], [], [], remaining)[0]:
                    raise DeadlineExpired("serial read timed out")
                chunk = os.read(self.fd, max(count, 4096))
            except BlockingIOError:
                continue
            except OSError as error:
                raise LinkError(f"serial read failed: {error}") from error
            if not chunk:
                raise LinkError("serial link closed")
            self.buffer.extend(chunk)
        result = bytes(self.buffer[:count])
        del self.buffer[:count]
        return result

    def exact(self, count, deadline):
        out = bytearray()
        while len(out) < count:
            out.extend(self.read(count - len(out), deadline))
        return bytes(out)

    def byte(self, deadline):
        value = self.exact(1, deadline)[0]
        if value == CAN:
            raise DebugError("peer cancelled YMODEM transfer")
        return value

    def line(self, deadline):
        out = bytearray()
        while True:
            value = self.exact(1, deadline)
            out.extend(value)
            if value == b"\n":
                return bytes(out)
            if len(out) > MAX_LINE:
                raise DebugError(
                    "terminal line exceeds the client limit; use file transfer for large data"
                )


def packet(block, payload, size=None):
    size = size or (128 if len(payload) <= 128 else 1024)
    if size not in (128, 1024) or len(payload) > size:
        raise ValueError("invalid YMODEM block size")
    data = payload.ljust(size, b"\x1a")
    crc = binascii.crc_hqx(data, 0)
    return (
        bytes((SOH if size == 128 else STX, block & 255, 255 - (block & 255)))
        + data
        + crc.to_bytes(2, "big")
    )


def metadata(name, size):
    encoded = name.encode("utf-8")
    validate_name(encoded)
    if not 0 <= size <= MAX_FILE_SIZE:
        raise DebugError("file size exceeds native fshell's 2 GiB limit")
    body = encoded + b"\0" + str(size).encode("ascii") + b"\0"
    if len(body) > 1024:
        raise DebugError("UTF-8 filename metadata exceeds a YMODEM block")
    return body.ljust(128 if len(body) <= 128 else 1024, b"\0")


def validate_name(encoded):
    try:
        name = encoded.decode("utf-8")
    except UnicodeDecodeError as error:
        raise DebugError("YMODEM filename is not valid UTF-8") from error
    if (
        not name
        or name in (".", "..")
        or any(c in name for c in "/\\:\0")
        or any(ord(c) < 32 or ord(c) == 127 for c in name)
        or len(name.encode("utf-16-le")) // 2 > 256
    ):
        raise DebugError("invalid YMODEM filename")
    return name


def parse_metadata(body):
    if not body or body[0] == 0:
        return None
    if b"\0" not in body:
        raise DebugError("unterminated YMODEM filename")
    name, tail = body.split(b"\0", 1)
    name = validate_name(name)
    size = tail.split(b"\0", 1)[0].split(b" ", 1)[0]
    if not size or len(size) > 10 or not size.isdigit() or int(size) > MAX_FILE_SIZE:
        raise DebugError("invalid YMODEM file size")
    return name, int(size)


class Ymodem:
    def __init__(self, stream, idle=30.0, retries=10):
        self.stream, self.idle, self.retries = stream, idle, retries
        self.last_progress = time.monotonic()

    def progress(self):
        self.last_progress = time.monotonic()

    def deadline(self):
        return self.last_progress + self.idle

    def cancel(self):
        try:
            self.stream.write(bytes((CAN, CAN)), time.monotonic() + 0.2)
        except DebugError:
            pass

    def expect(self, wanted):
        for _ in range(self.retries):
            value = self.stream.byte(self.deadline())
            if value == wanted:
                self.progress()
                return
            if value not in (ACK, NAK, CRC):
                raise DebugError(f"unexpected YMODEM control byte 0x{value:02x}")
        raise DebugError("YMODEM peer did not enter the requested state")

    def acknowledged(self, wire):
        for _ in range(self.retries):
            self.stream.write(wire, self.deadline())
            try:
                answer = self.stream.byte(min(self.deadline(), time.monotonic() + 1.0))
            except DeadlineExpired:
                if time.monotonic() >= self.deadline():
                    raise
                continue
            if answer == ACK:
                self.progress()
                return
            if answer not in (NAK, CRC):
                raise DebugError(f"unexpected YMODEM acknowledgement 0x{answer:02x}")
        raise DebugError("YMODEM retry limit reached")

    def send(self, file, name, size):
        try:
            self.expect(CRC)
            self.acknowledged(packet(0, metadata(name, size)))
            self.expect(CRC)
            remaining, block = size, 1
            while remaining:
                data = file.read(min(1024, remaining))
                if not data:
                    raise DebugError("source file shortened during transfer")
                self.acknowledged(packet(block, data, 1024))
                remaining -= len(data)
                block = (block + 1) & 255
            self.acknowledged(bytes((EOT,)))
            self.expect(CRC)
            self.acknowledged(packet(0, bytes(128), 128))
        except BaseException:
            self.cancel()
            raise

    def receive_packet(self, first):
        if first not in (SOH, STX):
            raise DebugError(f"unexpected YMODEM packet byte 0x{first:02x}")
        size = 128 if first == SOH else 1024
        rest = self.stream.exact(size + 4, self.deadline())
        block, complement, body, crc = rest[0], rest[1], rest[2:-2], rest[-2:]
        if block ^ complement != 255 or binascii.crc_hqx(body, 0) != int.from_bytes(
            crc, "big"
        ):
            return None
        return block, body

    def receive(self, file):
        self.stream.write(bytes((CRC,)), self.deadline())
        expected, previous, remaining, info, failures = 0, None, None, None, 0
        previous_header = False
        ending = False
        try:
            while True:
                try:
                    first = self.stream.byte(
                        min(self.deadline(), time.monotonic() + 1.0)
                    )
                except DeadlineExpired:
                    if time.monotonic() >= self.deadline():
                        raise
                    self.stream.write(
                        bytes((CRC if info is None or ending else NAK,)),
                        self.deadline(),
                    )
                    failures += 1
                    if failures >= self.retries:
                        raise DebugError("YMODEM retry limit reached")
                    continue
                if first == EOT:
                    if info is None or remaining != 0:
                        raise DebugError(
                            "YMODEM ended before the advertised file length"
                        )
                    self.stream.write(bytes((ACK, CRC)), self.deadline())
                    if ending:
                        failures += 1
                        if failures >= self.retries:
                            raise DebugError(
                                "YMODEM end-of-transfer retry limit reached"
                            )
                        continue
                    ending, expected, previous = True, 0, None
                    self.progress()
                    continue
                received = self.receive_packet(first)
                if received is None:
                    self.stream.write(bytes((NAK,)), self.deadline())
                    failures += 1
                    if failures >= self.retries:
                        raise DebugError("YMODEM CRC retry limit reached")
                    continue
                block, body = received
                if (
                    previous is not None
                    and block == previous[0]
                    and body == previous[1]
                ):
                    self.stream.write(
                        bytes((ACK, CRC)) if previous_header else bytes((ACK,)),
                        self.deadline(),
                    )
                    failures += 1
                    if failures >= self.retries:
                        raise DebugError("YMODEM duplicate-block retry limit reached")
                    continue
                if block != expected:
                    raise DebugError(
                        f"out-of-order YMODEM block {block}, expected {expected}"
                    )
                is_header = info is None or ending
                if is_header:
                    entry = parse_metadata(body)
                    if ending:
                        if entry is not None:
                            raise DebugError(
                                "only one file per YMODEM operation is supported"
                            )
                        self.stream.write(bytes((ACK,)), self.deadline())
                        return info
                    if entry is None:
                        raise DebugError("YMODEM batch contained no file")
                    info, remaining = entry, entry[1]
                    self.stream.write(bytes((ACK, CRC)), self.deadline())
                else:
                    if not remaining:
                        raise DebugError("YMODEM sent more data than advertised")
                    data = body[:remaining]
                    written = file.write(data)
                    if written != len(data):
                        raise DebugError("short local file write")
                    remaining -= len(data)
                    self.stream.write(bytes((ACK,)), self.deadline())
                previous, previous_header = received, is_header
                expected, failures = (expected + 1) & 255, 0
                self.progress()
        except BaseException:
            self.cancel()
            raise


_CSI = re.compile(rb"\x1b\[[0-?]*[ -/]*[@-~]")
_OSC = re.compile(rb"\x1b\][^\x07]*(?:\x07|\x1b\\)")


def terminal_text(data):
    data = _CSI.sub(b"", _OSC.sub(b"", data))
    data = re.sub(rb"\x1b[()][0-9A-Za-z]", b"", data)
    return data.replace(b"\r\n", b"\n")


def quote(value):
    if "\0" in value or "\r" in value or "\n" in value:
        raise DebugError("a native command argument contains a line break or NUL")
    return '"' + value.replace("^", "^^").replace('"', '^"').replace("$", "^$") + '"'


def remote_path(value):
    if (
        not re.match(r"^[A-Za-z]:[\\/]", value)
        or value.endswith(("/", "\\"))
        or any(ord(c) < 32 for c in value)
        or len(value.encode("utf-16-le")) // 2 > 256
    ):
        raise DebugError(
            "remote path must be a native absolute filename, such as C:\\data\\file.bin"
        )
    value = value.replace("/", "\\")
    if any(part in (".", "..") for part in value[3:].split("\\")):
        raise DebugError("remote paths must not contain . or .. components")
    return value


def file_sha256(file):
    digest = hashlib.sha256()
    for chunk in iter(lambda: file.read(1024 * 1024), b""):
        digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True)
class Result:
    status: int
    output: bytes


class Shell:
    def __init__(self, stream, timeout=60.0, idle=30.0):
        self.stream, self.timeout, self.idle = stream, timeout, idle

    def begin(self, command, *, script=None, timeout=None):
        nonce = secrets.token_hex(12)
        if script:
            # $? straight into END: "export NEO_RESULT $?" took a negative status for an option (and paged its help);
            # the script is removed after END, and prompt() waits for that before the next command
            command = f'fshell {script} &| echo "^r^nEND_{nonce} $?" &| rm {script}'
        else:
            command += f' &| echo "^r^nEND_{nonce} $?"'
        wire = f"echo BEGIN_{nonce} &| {command}\r".encode("ascii")
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        self.stream.write(wire, deadline)
        while (
            terminal_text(self.stream.line(deadline)).strip()
            != f"BEGIN_{nonce}".encode()
        ):
            pass
        return nonce

    def prompt(self, deadline):
        """Read up to fshell's next prompt ("c:\\>"): until it shows, fshell isn't reading, and input typed then
        loses its first bytes (seen on QEMU after a YMODEM command)."""
        text = bytearray()
        while not terminal_text(bytes(text)).rstrip().endswith(b">"):
            text.extend(self.stream.read(4096, deadline))
            del text[:-4096]

    def finish(self, nonce, deadline):
        marker = re.compile(rb"END_" + nonce.encode() + rb" ([-+]?\d{1,10})\n?$")
        output = bytearray()
        while True:
            line = self.stream.line(deadline)
            plain = terminal_text(line)
            match = marker.fullmatch(plain)
            if match:
                status = int(match[1])
                if not -(1 << 31) <= status < (1 << 31):
                    raise DebugError("native exit status is outside TInt range")
                # The wrapper adds one CRLF before END, independently of command output.
                if output.endswith(b"\r\n"):
                    del output[-2:]
                elif output.endswith(b"\n"):
                    del output[-1:]
                text = terminal_text(bytes(output))
                try:
                    text.decode("utf-8")
                except UnicodeDecodeError as error:
                    raise DebugError(
                        "command output is not UTF-8 text; use binary file transfer"
                    ) from error
                self.prompt(deadline)
                return Result(status, text)
            output.extend(line)
            if len(output) > MAX_OUTPUT:
                raise DebugError(
                    "command output exceeds 16 MiB; redirect it to a file and pull it"
                )

    def transfer_ready(self, nonce):
        # This native notice precedes the binary handshake. Reading until a random 'C' in terminal text would
        # confuse messages with CRC requests; starting a download before binary mode can also type into fshell.
        deadline = time.monotonic() + self.timeout
        marker = re.compile(rb"END_" + nonce.encode() + rb" ([-+]?\d{1,10})\n?$")
        output = bytearray()
        while True:
            line = self.stream.line(deadline)
            plain = terminal_text(line)
            if plain.strip() == b"Please start the file transfer in your terminal...":
                return
            match = marker.fullmatch(plain)
            if match:
                status = int(match[1])
                if status:
                    raise RemoteError(status, terminal_text(bytes(output)))
                raise DebugError(
                    "native command completed without entering file-transfer mode"
                )
            output.extend(line)
            if len(output) > MAX_OUTPUT:
                raise DebugError(
                    "native file-transfer preamble exceeds the client limit"
                )

    @staticmethod
    def require(result):
        if result.status:
            raise RemoteError(result.status, result.output)
        return result

    def upload(
        self, file, name, size, digest, destination, overwrite=False, script=None
    ):
        command = f"ymodem receive --output {quote(destination)} --sha256 {digest}"
        if overwrite:
            command += " --overwrite"
        nonce = self.begin(command, script=script)
        self.transfer_ready(nonce)
        Ymodem(self.stream, self.idle).send(file, name, size)
        return self.require(self.finish(nonce, time.monotonic() + self.timeout))

    def prepare(self, command):
        if "\0" in command:
            raise DebugError("command contains NUL")
        data = (command + "\n").encode("utf-8")
        name = f"neo_{secrets.token_hex(12)}.script"
        destination = "C:\\" + name
        self.upload(
            io.BytesIO(data),
            name,
            len(data),
            hashlib.sha256(data).hexdigest(),
            destination,
        )
        return destination

    def execute(self, command):
        script = self.prepare(command)
        deadline = time.monotonic() + self.timeout
        nonce = self.begin("", script=script)
        return self.finish(nonce, deadline)

    def push(self, local, remote, overwrite=False):
        remote = remote_path(remote)
        with open(local, "rb") as file:
            size = os.fstat(file.fileno()).st_size
            if size > MAX_FILE_SIZE:
                raise DebugError("file size exceeds native fshell's 2 GiB limit")
            digest = file_sha256(file)
            file.seek(0)
            command = f"ymodem receive --output {quote(remote)} --sha256 {digest}"
            if overwrite:
                command += " --overwrite"
            script = self.prepare(command)
            nonce = self.begin("", script=script)
            self.transfer_ready(nonce)
            Ymodem(self.stream, self.idle).send(file, remote.rsplit("\\", 1)[-1], size)
            self.require(self.finish(nonce, time.monotonic() + self.timeout))
        return digest

    def pull(self, remote, local, overwrite=False):
        remote = remote_path(remote)
        local = Path(local)
        if local.exists() and not overwrite:
            raise DebugError(f"destination exists: {local}; use --overwrite")
        result = self.require(
            self.execute(f"hash --algorithm SHA256 --input {quote(remote)} --hex")
        )
        hashes = re.findall(rb"(?m)^([0-9a-fA-F]{64})$", result.output)
        if len(hashes) != 1:
            raise DebugError("native hash did not return one SHA-256 value")
        expected = hashes[0].decode().lower()
        script = self.prepare(f"ymodem send {quote(remote)}")
        fd, temporary = tempfile.mkstemp(
            prefix=".neo-", suffix=".part", dir=local.parent
        )
        try:
            with os.fdopen(fd, "w+b") as file:
                nonce = self.begin("", script=script)
                self.transfer_ready(nonce)
                _, size = Ymodem(self.stream, self.idle).receive(file)
                self.require(self.finish(nonce, time.monotonic() + self.timeout))
                if file.tell() != size:
                    raise DebugError("download length differs from YMODEM metadata")
                file.flush()
                os.fsync(file.fileno())
                file.seek(0)
                actual = file_sha256(file)
                if actual != expected:
                    raise DebugError(
                        f"download SHA-256 mismatch: expected {expected}, got {actual}"
                    )
            if overwrite:
                os.replace(temporary, local)
            else:
                # Atomic no-clobber creation on the same filesystem, including a destination created mid-transfer.
                os.link(temporary, local)
                os.unlink(temporary)
            temporary = None
            return actual
        finally:
            if temporary is not None:
                try:
                    os.unlink(temporary)
                except FileNotFoundError:
                    pass
