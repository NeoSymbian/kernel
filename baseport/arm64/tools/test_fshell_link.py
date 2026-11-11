#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: SIM117 -- nested with: assertRaises must cover only the inner call, not the peer set-up
"""Host tests for fshell_link and phone.py sh/push/pull (roadmap T9): independent YMODEM and fshell peers over real
PTYs, no phone, standard library only. Run: python3 -m unittest test_fshell_link (in tools/)."""

import binascii
import contextlib
import hashlib
import io
import os
import pty
import re
import select
import struct
import subprocess
import sys
import tempfile
import threading
import time
import tty
import unittest
from pathlib import Path
from unittest import mock

import fshell_link as debug
import phone


def exact(fd, count):
    data = bytearray()
    deadline = time.monotonic() + 5
    while len(data) < count:
        if not select.select([fd], [], [], max(0, deadline - time.monotonic()))[0]:
            raise AssertionError(f"test peer timed out reading {count} bytes")
        chunk = os.read(fd, count - len(data))
        if not chunk:
            raise AssertionError("test peer EOF")
        data.extend(chunk)
    return bytes(data)


def wire_packet(block, body, size=1024):
    body = body.ljust(size, b"\x1a")
    return (
        bytes((1 if size == 128 else 2, block % 256, 255 - block % 256))
        + body
        + struct.pack(">H", binascii.crc_hqx(body, 0))
    )


def get_packet(fd):
    start = exact(fd, 1)[0]
    if start not in (1, 2):
        raise AssertionError(f"test peer expected packet, got {start}")
    length = 128 if start == 1 else 1024
    data = exact(fd, length + 4)
    assert data[0] ^ data[1] == 255
    assert binascii.crc_hqx(data[2:-2], 0) == int.from_bytes(data[-2:], "big")
    return data[0], data[2:-2]


def peer_receive(fd, retry=False, double_eot=False):
    os.write(fd, b"C")
    block, body = get_packet(fd)
    assert block == 0
    name, tail = body.split(b"\0", 1)
    size = int(tail.split(b"\0", 1)[0])
    os.write(fd, b"\x06C")
    data, expected = bytearray(), 1
    while len(data) < size:
        block, body = get_packet(fd)
        assert block == expected % 256
        if retry:
            os.write(fd, b"\x15")
            assert get_packet(fd) == (block, body)
            retry = False
        data.extend(body[: size - len(data)])
        expected += 1
        os.write(fd, b"\x06")
    assert exact(fd, 1) == b"\x04"
    if double_eot:
        os.write(fd, b"\x15")
        assert exact(fd, 1) == b"\x04"
    os.write(fd, b"\x06C")
    assert get_packet(fd) == (0, bytes(128))
    os.write(fd, b"\x06")
    return name.decode(), bytes(data)


def peer_send(fd, name, data, *, corrupt=False, duplicate=False, duplicate_eot=False):
    assert exact(fd, 1) == b"C"
    header = name.encode() + b"\0" + str(len(data)).encode() + b"\0"
    os.write(fd, wire_packet(0, header.ljust(1024, b"\0")))
    assert exact(fd, 2) == b"\x06C"
    for n, offset in enumerate(range(0, len(data), 1024), 1):
        packet = wire_packet(n, data[offset : offset + 1024])
        if corrupt:
            bad = bytearray(packet)
            bad[-1] ^= 1
            os.write(fd, bad)
            assert exact(fd, 1) == b"\x15"
            corrupt = False
        os.write(fd, packet)
        assert exact(fd, 1) == b"\x06"
        if duplicate:
            os.write(fd, packet)
            assert exact(fd, 1) == b"\x06"
            duplicate = False
    os.write(fd, b"\x04")
    assert exact(fd, 2) == b"\x06C"
    if duplicate_eot:
        os.write(fd, b"\x04")
        assert exact(fd, 2) == b"\x06C"
    os.write(fd, wire_packet(0, bytes(128), 128))
    assert exact(fd, 1) == b"\x06"


@contextlib.contextmanager
def pair(peer):
    master, slave = pty.openpty()
    tty.setraw(slave)
    errors = []

    def run():
        try:
            peer(master)
        except BaseException as error:  # noqa: BLE001 -- the peer's error is re-raised in the test thread
            errors.append(error)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    try:
        yield debug.Stream(slave)
    finally:
        thread.join(6)
        os.close(slave)
        with contextlib.suppress(OSError):
            os.close(master)
        if thread.is_alive():
            raise AssertionError("test peer did not finish")
        if errors:
            raise errors[0]


class FakeNative:
    """Protocol oracle parses host commands and independently serves file operations."""

    def __init__(
        self,
        files=None,
        output="done",
        status=0,
        hash_override=None,
        upload_status=0,
        disconnect_download=False,
        refuse_upload=0,
    ):
        self.files = {} if files is None else dict(files)
        self.output, self.status = output.encode(), status
        self.hash_override, self.upload_status = hash_override, upload_status
        self.disconnect_download = disconnect_download
        self.refuse_upload = refuse_upload
        self.commands = []
        self.stopped = False

    def __call__(self, fd):
        while not self.stopped:
            line = bytearray()
            while not line.endswith(b"\r"):
                line.extend(exact(fd, 1))
            text = line.decode("ascii").rstrip("\r")
            nonce = re.search(r"echo BEGIN_([0-9a-f]+)", text)[1]
            # The echoed command contains nonce text and $?; it is not a completion record.
            os.write(fd, bytes(line) + b"\n\x1b[2KBEGIN_" + nonce.encode() + b"\r\n")
            script = re.search(r"fshell (C:\\neo_[0-9a-f]+\.script)", text)
            if script:
                source = self.files.pop(script[1]).decode().rstrip("\n")
                self.commands.append(source)
            else:
                source = text.split(" &| ", 1)[1].split(" &| echo ", 1)[0]
            status, output = 0, b""
            if source.startswith("ymodem receive "):
                destination = re.search(r'--output "([^"]+)"', source)[1]
                want = re.search(r"--sha256 ([0-9a-f]{64})", source)[1]
                os.write(fd, b"Please start the file transfer in your terminal...\r\n")
                if self.refuse_upload and not destination.endswith(".script"):
                    # as fshell when it can't create the file: CAN after the header, then purge input until quiet
                    os.write(fd, b"C")
                    assert get_packet(fd)[0] == 0
                    os.write(fd, b"\x06\x18\x18")
                    while select.select([fd], [], [], 0.3)[0]:
                        os.read(fd, 4096)
                    os.write(
                        fd,
                        output
                        + f"\r\nEND_{nonce} {self.refuse_upload}\r\nc:\\>".encode(),
                    )
                    continue
                _, payload = peer_receive(fd)
                assert hashlib.sha256(payload).hexdigest() == want
                if destination.endswith(".script"):
                    self.files[destination] = payload
                elif self.upload_status:
                    status = self.upload_status
                else:
                    assert destination not in self.files or "--overwrite" in source
                    self.files[destination] = payload
            elif source.startswith("ymodem send "):
                remote = re.search(r'ymodem send "([^"]+)"', source)[1]
                os.write(fd, b"Please start the file transfer in your terminal...\r\n")
                if self.disconnect_download:
                    self.disconnect_download = False
                    self.stopped = True
                    payload = self.files[remote]
                    assert exact(fd, 1) == b"C"
                    os.write(
                        fd,
                        wire_packet(
                            0,
                            remote.rsplit("\\", 1)[1].encode()
                            + b"\0"
                            + str(len(payload)).encode()
                            + b"\0",
                        ),
                    )
                    assert exact(fd, 2) == b"\x06C"
                    os.write(fd, wire_packet(1, payload[:1024]))
                    assert exact(fd, 1) == b"\x06"
                    os.close(fd)
                    return
                peer_send(fd, remote.rsplit("\\", 1)[1], self.files[remote])
            elif source.startswith("hash "):
                remote = re.search(r'--input "([^"]+)"', source)[1]
                output = (
                    self.hash_override or hashlib.sha256(self.files[remote]).hexdigest()
                ).encode() + b"\r\n"
            else:
                output, status = self.output, self.status
            # fshell prints its prompt after every command line; the client waits for it (T7)
            os.write(fd, output + f"\r\nEND_{nonce} {status}\r\nc:\\>".encode())


@contextlib.contextmanager
def native_peer(peer):
    """FakeNative exits on a private test-only sentinel command, never sent by production."""
    with pair(peer) as stream:
        try:
            yield debug.Shell(stream, timeout=1, idle=1)
        finally:
            if not peer.stopped:
                peer.stopped = True
                # Wake a peer waiting for a command without closing its PTY beneath the test thread.
                os.write(stream.fd, b"echo BEGIN_deadbeef &| echo stop\r")
                deadline = time.monotonic() + 2
                try:
                    stream.line(deadline)
                    stream.line(deadline)
                    stream.line(deadline)
                except debug.DebugError:
                    pass


class DiscoveryTests(unittest.TestCase):
    def test_roles_follow_registry_interface_not_modem_name(self):
        # as macOS shows it: the ACM data driver under each interface repeats idVendor/idProduct
        acm = lambda path: {
            "idVendor": 0x1209,
            "idProduct": 1,
            "IORegistryEntryChildren": [{"IOCalloutDevice": path}],
        }
        tree = [
            {
                "idVendor": 0x1209,
                "idProduct": 1,
                "bDeviceClass": 0xEF,
                "USB Serial Number": "one",
                "locationID": 0x1234,
                "IORegistryEntryChildren": [
                    {
                        "bInterfaceNumber": 1,
                        "idVendor": 0x1209,
                        "idProduct": 1,
                        "IORegistryEntryChildren": [acm("/dev/cu.z-monitor")],
                    },
                    {
                        "bInterfaceNumber": 3,
                        "idVendor": 0x1209,
                        "idProduct": 1,
                        "IORegistryEntryChildren": [acm("/dev/cu.a-shell")],
                    },
                    {
                        "bInterfaceNumber": 5,
                        "IORegistryEntryChildren": [
                            {"IOCalloutDevice": "/dev/cu.not-serial"}
                        ],
                    },
                ],
            },
            {
                "idVendor": 0x9999,
                "idProduct": 1,
                "bInterfaceNumber": 1,
                "IOCalloutDevice": "/dev/cu.other",
            },
        ]
        ports = debug.ports_from_registry(tree)
        self.assertEqual(debug.select_port("monitor", ports=ports), "/dev/cu.z-monitor")
        self.assertEqual(
            debug.select_port("shell", ports=ports, device="one"), "/dev/cu.a-shell"
        )
        self.assertEqual(len(ports), 2)

    def test_ambiguity_is_an_error_and_override_is_explicit(self):
        ports = [
            debug.Port("a", "id-a", "a", 1, 1, "monitor"),
            debug.Port("b", "id-b", "b", 2, 1, "monitor"),
        ]
        with self.assertRaisesRegex(debug.DebugError, "ambiguous"):
            debug.select_port("monitor", ports=ports)
        self.assertEqual(debug.select_port("monitor", ports=ports, device="id-b"), "b")
        self.assertEqual(
            debug.select_port("shell", ports=[], override="/dev/test"), "/dev/test"
        )

    def test_common_flags_preserve_quoted_command_text(self):
        selected, rest = debug.connection_options(
            ["--device", "one", "sh", "echo --device two", "--shell-port", "/dev/test"]
        )
        self.assertEqual((selected.device, selected.shell_port), ("one", "/dev/test"))
        self.assertEqual(rest, ["sh", "echo --device two"])


class StreamTests(unittest.TestCase):
    def test_partial_writes_and_reads(self):
        data = bytes(range(256))
        got = []
        real_write = os.write
        with pair(lambda fd: got.append(exact(fd, len(data)))) as stream:
            with mock.patch.object(
                debug.os,
                "write",
                side_effect=lambda fd, value: real_write(fd, value[:3]),
            ):
                stream.write(data, time.monotonic() + 1)
        self.assertEqual(got, [data])

    def test_timeout_and_disconnect_are_errors(self):
        read, write = os.pipe()
        try:
            with self.assertRaises(debug.DeadlineExpired):
                debug.Stream(read).exact(1, time.monotonic() + 0.02)
            os.close(write)
            write = None
            with self.assertRaises(debug.LinkError):
                debug.Stream(read).exact(1, time.monotonic() + 1)
        finally:
            os.close(read)
            if write is not None:
                os.close(write)

    def test_shell_lock_rejects_second_operation(self):
        with tempfile.TemporaryDirectory() as directory:
            with debug.PortLock("/dev/test", directory):
                with self.assertRaises(debug.BusyError):
                    with debug.PortLock("/dev/test", directory):
                        self.fail("second lock acquired")
            with debug.PortLock("/dev/test", directory):
                pass

    def test_dtr_is_asserted_and_lowered_on_close(self):
        master, slave = pty.openpty()
        try:
            with (
                tempfile.TemporaryDirectory() as home,
                mock.patch.object(
                    debug.pwd, "getpwuid", return_value=mock.Mock(pw_dir=home)
                ),
                mock.patch.object(debug.fcntl, "ioctl") as ioctl,
            ):
                with debug.SerialConnection(os.ttyname(slave)):
                    pass
                self.assertEqual(
                    ioctl.call_args_list[0].args[1], debug.termios.TIOCMBIS
                )
                self.assertEqual(
                    ioctl.call_args_list[-1].args[1], debug.termios.TIOCMBIC
                )
        finally:
            os.close(master)
            os.close(slave)

    def test_default_lock_ignores_tmpdir_and_home_overrides(self):
        with (
            tempfile.TemporaryDirectory() as home,
            mock.patch.object(
                debug.pwd, "getpwuid", return_value=mock.Mock(pw_dir=home)
            ),
            mock.patch.dict(
                os.environ, {"TMPDIR": home + "/one", "HOME": home + "/fake-one"}
            ),
            mock.patch.object(tempfile, "tempdir", None),
        ):
            first = debug.PortLock("/dev/test-endpoint")
            with first:
                with (
                    mock.patch.dict(
                        os.environ,
                        {"TMPDIR": home + "/two", "HOME": home + "/fake-two"},
                    ),
                    mock.patch.object(tempfile, "tempdir", None),
                ):
                    second = debug.PortLock("/dev/test-endpoint")
                    self.assertEqual(first.path, second.path)
                    with self.assertRaises(debug.BusyError), second:
                        self.fail("environment changes bypassed the lock")
                self.assertEqual(first.path.parent.stat().st_mode & 0o777, 0o700)
                self.assertTrue(first.path.is_relative_to(Path(home)))

    def test_macos_tty_and_callout_aliases_share_the_lock(self):
        with (
            tempfile.TemporaryDirectory() as directory,
            mock.patch.object(debug.sys, "platform", "darwin"),
        ):
            first = debug.PortLock("/dev/cu.usbmodem-test", directory)
            second = debug.PortLock("/dev/tty.usbmodem-test", directory)
            self.assertEqual(first.path, second.path)
            with first, self.assertRaises(debug.BusyError), second:
                self.fail("tty alias bypassed the callout lock")

    def test_pty_nodes_remain_distinct(self):
        with (
            tempfile.TemporaryDirectory() as directory,
            mock.patch.object(debug.sys, "platform", "darwin"),
        ):
            self.assertNotEqual(
                debug.PortLock("/dev/ttys001", directory).path,
                debug.PortLock("/dev/ttys002", directory).path,
            )

    def test_second_process_cannot_bypass_lock_with_alias_and_different_environment(
        self,
    ):
        child = (
            "from types import SimpleNamespace\n"
            "import sys\n"
            "import fshell_link as d\n"
            "d.sys.platform = 'darwin'\n"
            "d.pwd.getpwuid = lambda uid: SimpleNamespace(pw_dir=sys.argv[1])\n"
            "try:\n"
            "    with d.PortLock('/dev/tty.usbmodem-test'):\n"
            "        pass\n"
            "except d.BusyError:\n"
            "    raise SystemExit(73)\n"
            "raise SystemExit(0)\n"
        )
        with (
            tempfile.TemporaryDirectory() as home,
            mock.patch.object(
                debug.pwd, "getpwuid", return_value=mock.Mock(pw_dir=home)
            ),
            mock.patch.object(debug.sys, "platform", "darwin"),
        ):
            environment = dict(
                os.environ, TMPDIR=home + "/child-tmp", HOME=home + "/child-home"
            )
            with (
                mock.patch.dict(
                    os.environ,
                    {"TMPDIR": home + "/parent-tmp", "HOME": home + "/parent-home"},
                ),
                debug.PortLock("/dev/cu.usbmodem-test"),
            ):
                result = subprocess.run(  # noqa: PLW1510 -- the test checks returncode itself
                    [sys.executable, "-c", child, home],
                    cwd=Path(__file__).parent,
                    env=environment,
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                self.assertEqual(result.returncode, 73, result.stderr)
            result = subprocess.run(  # noqa: PLW1510 -- the test checks returncode itself
                [sys.executable, "-c", child, home],
                cwd=Path(__file__).parent,
                env=environment,
                capture_output=True,
                text=True,
                timeout=5,
            )
            self.assertEqual(result.returncode, 0, result.stderr)


class YmodemTests(unittest.TestCase):
    def test_crc_known_vector_and_utf8_long_header(self):
        self.assertEqual(binascii.crc_hqx(b"123456789", 0), 0x31C3)
        name = "Հայերեն_" * 12 + ".bin"
        metadata = debug.metadata(name, 123)
        self.assertEqual(len(metadata), 1024)
        self.assertEqual(debug.parse_metadata(metadata), (name, 123))

    def test_invalid_metadata_rejected(self):
        for body in (
            b"../x\x000",
            b"x\x00-1",
            b"x\x009223372036854775808",
            b"\xff\x001",
            b"missing-nul",
        ):
            with self.subTest(body=body), self.assertRaises(debug.DebugError):
                debug.parse_metadata(body)

    def test_send_all_bytes_unicode_retry_double_eot_and_sequence_wrap(self):
        data, received = bytes(range(256)) * 1050, []
        name = "Հայերեն_" * 12 + ".bin"
        with pair(
            lambda fd: received.append(peer_receive(fd, retry=True, double_eot=True))
        ) as stream:
            debug.Ymodem(stream, idle=3).send(io.BytesIO(data), name, len(data))
        self.assertEqual(received, [(name, data)])

    def test_receive_crc_duplicate_eot_and_sequence_wrap(self):
        data = bytes(range(256)) * 1050
        output = io.BytesIO()
        with pair(
            lambda fd: peer_send(
                fd,
                "Հայերեն.bin",
                data,
                corrupt=True,
                duplicate=True,
                duplicate_eot=True,
            )
        ) as stream:
            info = debug.Ymodem(stream, idle=3).receive(output)
        self.assertEqual(info, ("Հայերեն.bin", len(data)))
        self.assertEqual(output.getvalue(), data)

    def test_empty_file_round_trip(self):
        received = []
        with pair(lambda fd: received.append(peer_receive(fd))) as stream:
            debug.Ymodem(stream, idle=1).send(io.BytesIO(), "empty.bin", 0)
        self.assertEqual(received, [("empty.bin", b"")])
        output = io.BytesIO()
        with pair(lambda fd: peer_send(fd, "empty.bin", b"")) as stream:
            self.assertEqual(
                debug.Ymodem(stream, idle=1).receive(output), ("empty.bin", 0)
            )

    def test_premature_eot_fails_and_cancels(self):
        def peer(fd):
            self.assertEqual(exact(fd, 1), b"C")
            os.write(fd, wire_packet(0, b"file\x005\x00".ljust(128, b"\0"), 128))
            self.assertEqual(exact(fd, 2), b"\x06C")
            os.write(fd, b"\x04")
            self.assertEqual(exact(fd, 2), b"\x18\x18")

        with pair(peer) as stream:
            with self.assertRaisesRegex(debug.DebugError, "advertised"):
                debug.Ymodem(stream, idle=1).receive(io.BytesIO())

    def test_no_progress_never_succeeds(self):
        read, write = os.pipe()
        try:
            with self.assertRaises(debug.DeadlineExpired):
                debug.Ymodem(debug.Stream(read), idle=0.02).send(
                    io.BytesIO(b"x"), "file", 1
                )
        finally:
            os.close(read)
            os.close(write)

    def test_cancelled_upload_sends_can_and_does_not_replay(self):
        def peer(fd):
            os.write(fd, b"C")
            self.assertEqual(get_packet(fd)[0], 0)
            os.write(fd, b"\x06C")
            self.assertEqual(exact(fd, 2), b"\x18\x18")

        source = mock.Mock()
        source.read.side_effect = KeyboardInterrupt
        with pair(peer) as stream, self.assertRaises(KeyboardInterrupt):
            debug.Ymodem(stream, idle=1).send(source, "file.bin", 1)
        source.read.assert_called_once()


class ShellTests(unittest.TestCase):
    def test_finite_unicode_script_and_signed_exit_no_prompt_parsing(self):
        peer = FakeNative(output="Հայերեն առանց newline", status=-5)
        command = 'echo "Հայերեն ^ $ apostrophe\'"\necho second'
        with native_peer(peer) as shell:
            result = shell.execute(command)
        self.assertEqual(result, debug.Result(-5, "Հայերեն առանց newline".encode()))
        self.assertEqual(peer.commands, [command])
        self.assertFalse(any(p.endswith(".script") for p in peer.files))

    def test_push_unicode_all_bytes_and_native_finalization_failure(self):
        data, remote = bytes(range(256)) * 9, "C:\\Հայերեն ֆայլ.bin"
        with tempfile.TemporaryDirectory() as directory:
            local = Path(directory) / "input.bin"
            local.write_bytes(data)
            peer = FakeNative()
            with native_peer(peer) as shell:
                self.assertEqual(
                    shell.push(local, remote), hashlib.sha256(data).hexdigest()
                )
            self.assertEqual(peer.files[remote], data)
            failed = FakeNative(upload_status=-22)
            with native_peer(failed) as shell:
                with self.assertRaises(debug.RemoteError) as error:
                    shell.push(local, remote)
                self.assertEqual(error.exception.status, -22)
            self.assertNotIn(remote, failed.files)
            refused = FakeNative(refuse_upload=-12)
            with native_peer(refused) as shell:
                with self.assertRaises(debug.RemoteError) as error:
                    shell.push(local, "C:\\nodir\\x.bin")
                self.assertEqual(error.exception.status, -12)
                # the shell is idle again: the next command is not lost in fshell's purge
                self.assertEqual(shell.execute("echo after").status, 0)

    def test_pull_integrity_overwrite_and_temporary_cleanup(self):
        data, remote = bytes(range(256)) * 9, "C:\\Հայերեն ֆայլ.bin"
        with tempfile.TemporaryDirectory() as directory:
            local = Path(directory) / "Հայերեն.bin"
            peer = FakeNative({remote: data})
            with native_peer(peer) as shell:
                self.assertEqual(
                    shell.pull(remote, local), hashlib.sha256(data).hexdigest()
                )
            self.assertEqual(local.read_bytes(), data)
            with self.assertRaisesRegex(debug.DebugError, "destination exists"):
                debug.Shell(None).pull(remote, local)
            peer = FakeNative({remote: b"replacement"})
            with native_peer(peer) as shell:
                shell.pull(remote, local, overwrite=True)
            self.assertEqual(local.read_bytes(), b"replacement")
            self.assertEqual(list(Path(directory).glob("*.part")), [])

    def test_hash_mismatch_preserves_existing_destination(self):
        remote = "C:\\file.bin"
        with tempfile.TemporaryDirectory() as directory:
            local = Path(directory) / "file.bin"
            local.write_bytes(b"original")
            peer = FakeNative({remote: b"new"}, hash_override="0" * 64)
            with native_peer(peer) as shell:
                with self.assertRaisesRegex(debug.DebugError, "SHA-256 mismatch"):
                    shell.pull(remote, local, overwrite=True)
            self.assertEqual(local.read_bytes(), b"original")
            self.assertEqual(list(Path(directory).glob(".neo-*.part")), [])

    def test_local_fsync_failure_preserves_destination(self):
        remote = "C:\\file.bin"
        with tempfile.TemporaryDirectory() as directory:
            local = Path(directory) / "file.bin"
            local.write_bytes(b"original")
            peer = FakeNative({remote: b"new"})
            with (
                native_peer(peer) as shell,
                mock.patch.object(debug.os, "fsync", side_effect=OSError("disk error")),
            ):
                with self.assertRaisesRegex(OSError, "disk error"):
                    shell.pull(remote, local, overwrite=True)
            self.assertEqual(local.read_bytes(), b"original")
            self.assertEqual(list(Path(directory).glob(".neo-*.part")), [])

    def test_destination_created_during_pull_is_never_overwritten(self):
        remote = "C:\\file.bin"
        with tempfile.TemporaryDirectory() as directory:
            local = Path(directory) / "file.bin"
            peer = FakeNative({remote: b"new"})
            real_link = os.link

            def racing_link(source, destination):
                local.write_bytes(b"created concurrently")
                real_link(source, destination)

            with (
                native_peer(peer) as shell,
                mock.patch.object(debug.os, "link", side_effect=racing_link),
            ):
                with self.assertRaises(FileExistsError):
                    shell.pull(remote, local)
            self.assertEqual(local.read_bytes(), b"created concurrently")
            self.assertEqual(list(Path(directory).glob(".neo-*.part")), [])

    def test_interrupted_pull_preserves_destination_and_never_replays(self):
        remote = "C:\\file.bin"
        with tempfile.TemporaryDirectory() as directory:
            local = Path(directory) / "file.bin"
            local.write_bytes(b"original")
            peer = FakeNative(
                {remote: bytes(range(256)) * 16}, disconnect_download=True
            )
            with native_peer(peer) as shell:
                with self.assertRaises(debug.LinkError):
                    shell.pull(remote, local, overwrite=True)
            self.assertEqual(local.read_bytes(), b"original")
            self.assertEqual(list(Path(directory).glob(".neo-*.part")), [])
            self.assertEqual(
                sum(c.startswith("ymodem send ") for c in peer.commands), 1
            )

    def test_completion_requires_exact_nonce_and_status(self):
        def peer(fd):
            line = bytearray()
            while not line.endswith(b"\r"):
                line.extend(exact(fd, 1))
            nonce = re.search(rb"BEGIN_([0-9a-f]+)", line)[1]
            os.write(
                fd, bytes(line) + b"\nBEGIN_" + nonce + b"\r\nEND_wrong 0\r\nfshell> "
            )
            time.sleep(0.1)

        with pair(peer) as stream:
            shell = debug.Shell(stream, timeout=0.05)
            nonce = shell.begin("echo test")
            with self.assertRaises(debug.DeadlineExpired):
                shell.finish(nonce, time.monotonic() + 0.05)

    def test_native_failure_before_binary_mode_is_reported(self):
        def peer(fd):
            line = bytearray()
            while not line.endswith(b"\r"):
                line.extend(exact(fd, 1))
            nonce = re.search(rb"BEGIN_([0-9a-f]+)", line)[1]
            os.write(
                fd,
                b"BEGIN_"
                + nonce
                + b"\r\nError: hash not supported\r\nEND_"
                + nonce
                + b" -5\r\n",
            )

        with pair(peer) as stream:
            shell = debug.Shell(stream, timeout=1)
            nonce = shell.begin("ymodem receive")
            with self.assertRaises(debug.RemoteError) as error:
                shell.transfer_ready(nonce)
            self.assertEqual(error.exception.status, -5)

    def test_legacy_selftest_and_help_still_run(self):
        with mock.patch("sys.stdout", new=io.StringIO()):
            self.assertEqual(phone.main(["selftest"]), 0)
            self.assertEqual(phone.main(["--help"]), 0)

    def test_legacy_monitor_command_uses_explicit_monitor_port(self):
        with (
            mock.patch.object(phone, "Console") as console,
            mock.patch("sys.stdout", new=io.StringIO()),
        ):
            console.return_value.cmd.return_value = b"boot log"
            self.assertEqual(
                phone.main(["--monitor-port", "/dev/monitor", "cmd", "log"]), 0
            )
        console.assert_called_once_with("/dev/monitor")
        console.return_value.cmd.assert_called_once_with("log")
        console.return_value.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
