"""Write an Android boot.img (header v0) for the Pixel 3a XL ABL: kernel with the DTB appended, no ramdisk.

Usage: python3 mkboot.py KERNEL DTB OUT
Addresses match the working ztOS boot.img for bonito.
"""

import hashlib
import struct
import sys
from pathlib import Path

PAGE = 4096


def pad(b: bytes) -> bytes:
    return b + b"\0" * (-len(b) % PAGE)


def mkboot(kernel: bytes, ramdisk: bytes = b"") -> bytes:
    sha = hashlib.sha1()
    for part in (
        kernel,
        ramdisk,
        b"",
    ):  # kernel, ramdisk, second, as mkbootimg hashes them
        sha.update(part)
        sha.update(struct.pack("<I", len(part)))
    hdr = b"ANDROID!" + struct.pack(
        "<10I16s512s32s1024s",
        len(kernel),
        0x8000,  # kernel size, addr
        len(ramdisk),
        0x1000000,  # ramdisk size, addr
        0,
        0xF00000,  # second size, addr
        0x100,
        PAGE,
        0,
        0,  # tags addr, page size, header version, os version
        b"",
        b"",
        sha.digest(),
        b"",  # name, cmdline, id, extra cmdline
    )
    return pad(hdr) + pad(kernel) + pad(ramdisk)


def main() -> None:
    kernel_path, dtb_path, out_path = sys.argv[1:4]
    kernel = Path(kernel_path).read_bytes() + Path(dtb_path).read_bytes()
    img = mkboot(kernel)
    # self-check: header parses back to what we wrote
    assert img[:8] == b"ANDROID!"
    ksize, _, rsize = struct.unpack_from("<3I", img, 8)
    assert ksize == len(kernel) and rsize == 0
    assert img[PAGE : PAGE + ksize] == kernel
    Path(out_path).write_bytes(img)
    print(f"{out_path}: {len(img)} bytes, kernel+dtb {ksize}")


if __name__ == "__main__":
    main()
