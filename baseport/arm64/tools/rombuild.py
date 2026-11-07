#!/usr/bin/env python3
# Copyright The Phorepoc Authors
# SPDX-License-Identifier: Apache-2.0
# From Phorepoc pdk tools/rombuild.py (4ccaa1f); NeoSymbian: rom_heap.tsv is read from beside this file.

#
# Emits the ROM container (TRomHeader, TRomImageHeader + payload per XIP file, the directory tree)
# and prelinks our emitted E32Images into it (imports resolved by ordinal, relocations applied
# for the fixed ROM placement).  Two modes: a whole ROM (a bootstrap supplies the opening bytes)
# and a ROM SECTION (--section), the blob an x86/x32 kernel image carries inside itself.
#
# Layout produced (all addresses linear = rom_base + file_offset):
#   [0 .. 512)         TRomHeader: the bootstrap supplies the reset/restart vectors and
#                      reserves the 512-byte header (TheRomHeader in bootmain.s); we fill
#                      iRomBase/iRomSize/iRomRootDirectoryList/iPrimaryFile/iTime/iCheckSum.
#   [512 .. )          rest of the bootstrap boot code (XIP at rom_base).
#   then, 4-aligned:   each XIP file = 120-byte TRomImageHeader + payload.
#   then:              the directory tree (rom_dir blocks) and root_dir_list.
#   iCheckSum is computed so the ROM's 32-bit word sum == 0 (e32rom.h iCheckSum semantics).
#
#             --primary ekern.bin --kinfo ... --variant ... --xip 'Sys\Bin\x.dll=build/x.dll' --data 'Z-path=host-path' ...

import argparse
import os
import struct
import sys
import time

CPU_X86 = 0x1000

UID1_EXE = 0x1000007A
FILE_ATTRIB_DIR = 0x10
XIP_HDR_SIZE = 120          # TRomImageHeader, epoc10
ROM_HEADER_SIZE = 512       # TRomHeader_sz (h2inc)

# TRomHeader field offsets (h2inc of e32rom.h)
H_iTime = 128
H_iRomBase = 140
H_iRomSize = 144
H_iRomRootDirectoryList = 148
H_iPrimaryFile = 160
H_iSecondaryFile = 164
                            # exstart.cpp SecondaryInfo reads it via Epoc::RomHeader().iSecondaryFile
H_iCheckSum = 168
H_iKernelConfigFlags = 184  # copied to the super page at boot (memmodel/epoc/putils.cpp:52)
# u32std.h: a process whose code (or any DLL it loads) is not marked SMP safe has its threads in one NThreadGroup
# (EKernelConfigSMPUnsafeCompat, one CPU at a time) or all on CPU 0 (EKernelConfigSMPUnsafeCPU0, the one set below).
# Our own images are marked SMP safe, so only shipped third-party apps are affected.
KERNEL_CONFIG_SMP_UNSAFE_COMPAT = 1 << 12
KERNEL_CONFIG_SMP_UNSAFE_CPU0 = 1 << 13
KROM_IMAGE_SMP_SAFE = 0x00004000  # KRomImageSMPSafe (e32rom.h) -> ECodeSegAttSMPSafe (putils.cpp / sf_lepoc.cpp)
H_iRomExceptionSearchTable = 188  # -> TRomExceptionSearchTable (e32rom.h; drtaeabi SearchEST)
H_iUncompressedSize = 244   # bootstrap's "Running from RAM" path reads ROM size from here
H_iPageableRomStart = 328   # 0 = not a pageable ROM (fall back to iUncompressedSize)
H_iKernDataAddress = 152    # RAM VMA the kernel .data/.bss are linked at + copied to
H_iKernelLimit = 156        # top of kernel data region
H_iTotalSvDataSize = 200    # total kernel .data + .bss (4K-rounded)
H_iVariantFile = 204        # -> TRomEntry of the variant extension DLL
H_iTraceMask = 256          # copied to super-page iDebugMask; enables __KTRACE_OPT(cat)
H_iUserDataAddress = 220    # multiple-MM Mmu::DoInit2 asserts this == iDllDataBase+iMaxDllDataSize
H_iTotalUserDataSize = 224  # must be page-aligned (0 is fine); ROM DLL static-data size

# KUserSharedDataEnd1GB (== KUserSharedDataBase2GB) = 0x40000000 for both the 1GB and 2GB user
# splits (mmboot.h). Mmu::DoInit2 (multiple/mmu.cpp:182) panics ERomUserDataAddressInvalid
# (Kern::Fault "MMU" reason 9) unless the ROM header carries this exact value.
KUserDataAddress_MultipleMM = 0x40000000
# per-process local region -- maddressspace.cpp reserves [iUserDataAddress - iTotalUserDataSize,
# iUserDataAddress) there and asserts iUserDataAddress == KUserLocalDataEnd.  Carving it below
# 0x40000000 (the multiple-model value) would put every ROM DLL's data outside the region.
KUserLocalDataEnd_Flexible = 0x80000000
# ARM flexible (include/memmodel/epoc/flexible/arm/mmboot.h): KUserLocalDataEnd.
KUserLocalDataEnd_FlexibleArm = 0x7f000000

# ---- capabilities in the ROM header
_CAPS_MAP = None
# Core images that do not come from the component registry, with the capabilities of their mmps (honoured-capabilities ROMs only).
# The loader refuses a DLL that lacks a capability of the EXE it is loaded into, and the file systems carry the F32 set:
# efile.exe: TCB ProtServ DiskAdmin AllFiles PowerMgmt CommDD (f32caps.mmh); e32strt.exe / estart.exe: TCB WriteDeviceData DiskAdmin
# ProtServ AllFiles PowerMgmt (estart.mmp).
_CORE_CAPS = {"efile.exe": 0xB07, "e32strt.exe": 0xB25, "estart.exe": 0xB25}


def _header_caps(img, fallback1, fallback2):
    """The long-standing rule: an EXE runs with its own header capabilities (an EXE whose header says none keeps the fallback, ALL by
    default); a DLL gets the fallback."""
    if img.is_exe and (img.cap1 or img.cap2):
        return img.cap1, img.cap2
    return fallback1, fallback2


def _cap_override(zname):
    """(cap1, cap2) for an XIP image when the ROM honours the mmp capabilities, else None (the header rule applies).
    x32_rom_args.py writes <build>/rom/caps.tsv (image name -> first capability word) unless ALLCAPS=1 / HONOUR_CAPS=0 / ROM_ARCH=arm.
    An image outside the map (euser, efsrv, hal ... from the core builders) gets the 20 defined capability bits, not 0xFFFFFFFF."""
    global _CAPS_MAP
    if os.environ.get("ALLCAPS") == "1" or os.environ.get("HONOUR_CAPS") == "0":
        return None
    if _CAPS_MAP is None:
        _CAPS_MAP = {}
        path = os.environ.get("ROM_CAPS_TSV", os.path.join(os.environ.get("PHOREPOC_BUILD", ""), "rom", "caps.tsv"))
        if os.path.isfile(path):
            for line in open(path):
                k, _, v = line.strip().partition("\t")
                if v:
                    _CAPS_MAP[k] = int(v)
    if not _CAPS_MAP:
        return None
    name = zname.replace("/", "\\").split("\\")[-1].lower()
    if name in _CAPS_MAP:
        return _CAPS_MAP[name], 0
    return _CORE_CAPS.get(name, 0x000FFFFF), 0



# EXE heap/stack limits: an mmp sets EPOCHEAPSIZE for the emulator only (#ifdef WINS: WSERV's 3 MB), a device
# ROM raises it at rombuild time (iby HEAPMAX).  rom_heap.tsv (from Phorepoc core/registry) holds the values (name, heap min,
# heap max, stack; hex); every EXE gets max(its own, the table's), and an unlisted EXE's heap max is at least
# 2 MB.  ROM_HEAP_LIMITS=0 = the image's own values.
_LIMITS = None
def _exe_limits(img):
    global _LIMITS
    hmin, hmax, stk = img.heap_min or 0x1000, img.heap_max or 0x100000, img.stack or 0x2000
    if not img.is_exe or os.environ.get("ROM_HEAP_LIMITS", "1") == "0":
        return dict(stack_size=stk, heap_min=hmin, heap_max=hmax)
    if _LIMITS is None:
        _LIMITS = {}
        t = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rom_heap.tsv")	# NeoSymbian: beside this script
        for line in open(t, encoding="utf-8"):
            if line.strip() and not line.startswith("#"):
                n, a, b, c = line.split()
                _LIMITS[n.lower()] = (int(a, 16), int(b, 16), int(c, 16))
    name = img.zname.replace("/", "\\").split("\\")[-1].lower()
    nmin, nmax, nstk = _LIMITS.get(name, (0, 0x200000, 0))
    return dict(stack_size=max(stk, nstk), heap_min=max(hmin, nmin), heap_max=max(hmax, nmax))

def uid_checksum(uid1, uid2, uid3):
    """The Symbian UID checksum (odd/even byte interleave rotate)."""
    b = struct.pack("<III", uid1 & 0xFFFFFFFF, uid2 & 0xFFFFFFFF, uid3 & 0xFFFFFFFF)
    even = bytes(b[i] for i in range(0, 12, 2))
    odd = bytes(b[i] for i in range(1, 12, 2))

    def crc(data):
        c = 0
        for byte in data:
            c ^= byte << 8
            for _ in range(8):
                c = ((c << 1) ^ 0x1021) & 0xFFFF if (c & 0x8000) else (c << 1) & 0xFFFF
        return c
    return (crc(even) | (crc(odd) << 16)) & 0xFFFFFFFF


def est_table(images, ro_limit):
    """TRomExceptionSearchTable bytes for images = [(code base, exception descriptor address)];
    ro_limit(descriptor address) = that descriptor's iROSegmentLimit (TExceptionDescriptor word 3)."""
    if not images:
        return b""
    entries = sorted(c for c, _ in images)
    fence = max(ro_limit(d) for _, d in images) or 0xFFFFFFFF
    return struct.pack("<%dI" % (len(entries) + 2), len(entries), *entries, fence)


def align4(ba):
    while len(ba) % 4:
        ba.append(0)


class SlotMap:
    """Stable file placement (for delta pushes of an image to a device). A sequentially packed ROM moves every file behind
    one whose size changed, so two builds differ in nearly every 64 KB block. Here each file owns a SLOT (its size
    plus slack: 1/8, within 1..128 KB) at a fixed address, remembered in a JSON map next to the build output: the
    next build keeps a file in its slot when it still fits; a file that outgrew its slot, and every new file, goes to
    the end of everything placed before; a vanished file leaves a hole (zeros). fresh=True (FULL=1) ignores the old map
    and packs compactly. Nothing else changes: run addresses of data/bss are allocated as before."""
    SLACK_DIV, SLACK_MIN, SLACK_MAX, UNIT = 8, 0x400, 0x20000, 16     # slack = size/8, within 1 KB .. 128 KB

    def __init__(self, path, fresh=False):
        import json
        self.path, self.old, self.new, self.kept, self.moved = path, {}, {}, 0, []
        self.region0 = self.tail = 0
        self.old_region0 = self.old_end = None
        if path and not fresh and os.path.isfile(path):
            try:
                d = json.load(open(path))
                self.old = {k: tuple(v) for k, v in d["slots"].items()}
                self.old_region0, self.old_end = d["region0"], d["end"]
            except (ValueError, KeyError, OSError):
                self.old = {}

    def begin(self, region0):
        """The first slotted address; a map made for another region start is dropped."""
        self.region0 = (region0 + 3) & ~3
        if self.old_region0 is not None and self.old_region0 != self.region0:
            print("rombuild: placement map was made for a region at 0x%08X, now 0x%08X: starting fresh"
                  % (self.old_region0, self.region0), file=sys.stderr)
            self.old, self.old_end = {}, None
        self.tail = max(self.region0, self.old_end or 0)

    def reserve(self, name, worst):
        """A slot of at least `worst` bytes for `name`; returns its start address."""
        key, n = name.lower(), 2
        while key in self.new:                      # a name listed twice
            key, n = "%s#%d" % (name.lower(), n), n + 1
        o = self.old.get(key)
        if o and o[1] >= worst:
            start, size = o
            self.kept += 1
        else:
            slack = min(max(worst // self.SLACK_DIV, self.SLACK_MIN), self.SLACK_MAX)
            size = (worst + slack + self.UNIT - 1) // self.UNIT * self.UNIT
            start = (self.tail + 3) & ~3
            self.tail = start + size
            self.moved.append(key)
        self.new[key] = (start, size)
        return start

    def end(self):
        return max(self.tail, max((a + b for a, b in self.new.values()), default=self.region0))

    def save(self):
        import json
        if self.path:
            with open(self.path, "w", newline="\n") as fh:
                json.dump({"region0": self.region0, "end": self.end(),
                           "slots": {k: list(v) for k, v in sorted(self.new.items())}}, fh, indent=0)
        print("rombuild: placement: %d file(s) kept in their slot, %d placed at the end%s; region 0x%08X..0x%08X"
              % (self.kept, len(self.moved), (" (e.g. %s)" % ", ".join(self.moved[:4])) if self.moved else "",
                 self.region0, self.end()), file=sys.stderr)


def placement_from_env(path=None):
    """A SlotMap for ROM_PLACEMENT=<map file> (or the --placement argument); FULL=1 / ROM_PLACEMENT_FRESH=1 repack."""
    path = path or os.environ.get("ROM_PLACEMENT")
    if not path:
        return None
    return SlotMap(path, fresh=os.environ.get("FULL") == "1" or os.environ.get("ROM_PLACEMENT_FRESH") == "1")


class RomBuilder:
    slots = None                     # a SlotMap: images are put at their reserved address (add_xip_file place_at)

    def put(self, off, data):
        """Write `data` at ROM offset `off` (zero-extending the ROM), for slotted files placed out of order."""
        if len(self.rom) < off + len(data):
            self.rom.extend(bytes(off + len(data) - len(self.rom)))
        self.rom[off:off + len(data)] = data

    def pad_to_lin(self, lin_addr):
        if len(self.rom) < lin_addr - self.rom_base:
            self.rom.extend(bytes(lin_addr - self.rom_base - len(self.rom)))

    def __init__(self, bootstrap: bytes, rom_base: int):
        self.rom_base = rom_base
        self.rom = bytearray(bootstrap)
        align4(self.rom)
        self.primary_addr = 0        # primary's TRomImageHeader linear address
        self.primary_entry_addr = 0  # primary's TRomEntry linear address (-> iPrimaryFile)
        self.secondary_addr = 0      # secondary (first user EXE) TRomImageHeader linear address
        self.secondary_entry_addr = 0  # secondary's TRomEntry linear address (-> iSecondaryFile)
        self.est_images = []         # (code base, exception descriptor address) of each exception-aware image

    def lin(self, off):
        return self.rom_base + off

    def add_xip_file(self, payload: bytes, *, uid1=UID1_EXE, uid2=0, uid3=0, entry_off=0,
                     is_primary=False, place_at=None, entry_abs=None, code_size=None,
                     data_addr=0, data_size=0, bss_size=0, stack_size=0x2000,
                     heap_min=0x1000, heap_max=0x100000, data_bss_lin_base=0,
                     priority=0x200, cap1=0, cap2=0,
                     export_count=0, export_addr=0, secure_id=0, vendor_id=0,
                     dll_ref_table=0, extra_flags=0, exc_desc=0, module_version=0):
        """Append a TRomImageHeader + payload; return its linear address and total size.

        For a real kernel: place_at pins the header linear address (the ROM is padded to it,
        since the kernel is not position-independent); entry_abs is the absolute entry point
        (_E32Startup); code_size is the execute-in-place ROM part (text+rodata, < len(payload)
        when .data trails); data_addr = ROM LMA of .data; data_size/bss_size drive the
        bootstrap's .data copy + .bss zero.
        """
        slotted = self.slots is not None and place_at is not None
        if place_at is not None and not slotted:
            target = place_at - self.rom_base
            while len(self.rom) < target:
                self.rom.append(0)
        if slotted:
            addr = place_at
        else:
            align4(self.rom)
            addr = self.lin(len(self.rom))
        code_addr = addr + XIP_HDR_SIZE
        total = XIP_HDR_SIZE + len(payload)
        csz = code_size if code_size is not None else len(payload)
        hdr = bytearray(XIP_HDR_SIZE)
        struct.pack_into("<21I", hdr, 0,
                         uid1, uid2, uid3, uid_checksum(uid1, uid2, uid3),
                         entry_abs if entry_abs is not None else code_addr + entry_off,
                         code_addr,               # code_address
                         data_addr,               # data_address (ROM LMA of .data)
                         csz,                      # code_size (execute-in-place part)
                         csz,                      # text_size
                         data_size, bss_size,
                         heap_min, heap_max,
                         stack_size,
                         dll_ref_table,           # dll_ref_table_address (TDllRefTable, or 0)
                         export_count,            # export_dir_count
                         export_addr,             # export_dir_address
                         secure_id, vendor_id, cap1, cap2)   # SSecurityInfo
        struct.pack_into("<BBH", hdr, 84, 1, 0, 0)          # major, minor, build
        struct.pack_into("<8I", hdr, 88,
                         0x20 | extra_flags,  # @88 iFlags: KRomImageEpt_Eka2 (CHECK_ROM_ENTRY_POINT)
                                    #      + KRomImageFlagData/DataInit/DataPresent (ROM DLL WSD)
                         priority,  # @92  iPriority: must be a valid TProcessPriority for a USER
                                    #      EXE (DProcess::ConvertPriority; e.g. EPriorityForeground
                                    #      =350) or DProcess::Create returns KErrArgument. Ignored
                                    #      for the kernel.
                         data_bss_lin_base,  # @96  iDataBssLinearBase
                         0,        # @100 iNextExtension
                         0,        # @104 iHardwareVariant
                         data_size + bss_size,  # @108 iTotalDataSize (data+bss + DLL reserve).
                                    # MUST be 0 when the EXE has no data/bss, else pcodeseg.cpp:236
                                    # (user proc) faults K::EProcessDataAddressInvalid=KERN 77 when
                                    # iDataBssRunAddress != iDataBssLinearBase. (Was wrongly the
                                    # image size; harmless for the kernel, fatal for a user EXE.)
                         module_version,  # @112 iModuleVersion: the E32 header's (major<<16|minor). The loader matches
                                    #      versioned imports (name{000a0000}.dll = 10.0 or later, every SDK app) against it;
                                    #      0 made each such import "not found" and the process was killed at load
                         # @116 iExceptionDescriptor: the E32 header's (code offset | 1) as the descriptor's ADDRESS, the
                         # present bit cleared, as rombuild does (r_build.cpp: (xd & ~1) + iCodeAddress). The C++ unwinder
                         # reads it through the ROM EST without masking (symbian_support.cpp SearchEST); bit 0 left set made
                         # it read the descriptor one byte off and abort
                         (code_addr + (exc_desc & ~1)) if exc_desc and exc_desc != 0xFFFFFFFF else 0)
        if slotted:
            self.put(addr - self.rom_base, bytes(hdr) + bytes(payload))
        else:
            self.rom += hdr
            self.rom += payload
        if is_primary:
            self.primary_addr = addr
        # rombuild (r_rom.cpp ReserveRomExceptionSearchTable): an image is an EST entry when its exception descriptor
        # is set (bit 0 = ELFTRAN's mark) and not 0xffffffff; the entry is the image's code base (iCodeBase)
        if exc_desc and (exc_desc & 1) and exc_desc != 0xFFFFFFFF:
            self.est_images.append((code_addr, code_addr + (exc_desc & ~1)))
        return addr, total

    def build_est(self):
        """The TRomExceptionSearchTable bytes (r_rom.cpp ConstructRomExceptionSearchTable): iNumEntries, the code
        bases sorted ascending, then a fencepost = the greatest iROSegmentLimit of any entry's TExceptionDescriptor
        (0xffffffff when there is none). b'' when no image is exception-aware."""
        return est_table(self.est_images, lambda a: struct.unpack_from("<I", self.rom, a - self.rom_base + 12)[0])

    # kernel-data ROM header fields (set for a real ekern primary)
    total_user_data = 0      # ROM DLL .data+.bss, page-rounded per DLL (prelink_e32)
    kern_data_base = 0
    kern_data_limit = 0
    total_sv_data = 0
    variant_entry_addr = 0   # the variant file's TRomEntry (-> TRomHeader.iVariantFile)
    variant_img_addr = 0     # the variant file's TRomImageHeader
    trace_mask = 0           # TRomHeader.iTraceMask (KBOOT trace etc.)

    def add_variant_file(self, export_addrs, entry_abs, hw_variant=0x01000000,
                         uid3=0x100039E8):
        """Append a minimal variant TRomImageHeader whose export table points at ekern's
        statically-linked variant functions (ordinal N = export_addrs[N-1]). The kernel's
        A::CreateVariant reads iExportDir[0] (VariantInitialise) to create the Asic."""
        align4(self.rom)
        addr = self.lin(len(self.rom))              # TRomImageHeader linear address
        expdir = addr + XIP_HDR_SIZE                # export table sits right after the header
        hdr = bytearray(XIP_HDR_SIZE)
        struct.pack_into("<21I", hdr, 0,
                         0x10000079, 0, uid3, uid_checksum(0x10000079, 0, uid3),  # uid1=DLL
                         entry_abs,                 # entry_point (_variant_e32dll)
                         expdir,                    # code_address (nominal)
                         0, 0, 0, 0, 0,             # data_addr, code/text/data/bss sizes
                         0x1000, 0x100000, 0x2000,  # heap_min/max, stack
                         0,                         # dll_ref_table_address
                         len(export_addrs),         # export_dir_count  (@60)
                         expdir,                    # export_dir_address (@64)
                         0, 0, 0, 0)
        struct.pack_into("<BBH", hdr, 84, 1, 0, 0)
        struct.pack_into("<8I", hdr, 88,
                         0x20,        # iFlags: KRomImageEpt_Eka2 (CHECK_ROM_ENTRY_POINT)
                         0x200,       # priority
                         0,           # data_bss_linear_base_address
                         0,           # next_extension_linear_address
                         hw_variant,  # iHardwareVariant (@104; top byte<=3 -> compatible)
                         XIP_HDR_SIZE + len(export_addrs) * 4,  # total_size
                         0, 0)
        self.rom += hdr
        for a in export_addrs:
            self.rom += struct.pack("<I", a)
        self.variant_img_addr = addr
        return addr

    def add_builtin_device(self, uid2, dev_id):
        """A device driver (.ldd/.pdd) that is LINKED INTO the kernel image rather
        than shipped as its own kernel DLL. The record is a real TRomImageHeader -- uid1 DLL, uid2
        LDD/PDD, KRomImageFlagDevice, one export -- so F32's loader finds `Sys/Bin/enet.ldd` in
        the Z: tree and the kernel's ExecHandler::DeviceLoad calls export ordinal 1 (the driver
        factory), exactly as for a device in a device ROM. What a ROM builder cannot know here is
        the kernel's own addresses (the section is built before the kernel is linked around it),
        so the entry point and export slot carry `dev_id` and the bootstrap (xboot.cpp) patches
        them with SYMVALs at boot -- the pattern its fabricated variant record already uses."""
        align4(self.rom)
        addr = self.lin(len(self.rom))
        expdir = addr + XIP_HDR_SIZE
        hdr = bytearray(XIP_HDR_SIZE)
        struct.pack_into("<21I", hdr, 0,
                         0x10000079, uid2, 0, uid_checksum(0x10000079, uid2, 0),
                         dev_id,                    # entry point: patched at boot
                         expdir,                    # code_address (nominal, as the variant's)
                         0, 4, 4, 0, 0,             # data_addr, code/text/data/bss sizes
                         0, 0, 0,                   # heap_min/max, stack
                         0,                         # dll_ref_table_address: no imports
                         1,                         # export_dir_count
                         expdir,                    # export_dir_address
                         0, 0,                      # secure id, vendor id
                         0xFFFFFFFF, 0xFFFFFFFF)    # capabilities: loaded into the kernel
        struct.pack_into("<BBH", hdr, 84, 1, 0, 0)
        struct.pack_into("<8I", hdr, 88,
                         0x10000000 | 0x20 | 0x1,   # KRomImageFlagDevice | Eka2 entry | KImageDll
                         0x200, 0, 0,
                         0x01000000,                # iHardwareVariant: compatible
                         XIP_HDR_SIZE + 4, 0, 0)
        self.rom += hdr
        self.rom += struct.pack("<I", dev_id)       # export ordinal 1: patched at boot
        return addr, XIP_HDR_SIZE + 4


    def _dir_block(self, entries):
        """entries: list of (name, addr_lin, size, is_dir). Returns the packed rom_dir.

        REAL TRomDir format (e32rom.h + srom/sr_rom.cpp -- the fileserver's rom fs
        binary-searches these):
          int32 iSize            bytes of the TRomEntry region ONLY (not this field)
          TRomEntry iEntry...    each {i32 size, u32 addr_lin, u8 att, u8 nameLen,
                                 utf16 name}, 4-aligned
          TRomDirSortInfo        at &iEntry + align4(iSize):
              u16 iSubDirCount, u16 iFileCount,
              u16 subdir_offsets[iSubDirCount]   (entry_off_from_first_entry / 4)
              u16 file_offsets[iFileCount]
          each table sorted ascending by CRomMountCB::Compare (ASCII fold to
          lowercase over the UTF-16 units, ties broken shorter-first).
        """
        body = bytearray()
        offsets = []                       # byte offset of each entry within the block
        for name, addr_lin, size, is_dir, attrib in entries:
            offsets.append(4 + len(body))  # +4 for the leading int32 dir size
            nm = name.encode("utf-16-le")
            if is_dir:
                attrib = FILE_ATTRIB_DIR
            body += struct.pack("<IIBB", size, addr_lin, attrib, len(name))
            body += nm
            while len(body) % 4:
                body.append(0)

        def sort_key(i):
            name = entries[i][0]
            folded = tuple(ord(c.lower() if 'A' <= c <= 'Z' else c) for c in name)
            return (folded, len(name))
        subdirs = sorted((i for i, e in enumerate(entries) if e[3]), key=sort_key)
        files   = sorted((i for i, e in enumerate(entries) if not e[3]), key=sort_key)
        sort_info = struct.pack("<HH", len(subdirs), len(files))
        for i in subdirs + files:
            sort_info += struct.pack("<H", (offsets[i] - 4) // 4)  # scaled from first entry
        while len(sort_info) % 4:
            sort_info += b"\0"
        return struct.pack("<i", len(body)) + body + sort_info, offsets

    def add_tree(self, files):
        """files: list of (z_path, addr_lin, size[, attrib]). Builds the tree bottom-up.

        z_path like 'Sys\\Bin\\ekern.exe' (no drive).  Returns root_dir_list lin addr.
        attrib defaults to ReadOnly|XIP (0x81, an XIP E32Image -- without KEntryAttXIP
        the loader ignores the ROM address, RFile::Att gate in sf_cache.cpp); plain
        data files (e.g. estart.txt) pass 0x01 (ReadOnly).
        """
        # build a nested dict:  name -> either ('file', addr, size, attrib) or subdict
        root = {}
        for entry in files:
            path, addr, size = entry[0], entry[1], entry[2]
            attrib = entry[3] if len(entry) > 3 else 0x81
            parts = path.replace("/", "\\").strip("\\").split("\\")
            node = root
            for d in parts[:-1]:
                # ROM names are case-insensitive: `resource\apps` must merge into an
                # existing `Resource` dir (two entries folding equal = one is unreachable)
                d = next((k for k in node if k.lower() == d.lower()
                          and isinstance(node[k], dict)), d)
                node = node.setdefault(d, {})
            node[parts[-1]] = ("file", addr, size, attrib)

        def emit(node):
            """Emit sub-dir blocks first (need their addrs), then this dir block. Returns addr."""
            entries = []
            for name, val in sorted(node.items()):
                if isinstance(val, tuple):
                    entries.append((name, val[1], val[2], False, val[3]))
                else:
                    sub_addr = emit(val)
                    # size of a dir entry's target is unused by the parser; use its block size
                    entries.append((name, sub_addr, 0, True, 0))
            block, offsets = self._dir_block(entries)
            align4(self.rom)
            addr = self.lin(len(self.rom))
            self.rom += block
            # iPrimaryFile must point at the primary file's TRomEntry (its iAddressLin@4
            # then points at the TRomImageHeader), not at the header itself.
            for (name, tgt, size, is_dir, _att), off in zip(entries, offsets):
                if not is_dir and self.primary_addr and tgt == self.primary_addr:
                    self.primary_entry_addr = addr + off
                if not is_dir and tgt == getattr(self, "variant_img_addr", 0) and self.variant_img_addr:
                    self.variant_entry_addr = addr + off
                if not is_dir and self.secondary_addr and tgt == self.secondary_addr:
                    self.secondary_entry_addr = addr + off
            return addr

        root_addr = emit(root)
        align4(self.rom)
        rdl_addr = self.lin(len(self.rom))
        self.rom += struct.pack("<iII", 1, 0, root_addr)   # num_root=1, hw_variant, addr
        return rdl_addr

    def finalize(self, root_dir_list_addr):
        align4(self.rom)
        est = self.build_est()
        if est:                          # rombuild reserves it after the files; here after the directory tree
            struct.pack_into("<I", self.rom, H_iRomExceptionSearchTable, self.lin(len(self.rom)))
            self.rom += est
            align4(self.rom)
        # iTime is a TTimeK: microseconds since midnight 1 Jan year 0 AD (Symbian epoch),
        # NOT the Unix epoch. P::InitSystemTime falls back to this ROM timestamp when the RTC
        # read fails and passes it to K::SecondsFrom2000, which range-checks (result/1e6 -
        # Year2000InSeconds) against +/-TInt; a Unix-epoch value underflows -> KErrOverflow ->
        # PP::Panic(EInitialSystemTimeInvalid) = Kern::Fault("PLAT", 0). Convert to the year-0
        # epoch. Year2000InSeconds = KDaysFrom0ADTo2000AD(730497) * 86400; Unix time of
        # 2000-01-01T00:00:00Z = 946684800.
        K_YEAR2000_IN_SECONDS = 730497 * 86400
        K_UNIX_2000 = 946684800
        itime_symbian = (K_YEAR2000_IN_SECONDS + (int(time.time()) - K_UNIX_2000)) * 1_000_000
        struct.pack_into("<q", self.rom, H_iTime, itime_symbian)
        struct.pack_into("<I", self.rom, H_iRomBase, self.rom_base)
        # A whole number of pages, as a real rombuild image: the bootstrap's ROM bank size (-> the super page's
        # many bytes of ROM pages -- a non-page-multiple is KErrArgument -> FAULT mrom.cpp:475.
        self.rom.extend(b"\0" * (-len(self.rom) % 0x1000))
        struct.pack_into("<I", self.rom, H_iRomSize, len(self.rom))
        struct.pack_into("<I", self.rom, H_iUncompressedSize, len(self.rom))
        struct.pack_into("<I", self.rom, H_iPageableRomStart, 0)
        struct.pack_into("<I", self.rom, H_iRomRootDirectoryList, root_dir_list_addr)
        struct.pack_into("<I", self.rom, H_iPrimaryFile, self.primary_entry_addr)
        # No SMP compatibility mode: apps not marked SMP safe run on every core. (Fragger / Gravity Guy's "Mixer" crash,
        # first blamed on that, was a kernel debug hook stalling every new thread on the SD lock -- ekern_seam.cpp
        # ekseam_dump_thread. EKernelConfigSMPUnsafeCompat was followed by a silent hang after a crashing game, boot 2199;
        # EKernelConfigSMPUnsafeCPU0 pins those apps to one core.)
        struct.pack_into("<I", self.rom, H_iKernelConfigFlags, 0)
        if self.kern_data_base:
            struct.pack_into("<I", self.rom, H_iKernDataAddress, self.kern_data_base)
            struct.pack_into("<I", self.rom, H_iKernelLimit, self.kern_data_limit)
            struct.pack_into("<I", self.rom, H_iTotalSvDataSize, self.total_sv_data)
            # user-data area (multiple MM Mmu::DoInit2 consistency asserts). ROM DLL static data
            # is laid out DOWNWARD from iUserDataAddress (prelink_e32); the kernel reserves
            # iTotalUserDataSize (page multiple) at the top of every process's DLL-data chunk
            struct.pack_into("<I", self.rom, H_iUserDataAddress,
                             getattr(self, "user_data_top", KUserDataAddress_MultipleMM))
            struct.pack_into("<I", self.rom, H_iTotalUserDataSize, self.total_user_data)
        if self.variant_entry_addr:
            struct.pack_into("<I", self.rom, H_iVariantFile, self.variant_entry_addr)
        if self.secondary_entry_addr:
            struct.pack_into("<I", self.rom, H_iSecondaryFile, self.secondary_entry_addr)
        if self.trace_mask:
            struct.pack_into("<I", self.rom, H_iTraceMask, self.trace_mask)
        struct.pack_into("<I", self.rom, H_iCheckSum, 0)
        s = sum(struct.unpack_from("<%dI" % (len(self.rom) // 4), self.rom, 0)) & 0xFFFFFFFF
        struct.pack_into("<I", self.rom, H_iCheckSum, (-s) & 0xFFFFFFFF)
        return bytes(self.rom)


# ── XIP prelink: place our E32Images in ROM with imports resolved ────────────
# What real rombuild does for XIP files: each emitted E32Image (emitter/) is
# assigned its final ROM address, its relocations are applied (TEXT += the
# code delta, DATA += the data delta), and every import slot -- the veneer's
# export's ROM address. At runtime nothing is left to fix up: the images are
# plain execute-in-place ROM code calling each other at absolute addresses.
# (euser/efsrv/hal have ZERO static data, so no ROM-DLL data machinery needed;
# the EXE's .data/.bss are handled by the kernel's XIP process-create path via
# the TRomImageHeader fields, like the kernel's own data.)

E32_UID1_DLL = 0x10000079
RELOC_TEXT_NIBBLE = 0x1000
RELOC_DATA_NIBBLE = 0x2000


class E32Xip:
    """A parsed, uncompressed E32Image ready for XIP placement."""

    def __init__(self, path, zname):
        self.path, self.zname = path, zname
        d = open(path, "rb").read()
        if struct.unpack_from("<I", d, 28)[0] != 0:
            raise ValueError(f"{path}: compressed E32Image not supported for XIP")
        g = lambda off: struct.unpack_from("<I", d, off)[0]
        self.uid1, self.uid2, self.uid3 = g(0), g(4), g(8)
        self.is_exe = self.uid1 != E32_UID1_DLL
        self.code_size, self.data_size = g(48), g(52)
        self.heap_min, self.heap_max, self.stack = g(56), g(60), g(64)
        self.bss_size, self.entry_off = g(68), g(72)
        self.link_code_base, self.link_data_base = g(76), g(80)
        ref_count, exp_off, self.export_count = g(84), g(88), g(92)
        code_off, data_off, imp_off = g(100), g(104), g(108)
        creloc_off, dreloc_off = g(112), g(116)
        self.priority = struct.unpack_from("<H", d, 120)[0]
        # E32ImageHeaderV SSecurityInfo @128: the SID must survive into the
        # TRomImageHeader -- ECOM rejects a plug-in whose SID != its UID3
        # (CDllData::SaveSecurityInfoL), and servers police clients by SID.
        self.secure_id, self.vendor_id = g(128), g(132)
        self.cap1, self.cap2 = g(136), g(140)       # SSecurityInfo.iCaps (the image header's capabilities)
        self.module_version = g(24)  # E32ImageHeader iModuleVersion
        # the ABI bits (KImageABIMask == KRomImageABIMask 0x18; EABI = 0x08): the loader refuses an explicit RLibrary::Load of a
        # DLL whose ABI differs from the process (sf_lepoc.cpp ABIFAIL) -- with 0 here every ROM DLL was 'GCC98r2' and an SDK
        # (EABI) app's RLibrary::Load("z:euser") got KErrNotSupported
        self.abi_flags = g(44) & 0x18
        # SMP safe: KImageSMPSafe in the E32 header, or tools version 0.0.0 = our own emitter (images it wrote before it
        # set the flag); an SDK-built image says 2.x and keeps the compatibility mode
        self.smp_safe = bool(g(44) & 0x4000) or d[32:36] == b"\0\0\0\0"
        self.exc_desc = g(144)       # E32ImageHeaderV iExceptionDescriptor (code offset | 1; 0 = no unwind tables)
        self.code = bytearray(d[code_off:code_off + self.code_size])
        self.data = bytearray(d[data_off:data_off + self.data_size] if data_off else b"")
        self.exports = list(struct.unpack_from(f"<{self.export_count}I", d, exp_off)
                            ) if self.export_count else []
        self.code_relocs = self._parse_relocs(d, creloc_off)
        self.data_relocs = self._parse_relocs(d, dreloc_off)
        self.imports = self._parse_imports(d, imp_off, ref_count)
        # assigned at layout time:
        self.hdr_addr = self.code_addr = self.data_lma = self.export_addr = 0
        self.payload = b""
        # ROM DLL writable static data (prelink_e32): run address, "has data" closure, and the
        # TDllRefTable = the direct dependencies the loader must build code segments for
        self.own_wsd = self.data_present = False
        self.data_run = self.ref_table_addr = 0
        self.ref_deps = []
        self.cpu = struct.unpack_from("<H", d, 122)[0]
        # What the linker assumed about placement. 4 unless a sidecar says otherwise, which
        # that faults on a misaligned datum.
        self.code_align = self.data_align = 4
        self.pcimp = []
        self.pcrel_data = self._read_pcrel_sidecar()
        # ARM images: NEON code (FFmpeg) does 128-bit aligned vld1/vst1 (:128) on DECLARE_ALIGNED constants in the code
        # segment, so the image code must sit on a 16-byte boundary (the ELF link base is 0x8000, 16-aligned); an unaligned
        # base is an alignment fault (fsr 0x1) on the A53. Costs <= 12 bytes of padding per image.
        if self.cpu != CPU_X86 and self.code_align < 16:
            self.code_align = 16

    def _read_pcrel_sidecar(self):
        """x32 images carry their %rip-relative code->data sites beside them (see
        emitter/e32image.py write_pcrel_sidecar). They cannot live in the E32Image, because
        the fixup they need -- (data_delta - code_delta) -- is not a thing E32's two-delta
        relocation model can express; only a PRELINKER, which fixes both addresses once and
        for all, can apply it.

        An x32 image WITH writable static data and no sidecar is the exact shape that boots
        and then writes to ROM, so refuse it rather than emit it."""
        path = self.path + ".pcrel"
        if not os.path.exists(path):
            if self.cpu == CPU_X86 and (self.data_size or self.bss_size):
                raise ValueError(
                    "%s: x32 image with %d B data + %d B bss and no %s. Rebuild it with the "
                    "emitter: without that list its %%rip-relative references to its own "
                    "statics stay aimed at ROM." % (self.path, self.data_size, self.bss_size,
                                                    os.path.basename(path)))
            return []
        offs, declared = [], None
        for line in open(path):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            key, _, val = line.partition(" ")
            if key in ("code_base", "data_base"):
                continue
            if key in ("code_align", "data_align"):
                setattr(self, key, int(val, 0))
                continue
            if key == "count":
                declared = int(val)
                continue
            if key == "pcimp":
                dll, ordn, off, addend = val.split()
                self.pcimp.append((dll, int(ordn), int(off, 16), int(addend)))
                continue
            offs.append(int(line, 16))
        if declared is not None and declared != len(offs):
            raise ValueError("%s: says count %d but lists %d" % (path, declared, len(offs)))
        for off in offs:
            if not 0 <= off <= self.code_size - 4:
                raise ValueError("%s: site 0x%X is outside the %d-byte code"
                                 % (path, off, self.code_size))
        return offs

    @staticmethod
    def _parse_relocs(d, off):
        if not off:
            return []
        # E32RelocSection.iSize is the size of the BLOCKS, not counting its own 8-byte header (elf2e32
        # e32imagefile.cpp, F32's ValidateRelocations). Older images counted the header, and
        # images of both kinds are about, so walk by the relocation COUNT, which both agree on.
        _size, total = struct.unpack_from("<II", d, off)
        out, p = [], off + 8
        while len(out) < total:
            base, bsize = struct.unpack_from("<II", d, p)
            assert bsize >= 8, f"bad relocation block at {p:#x}"
            for k in range((bsize - 8) // 2):
                (info,) = struct.unpack_from("<H", d, p + 8 + 2 * k)
                if info:
                    out.append((base + (info & 0xFFF), info & 0xF000))
            p += bsize
        return out

    @staticmethod
    def _parse_imports(d, off, ref_count):
        blocks, p = [], off + 4
        for _ in range(ref_count):
            name_off, n = struct.unpack_from("<II", d, p)
            p += 8
            offs = list(struct.unpack_from(f"<{n}I", d, p))
            p += 4 * n
            nm = d[off + name_off:d.index(b"\x00", off + name_off)].decode()
            blocks.append((nm, offs))
        return blocks

    def export_run(self, e):
        """The run-time address of an export given at its link address. An exported STATIC (Qt 4 exports
        QListData::shared_null & co, which its inline code refcounts) lives in the DLL's data, so it moves with the
        DATA delta -- to the DLL's one data address in RAM -- not with the code: given the code delta, the entry
        pointed at the initial copy of the data inside the ROM image and the first write faulted."""
        dsz = self.data_size + self.bss_size
        if dsz and self.data_run and self.link_data_base <= e < self.link_data_base + dsz:
            return e - self.link_data_base + self.data_run
        return e - self.link_code_base + self.code_addr

    def resolve(self, by_name, data_run_base):
        """Apply relocs + imports for the assigned addresses; build payload."""
        dcode = self.code_addr - self.link_code_base
        ddata = (data_run_base - self.link_data_base) if self.data_size or self.bss_size else 0
        # Relocation is a SHIFT, so it preserves alignment only when the shift is a multiple
        # of it. Both are checked here rather than assumed: on x86-64 a misaligned `movdqa`
        # is a #GP, not a slow access, and it shows up as a fault in a constructor with no
        # hint of where it came from.
        if dcode % self.code_align:
            raise ValueError("%s: code placed at 0x%08X, %d off the %d-byte alignment its "
                             "link base 0x%08X assumed"
                             % (self.zname, self.code_addr, dcode % self.code_align,
                                self.code_align, self.link_code_base))
        if ddata % self.data_align:
            raise ValueError("%s: static data placed at 0x%08X, %d off the %d-byte alignment "
                             "its link base 0x%08X assumed"
                             % (self.zname, data_run_base, ddata % self.data_align,
                                self.data_align, self.link_data_base))

        dsz = self.data_size + self.bss_size

        def fix(buf, off, typ):
            (w,) = struct.unpack_from("<I", buf, off)
            if typ == 0x3000:		# inferred (the export table, e32image.py): data if the value lies in the data
                typ = RELOC_DATA_NIBBLE if dsz and self.link_data_base <= w < self.link_data_base + dsz \
                    else RELOC_TEXT_NIBBLE
            w = (w + (dcode if typ == RELOC_TEXT_NIBBLE else ddata)) & 0xFFFFFFFF
            struct.pack_into("<I", buf, off, w)

        for off, typ in self.code_relocs:
            if typ == 0x4000:
                continue        # KX32PcRelRelocType: the same sites as the sidecar, re-aimed below
            fix(self.code, off, typ)
        for off, typ in self.data_relocs:
            fix(self.data, off, typ)
        # %rip-relative code->data: the displacement is a DISTANCE, not an address, so it
        # moves by the DIFFERENCE of the two deltas rather than by either one of them. And
        # a distance is what x86-64 gives only 32 signed bits for, which turns a relocation
        # question into an ADDRESS MAP question: XIP code and the data it reaches have to be
        # within 2 GiB of each other, or the reference cannot be encoded at all.
        for off in self.pcrel_data:
            (w,) = struct.unpack_from("<i", self.code, off)
            nw = w + (ddata - dcode)
            if not -0x80000000 <= nw < 0x80000000:
                raise ValueError(
                    "%s: a %%rip-relative reference from code at 0x%08X to its static data "
                    "near 0x%08X needs a displacement of %d bytes; x86-64 encodes 32 signed "
                    "bits. ROM code and the writable segment must be within 2 GiB -- see "
                    "KX64ExeDataBase (the kernel's mmboot.h)."
                    % (self.zname, self.code_addr + off, data_run_base, nw))
            struct.pack_into("<i", self.code, off, nw)
        # %rip-relative references to IMPORTED statics (sidecar pcimp lines): disp = export + A - site
        for dll, ordn, off, addend in self.pcimp:
            tgt = by_name.get(dll.split("{")[0].lower())
            if tgt is None:
                raise ValueError(f"{self.zname}: imports data from {dll} which is not in the ROM bundle")
            nw = tgt.export_run(tgt.exports[ordn - 1]) + addend - (self.code_addr + off)
            if not -0x80000000 <= nw < 0x80000000:
                raise ValueError(f"{self.zname}: %rip reference to {dll} ordinal {ordn} out of 32-bit range")
            struct.pack_into("<i", self.code, off, nw)
        for dll, offs in self.imports:
            key = dll.split("{")[0].lower()
            tgt = by_name.get(key)
            if tgt is None:
                raise ValueError(f"{self.zname}: imports {dll} which is not in the ROM bundle")
            for off in offs:
                (w,) = struct.unpack_from("<I", self.code, off)
                ordn, adj = w & 0xFFFF, w >> 16
                if adj >= 0x8000:
                    adj -= 0x10000      # a signed 16-bit addend (Qt: &QVectorData::shared_null - 8)
                if not 1 <= ordn <= tgt.export_count:
                    raise ValueError(f"{self.zname}: ordinal {ordn} out of range for {dll}")
                # export entries are ABSOLUTE link-space addresses (code_base+off)
                exp = tgt.export_run(tgt.exports[ordn - 1])
                struct.pack_into("<I", self.code, off, (exp + adj) & 0xFFFFFFFF)

        parts = bytes(self.code)
        if self.data:
            parts += b"\x00" * (-len(parts) % 4) + bytes(self.data)
        if self.exports:
            reb = [self.export_run(e) & 0xFFFFFFFF for e in self.exports]
            parts += b"\x00" * (-len(parts) % 4)
            self.export_addr = self.code_addr + len(parts)
            # CheckRomExports (sf_lepoc.cpp) tests xd[ordinal] with xd = iExportDir, i.e. the entry AFTER the one an
            # import names: for the LAST ordinal it reads the word past the table and refuses the import (KErrNotSupported)
            # when that is 0. The ROM builder links statically and never hits it; a RAM-loaded image (SIS) does.
            parts += struct.pack(f"<{len(reb)}I", *reb) + struct.pack("<I", 0xFFFFFFFF)
        if self.ref_deps:
            # TDllRefTable { TUint16 iFlags; TUint16 iNumberOfEntries; TRomImageHeader* iEntry[]; }
            parts += b"\x00" * (-len(parts) % 4)
            assert self.ref_table_addr == self.code_addr + len(parts), self.zname
            parts += struct.pack(f"<HH{len(self.ref_deps)}I", 0, len(self.ref_deps),
                                 *[d.hdr_addr for d in self.ref_deps])
        self.payload = parts


LAST_XIP_MAP = []


def prelink_e32(rom_builder, imgs, exe_data_run_base, data_top=KUserDataAddress_MultipleMM,
                flexible=None, min_block=0x1000):
    """Assign ROM addresses for the E32 bundle (sequential append), resolve, and
    return per-image (payload, fields) ready for add_xip_file. Layout must match
    exactly what add_xip_file will do: align4 -> 120-byte header -> payload."""
    by_name = {}
    for img in imgs:
        by_name[img.zname.rsplit("\\", 1)[-1].lower()] = img
    # PRUNE_UNRESOLVED=1: drop (to a fixpoint) images whose imports are not in the bundle instead of failing --
    # for a grab-bag GUI ROM over a partly stale emitter/ (the QEMU virt GUI=1 build); a named note per drop
    if os.environ.get("PRUNE_UNRESOLVED") == "1":
        while True:
            bad = [img for img in imgs
                   if any(n.split("{")[0].lower() not in by_name for n, _ in list(img.imports) +
                          [(d, None) for d, *_ in img.pcimp])]
            if not bad:
                break
            for img in bad:
                miss = sorted({n.split("{")[0].lower() for n, _ in img.imports} - set(by_name))
                print(f"rombuild: pruned {img.zname} (imports {', '.join(miss) or 'data'} not in bundle)",
                      file=sys.stderr)
                imgs.remove(img)
                by_name.pop(img.zname.rsplit("\\", 1)[-1].lower(), None)   # a name can be listed twice
    # --- ROM DLL writable static data (what real rombuild does; e32rom.h / sf_lepoc.cpp) ---
    # A DLL with .data/.bss gets a fixed run address carved DOWNWARD from 0x40000000 and
    # KRomImageFlagData. "DataPresent" = self or any dependency has data: the loader skips
    # code-segment creation for a ROM DLL WITHOUT it (E32Image::AlwaysLoaded) and walks
    # iDllRefTable for the rest, so every image lists its DataPresent direct dependencies;
    # the kernel then commits + initialises the data per process (DEpocCodeSeg::InitData).
    top = data_top
    for img in imgs:
        img.deps = [d for d in (by_name.get(n.split("{")[0].lower()) for n, _ in img.imports)
                    if d is not None and d is not img]
        img.own_wsd = (not img.is_exe) and (len(img.data) + img.bss_size) > 0
        img.data_present = img.own_wsd
    wsd = [img for img in imgs if img.own_wsd]
    if flexible is None:
        flexible = data_top == KUserLocalDataEnd_Flexible
    rom_builder.user_data_top = data_top
    if flexible:
        # The flexible model maps each DLL's data at its fixed address through RVirtualAllocator,
        # which widens every request to a NATURALLY ALIGNED power-of-two block (mvalloc.cpp
        # AdjustRegion). So each block is sized to a power of two and aligned to it, largest first
        # (the small ones then pack beneath without straddling). A 64 KB block at 0x7FFED000
        # would become a 128 KB block over three 4 KB neighbours -> KErrAlreadyExists at process load.
        # min_block: ARM's flexible model has 4 page colours (nk_cpu.h __CPU_CACHE_HAS_COLOUR on every
        # ARMv6+ core -> KPageColourShift 2): a fixed-address mapping must start on a 16 KB colour boundary
        # (mmapping.cpp "wrong colour" -> KErrArgument from ProcessLoaded), and RVirtualAllocator works in
        # 16 KB units. x86 has no colouring (4 KB).
        def pow2(n):
            p = min_block
            while p < n:
                p <<= 1
            return p
        for img in sorted(wsd, key=lambda i: -pow2(len(i.data) + i.bss_size)):
            size = pow2(len(img.data) + img.bss_size)
            top = (top - size) & ~(size - 1)
            img.data_run = top
    else:
        for img in wsd:
            top -= (len(img.data) + img.bss_size + 0xFFF) & ~0xFFF
            img.data_run = top
    rom_builder.total_user_data = data_top - top
    changed = True
    while changed:                                   # closure over the (cyclic) import graph
        changed = False
        for img in imgs:
            if not img.data_present and any(d.data_present for d in img.deps):
                img.data_present = changed = True
    # iDllRefTable exactly as the 2010 rombuild (r_rom.cpp E32Rom::ProcessDependencies / TRomFile::MarkDeps): the FLATTENED transitive
    # closure of the image's dependencies, filtered to the EXEs (first) plus the DLLs that OWN data (KRomImageFlagData == own_wsd), in
    # image order -- NOT the direct DataPresent deps (that made one WSD DLL, e.g. a euser with a 4-byte bss, drag every intermediate
    # DataPresent DLL into every table and the kernel's ListDeps closure past 256 -> KERN-EXEC 39). An image that reaches an EXE gets
    # KRomImageFlagExeInTree; EXE->other EXE and DLL->two EXEs are errors in the real tool (warnings here).
    order = {id(i): n for n, i in enumerate(imgs)}
    old_total = sum(len([d for d in img.deps if d.data_present]) for img in imgs)
    for img in imgs:
        seen, stack = set(), list(img.deps)
        while stack:
            d = stack.pop()
            if id(d) in seen:
                continue
            seen.add(id(d))
            stack.extend(d.deps)
        reached = sorted((i for i in imgs if id(i) in seen), key=lambda i: order[id(i)])
        exes = [i for i in reached if i.is_exe]
        keep = [i for i in reached if not i.is_exe and i.own_wsd]
        if not img.is_exe and len(exes) > 1:
            print(f"rombuild: WARNING DLL {img.zname} links to more than one EXE: {[e.zname for e in exes]}", file=sys.stderr)
        if img.is_exe and any(e is not img for e in exes):
            print(f"rombuild: WARNING EXE {img.zname} links to other EXEs: {[e.zname for e in exes if e is not img]}", file=sys.stderr)
        img.exe_in_tree = bool(exes)
        img.ref_deps = exes + keep
    new_total = sum(len(img.ref_deps) for img in imgs)
    print(f"rombuild: dll-ref tables: {old_total} direct-DataPresent entries (old) -> {new_total} flattened-KEEP entries (2010 ProcessDependencies); "
          f"max per image {max((len(i.ref_deps) for i in imgs), default=0)}", file=sys.stderr)
    cur = len(rom_builder.rom)
    for img in imgs:
        if rom_builder.slots is not None:
            # the file's slot: its worst-case size (header + payload + the code-alignment padding) decides whether the old slot fits
            worst = XIP_HDR_SIZE + len(img.code) + img.code_align + 16
            if img.data:
                worst += len(img.data) + 4
            if img.exports:
                worst += 4 * len(img.exports) + 8
            if img.ref_deps:
                worst += 8 + 4 * len(img.ref_deps)
            cur = rom_builder.slots.reserve(img.zname, worst) - rom_builder.rom_base
        cur = (cur + 3) & ~3
        # An image keeps its INTERNAL alignment only if it lands congruent to its link base.
        # The XIP header is 120 bytes -- 24 mod 32 -- so a 32-aligned header would put a
        # 32-aligned .text on an odd multiple of 8, and x86-64's `movdqa` on a 16-byte
        # constant then #GPs. Pad the header back so the CODE, not the header, is aligned.
        a = img.code_align
        while (rom_builder.lin(cur) + XIP_HDR_SIZE - img.link_code_base) % a:
            cur += 4
        img.hdr_addr = rom_builder.lin(cur)
        img.code_addr = img.hdr_addr + XIP_HDR_SIZE
        # payload = code [+ pad + data] [+ pad + export table]
        plen = len(img.code)
        if img.data:
            plen += (-plen % 4)
            img.data_lma = img.code_addr + plen
            plen += len(img.data)
        if img.exports:
            plen += (-plen % 4) + 4 * len(img.exports) + 4   # + the guard word (below)
        if img.ref_deps:
            plen += (-plen % 4)
            img.ref_table_addr = img.code_addr + plen
            plen += 4 + 4 * len(img.ref_deps)
        cur += XIP_HDR_SIZE + plen
    for img in imgs:
        img.resolve(by_name, exe_data_run_base if img.is_exe else img.data_run)
    # placement map for a symbolizer: name, ROM code addr, code len, link base
    global LAST_XIP_MAP
    LAST_XIP_MAP = [(img.zname.rsplit("\\", 1)[-1].lower(), img.code_addr,
                            len(img.code), img.link_code_base) for img in imgs]


# --------------------------------------------------------------------------------------------
# x86/x32: a ROM SECTION rather than a whole ROM
# --------------------------------------------------------------------------------------------
# On x86/x32 the ROM *is* the kernel image and its TRomHeader
# is written at run time by the bootstrap, so there is no container to build here -- what is
# needed is the part a container would hold: the XIP images, prelinked, with their TRomImageHeader
# and TRomEntry records, laid out for one fixed linear base. That blob is .incbin'd into the
# kernel image (xk_uimage.S) at the address it was laid out for, and xboot.cpp points
# TRomHeader.iSecondaryFile straight at a record inside it instead of fabricating one.
#
# Everything below the section header is REAL ROM structure. The prelinking is the same
# arch-neutral machinery the ARM path uses (E32Xip/prelink_e32): E32 relocations are word
# patches, import slots are `ordinal | adj<<16` words, and export entries are absolute
# link-space addresses -- none of that is ARM- or x86-specific.
#
# v2 carries a REAL DIRECTORY TREE. v1 had a flat table of TRomEntry records, which was enough
# for the kernel (it only ever dereferences iSecondaryFile) but not for the FILESERVER: CRom
# mounts Z: from TRomRootDirectoryList -> TRomDir blocks and binary-searches them, so a ROM
# without a tree has no files in it. add_tree()/_dir_block() already emit the real format for
# the ARM path; the section format just had to grow a place to put them.
SECTION_MAGIC = 0x43535250          # 'PRSC', little-endian
SECTION_VERSION = 4                 # v3: word 7 = the ROM DLL data total (-> iTotalUserDataSize)
                                    # v4: word 8 = the built-in device table (0 = none):
                                    #   { count, (TRomImageHeader address, device id) * count }
SECTION_HDR_SIZE = 48
KROM_ENTRY_SIZE = 10                # e32rom.h KRomEntrySize: sizeof(TRomEntry) - 2 (name array)
FILE_ATTRIB_RO = 0x01               # KEntryAttReadOnly
FILE_ATTRIB_RO_XIP = 0x81           # ... | KEntryAttXIP (an execute-in-place E32Image)


def build_xip_section(imgs, base, secondary_name, exe_data_run_base=0, data_files=None,
                      builtin_devices=None, placement=None, data_top=KUserLocalDataEnd_Flexible):
    """Lay `imgs` (E32Xip) out as a self-describing prelinked XIP section based at `base`.

    Layout:  [32-byte section header]
             [align4, TRomImageHeader + payload, per image]
             [align4, raw bytes, per plain data file]
             [the directory tree: a TRomDir block per directory, deepest first, then the
              TRomRootDirectoryList that TRomHeader.iRomRootDirectoryList points at]
    data_files = [(z_path, bytes)] for non-E32 ROM content (estart.txt and friends).
    Returns (blob, info) where info lists (zname, hdr_addr, code_addr, size) per entry.
    """
    b = RomBuilder(bytes(SECTION_HDR_SIZE), base)
    info = []
    tree = []                       # (z_path, addr_lin, file_size, attrib) -> add_tree

    # Plain (non-E32) ROM files FIRST: resources and skins rarely change, the
    # XIP images after them do -- a rebuilt EXE near the end of the ROM then changes only the blocks after it (the
    # TRomImageHeader: the fileserver reads them straight out of the ROM at the address their TRomEntry carries.
    slots = placement_from_env(placement)
    if slots is not None:
        align4(b.rom)
        slots.begin(b.lin(len(b.rom)))
        b.slots = slots
    for zname, blob in (data_files or []):
        if slots is not None:
            addr = slots.reserve("d:" + zname, len(blob) + 4)
            b.put(addr - b.rom_base, blob)
        else:
            align4(b.rom)
            addr = b.lin(len(b.rom))
            b.rom += blob
        info.append((zname, addr, addr, len(blob)))
        tree.append((zname, addr, len(blob), FILE_ATTRIB_RO))

    prelink_e32(b, imgs, exe_data_run_base, data_top=data_top, flexible=True)
    for img in sorted(imgs, key=lambda i: i.hdr_addr):
        addr, size = b.add_xip_file(
            img.payload, place_at=img.hdr_addr,
            uid1=img.uid1, uid2=img.uid2, uid3=img.uid3,
            entry_abs=img.code_addr + img.entry_off, code_size=len(img.code),
            data_addr=img.data_lma, data_size=len(img.data),
            bss_size=img.bss_size if (img.is_exe or img.own_wsd) else 0,
            **_exe_limits(img),
            data_bss_lin_base=exe_data_run_base if img.is_exe else img.data_run,
            dll_ref_table=img.ref_table_addr,
            # KImageDll: the loader's ONLY "is a DLL" test for a ROM image (E32Image::Construct)
            extra_flags=(0 if img.is_exe else 0x00000001)
                        | img.abi_flags | (KROM_IMAGE_SMP_SAFE if img.smp_safe else 0)
                        | (0x04000000 if img.own_wsd else 0)
                        | (0x00800000 if getattr(img, 'exe_in_tree', False) else 0)
                        | (0x03000000 if img.data_present else 0),
            priority=(img.priority or 350) if img.is_exe else 0x200,
            # An EXE runs with its own header capabilities (build_mmp writes the mmp's CAPABILITY): a process that held every
            # capability, TCB included, could never load or list an engine plug-in installed on C: (ECom and the loader require
            # the plug-in's capabilities to cover the client's). An EXE whose header says none keeps all of them (an image
            # from a tool that does not write capabilities). DLLs keep all capabilities: such a DLL loads into every process.
            cap1=(_cap_override(img.zname) or _header_caps(img, 0xFFFFFFFF, 0xFFFFFFFF))[0],
            cap2=(_cap_override(img.zname) or _header_caps(img, 0xFFFFFFFF, 0xFFFFFFFF))[1],
            export_count=img.export_count, export_addr=img.export_addr,
            secure_id=img.secure_id, vendor_id=img.vendor_id, exc_desc=img.exc_desc,
            module_version=img.module_version)
        assert addr == img.hdr_addr, f"{img.zname}: layout drift 0x{addr:X} != 0x{img.hdr_addr:X}"
        info.append((img.zname, addr, img.code_addr, size))
        # Each TRomEntry points at the TRomImageHeader and carries the whole file's size --
        # header + payload -- exactly as a real ROM directory entry does. KEntryAttXIP is what
        # tells the loader the ROM address is usable (RFile::Att gate, sf_cache.cpp).
        tree.append((img.zname, addr, size, FILE_ATTRIB_RO_XIP))

    if slots is not None:
        b.pad_to_lin(slots.end())           # the device table and the directory tree behind the last slot: a stable address
        slots.save()
    # Built-in device drivers (add_builtin_device): real ROM files, patched by the bootstrap.
    # builtin_devices = [(z_path, "ldd"|"pdd", device id)]
    dev_table = []
    for zname, kind, dev_id in (builtin_devices or []):
        uid2 = {"ldd": 0x100000AF, "pdd": 0x100039D0}[kind]     # KLogical/KPhysicalDeviceDriverUid
        addr, size = b.add_builtin_device(uid2, dev_id)
        info.append((zname, addr, addr, size))
        tree.append((zname, addr, size, FILE_ATTRIB_RO_XIP))
        dev_table.append((addr, dev_id))
    dev_table_addr = 0
    if dev_table:
        align4(b.rom)
        dev_table_addr = b.lin(len(b.rom))
        b.rom += struct.pack("<I", len(dev_table))
        for addr, dev_id in dev_table:
            b.rom += struct.pack("<2I", addr, dev_id)

    # The secondary's TRomEntry now lives inside a directory block rather than a flat table,
    # so name it BEFORE the tree is emitted: add_tree records the address as it lays it down.
    match = [i for i in imgs if i.zname.rsplit("\\", 1)[-1].lower() == secondary_name.lower()]
    if not match:
        raise ValueError("secondary %r is not in the bundle: %s"
                         % (secondary_name, [i.zname for i in imgs]))
    b.secondary_addr = match[0].hdr_addr
    rdl_addr = b.add_tree(tree)
    assert b.secondary_entry_addr, "secondary TRomEntry not found in the emitted tree"

    struct.pack_into("<9I", b.rom, 0, SECTION_MAGIC, SECTION_VERSION, base, len(b.rom),
                     len(imgs), rdl_addr, b.secondary_entry_addr, b.total_user_data, dev_table_addr)
    blob = bytes(b.rom)
    verify_section_tree(blob, base, tree)
    return blob, info


# ── reading the tree back the way the FILESERVER does ────────────────────────────────────────
# A directory block is only as good as its sort tables, and those are the part a writer gets
# wrong silently: a mis-sorted table still parses and still lists -- it only fails when somebody
# binary-searches it, which is the one thing the fileserver ever does. So the verifier below
# does NOT re-read what we just wrote. It resolves each path with TRomDir::BinarySearch's own
# algorithm over the sort tables (srom/sr_rom.cpp), folding names through CRomMountCB::Compare.

def _rom_compare(left, right):
    """CRomMountCB::Compare -- fold ASCII 'A'-'Z' to lowercase, then shorter string first."""
    def fold(s):
        return [ord(c) + 32 if 'A' <= c <= 'Z' else ord(c) for c in s]
    l, r = fold(left), fold(right)
    for lc, rc in zip(l, r):
        if lc != rc:
            return lc - rc
    return len(l) - len(r)


class RomDirView:
    """One TRomDir block inside a blob, read exactly as e32rom.h lays it out."""

    def __init__(self, data, base, lin):
        self.data, self.base = data, base
        off = lin - base
        if not 0 <= off < len(data):
            raise ValueError("TRomDir at 0x%08X is outside the section" % lin)
        self.size = struct.unpack_from("<i", data, off)[0]
        self.first = off + 4                        # first TRomEntry
        si = self.first + ((self.size + 3) & ~3)    # TRomDirSortInfo
        self.n_sub, self.n_file = struct.unpack_from("<HH", data, si)
        self.sorted = struct.unpack_from("<%dH" % (self.n_sub + self.n_file), data, si + 4)

    def entry(self, i):
        """The i'th entry OF THE SORT TABLES -> (name, addr_lin, size, attrib)."""
        off = self.first + 4 * self.sorted[i]
        size, addr, att, nlen = struct.unpack_from("<IIBB", self.data, off)
        name = self.data[off + KROM_ENTRY_SIZE:off + KROM_ENTRY_SIZE + 2 * nlen] \
            .decode("utf-16-le")
        return name, addr, size, att

    def search(self, name, want_dir):
        """TRomDir::BinarySearch(name, KMaxTInt, EArrayFindMode_Any, want_dir)."""
        lo = 0 if want_dir else self.n_sub
        hi = self.n_sub if want_dir else self.n_sub + self.n_file
        while hi > lo:
            m = (lo + hi) >> 1
            k = _rom_compare(name, self.entry(m)[0])
            if k == 0:
                return self.entry(m)
            lo, hi = (m + 1, hi) if k > 0 else (lo, m)
        return None


def rom_dir_find(blob, base, root_lin, path):
    """CRomMountCB::FindEntryL: walk the path's directories, then binary-search the leaf.

    Returns (name, addr_lin, size, attrib) or None. Directory components resolve through the
    subdir half of the sort table, the last component through the file half -- exactly the
    split TRomDir::BinarySearch makes on its aDir argument."""
    d = RomDirView(blob, base, root_lin)
    parts = path.replace("/", "\\").strip("\\").split("\\")
    for comp in parts[:-1]:
        hit = d.search(comp, True)
        if not hit:
            return None
        d = RomDirView(blob, base, hit[1])
    return d.search(parts[-1], False)


def verify_section_tree(blob, base, expect):
    """Resolve every (z_path, addr, size, attrib) in `expect` through the emitted tree."""
    magic, ver, hbase, size, _count, rdl, sec, _ = struct.unpack_from("<8I", blob, 0)
    assert magic == SECTION_MAGIC and ver == SECTION_VERSION, "section header magic/version"
    assert hbase == base and size == len(blob), "section header base/size"
    assert base <= sec < base + size, "secondary TRomEntry outside the section"
    n_root, _hw, root = struct.unpack_from("<iII", blob, rdl - base)
    assert n_root == 1, "root dir list: %d root dirs" % n_root
    for path, addr, fsize, att in expect:
        hit = rom_dir_find(blob, base, root, path)
        assert hit, "%s: not found by binary search in the emitted tree" % path
        assert hit[1:] == (addr, fsize, att), \
            "%s: entry mismatch, got addr 0x%08X size %d att 0x%02X" % ((path,) + hit[1:])
    return True


# a minimal placeholder primary: ARM `b .` (infinite loop) + a small identity blob
PLACEHOLDER_EKERN = struct.pack("<I", 0xEAFFFFFE) + b"phorepoc-minimal-primary\x00"


def build_minimal(bootstrap: bytes, rom_base: int, primary_payload: bytes = None) -> bytes:
    b = RomBuilder(bootstrap, rom_base)
    payload = primary_payload if primary_payload is not None else PLACEHOLDER_EKERN
    addr, size = b.add_xip_file(payload, uid3=0x100000AE, is_primary=True)
    rdl = b.add_tree([("Sys\\Bin\\ekern.exe", addr, size)])
    return b.finalize(rdl)


def build_kernel_rom(bootstrap, rom_base, kernel_bin, k, variant=None, secondary=None,
                     xip_e32=None, data_files=None, placement=None):
    """Build a ROM with a real ekern primary. k = dict of ints (see --kinfo).
    variant = (entry_abs, [export_addrs]) to also add the variant extension DLL.
    secondary = (payload_bytes, sk) to add the first USER process (-> iSecondaryFile); sk is a
    dict of ints: entry_off, code_size, data_size, bss_size, data_bss_lin_base, stack, heap_max.
    xip_e32 = ([E32Xip...], xk) to XIP-prelink our emitted E32Images into the ROM (imports
    resolved at build time; the EXE among them becomes iSecondaryFile); xk keys: databss,
    cap1, cap2."""
    b = RomBuilder(bootstrap, rom_base)
    b.trace_mask = k.get("trace_mask", 0)
    total_sv = (k["data_size"] + k["bss_size"] + 0xFFF) & ~0xFFF   # 4K-round data+bss
    b.kern_data_base = k["kern_data_base"]
    b.total_sv_data = total_sv
    # kernel region = data/bss + stack + heap headroom; a generous limit for first boot.
    # MUST be rounded UP to a 1MB (chunk) boundary: MmuBase::CreateKernelSection uses
    # iKernelLimit VERBATIM as the kernel linear-section base (TLinearSection::New does NOT
    # align it), and the section allocator hands out chunk-aligned addresses as base+r*1MB.
    # If iKernelLimit is only 4K-aligned, every kernel chunk is skewed by that sub-1MB
    # remainder, and ArmMmu::MapRamPages -- which indexes PTEs from the 1MB boundary
    # (offset>>12 & 0xFF) -- installs each chunk's PTEs at the wrong page-table slot, so the
    # chunk's own base VA lands on an unmapped PTE and the first access data-aborts.
    # The kernel heap max = the primary's iHeapSizeMax (sinit.cpp Mutate). It was 1 MB here (x86: 16 MB, xboot.cpp):
    # the full S60 boot ran the kernel heap dry -> ChunkCreate -4 / a constructor on a NULL new with 800 MB RAM free
    # The region must hold data/bss + stack + the whole heap.
    kheap_max = k.get("heap_max", 0x02000000)   # min 10 MB, max 32 MB
    b.kern_data_limit = (k["kern_data_base"] + total_sv + k.get("stack", 0x2000) + kheap_max + 0x400000 + 0xFFFFF) & ~0xFFFFF
    addr, size = b.add_xip_file(
        # ekern is an executable image: UID1 must be KExecutableImageUidValue (0x1000007A) or
        # DCodeSeg::Create rejects the kernel codeseg with KErrNotSupported (scodeseg.cpp:677) ->
        # InitialiseMicrokernel returns -5 -> KernelMain faults EInitMicrokernelFailed(50).
        kernel_bin, uid1=k.get("uid1", UID1_EXE), uid3=k.get("uid3", 0x00000000),
        is_primary=True, place_at=k["place_at"], entry_abs=k["entry"],
        code_size=k["code_size"], data_addr=k["data_lma"], data_size=k["data_size"],
        bss_size=k["bss_size"], stack_size=k.get("stack", 0x2000),
        heap_min=0x1000, heap_max=kheap_max,
        data_bss_lin_base=k["kern_data_base"])
    files = [("Sys\\Bin\\ekern.exe", addr, size)]
    if variant is not None:                       # variant = (entry_abs, [export_addrs])
        v_entry, v_exports = variant
        v_addr = b.add_variant_file(v_exports, v_entry)
        files.append(("Sys\\Bin\\kasyborg.dll", v_addr, XIP_HDR_SIZE + len(v_exports) * 4))
    if xip_e32 is not None and xip_e32[1].get("user_at"):
        # user_at: the user content starts at a FIXED address, the data files
        # first, then the XIP images -- a kernel that grows or shrinks no longer shifts the 70 MB behind it (every
        # block of the image changed), and a rebuilt EXE at the end changes only the end. The gap is slack.
        target = xip_e32[1]["user_at"] - b.rom_base
        if len(b.rom) > target:
            raise ValueError("user_at 0x%08X: the kernel part already ends at 0x%08X"
                             % (xip_e32[1]["user_at"], b.lin(len(b.rom))))
        b.rom.extend(bytes(target - len(b.rom)))
        slots = placement_from_env(placement)
        if slots is not None:
            slots.begin(xip_e32[1]["user_at"])
            b.slots = slots
        for zname, payload in (data_files or []):
            if slots is not None:
                d_addr = slots.reserve("d:" + zname, len(payload) + 4)
                b.put(d_addr - b.rom_base, payload)
            else:
                align4(b.rom)
                d_addr = b.lin(len(b.rom))
                b.rom += payload
            files.append((zname, d_addr, len(payload), 0x01))
        data_files = None
    if xip_e32 is not None:                       # xip_e32 = ([E32Xip...], xk)
        imgs, xk = xip_e32
        databss = xk.get("databss", 0x00400000)   # multiple-MM iUserLocalBase
        if xk.get("flexible"):                    # ARM flexible: DLL data under KUserLocalDataEnd
            prelink_e32(b, imgs, databss, data_top=KUserLocalDataEnd_FlexibleArm, flexible=True,
                        min_block=0x4000)
        else:
            prelink_e32(b, imgs, databss)
        for img in sorted(imgs, key=lambda i: i.hdr_addr):
            is_exe = img.is_exe
            addr, size = b.add_xip_file(
                img.payload, place_at=img.hdr_addr,
                uid1=img.uid1, uid2=img.uid2, uid3=img.uid3,
                entry_abs=img.code_addr + img.entry_off,
                code_size=len(img.code),
                data_addr=img.data_lma, data_size=len(img.data),
                bss_size=img.bss_size if (is_exe or img.own_wsd) else 0,
                **_exe_limits(img),
                data_bss_lin_base=databss if is_exe else img.data_run,
                dll_ref_table=img.ref_table_addr,
                # KImageDll: the loader's ONLY "is a DLL" test for a ROM image (E32Image::Construct)
                # -- without it a dependency is code-seg-created as an EXE and the load fails
                extra_flags=(0 if is_exe else 0x00000001)
                            | img.abi_flags                              # KRomImageABIMask (E32 iFlags bits 3-4)
                            | (KROM_IMAGE_SMP_SAFE if img.smp_safe else 0)
                            | (0x04000000 if img.own_wsd else 0)        # KRomImageFlagData
                            | (0x00800000 if getattr(img, 'exe_in_tree', False) else 0)   # KRomImageFlagExeInTree
                            | (0x03000000 if img.data_present else 0),  # ...DataInit|DataPresent
                priority=(img.priority or 350) if is_exe else 0x200,
                # an EXE runs with its header capabilities (see the same rule in build_xip_section); the --xinfo caps (all, by
                # the recipes) are the fallback for an EXE whose header has none, and what every DLL gets
                cap1=(_cap_override(img.zname) or _header_caps(img, xk.get("cap1", 0xFFFFFFFF), xk.get("cap2", 0xFFFFFFFF)))[0],
                cap2=(_cap_override(img.zname) or _header_caps(img, xk.get("cap1", 0xFFFFFFFF), xk.get("cap2", 0xFFFFFFFF)))[1],
                export_count=img.export_count, export_addr=img.export_addr,
                secure_id=img.secure_id, vendor_id=img.vendor_id, exc_desc=img.exc_desc,
            module_version=img.module_version)
            assert addr == img.hdr_addr, \
                f"{img.zname}: layout drift 0x{addr:X} != 0x{img.hdr_addr:X}"
            # Only the FILE SERVER is the ROM secondary (iSecondaryFile -> the first
            # USER process the kernel launches). The kernel's SecondaryInfo hard-codes
            # the fileserver's loader launches later -- they must NOT become the
            # secondary, or the kernel boots them instead of the fileserver.
            if is_exe and img.zname.lower().replace("/", "\\").endswith("\\efile.exe"):
                b.secondary_addr = addr
            files.append((img.zname, addr, size))
    if secondary is not None:                     # secondary = (payload, sk)
        payload, sk = secondary
        s_addr, s_size = b.add_xip_file(
            payload, uid1=UID1_EXE, uid3=sk.get("uid3", 0),
            priority=sk.get("priority", 350),   # EPriorityForeground (valid TProcessPriority)
            entry_off=sk.get("entry_off", 0),
            code_size=sk.get("code_size", len(payload)),
            data_addr=sk.get("data_addr", 0), data_size=sk.get("data_size", 0),
            bss_size=sk.get("bss_size", 0), stack_size=sk.get("stack", 0x2000),
            heap_min=0x1000, heap_max=sk.get("heap_max", 0x100000),
            data_bss_lin_base=sk.get("data_bss_lin_base", 0),
            cap1=sk.get("cap1", 0), cap2=sk.get("cap2", 0))
        b.secondary_addr = s_addr
        # name is cosmetic for the launch (iSecondaryFile points straight at the TRomEntry), but
        files.append(("Sys\\Bin\\efile.exe", s_addr, s_size))
    for zname, payload in (data_files or []):
        # plain data file (e.g. estart.txt): raw bytes, no image header, attrib
        # ReadOnly only (NOT XIP -- it is not an E32Image).
        align4(b.rom)
        d_addr = b.lin(len(b.rom))
        b.rom += payload
        files.append((zname, d_addr, len(payload), 0x01))
    if b.slots is not None:
        b.pad_to_lin(b.slots.end())         # the directory tree behind the last slot: a stable address
        b.slots.save()
    rdl = b.add_tree(files)
    return b.finalize(rdl)


def selftest_tree():
    """The directory tree, writer vs the fileserver's reader -- no toolchain needed.

    The sort tables are the part of a TRomDir that fails silently: a mis-sorted table still
    parses and still lists, and only a binary search notices. So this drives add_tree() with
    names that exercise CRomMountCB::Compare (case folding, shorter-first ties, a directory
    spelled two ways) and then proves the check has teeth by breaking a sort table."""
    base = 0x80400000
    files = [(r"Sys\Bin\EUSER.DLL",    base + 0x1000, 16, FILE_ATTRIB_RO_XIP),
             (r"sys\bin\efile.exe",    base + 0x2000, 32, FILE_ATTRIB_RO_XIP),  # dir folds equal
             (r"Sys\Bin\ef.dll",       base + 0x3000, 48, FILE_ATTRIB_RO_XIP),  # tie, shorter
             (r"Sys\Data\estart.txt",  base + 0x4000, 8,  FILE_ATTRIB_RO),
             (r"Resource\z.rsc",       base + 0x5000, 4,  FILE_ATTRIB_RO)]
    b = RomBuilder(bytes(SECTION_HDR_SIZE), base)
    rdl = b.add_tree(files)
    blob = bytes(b.rom)
    root = struct.unpack_from("<I", blob, rdl - base + 8)[0]

    for path, addr, size, att in files:
        assert rom_dir_find(blob, base, root, path)[1:] == (addr, size, att), path
    # ROM lookups are case-insensitive, so every spelling of a path must land on one entry
    assert rom_dir_find(blob, base, root, r"SYS\BIN\euser.dll")[1] == base + 0x1000
    assert rom_dir_find(blob, base, root, r"sYs\bIn\EfIlE.ExE")[1] == base + 0x2000
    # 'sys\bin\...' must have MERGED into 'Sys\Bin' -- two dirs folding equal means one of
    # them is unreachable, which is the bug add_tree's case-insensitive setdefault prevents
    assert RomDirView(blob, base, root).n_sub == 2, "Sys/sys did not merge"
    assert rom_dir_find(blob, base, root, r"Sys\Bin\nosuch.dll") is None
    assert rom_dir_find(blob, base, root, r"Nosuch\x.dll") is None
    # a file is not findable as a directory, and a directory not as a file
    assert RomDirView(blob, base, root).search("Sys", False) is None
    assert RomDirView(blob, base, root).search("Sys", True) is not None

    # now break it: reverse Sys\Bin's file sort table. Every entry is still present and the
    # block still parses -- only the binary search can tell, which is the point.
    sysbin = RomDirView(blob, base, RomDirView(blob, base, root).search("Sys", True)[1])
    d = RomDirView(blob, base, sysbin.search("Bin", True)[1])
    si = d.first + ((d.size + 3) & ~3) + 4 + 2 * d.n_sub
    bad = bytearray(blob)
    struct.pack_into("<%dH" % d.n_file, bad, si, *reversed(d.sorted[d.n_sub:]))
    broken = [p for p, _a, _s, _t in files
              if rom_dir_find(bytes(bad), base, root, p) is None]
    assert broken, "a reversed sort table went unnoticed -- the check has no teeth"
    print("rombuild selftest: tree PASS (%d paths by binary search; a reversed sort table "
          "loses %d of them)" % (len(files), len(broken)))


def selftest_est():
    """The exception search table: sorted code bases, the fencepost, the header field -- no toolchain needed."""
    base = 0x80000000
    boot = bytearray(ROM_HEADER_SIZE + 64)
    b = RomBuilder(bytes(boot), base)
    lims = {}
    # descriptor payload: iExIdxBase, iExIdxLimit, iROSegmentBase, iROSegmentLimit, then code
    def img(code_len, exc_off, limit_delta):
        payload = bytearray(code_len)
        a, _ = b.add_xip_file(bytes(payload), exc_desc=exc_off | 1)
        ca = a + XIP_HDR_SIZE
        struct.pack_into("<4I", b.rom, ca + exc_off - base, 0, 0, ca, ca + limit_delta)
        lims[ca] = ca + limit_delta
        return ca
    c1 = img(0x40, 0x10, 0x30)
    c2 = img(0x80, 0x20, 0x70)
    b.add_xip_file(bytes(0x20))                      # no exception descriptor: not an entry
    b.est_images.reverse()                            # builder order must not matter
    rom = b.finalize(0)
    est = struct.unpack_from("<I", rom, H_iRomExceptionSearchTable)[0]
    assert est and est % 4 == 0, "EST header field not set"
    o = est - base
    n = struct.unpack_from("<I", rom, o)[0]
    ent = list(struct.unpack_from("<%dI" % (n + 1), rom, o + 4))
    assert n == 2 and ent[:2] == [c1, c2] and ent[:2] == sorted(ent[:2]), ent
    assert ent[2] == max(lims.values()), "fencepost != greatest iROSegmentLimit"
    for ca, off in ((c1, 0x10), (c2, 0x20)):       # what the unwinder reads: rih[-1].iExceptionDescriptor, unmasked
        xd = struct.unpack_from("<I", rom, ca - base - XIP_HDR_SIZE + 116)[0]
        assert xd == ca + off, "iExceptionDescriptor %08x != aligned descriptor address %08x" % (xd, ca + off)
    s = sum(struct.unpack_from("<%dI" % (len(rom) // 4), rom, 0)) & 0xFFFFFFFF
    assert s == 0, "checksum"
    none = RomBuilder(bytes(boot), base).finalize(0)
    assert struct.unpack_from("<I", none, H_iRomExceptionSearchTable)[0] == 0
    print("rombuild selftest: PASS (exception search table: 2 entries sorted, fencepost, header field)")


def selftest():
    """Round-trip the writer: a synthetic bootstrap, the ROM checksum, the directory tree."""
    boot = bytearray(ROM_HEADER_SIZE + 64)
    struct.pack_into("<I", boot, 0, 0xEA000000)   # a plausible reset branch
    rom = build_minimal(bytes(boot), 0x80000000)
    assert struct.unpack_from("<I", rom, H_iRomBase)[0] == 0x80000000
    assert struct.unpack_from("<I", rom, H_iRomSize)[0] == len(rom)
    s = sum(struct.unpack_from("<%dI" % (len(rom) // 4), rom, 0)) & 0xFFFFFFFF
    assert s == 0, "word-sum != 0"
    print("rombuild selftest: PASS (header fields, checksum word-sum=0)")
    selftest_tree()
    selftest_est()


def main(argv=None):
    ap = argparse.ArgumentParser(description="Minimal Symbian XIP ROM writer",
                                 fromfile_prefix_chars="@")   # @file: one argument per line
    if argv is None:
        argv = sys.argv[1:]
    if argv and argv[0] == "--selftest":
        selftest()
        return 0
    ap.add_argument("bootstrap", nargs="?", help="raw bootstrap binary (objcopy -O binary); "
                    "not used with --section")
    ap.add_argument("out", help="output .rom path (or the .bin section with --section)")
    ap.add_argument("--section", help="x86/x32: emit only a prelinked XIP "
                    "ROM SECTION based at this address (e.g. 0x80400000) instead of a whole "
                    "ROM -- the blob xk_uimage.S carries inside the kernel image. Takes --xip "
                    "and --secondary-name; no bootstrap.")
    ap.add_argument("--secondary-name", default="efile.exe",
                    help="with --section: which bundled image the section header names as the "
                    "ROM secondary (the first user process).")
    ap.add_argument("--exe-data-base", default="0x70000000",
                    help="with --section: the EXE's .data/.bss run address. MUST equal "
                    "KX64ExeDataBase (the kernel's mmboot.h), where the kernel pins every EXE's "
                    "statics (the kernel's mprocess.cpp) -- pcodeseg.cpp faults KERN 77 if "
                    "the TRomImageHeader disagrees.")
    ap.add_argument("--rom-base", default="0x80000000")
    ap.add_argument("--data-top", default="0x80000000",
                    help="--section: KUserLocalDataEnd of the kernel's flexible memory model, under which ROM DLL "
                         "static data is carved (x86 0x80000000; ARM, e.g. the AArch64 port, 0x7f000000)")
    ap.add_argument("--primary", help="raw binary to use as the primary (ekern) payload "
                                       "instead of the built-in placeholder")
    ap.add_argument("--kinfo", help="real-kernel primary: comma-separated k=v ints "
                    "(place_at,entry,code_size,data_lma,data_size,bss_size,kern_data_base,"
                    "stack,heap_max,uid1,uid3). Values accept 0x hex. Requires --primary.")
    ap.add_argument("--variant", help="add the variant DLL: 'entry=0x..,exports=0xa:0xb:..' "
                    "(entry=_variant_e32dll, exports=ekern variant-function addrs, ord 1..N).")
    ap.add_argument("--secondary", help="path to the raw binary for the first USER process "
                    "(-> iSecondaryFile), e.g. minisec.bin. PIC EXE placed as z:\\sys\\bin\\efile.exe.")
    ap.add_argument("--sinfo", help="secondary params: comma-separated k=v ints "
                    "(entry_off,code_size,data_size,bss_size,data_bss_lin_base,stack,heap_max).")
    ap.add_argument("--xip", action="append", default=[],
                    help="XIP-prelink an emitted E32Image into the ROM: 'Z-path=e32-path', "
                    "e.g. 'Sys\\Bin\\euser.dll=emitter/euser_dev.dll'. Repeatable. Imports "
                    "resolve by ordinal against the bundle; an EXE becomes iSecondaryFile.")
    ap.add_argument("--xinfo", help="XIP bundle params: comma-separated k=v ints "
                    "(databss,cap1,cap2).")
    ap.add_argument("--data", action="append", default=[],
                    help="add a plain data file: 'Z-path=host-path', e.g. "
                    r"'Sys\Data\estart.txt=path/to/estart.txt'. Repeatable.")
    ap.add_argument("--builtin-device", action="append", default=[],
                    help="with --section: a kernel-linked device driver's ROM file, "
                    r"'Z-path=ldd|pdd:<id>', e.g. 'Sys\Bin\enet.ldd=ldd:1' (xboot.cpp patches "
                    "its entry point + export 1 by id). Repeatable.")
    ap.add_argument("--placement", help="stable file placement: the JSON map of per-file ROM slots (read, and rewritten after "
                    "the build); env ROM_PLACEMENT does the same. FULL=1 / ROM_PLACEMENT_FRESH=1 ignores the old map and packs "
                    "compactly. Files keep their address while they fit their slot, so a delta push sends only what changed.")
    args = ap.parse_args(argv)

    # the same data file named twice (the registry's rom_data and a package export, say): kept once; the same ROM path
    # with two different sources is an error (which one ships would be an accident)
    seen, data = {}, []
    for spec in args.data:
        zn, _, p = spec.partition("=")
        key = zn.lower().replace("/", "\\")
        if key in seen:
            if (os.path.normcase(os.path.abspath(seen[key])) != os.path.normcase(os.path.abspath(p))
                    and open(seen[key], "rb").read() != open(p, "rb").read()):
                sys.exit(f"rombuild: {zn} given twice with different content: {seen[key]} and {p}")
            continue
        seen[key] = p
        data.append(spec)
    args.data = data

    rom_base = int(args.rom_base, 0)
    if args.section:
        base = int(args.section, 0)
        imgs = [E32Xip(path, zname) for zname, _, path in
                (spec.partition("=") for spec in args.xip)]
        dfiles = [(zn, open(p, "rb").read()) for zn, _, p in
                  (spec.partition("=") for spec in args.data)]
        devs = [(zn, v.split(":")[0], int(v.split(":")[1], 0)) for zn, _, v in
                (spec.partition("=") for spec in args.builtin_device)]
        blob, info = build_xip_section(imgs, base, args.secondary_name,
                                       int(args.exe_data_base, 0), dfiles, devs, args.placement,
                                       int(args.data_top, 0))
        open(args.out, "wb").write(blob)
        rdl, sec = struct.unpack_from("<2I", blob, 20)
        print("wrote %s: %d bytes, section base=0x%08X, %d image(s) + %d data file(s)"
              % (args.out, len(blob), base, len(imgs), len(dfiles)))
        # <out>.map: "<zname> <code run addr> <code size> <source image>" -- for the x32
        # symbolizer, as the whole-ROM path's map is for a symbolizer too
        srcs = {zn: p for zn, _, p in (spec.partition("=") for spec in args.xip)}
        with open(args.out + ".map", "w", newline="\n") as fh:
            for zname, hdr, code, size in info:
                print("   %-22s hdr 0x%08X  code 0x%08X  %7d bytes" % (zname, hdr, code, size))
                fh.write("%s 0x%08X 0x%X %s\n" % (zname, code, size, srcs.get(zname, "?")))
        print("   root dir list 0x%08X   secondary TRomEntry 0x%08X   (tree verified by "
              "binary search)" % (rdl, sec))
        return 0

    bootstrap = open(args.bootstrap, "rb").read()
    primary = open(args.primary, "rb").read() if args.primary else None
    if args.kinfo:
        k = {kv.split("=")[0]: int(kv.split("=")[1], 0) for kv in args.kinfo.split(",")}
        variant = None
        if args.variant:
            vd = {kv.split("=")[0]: kv.split("=")[1] for kv in args.variant.split(",")}
            variant = (int(vd["entry"], 0), [int(x, 0) for x in vd["exports"].split(":")])
        secondary = None
        if args.secondary:
            payload = open(args.secondary, "rb").read()
            sk = {}
            if args.sinfo:
                sk = {kv.split("=")[0]: int(kv.split("=")[1], 0) for kv in args.sinfo.split(",")}
            secondary = (payload, sk)
        xip_e32 = None
        if args.xip:
            imgs = []
            for spec in args.xip:
                zname, _, path = spec.partition("=")
                imgs.append(E32Xip(path, zname))
            xk = {}
            if args.xinfo:
                xk = {kv.split("=")[0]: int(kv.split("=")[1], 0)
                      for kv in args.xinfo.split(",")}
            xip_e32 = (imgs, xk)
        data_files = []
        for spec in args.data:
            zname, _, path = spec.partition("=")
            data_files.append((zname, open(path, "rb").read()))
        rom = build_kernel_rom(bootstrap, rom_base, primary, k, variant, secondary,
                               xip_e32, data_files, args.placement)
    else:
        rom = build_minimal(bootstrap, rom_base, primary)
    open(args.out, "wb").write(rom)
    print("wrote %s: %d bytes, rom_base=0x%08X" % (args.out, len(rom), rom_base))
    if LAST_XIP_MAP:
        with open(args.out + ".map", "w") as fh:
            for name, addr, size, link in LAST_XIP_MAP:
                fh.write("%s 0x%08X 0x%X 0x%X\n" % (name, addr, size, link))

    return 0


if __name__ == "__main__":
    sys.exit(main())
