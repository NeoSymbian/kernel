<!-- SPDX-License-Identifier: Apache-2.0 -->
# NeoSymbian kernel

This is the Symbian EKA2 kernel and hardware services package (`kernelhwsrv`, Symbian^3, 2010), ported to
**AArch64 with 32-bit pointers (ILP32)**. The 2010 user side runs unchanged in 32-bit mode at AArch32 EL0. The first
targets are QEMU `virt` and the Google Pixel 3a XL (bonito, Qualcomm SDM670).

- **Fork:** of [SymbianSource/oss.FCL.sf.os.kernelhwsrv](https://github.com/SymbianSource/oss.FCL.sf.os.kernelhwsrv)
  (renamed 2026-11-06). Our work is on `main`; the upstream branches stay untouched.
- **Builds:** [NeoSymbian/build](https://github.com/NeoSymbian/build) builds it (`neo`) and pins it in `sources.lock`.
  The build scripts expect it in a folder named `oss.FCL.sf.os.kernelhwsrv`.
- **Docs:** design and plans are in [NeoSymbian/docs](https://github.com/NeoSymbian/docs): `roadmap.md`,
  `port-map.md`, `ekern-backend.md`, `filesystem.md`, `telephony.md`.

## Repository map

The top-level folders are Symbian's package layout. NeoSymbian adds `arm64` folders next to Symbian's `arm/` and `x86/`
folders, plus a few marked edits in generic files.

| Folder | What | NeoSymbian changes |
|---|---|---|
| `kernel/eka/` | The kernel: nanokernel (`nkern`, `nkernsmp`), kernel (`kernel`), memory models (`memmodel`), shared code (`common`, `euser`), generic drivers (`drivers`), headers (`include`) | **Yes**: the `arm64` folders + marked edits |
| `kerneltest/` | Kernel and file-server tests (`e32test`, `f32test`, `e32utils`); `e32test/nkernsa` = standalone nanokernel tests | **Yes**: `e32test/arm64`, `e32test/nkernsa/arm64` |
| `baseport/` | Board support. **NeoSymbian's own folder**: `baseport/arm64` (see [its README](baseport/arm64/README.md)) | **Ours** |
| `userlibandfileserver/` | The file server (F32), file systems, `domainmgr` | One edit (`fileserver/sfile/sf_lepoc.cpp`) |
| `halservices/` | The user-side HAL (`hal.dll`) and its attribute tables | No |
| `brdbootldr/` | Symbian's board bootloaders (ARM) | No (ABL or QEMU loads our kernel) |
| `bsptemplate/` | Symbian's board-support template (`asspandvariant`): the model `baseport/arm64` follows | No |
| `kernelhwsrv_info/` | Package documentation and metadata | No |
| `package_definition.xml`, `package_map.xml`, `build.config.xml` | Symbian build-system metadata | No (we don't use Symbian's build system) |

## The AArch64 port

| Part | Where |
|---|---|
| SMP nanokernel back end, GICv3, platform layer | `kernel/eka/nkernsmp/arm64`, `kernel/eka/include/nkernsmp/arm64` |
| Kernel back end, boot stub, board interface (`board.h`) | `kernel/eka/kernel/arm64`, `kernel/eka/include/kernel/arm64` |
| Flexible memory model | `kernel/eka/memmodel/epoc/flexible/{arm64,mmu/arm64}`, `kernel/eka/include/memmodel/epoc/flexible/arm64` |
| Atomics, compiler preludes | `kernel/eka/common/arm64`, `kernel/eka/include/nkern/arm64` |
| Tests | `kerneltest/e32test/nkernsa/arm64` (nkernsa image), `kerneltest/e32test/arm64` (AArch32 EL0 checks) |
| Boards | `baseport/arm64` |

`kernel/eka/kernel/arm64`, file by file:

| File | Role |
|---|---|
| `boot.S` | Entry of the arm64 `Image`; early framebuffer progress bands; jumps to `A64_BootMain` |
| `a64boot.cpp` | MMU-off bootstrap: page tables, the board's I/O map, the ROM header and the variant/extension records |
| `a64e_entry.cpp` | `_E32Startup`: vectors, I/O window, static constructors, `KernelMain` |
| `cinit.cpp` | Copy of `kernel/arm/cinit.cpp` with marked changes (CPU count from the board) |
| `a64e_exc.cpp`, `a64e_trap.S` | Exceptions, traps, user-memory access with fault recovery |
| `a64e_exec.cpp` | Executive-call tables, the exec gate, thread entry |
| `a64e_a32fix.cpp` | Emulates the AArch32 EL0 instructions ARMv8 dropped |
| `a64e_cache.cpp` | Cache maintenance |
| `ekern.lds` | Linker script for the single kernel image (kernel + ROM sections) |

**Generic Symbian files with marked edits** (an `__CPU_ARM64` section or a `NeoSymbian` comment):
- headers: `include/kernel/execs.h`, `include/u32exec.h`, `include/memmodel/epoc/flexible/mmtypes.h`;
- kernel: `kernel/{eventq,sinit,sthread}.cpp`, `kernel/arm/{cutils,cache_maintenance}.cpp`;
- nanokernel and memory model: `nkernsmp/{nkern,nk_bal}.cpp`, `memmodel/epoc/flexible/mmu/{mpager,mpdalloc}.cpp`;
- tests: `kerneltest/e32test/nkernsa/{fastmutex,tiedevents}.cpp`.

To list them all: `git diff --name-only origin/master...main | grep -v /arm64/`.

## Boards

`baseport/arm64` follows Symbian's chip/board split. The chip (ASSP) and the board (variant) are separate:

| Folder | What |
|---|---|
| `common/` | Code every arm64 board shares: variant, display HAL, block media, debug monitor |
| `<soc>_assp/` | One SoC's drivers: `virt_assp` (QEMU virt), `sdm670_assp` (Qualcomm SDM670) |
| `<board>_variant/` | One board: `qemu_variant`, `bonito_variant` (Pixel 3a XL) |

[`baseport/arm64/README.md`](baseport/arm64/README.md) covers:
- how drivers plug in;
- how to add a device or a driver;
- the known layering debts and the planned layout for more devices.

## Kernel or user side?

| Work | Where | Language |
|---|---|---|
| CPU, MMU, interrupts, timers, MMIO, DMA, IRQ handlers, transport to other processors | this repo | C++ (no Rust: Rust has no ILP32 target) |
| Protocols, policy, daemons, UI | NeoSymbian/e32-rs, neoui, telephony | Rust (`armv5te-none-eabi`) |

Rules for the boundary:
- **Use Symbian's own interface wherever the 2010 user side already consumes one.** Examples: the HAL groups (display,
  power, variant), `locmedia` for drives, `Kern::AddEvent` for keys and pointer, `usbc`, `soundsc`, `power`,
  `resourceman`, `iic`.
- **Otherwise, a kernel service a NeoSymbian daemon needs** starts as a NeoSymbian function in `EHalGroupVariant`
  (numbers `0x4e00`+; see `common/specific/a64_variant.cpp` and `sdm670_assp/qcom_telhal.cpp`). It becomes an LDD once
  it needs asynchronous notification.

## Building

Use [`neo`](https://github.com/NeoSymbian/build), run from a `build` checkout next to this one (on the ZimaOS build
machine):

```sh
neo nkernsa qemu                         # standalone nanokernel tests on QEMU virt
neo kernel qemu && neo qemu 60           # full kernel + the 2010 user side, booted for 60 s
DTB=../private/bonito.dtb neo kernel bonito && neo flash bonito   # the Pixel 3a XL
```

By hand, inside the toolchain image, run `BOARD=<board> baseport/arm64/tools/build-ekern.sh` (and see `tools/`).
`DTB` is the phone's own device tree. It is never committed.

## Licences

- **Symbian code:** EPL-1.0.
- **Symbian files we change:** EPL-2.0. The SPDX line goes on the header's blank line 7, and a
  `The NeoSymbian Authors (2026): …` line goes under Contributors, so `__LINE__` values don't move. Each change is
  marked `NeoSymbian`.
- **New files derived from Symbian code:** EPL-2.0, with the source named in the SPDX line. Examples:
  `a64_variant.cpp`, `a64_medverity.cpp`, `nk_plat.h`.
- **Our own new files:** Apache-2.0, with `SPDX-License-Identifier: Apache-2.0`. Most of `baseport/arm64` and the
  `arm64` folders are in this group.
