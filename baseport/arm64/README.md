# baseport/arm64: board support for the AArch64 kernel

NeoSymbian's kernel is EKA2, ported to AArch64 with 32-bit pointers (ILP32). The 2010 user side runs unchanged at
AArch32 EL0. The architecture code sits in this tree next to Symbian's `arm/` and `x86/` folders:

| Part | Where |
|---|---|
| SMP nanokernel back end, GICv3, platform layer | `kernel/eka/nkernsmp/arm64`, `kernel/eka/include/nkernsmp/arm64` |
| Kernel back end, boot stub, board interface (`board.h`) | `kernel/eka/kernel/arm64`, `kernel/eka/include/kernel/arm64` |
| Flexible memory model | `kernel/eka/memmodel/epoc/flexible/{arm64,mmu/arm64}`, `kernel/eka/include/memmodel/epoc/flexible/arm64` |
| Atomics, CPU preludes | `kernel/eka/common/arm64`, `kernel/eka/include/nkern/arm64` |
| Tests | `kerneltest/e32test/nkernsa/arm64`, `kerneltest/e32test/arm64` |
| Generic files with an `__CPU_ARM64` section | `include/kernel/execs.h`, `include/u32exec.h`, `include/memmodel/epoc/flexible/mmtypes.h`, `memmodel/epoc/flexible/mmu/mpdalloc.cpp` |

Changes to Symbian files are marked `NeoSymbian`.

This folder holds the boards. It follows `bsptemplate/asspandvariant`, where the chip (ASSP) is split from the
board (variant):

| Folder | What |
|---|---|
| `common/` | The variant code every arm64 board shares: static variant, display HAL, block media, debug monitor, `estart.txt` |
| `<soc>_assp/` | One SoC's drivers: `virt_assp` (the QEMU virt machine), `sdm670_assp` (Qualcomm SDM670) |
| `<board>_variant/` | One board: `variant.conf`, `specific/` (its `board.h` implementation and drivers) |
| `tools/` | Build, run and phone tools |

## Building

The scripts run in the `symbian-a64dev` image (aarch64 GCC 14, arm-none-eabi binutils, QEMU). `KHS` defaults to
this checkout.

```sh
BOARD=qemu   baseport/arm64/tools/build-kernel.sh      # nanokernel test image (nkernsa)
BOARD=qemu   baseport/arm64/tools/build-ekern.sh       # full kernel + the 2010 user side (needs the symbian-2010 build at S2010)
baseport/arm64/tools/run-qemu.sh 60                    # boot out/kernel-qemu/Image on QEMU virt
BOARD=bonito DTB=bonito.dtb baseport/arm64/tools/build-ekern.sh   # -> out/ekern-bonito/boot.img
fastboot boot out/ekern-bonito/boot.img
```

## Adding a device

1. If its SoC is new, add `<soc>_assp/` with the SoC's drivers.
2. Add `<board>_variant/` with:
   - `specific/<board>.cpp`: the `board.h` interface (GIC and timer, console, I/O map, framebuffer);
   - its own drivers in `specific/`;
   - a `variant.conf`. Copy `qemu_variant/variant.conf` or `bonito_variant/variant.conf` and set `ASSP`,
     `KERNEL_BASE`, `ROM_LIN`, `BOARD_SRC`, `BOOT_DEFS` and `BOOTIMG`, and list the board's drivers in
     `variant_drivers`.
3. Build with `BOARD=<board>`. `tools/paths.sh` documents every `variant.conf` key.
