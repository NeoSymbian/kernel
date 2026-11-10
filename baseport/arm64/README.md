# baseport/arm64: board support for the AArch64 kernel

NeoSymbian's kernel is EKA2, ported to AArch64 with 32-bit pointers (ILP32). The architecture code (nanokernel,
kernel, memory model) is in `kernel/eka/*/arm64`; the [repository README](../../README.md) maps it. This folder holds
the boards.

It follows `bsptemplate/asspandvariant`: the chip (ASSP) is split from the board (variant).

| Folder | What |
|---|---|
| `common/` | The variant code every arm64 board shares: static variant and HAL (`a64_variant.cpp`), display HAL, block media and drives (`a64_blk`, `a64_med*`), debug monitor and log ring (`a64_dbgmon.cpp`), the CPU's SHA-256, `estart.txt` |
| `<soc>_assp/` | One SoC's drivers: `virt_assp` (the QEMU virt machine), `sdm670_assp` (Qualcomm SDM670) |
| `<board>_variant/` | One board: `variant.conf`, `specific/` (its `board.h` implementation and board-only drivers) |
| `bonito_variant/probes/` | Standalone bare-metal probe images used to study the Pixel's bootloader (ABL). The kernel build does not use them. |
| `tools/` | Build, run and phone tools (`build-ekern.sh`, `paths.sh`, `phone.py`, …) |

## Building

The easy way is [`neo`](https://github.com/NeoSymbian/build), from a `build` checkout next to this one:
`neo kernel qemu`, `DTB=bonito.dtb neo kernel bonito`, `neo qemu 60`, `neo flash bonito`. It runs the scripts below in
the pinned build image (`ghcr.io/neosymbian/toolchain`: aarch64 GCC 14, the Arm GNU toolchain, QEMU) with the user
side's build volume mounted. By hand, inside that image (`KHS` defaults to this checkout):

```sh
BOARD=qemu   baseport/arm64/tools/build-kernel.sh      # nanokernel test image (nkernsa)
BOARD=qemu   baseport/arm64/tools/build-ekern.sh       # full kernel + the 2010 user side (needs the NeoSymbian/build volume at S2010)
baseport/arm64/tools/run-qemu.sh 60                    # boot out/kernel-qemu/Image on QEMU virt
BOARD=bonito DTB=bonito.dtb baseport/arm64/tools/build-ekern.sh   # -> out/ekern-bonito/boot.img
fastboot boot out/ekern-bonito/boot.img
```

## How a driver plugs in (today)

Everything is linked statically into one `ekern` image (`tools/build-ekern.sh`). There are no `.mmp` files and no
loadable LDDs or PDDs. The build compiles:
- the `common/` files, from a fixed list in `build-ekern.sh`;
- the `BOARD_SRC` file;
- each `ekc` line in the board's `variant_drivers()` (`variant.conf`).

`tools/paths.sh` documents every `variant.conf` key.

A driver joins the running kernel in one of four ways:

| Way | How | Examples |
|---|---|---|
| **Board interface** | Implements `kernel/eka/include/kernel/arm64/board.h` in the board's `BOARD_SRC` file | `pixel_bonito.cpp`, `qemu_virt.cpp`: GIC, timer, console, I/O map (`A64_IoRegions`), framebuffer |
| **Start hook** | A strong `A64Start*()` overrides a weak empty default in `a64_variant.cpp`. The one kernel extension, `A64ExtensionEntry`, calls the hooks **in a fixed order** | `A64StartBuses`, `A64StartCpufreq`, `A64StartKeys`, `A64StartTouch`, `A64StartUsbConsole`, `A64StartSerialKeyboard`, `A64StartReboot` |
| **Weak override** | A strong definition replaces a weak default that generic code calls | `A64BlkDev_*` (sdhci, virtio_blk), `A64_RtcSeconds` (rtc), `A64CpuBoost` (cpufreq, called from `kernel/eventq.cpp`), `A64_Restart`/`A64_CrashReset` (reboot), `A64Tel_Hal` (telhal) |
| **Library** | Plain functions that other drivers call | `Spmi_*`, `I2c_*`, `Bcm/Arc/Vrm_Vote` (rpmh), `Smem_*`, `Scm_*`, GLINK, SMP2P |

User-facing interfaces:

| Interface | Used for |
|---|---|
| HAL groups | Display (`a64_display.cpp`, `EHalGroupDisplay`); NeoSymbian functions in `EHalGroupVariant` |
| `EHalGroupVariant` functions `0x4e00`+ | `a64_variant.cpp`: log, console, boot slot, power, RTC. `qcom_telhal.cpp`: the modem daemon, from `0x4e0d`. **Take the next free number in both files' tables.** |
| `locmedia` drives | `a64_medblk/medrofs/medverity.cpp` |
| `Kern::AddEvent` | Keys and pointer events |

Debug-monitor commands (`A64Mon_Line` in `a64_dbgmon.cpp`):
- They are an `if (Eq(c, "…"))` chain that calls driver functions.
- Each of those functions has a weak "not on this board" stub in the same file.
- `blk` passes the rest of the line to `A64BlkDev_Monitor`; `scm`, `smem`, `fw`, `smp2p`, `qmp`, `mss` and `glink`
  pass it to `A64Tel_Monitor` (`qcom_telmon.cpp`). The command names themselves are listed in the common file.
- Answers go through `A64Mon_Say` and must stay under 256 bytes.

## Adding a device

1. If its SoC is new, add `<soc>_assp/` with the SoC's drivers.
2. Add `<board>_variant/` with:
   - `specific/<board>.cpp`: the `board.h` interface (GIC and timer, console, I/O map, framebuffer);
   - its own drivers in `specific/`;
   - a `variant.conf`. Copy `qemu_variant/variant.conf` or `bonito_variant/variant.conf` and set `ASSP`,
     `KERNEL_BASE`, `ROM_LIN`, `BOARD_SRC`, `BOOT_DEFS` and `BOOTIMG`, and list the board's drivers, including the
     SoC's, in `variant_drivers`.
3. Build with `BOARD=<board>`.

A second board on an existing SoC (the Pixel 3a "sargo" on SDM670) copies the SoC's `ekc` lines from the first board's
`variant.conf` for now; see "Planned layout".

## Adding a driver

1. **Pick the folder** (the rule: code lives at the most specific level that owns it, and moves up once a second user
   appears):

   | The code is… | Folder |
   |---|---|
   | needed by every board (a file-system reader, the media stack, a HAL) | `common/specific/` (+ `common/inc/` for its header) |
   | an SoC block (bus, clocks, storage controller, remote processor) | `<soc>_assp/` |
   | a part on one board (touch IC, panel, keys) or one board's policy (partition windows, reboot policy) | `<board>_variant/specific/` |

2. **Name it:**
   - `a64_*` for common and board files;
   - the vendor prefix (`qcom_*`) for SoC IP;
   - a unique basename, because `ekc` names objects by basename, so `x/foo.cpp` and `y/foo.cpp` overwrite each other.
3. **Add an `ekc` line** to the board's `variant_drivers()`.
4. **If it runs at boot**, add an `A64Start<Name>()` hook: a weak default in `a64_variant.cpp` and a call in
   `A64ExtensionEntry` at the right point. Buses come before their users: `A64StartBuses` initializes SPMI before the peripheral hooks; keys only read its cached readiness.
5. **Map its registers** in the board's `A64_IoRegions` (`board.h`, `SA64IoRegion`).
6. **For user access,** use a Symbian HAL group or framework if the 2010 user side already speaks it. Otherwise add a
   NeoSymbian `EHalGroupVariant` function (or an LDD once it needs asynchronous notification). User-side logic goes in
   Rust (NeoSymbian/e32-rs, neoui, telephony).
7. **For debugging,** add a monitor command in `a64_dbgmon.cpp` with a weak stub.
8. **For status,** describe the part with one `A64_DEVICE(tag, name, driver, statusFn)` at the end of the driver file
   (`common/inc/a64_device.h`): the status function returns the state (absent, off, starting, ok, partial, error) and
   appends one detail line. HAL `ENeoHalDeviceList` (0x4e19) and the monitor's `drivers` command list every entry, and
   neoui's Dev tools shows them on its Devices page. A driver that is not linked has no entry.

## Known debts and planned layout

The layout is right for one SoC and two boards. Telephony, power, audio and GPU will add many SoC drivers, and a second
SDM670 board is likely. These debts make the next device harder. They are recorded here so a dedicated pass can fix
them; this list is **not done yet**.

| Debt | Where | Fix |
|---|---|---|
| Every boot-time driver edits `common/`: one weak `A64Start*` per driver and a fixed call list | `a64_variant.cpp` (`A64ExtensionEntry`) | Registry: `A64_DRIVER(name, startFn, order)` in a `.a64drv` linker section (`ekern.lds`, `__start_`/`__stop_`), run in `order` |
| The monitor knows every driver: about 20 weak stubs and an `if` chain; `a64_dbgmon.h` declares the Qualcomm `A64Tel_Monitor` | `a64_dbgmon.cpp`, `common/inc/a64_dbgmon.h` | Registry: `A64_MONCMD("name", fn, "help")` in `.a64mon`; `help` lists them. (The `drivers` command and the status half of the driver registry exist: `A64_DEVICE` in `.a64dev`, `a64_devices.cpp`.) |
| A dropped `ekc` line fails silently into the weak default | `variant.conf`, weak stubs | The two registries, plus a `drivers` check in the gates |
| Per-driver start order is still a fixed call list | `a64_variant.cpp` (`A64StartBuses` now starts SPMI independently of keys) | Keep bus-before-peripheral ordering when adding registry-based startup |
| The SoC's driver list sits in the board's `variant.conf` | `bonito_variant/variant.conf` | `<soc>_assp/assp.conf` with `assp_drivers()`, called before `variant_drivers()` |
| `ekc` object names collide on equal basenames | `tools/build-ekern.sh` (`ekc`) | Name objects after the path under `baseport/arm64` |
| Generic code with a `qcom_` name: a GPT + FAT16 reader | `sdm670_assp/qcom_fat16.cpp` | `common/specific/a64_gpt.cpp` + `a64_fat16.cpp` |
| Four GPT parsers | `qcom_fat16.cpp`, `qcom_sdhci.cpp`, `qcom_bootctrl.cpp`, `a64_dbgmon.cpp` | One `Gpt_Find` in `common/` |
| Android/ABL A/B slot logic in the SoC folder | `sdm670_assp/qcom_bootctrl.cpp` | `common/specific/a64_bootctrl.cpp` |
| SoC code in the board folder: GCC clocks and TLMM pins in touch; PM660 PON in reboot | `bonito_variant/specific/a64_touch.cpp`, `a64_reboot.cpp` | `qcom_gcc`, `qcom_tlmm`, `qcom_pon` in the Qualcomm folder; the board keeps the RMI4 chip and its reboot policy |
| Board policy in the SoC eMMC driver (the userdata window, the partition allow-list) | `sdm670_assp/qcom_sdhci.cpp` | The board file |
| SoC facts spread over drivers (MMIO bases, IRQs; the GCC base defined in 3 files) | `qcom_*.cpp`, `a64_touch.cpp` | `sdm670_assp/sdm670.h` |
| `board.h` omits hooks that generic code calls | `A64CpuBoost`, `A64_LastLogPhys`, `A64_FbLinear`, `A64_WatchdogBite` (declared in .cpp files) | Declare them in `board.h` with the file that implements each; one file for the weak defaults |

Planned tree, once a second Qualcomm SoC or many more SDM670 drivers make it worthwhile:

```
baseport/arm64/
  common/            no SoC knowledge: variant, monitor core, display HAL, media, blk, GPT + FAT16, A/B boot control
  qcom/              Qualcomm IP blocks that other Qualcomm SoCs reuse (Symbian precedent: omap3530/shared)
    bus/             qcom_spmi, qcom_geni_i2c, qcom_rpmh
    soc/             qcom_scm, qcom_smem, qcom_llcc, qcom_cpufreq, qcom_gcc, qcom_tlmm
    storage/         qcom_sdhci
    usb/             qcom_dwc3, qcom_usbpower
    pmic/            qcom_rtc, qcom_pon (later: charger, RRADC, haptics, WLED)
    remoteproc/      qcom_smp2p, qcom_qmp, qcom_glink, qcom_mss, qcom_telhal, qcom_telmon (later: ADSP, IPA)
    display/ gpu/ audio/   (later: DPU/DSI, Adreno + GMU, LPASS/WCD)
  sdm670_assp/       SDM670 facts only: sdm670.h (MMIO bases, SPIs, carve-outs) + assp.conf
  virt_assp/
  <board>_variant/   board.h implementation, board-only parts and policy, variant.conf (board extras only)
  tools/
```

**Order of the code pass:**
1. Registries and `ekc` names. Behaviour stays the same; check `drivers` and an `nm` diff on QEMU and bonito.
2. Moves, as one commit at a quiet point in the telephony work.

## Read-only SoC temperatures (bonito)

`thermal` in the debug monitor lists both SDM670 TSENS v2 controllers (13 and 8
controller-local sensor IDs) in milli-Celsius. `drivers`/the device-list HAL report
valid sample count and maximum temperature. The registry deliberately reports
`partial`: telemetry provides no throttling, trip interrupts or thermal policy.
The driver never enables, resets or calibrates a controller; disabled/invalid
samples return unavailable. Sensor IDs are not physical CPU labels.

Host regression check: `python3 baseport/arm64/tools/test-tsens-host.py`.
The new `thermal` tag is mapped to neoui's existing Thermal device row; showing it
there requires rebuilding the UI as well as the kernel.

## Read-only battery observations (bonito)

`battery` reads PM660 fuel-gauge/charger observations through SPMI. It prints a
validity mask before raw fields: an unset bit means unknown, never an actual zero.
`drivers` reports the available percentage/voltage as `partial`, because this is
telemetry without charging policy. No PMIC writes, profile/SRAM access, ADC
triggers or charger-watchdog service are performed. Hardware readiness/shadow
agreement cannot establish profile correctness or sample age; the percentage is
the retained PMIC estimate, not Android's smoothed/calibrated UI value.

HAL `EHalGroupVariant` function `0x4e1a` takes an output descriptor in a1 and
requires `ReadDeviceData`. Exactly 32 little-endian bytes contain eight u32 words:
version (1), validity mask, presence, reported percentage, microvolts, USB presence,
USB online, normalized charge state. Validity bits 0..5 correspond to the six data
words; charge states are 0 unknown, 1 discharging, 2 charging, 3 not charging,
4 full. Partial data returns success; no data returns `KErrNotReady`; unsupported
PMIC/boards return `KErrNotSupported`. A short output descriptor returns overflow
through the standard descriptor write path. Older user interfaces remain unchanged.

`python3 baseport/arm64/tools/test-battery-host.py` checks actual driver code with
fake SPMI input. Source facts are from the AOSP PM660 Gen3 FG and Linux v6.16
PMI8998/PM660 charger references cited in `qcom_battery.cpp`.
