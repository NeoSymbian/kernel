<!-- SPDX-License-Identifier: Apache-2.0 -->
# NeoSymbian/kernel

This is EKA2 (Symbian's kernelhwsrv, 2010), ported to AArch64 with 32-bit pointers (ILP32). The 2010 user side runs
unchanged at AArch32 EL0. The maps:
- [`README.md`](README.md): the repository, the arm64 port, the kernel/user split, licences;
- [`baseport/arm64/README.md`](baseport/arm64/README.md): boards, how drivers plug in, adding a device or driver, and
  the known debts with their planned layout.

Read the relevant map before adding files. The workspace `../CLAUDE.md` and NeoSymbian/docs (`roadmap.md`,
`port-map.md`, `telephony.md`) hold the wider plan.

## Where new code goes

| Code | Place |
|---|---|
| CPU, exceptions, MMU, scheduler back end, GIC, atomics | `kernel/eka/<part>/arm64`, headers in `kernel/eka/include/<part>/arm64` |
| A fix in a generic Symbian file | In place, marked `NeoSymbian` (or an `__CPU_ARM64` section) |
| A driver every board needs (media, HAL, file-system readers, the monitor) | `baseport/arm64/common/{specific,inc}` |
| An SoC block (bus, clocks, storage, USB, PMIC, remote processor, GPU, audio) | `baseport/arm64/<soc>_assp/` (the planned `qcom/<subsystem>/` split is not done yet; don't start it piecemeal) |
| A part on one board, or one board's policy | `baseport/arm64/<board>_variant/specific/` |
| A new device | A new `<board>_variant/` (+ `<soc>_assp/` if the SoC is new) |
| An interface for the user side | A Symbian HAL group or framework the 2010 user side already uses; else a NeoSymbian `EHalGroupVariant` function (`0x4e00`+, take the next free number) or an LDD |
| Protocols, policy, daemons, UI | Not here: Rust in NeoSymbian/e32-rs, neoui, telephony |
| Tests | `kerneltest/e32test/nkernsa/arm64` (nanokernel), `kerneltest/e32test/arm64` |
| Build, run and phone tools | `baseport/arm64/tools/` |

Code lives at the most specific level that owns it, and moves up when a second user appears.

## Wiring a driver (today)

There is no registry yet: every step is explicit, and a missed step fails **silently** into a weak default.
1. Add an `ekc "$ASSPDIR/…"` or `ekc "$VDIR/specific/…"` line to the board's `variant_drivers()` in `variant.conf`.
   Give the file a unique basename: `ekc` names objects by basename.
2. If it starts at boot, add an `A64Start<Name>()` weak default plus a call in `A64ExtensionEntry`
   (`common/specific/a64_variant.cpp`). Order matters: a bus must come up before its users (SPMI starts inside
   `A64StartKeys` today).
3. Map its registers in the board's `A64_IoRegions`.
4. For a monitor command, add it to the `A64Mon_Line` chain and add a weak stub in `common/specific/a64_dbgmon.cpp`.
5. Afterwards, confirm on the target that it actually ran (its log line or monitor command), not just that the build
   linked.

## Rules

- **No Rust in the kernel.** Rust has no ILP32 target, so drivers are C++ here or Rust on the user side.
- **ILP32:** `long` and pointers are 32 bits. Use `TUint64` for 64-bit system registers and values; don't port
  LP64 assumptions from Linux drivers.
- **Never use the `EFastExecFastAtomic*` exec calls**: this kernel lacks them.
- **Debug-monitor output** through `Say`/`A64Mon_Say`/`Kern::Printf` must stay under 256 bytes per line, or the
  kernel faults.
- **Licences:**
  - our new files: `SPDX-License-Identifier: Apache-2.0`;
  - files derived from Symbian code: EPL-2.0, naming the source;
  - a changed Symbian file: EPL-2.0 SPDX on header line 7 + a `The NeoSymbian Authors (2026): …` Contributors line
    (keep `__LINE__` stable).
- **A known limit** gets a `shortcut: <limit>, <when to upgrade>` comment.
- **Never commit** the phone's DTB, Qualcomm firmware, IMEI/ICCID, or partition dumps (`.gitignore` covers the usual
  names). They belong in `../private`.

## Gates

Build on ZimaOS (`ssh zimaos`, clone at `/DATA/AppData/symbian-build/kernel`), from the `build` checkout beside it:
```sh
neo nkernsa qemu                 # after nanokernel, GIC, timer, ekern.lds or boot changes
neo kernel qemu && neo qemu 60   # every kernel change: the full kernel + 2010 user side boots
DTB=../private/bonito.dtb neo kernel bonito   # anything under sdm670_assp/ or bonito_variant/; then neo flash bonito
```
Phone runs: a RAM boot of an unmarked slot uses up its tries. See the workspace CLAUDE.md, "The phone".

After a kernel commit that `build` should use, re-pin `build/sources.lock`.

## Git

- `main` is ours; the upstream branches (`master`, `BRANCH_*`) stay untouched. Never push to SymbianSource.
- Kernel (telephony, board bring-up) and Rust sessions run in parallel, often in this checkout:
  - commit only your own paths (`git commit -- <paths>`);
  - never stash, revert or commit another session's edits;
  - check `git status` before you start.
