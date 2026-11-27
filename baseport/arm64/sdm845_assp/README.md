<!-- SPDX-License-Identifier: Apache-2.0 -->
# sdm845_assp: Qualcomm SDM845

The SoC drivers for SDM845 boards: today the OnePlus 6T (`fajita_variant`). The port plan, the facts read from the
phone and the stages OP0–OP13 are in docs `fajita.md`.

Most SDM845 blocks sit at the same addresses as SDM670's (GIC, SPMI, USB, RPMh, SMEM, QMP, MSS, IPA, WCN3990). A
driver is copied here from `sdm670_assp` when the stage that needs it comes, with SDM845's tables (owner, 2026-11-26).
A fix to a shared block goes into both copies until the two folders are merged.

Empty in OP2: the board boots on the framebuffer console with no SoC drivers.
