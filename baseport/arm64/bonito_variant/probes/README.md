# Boot probes for the Pixel 3a XL (bonito)

Tiny bare-metal images, RAM-booted with `fastboot boot` (nothing flashed), that paint the bootloader's
framebuffer one color and halt. They answer: what does Google's ABL accept, and can it start a 32-bit kernel?

```sh
probes/build.sh path/to/bonito.dtb      # DTB = the one appended to a working bonito kernel
fastboot boot probes/out/probe-a64.img  # screen turns green; hold Power+VolDown to get back to fastboot
```

The framebuffer is the bootloader's continuous splash from the DTB: `/chosen/framebuffer@9c000000`,
1080x2160, stride 4320, a8r8g8b8 (`fb.h`).

## Results (2026-11-05, unlocked Pixel 3a XL, SDC panel)

| Image | ABL answer | Screen |
|---|---|---|
| arm64 Image (gzip + appended DTB), `text_offset = 0` | `Invalid Parameter` | — |
| same, padded to 36 MB, `image_size` = Linux's | `Invalid Parameter` | — |
| arm64 Image, `text_offset = 0x80000` (`probe-a64.img`) | `OKAY` | green ✅ |
| ARM zImage (uncompressed, 0x016f2818 magic, DTB at `end`) | `Unsupported` | — |
| same with a `Linux version` string / Linux-style zImage fields | `Unsupported` | — |
| ARM zImage gzipped | `Volume Corrupt` | — |
| arm64 stub that `eret`s into AArch32 EL0 code (`probe-el0.img`) | `OKAY` | blue ✅ |

Conclusions:
- Our own code runs at boot, at EL1 in AArch64 with the MMU off, and can draw on the screen. The only ABL
  requirement found beyond the Linux packaging was `text_offset = 0x80000`.
- Google's ABL recognises a 32-bit ARM kernel and refuses it (`Unsupported`). Qualcomm's public ABL has a
  `SwitchTo32bitModeBooting` path; the Pixel build doesn't allow it. So Phorepoc's 32-bit ARM EKA2 kernel
  can't be booted as is.
- AArch32 at EL0 under our AArch64 EL1 code works (`el0.S`): the 32-bit user code painted the screen, no
  exception came back. So the 32-bit Symbian user side (Phorepoc's `arm` build) can run unchanged; only the
  kernel side has to become AArch64.
