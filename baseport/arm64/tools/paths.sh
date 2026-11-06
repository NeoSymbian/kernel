# SPDX-License-Identifier: Apache-2.0
# paths.sh -- where the arm64 port sits in the kernel tree. Sourced by the build scripts, which set HERE to
# baseport/arm64. KHS = the kernel tree to build (default: the checkout this script is in).
KHS=${KHS:-$(cd "$HERE/../.." && pwd)}
EKA=$KHS/kernel/eka
FLX=$EKA/memmodel/epoc/flexible
NK64=$EKA/nkernsmp/arm64			# SMP nanokernel backend, GICv3, platform layer
K64=$EKA/kernel/arm64				# kernel backend, boot
NKSA64=$KHS/kerneltest/e32test/nkernsa/arm64	# nkernsa glue
BSP=$HERE					# board support: common, <soc>_assp, <board>_variant
# the arm64 headers that stand in for the ARM ones: searched first
INC64="-I $EKA/include/nkernsmp/arm64 -I $EKA/include/kernel/arm64 -I $EKA/include/memmodel/epoc/flexible/arm64 -I $FLX/mmu/arm64"
PRE="-include $EKA/include/nkern/arm64/port_prelude.h -include $EKA/include/nkern/arm64/arm64_prelude.h -include $EKA/include/e32def.h"

# The board: baseport/arm64/<BOARD>_variant/variant.conf (the role variant.mmh plays in Symbian's own builds) sets
# ASSP (the SoC folder <ASSP>_assp), KERNEL_BASE, ROM_LIN, BOARD_SRC, BOOT_DEFS, BOOTIMG and optionally DTB, FONT,
# NKSA_FAST_CPU, and defines variant_drivers (its kernel extensions and drivers, compiled with ekc <src> [flags]).
# A new device is a new <board>_variant folder, plus an <soc>_assp folder unless it reuses one.
BOARD=${BOARD:-qemu}
VDIR=$BSP/${BOARD}_variant
[ -f "$VDIR/variant.conf" ] || { echo "unknown BOARD=$BOARD (no $VDIR/variant.conf)" >&2; exit 2; }
FONT=""; NKSA_FAST_CPU=""; DTB=${DTB:-}; BOOTIMG=none
. "$VDIR/variant.conf"
ASSPDIR=$BSP/${ASSP}_assp

# make_bootimg <Image> <out dir>: the image the board's bootloader takes (BOOTIMG=fastboot: gzip(Image) + DTB in an
# Android boot.img)
make_bootimg() {
	case $BOOTIMG in
	none) ;;
	fastboot)
		gzip -9nc "$1" > "$2/Image.gz"
		python3 -I "$HERE/tools/mkboot.py" "$2/Image.gz" "${DTB:?set DTB to the board's device tree blob}" "$2/boot.img" ;;
	*) echo "unknown BOOTIMG=$BOOTIMG" >&2; exit 2 ;;
	esac
}
