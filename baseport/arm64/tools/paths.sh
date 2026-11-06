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
