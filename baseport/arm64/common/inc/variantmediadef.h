/* Portions Copyright The Phorepoc Authors
 * SPDX-License-Identifier: Apache-2.0
 *
 * variantmediadef.h -- the local drives of the AArch64 boards, read by the unmodified media drivers
 * (drivers/medint/iram.cpp): one internal RAM drive, local drive 0 (estart.txt mounts it as C:).
 */
#ifndef __VARIANTMEDIADEF_H__
#define __VARIANTMEDIADEF_H__

#define IRAM_DRIVECOUNT 1
#define IRAM_DRIVELIST  0
#define IRAM_NUMMEDIA   1
#define IRAM_DRIVENAME "IRam"

#endif
