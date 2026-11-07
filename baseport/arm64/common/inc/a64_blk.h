// SPDX-License-Identifier: Apache-2.0
// a64_blk.h -- the block device behind the persistent C: drive (board/a64_medblk.cpp): 512-byte sectors, polled,
// thread context. Each board links one backend (QEMU virt: virtio_blk.cpp; Pixel: qcom_sdhci.cpp). Everything above
// the backend sees only a WINDOW of the device (the whole disk, or one GPT partition): window-relative sector
// numbers, checked before any command is issued, so no request can reach outside it.
#ifndef A64_BLK_H
#define A64_BLK_H

const TInt KA64BlkSector = 512;

// ---- backend, one per board (weak defaults: no device)
TInt A64BlkDev_Init(TUint64& aSectors);					// KErrNone and the device size, or an error
TInt A64BlkDev_Read(TUint64 aLba, TInt aCount, TUint8* aBuf);
TInt A64BlkDev_Write(TUint64 aLba, TInt aCount, const TUint8* aBuf);
// the board's window on the device (weak default: the whole device): KErrNone and [aFirst, aFirst + aCount)
TInt A64BlkBoard_Window(TUint64 aSectors, TUint64& aFirst, TUint64& aCount);
// the device's erase unit in bytes, a power of 2 dividing the window's start (weak default: 0, unknown); FAT's
// format aligns its data area to it
TUint32 A64BlkDev_EraseBlock();
// tell the device a range no longer holds data (eMMC TRIM; weak default: nothing to do). Advisory: KErrNone even if
// the device failed it (FAT's DeleteNotify faults the file server on an error)
TInt A64BlkDev_Discard(TUint64 aLba, TInt aCount);
// a GPT partition of the allow-list by name (weak default: KErrNotFound): [aFirst, aFirst + aCount) on the device
TInt A64BlkDev_Partition(const char* aName, TUint64& aFirst, TUint64& aCount);

// one sector of the GPT itself: the primary or backup header, or a sector of either entry array -- nothing else
// (Pixel: qcom_sdhci.cpp, for qcom_bootctrl.cpp; weak default: KErrNotSupported)
TInt A64BlkDev_WriteGpt(TUint64 aLba, const TUint8* aSector);

// ---- A/B slots (docs/filesystem.md FS5b; Pixel: qcom_bootctrl.cpp; weak defaults: one slot, 'a')
TInt A64Slot_Current();									// 'a' or 'b': the slot ABL booted
void A64Slot_Show(TDes8& aOut);							// both slots' ABL attributes, one line
// mark the current slot's boot "successful" in both GPT copies (backup first, each read back), so ABL stops
// counting its retries down; aDryRun: only say what would be written. KErrNone if done or already set
TInt A64Slot_MarkOk(TBool aDryRun, TDes8& aOut);
void A64Slot_BootDry(TDes8& aOut);						// what the boot-time try count would write (never writes)

// ---- the window
TInt A64Blk_Start();										// backend init + the board's window
TUint64 A64Blk_Sectors();								// the window's size, 0 if none
TUint64 A64Blk_First();									// the window's first sector on the device
TInt A64Blk_Read(TUint64 aLba, TInt aCount, TUint8* aBuf);			// window-relative
TInt A64Blk_Write(TUint64 aLba, TInt aCount, const TUint8* aBuf);	// window-relative
TInt A64Blk_Discard(TUint64 aLba, TInt aCount);						// window-relative
#endif
