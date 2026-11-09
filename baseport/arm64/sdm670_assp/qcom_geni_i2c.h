// SPDX-License-Identifier: Apache-2.0
// qcom_geni_i2c.h -- an I2C master on a Qualcomm GENI serial engine, FIFO mode, polled (no interrupt, no GPI DMA).
#ifndef QCOM_GENI_I2C_H
#define QCOM_GENI_I2C_H
// Set the engine at physical aSe (clocked, pins muxed, I2C firmware loaded) up for 100 kHz from a 19.2 MHz source:
// the firmware's protocol (3 = I2C) or a negative error
int I2c_Init(unsigned long aSe);
// Write aWn bytes, then (if aRn) a repeated start and read aRn bytes from 7-bit address aAddr: KErrNone or an error
// (KErrNotFound = NACK). Thread context; never hangs.
int I2c_WriteRead(unsigned aAddr, const unsigned char* aW, int aWn, unsigned char* aR, int aRn);
// The same for engine aEngine (0 = touch, as above; 1 = the audio codecs' I2C10), at aKhz 100 or 400
int I2c_Open(TInt aEngine, unsigned long aSe, TInt aKhz);
int I2c_Xfer(TInt aEngine, unsigned aAddr, const unsigned char* aW, int aWn, unsigned char* aR, int aRn);
#endif
