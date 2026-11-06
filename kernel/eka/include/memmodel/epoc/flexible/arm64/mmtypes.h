// Portions Copyright (c) Nokia Corporation and/or its subsidiary(-ies).
// SPDX-License-Identifier: EPL-2.0
//
// mmtypes.h -- shadow of include/memmodel/epoc/flexible/mmtypes.h for the AArch64 kernel (its arm64 dir first on
// -I). VMSAv8 translation table descriptors are 64-bit at every level (Phorepoc's x64mm/mmtypes.h did the same for
// long mode).
#ifndef MMTYPES_H
#define MMTYPES_H

typedef TUint64 TPde;

typedef TUint64 TPte;

#endif
