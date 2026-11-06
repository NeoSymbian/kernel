// Copyright (c) 2008-2009 Nokia Corporation and/or its subsidiary(-ies).
// All rights reserved.
// This component and the accompanying materials are made available
// under the terms of the License "Eclipse Public License v1.0"
// which accompanies this distribution, and is available
// at the URL "http://www.eclipse.org/legal/epl-v10.html".
// SPDX-License-Identifier: EPL-2.0 (changed by the NeoSymbian Authors)
// Initial Contributors:
// Nokia Corporation - initial contribution.
//
// Contributors:
// The NeoSymbian Authors (2026): 64-bit VMSAv8 descriptors under __CPU_ARM64.
// Description:
//

/**
 @file
 @internalComponent
*/

#ifndef MMTYPES_H
#define MMTYPES_H


#ifdef __CPU_ARM64	// NeoSymbian: VMSAv8 translation table descriptors are 64-bit at every level
typedef TUint64 TPde;

typedef TUint64 TPte;
#else
typedef TUint32 TPde;

typedef TUint32 TPte;
#endif


#endif


