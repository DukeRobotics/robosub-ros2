/** @file PDD_Definitions.h
 *  @brief Generic system dependent macros and includes
 * 
 * Copyright (c) Teledyne Marine 2021. All rights reserved.
 *
 * This file is part of TRDI.Drivers.PDDecoder library.
 */
#ifndef PDD_DEFINITIONS_H
#define PDD_DEFINITIONS_H

#ifdef _WIN32
// Windows support
#define WIN32_LEAN_AND_MEAN    
#include "windows.h"

#ifndef TRDI_PDD_EXPORTS
#define PDD_DECL __declspec(dllexport)
#else
#define PDD_DECL __declspec(dllimport)
#endif

#else
// Linux support
#define PDD_DECL
#endif

#endif