/*******************************************************************************
 *  Copyright (c) 2017 BSH Hausgeraete GmbH,
 *  Carl-Wery-Str. 34, 81739 Munich, Germany, www.bsh-group.de
 *
 *  All rights reserved. This program and the accompanying materials
 *  are protected by international copyright laws.
 *  Please contact copyright holder for licensing information.
 *
 *******************************************************************************
 *  PROJECT          PP PED Framework
 *  COMP_ABBREV      --
 ******************************************************************************/

#ifndef LIBTYPES_H
#define LIBTYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/* DOCUMENTATION                                                              */
/******************************************************************************/
/** \file
 *
 *  \brief    Standard data types, independent of processor/compiler.
 *
 *  \details  As C99 defines portable data types in stdint.h this header should
 *            be used instead. Support of old standard may be removed in future.
 *            \b Note: Due to portability reasons it is not allowed to use 
 *            C base types. Exception: automatic storage loop counters 
 */


#include <stdint.h>
#include <stdbool.h>


typedef uint8_t  uchar;   ///< Standard type, 8  bit unsigned. Hint: Not to use in new code, use C99 uint8_t  instead
typedef uint16_t uint16;  ///< Standard type, 16 bit unsigned. Hint: Not to use in new code, use C99 uint16_t instead
typedef uint32_t uint32;  ///< Standard type, 32 bit unsigned. Hint: Not to use in new code, use C99 uint32_t instead
typedef int16_t  int16;   ///< Standard type, 16  bit  signed. Hint: Not to use in new code, use C99 int16_t  instead
typedef int32_t  int32;   ///< Standard type, 32  bit  signed. Hint: Not to use in new code, use C99 int32_t  instead
typedef uint16_t uint;    ///< Old standard type, 16 bit unsigned. Hint: support will be removed soon
typedef uint32_t ulong;   ///< Old standard type, 32 bit unsigned. Hint: support will be removed soon
typedef int8_t   schar;   ///< Old standard library type, 8 bit signed. Hint: support will be removed soon 


#ifdef __cplusplus
}
#endif

#endif
