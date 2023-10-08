#ifndef __LIBDEFINES_H

#define __LIBDEFINES_H

#include "libtypes.h"

#ifndef NULL

#define NULL   ((void *) 0)    ///< zero pointer

#endif
#define OFF            ((BOOL)0)       //!< logical OFF
#define LOW            ((BOOL)0)       //!< logical LOW

#ifndef FALSE

#define FALSE          ((BOOL)0)       //!< logical FALSE

#endif
#define ON             ((BOOL)1)       //!< logical ON
#define HIGH           ((BOOL)1)       //!< logical HIGH

#ifndef TRUE

#define TRUE           ((BOOL)1)       //!< logical TRUE

#endif
#define INVALID        ((uchar)0xFFU)        //!< INVALID is used to symbolize an invalid result for 8 bit variables
#define UI_INVALID     0xFFFFU               //!< UI_INVALID is used to symbolise an invalid result for 16 bit variables
#define UL_INVALID     0xFFFFFFFFUL          //!< UL_INVALID is used to symbolise an invalid result for 32 bit variables


#define TASK_NOT_INITIALISED ((uchar)0)      //!< task/init handlers have to return this when they are not initilised
#define TASK_INITIALISED     ((uchar)1)      //!< task handlers have to return this when they are initilised, do not use for init handlers
#define TASK_TERMINATE       ((uchar)2)      //!< whan task/init handler are returning this value, they will be disbled (not called again)

#define NIBBLE_SIZE      ((uchar)0x04U)      //!< Number of bits in a nibble
#define BYTE_SIZE        ((uchar)0x08U)      //!< Number of bits in a byte
#define INT_SIZE         ((uchar)0x10U)      //!< Standardised number of bits in an integer (which implicates that a uint32 is not called integer)
#define LONG_SIZE        ((uchar)0x20U)      //!< Standardised number of bits in a ulong integer (which BTW is not called integer, see INT_SIZE)

#define MSB_BYTE         ((uchar)0x80U)      //!< Most significant bit of a byte
#define LSB_BYTE         ((uchar)0x01U)      //!< Least significant bit of a byte


#define MK_BIT(BitType,BitPos) ((BitType)(((BitType)(1))<<(BitPos)))



#define BIT_0  MK_BIT(uchar, 0)  //!< Bit  0
#define BIT_1  MK_BIT(uchar, 1)  //!< Bit  1
#define BIT_2  MK_BIT(uchar, 2)  //!< Bit  2
#define BIT_3  MK_BIT(uchar, 3)  //!< Bit  3
#define BIT_4  MK_BIT(uchar, 4)  //!< Bit  4
#define BIT_5  MK_BIT(uchar, 5)  //!< Bit  5
#define BIT_6  MK_BIT(uchar, 6)  //!< Bit  6
#define BIT_7  MK_BIT(uchar, 7)  //!< Bit  7
#define BIT_8  MK_BIT(uint,  8)  //!< Bit  8
#define BIT_9  MK_BIT(uint,  9)  //!< Bit  9
#define BIT_10 MK_BIT(uint, 10)  //!< Bit 10
#define BIT_11 MK_BIT(uint, 11)  //!< Bit 11
#define BIT_12 MK_BIT(uint, 12)  //!< Bit 12
#define BIT_13 MK_BIT(uint, 13)  //!< Bit 13
#define BIT_14 MK_BIT(uint, 14)  //!< Bit 14
#define BIT_15 MK_BIT(uint, 15)  //!< Bit 15
#define BIT_16 MK_BIT(ulong,16)  //!< Bit 16
#define BIT_17 MK_BIT(ulong,17)  //!< Bit 17
#define BIT_18 MK_BIT(ulong,18)  //!< Bit 18
#define BIT_19 MK_BIT(ulong,19)  //!< Bit 19
#define BIT_20 MK_BIT(ulong,20)  //!< Bit 20
#define BIT_21 MK_BIT(ulong,21)  //!< Bit 21
#define BIT_22 MK_BIT(ulong,22)  //!< Bit 22
#define BIT_23 MK_BIT(ulong,23)  //!< Bit 23
#define BIT_24 MK_BIT(ulong,24)  //!< Bit 24
#define BIT_25 MK_BIT(ulong,25)  //!< Bit 25
#define BIT_26 MK_BIT(ulong,26)  //!< Bit 26
#define BIT_27 MK_BIT(ulong,27)  //!< Bit 27
#define BIT_28 MK_BIT(ulong,28)  //!< Bit 28
#define BIT_29 MK_BIT(ulong,29)  //!< Bit 29
#define BIT_30 MK_BIT(ulong,30)  //!< Bit 30
#define BIT_31 MK_BIT(ulong,31)  //!< Bit 31


#define LD_STRINGIZE_(s) #s ///< Internally used in macro LD_STRINGIZE()

/** Creates a quoted string from argument 's'.
 * 
 * Helps to create a valid quoted c-string from a string constant e.g. from build process.\n
 * LD_STRINGIZE(PROCESSOR) will expand PROCESSOR and set quotes --> e.g. "R8C25"\n
 * This is usefull, as a quoted definition will not be expanded.
 */
#define LD_STRINGIZE(s) LD_STRINGIZE_(s)


#define LD_CONCAT_(a,b)  a ## b

/** Concatenate the strings 'a' and 'b'.\n
 *  When 'a' and/or 'b' are definitions they will be expanded before concatenation
 */
#define LD_CONCAT(a,b)  LD_CONCAT_(a,b)

/* The following definitions are shared between bootloader and D-Bus-2 */
#define LD_BLD_FAST_BOOT_ACTIVE     0x55AA  //!< If Bootloader variable BLD_uiFastBoot has this value -> jump to APP
#define LD_BLD_LONG_BOOT_TIMEOUT    0x55EE  //!< If Bootloader variable BLD_uiFastBoot has this value -> long TMO used


#endif
