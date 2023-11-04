/**
  ******************************************************************************
  * @file    IEC60730_B_param.h
  * @author
  * @version V1.2.0
  * @date    19-Jun-2019
  * @brief   This file contains the parameters to be customized for
  *                      the final application.
  ******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
/// @attention
///
/// THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
/// CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
/// TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
/// CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
/// HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
/// CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
///
/// <H2><CENTER>&COPY; COPYRIGHT MINDMOTION </CENTER></H2>
////////////////////////////////////////////////////////////////////////////////



/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IEC60730_B_PARAM_H
#define __IEC60730_B_PARAM_H

#include <stdint.h>
#include <types.h>

/* Includes ------------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* This is for having self-diagnostic messages reported on a PC via UART */
#define ClassB_VERBOSE_POR        /* During Power-on phase only */
#define ClassB_VERBOSE            /* During main program execution  */

/* uncoment next lines to configure watchdogs & resets for debug purposes */
#define USE_INDEPENDENT_WDOG
//#define USE_WINDOW_WDOG

/* comment out next line to force micro reset at fail mode */
#define NO_RESET_AT_FAIL_MODE

/* comment out next line when CRC unit is not configurable */
#define CRC_UNIT_CONFIGURABLE

/* comment out next line when there is no window feature at IWDG */
#define IWDG_FEATURES_BY_WINDOW_OPTION

/* comment out next line when HSE clock is not used - clock test then runs based on HSI */
//#define HSE_CLOCK_APPLIED */

/* system clock & HSE when HSE is applied as PLL source */
#if !defined  (HSE_VALUE)
#define HSE_VALUE    ((uint32_t)8000000)
#endif /* HSE_VALUE */

/* Reserved area for RAM buffer, incl overlap for test purposes */
/* Don't change this parameter as it is related to physical technology used! */
#define RT_RAM_BLOCKSIZE   ((uint32_t)6u)
/* Min overlap to cover coupling fault from one tested row to the other */
#define RT_RAM_BLOCK_OVERLAP  ((uint32_t)1u)

/* These are the direct and inverted data (pattern) used during the RAM
test, performed using March C- Algorithm */
#define BCKGRND     ((uint32_t)0x00000000uL)
#define BCKGRND_VA  ((uint32_t)0xAAAAAAAAuL)
#define INV_BCKGRND ((uint32_t)0xFFFFFFFFuL)
/* uncoment next line to use March-X test instead of March-C */
/* #define USE_MARCHX_RAM_TEST */

/* This is to provide a time base longer than the SysTick for the main */
/* For instance thi scan be used to signalize refresh the LSI watchdog and window watchdog at main */
#define SYSTICK_10ms_TB  ((uint32_t)10uL) /* 10*1ms */

#define LSI_Min       18570u
#define LSI_Max       69190u
#define HSE_Min       60937u
#define HSE_Max       64063u // hse_128div incap
#define HSI_Min       60937u
#define HSI_Max       64063u // hsi_128div incap
//#define HSE_INCAP     3u
//#define HSI_INCAP     2u
#define LSI_INCAP     1u

/* define the maximum U32 */
//#define U32_MAX    ((uint32_t)4294967295uL)

/* -------------------------------------------------------------------------- */
/* ------------------ CONTROL FLOW TAGS and CHECKPOINTS --------------------- */
/* -------------------------------------------------------------------------- */
/* RAM_TEST_CALLEE is only needed for CtrlFlowCntInv when exiting routine */
/* This is because the RAM test routines destroys the control flow counters */
#define RAM_TEST_CALLEE         ((uint32_t)0xFFFFFFFFuL)

#define CPU_TEST_CALLER         ((uint32_t)2)
#define CPU_TEST_CALLEE         ((uint32_t)3) /* Do not modify: hard coded in assembly file */
#define WDG_TEST_CALLER         ((uint32_t)5)
#define CRC32_INIT_CALLER       ((uint32_t)7)
#define CRC32_TEST_CALLER       ((uint32_t)11)
#define CRC32_TEST_CALLEE       ((uint32_t)13)
#define CRC_TEST_CALLER         ((uint32_t)17)
#define CRC_STORE_CALLER        ((uint32_t)19)
#define CLOCK_TEST_CALLER       ((uint32_t)23)
#define CLOCK_TEST_CALLEE       ((uint32_t)29)
#define LSI_INIT_CALLEE         ((uint32_t)31)
#define HSE_INIT_CALLEE         ((uint32_t)37)
#define RTC_INIT_CALLEE         ((uint32_t)41)
#define SYSTICK_INIT_CALLEE     ((uint32_t)43)
#define CLOCK_SWITCH_CALLEE     ((uint32_t)47)
#define STACK_OVERFLOW_TEST     ((uint32_t)53)
#define STACK_OVERFLOW_CALLEE   ((uint32_t)59)
#define CLOCKPERIOD_TEST_CALLEE ((uint32_t)61)
#define FLASH_TEST_CALLER       ((uint32_t)63)
#define CRC32_RUN_TEST_CALLEE   ((uint32_t)63)
#define Inter_error             ((uint32_t)80)

#define CHECKPOINT1 ((uint32_t)CPU_TEST_CALLER + \
                          CPU_TEST_CALLEE + \
                          WDG_TEST_CALLER + \
                          CRC32_TEST_CALLER + \
                          CRC_TEST_CALLER)

#define CHECKPOINT2 ((uint32_t)CRC_STORE_CALLER +\
                          CLOCK_TEST_CALLER + \
                          CLOCK_TEST_CALLEE + \
                          STACK_OVERFLOW_TEST)

/* This is for run-time tests with 32-bit CRC */
#define DELTA_MAIN  ((uint32_t)CPU_TEST_CALLER + \
                          CPU_TEST_CALLEE + \
                          STACK_OVERFLOW_TEST + \
                          STACK_OVERFLOW_CALLEE + \
                          CLOCK_TEST_CALLER + \
                          CLOCKPERIOD_TEST_CALLEE + \
                          FLASH_TEST_CALLER + \
                          CRC32_RUN_TEST_CALLEE)

#define LAST_DELTA_MAIN ((uint32_t) DELTA_MAIN + CRC32_INIT_CALLER)
//#define FULL_FLASH_CHECKED ((uint32_t)DELTA_MAIN * STEPS_NUMBER + LAST_DELTA_MAIN)

#define MEASPERIOD_ISR_CALLER   ((uint32_t)2)
#define MEASPERIOD_ISR_CALLEE   ((uint32_t)3)
#define CLOCKPERIOD_ISR_CALLEE  ((uint32_t)5)
#define RAM_MARCHC_ISR_CALLER   ((uint32_t)7)
#define RAM_MARCHC_ISR_CALLEE   ((uint32_t)11)

/* This is for March C tests */
#define DELTA_ISR  (uint32_t)(RAM_MARCHC_ISR_CALLER + \
                           RAM_MARCHC_ISR_CALLEE)



/* Exported macro ------------------------------------------------------------*/
#define init_control_flow() (CtrlFlowCntInv = ~(CtrlFlowCnt=0))
#define control_flow_call(a) CtrlFlowCnt += (a)
#define control_flow_resume(a) CtrlFlowCntInv -= (a)




#ifdef __CC_ARM   /* KEIL Compiler */

/* This is the KEIL compiler entry point, usually executed right after reset */
extern void Reset_Handler( void );
extern const uint32_t __Check_Sum;
extern void __main( void );

/* Constants necessary for Flash CRC calculation (ROM_SIZE in byte) */
/* byte-aligned addresses */

/* Constants necessary for execution initial March test */
#define RAM_START ((uint32_t *)0x20000000uL)

#define CRC_FLAG	CRC->IDR
#define GotoCompilerStartUp()  { CRC_FLAG = 0xAAu; __main(); } /* entry to init C before main() */

#endif  /* __CC_ARM */




#ifdef __GNUC__   /* GCC Compiler */


/* This is the KEIL compiler entry point, usually executed right after reset */
extern void Reset_Handler( void );
extern const uint32_t __Check_Sum;
extern void __main( void );

/* Constants necessary for Flash CRC calculation (ROM_SIZE in byte) */
/* byte-aligned addresses */

/* Constants necessary for execution initial March test */
#define RAM_START ((uint32_t *)0x20000000uL)

#define CRC_FLAG	CRC->IDR
#define GotoCompilerStartUp()  { CRC_FLAG = 0xAAu; __main(); } /* entry to init C before main() */
#endif  /* __GNUC__ */

/* Exported functions ------------------------------------------------------- */
void FailSafePOR(uint32_t E_ID);

#if defined(ClassB_VERBOSE) || defined(ClassB_VERBOSE_POR)
#define ClassB_Uart UART2
#endif /* ClassB_VERBOSE */

ErrorStatus control_flow_check_point(uint32_t chck);

#endif /* __SELFTEST_PARAM_H */
/******************* (C) COPYRIGHT Mindmotion *****END OF FILE****/
