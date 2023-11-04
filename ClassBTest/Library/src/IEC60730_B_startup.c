/**
  ******************************************************************************
  * @file    mm32_IEC60730startup.c
  * @author  NJ ClassB TEAM
  * @date    19-Jun-2019
  * @brief   This file contains sequence of self-test routines to be executed just
  *          after initialization of the application before main execution starts
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



/* Includes ------------------------------------------------------------------*/
#define ALLOC_GLOBALS
//#include "main.h"
#include "hal_conf.h"
#include "IEC60730_B_param.h"
#include "IEC60730_B_lib.h"
#include "IEC60730_B_startup.h"
#include "uart_classb.h"
#include <stdio.h>
extern uint32_t Incap_Freq;
/* Exported functions ------------------------------------------------------- */

#ifdef __IAR_SYSTEMS_ICC__  /* IAR Compiler */
extern void __iar_program_start(void);
#define GotoCompilerStartUp() { __iar_program_start(); }
#define CRC_BLOCK_BYTE          64u
/* The 4 variables here above are set by IAR linker */
extern uint32_t __ICFEDIT_region_ROM_start__;
extern uint32_t __ICFEDIT_region_ROM_end__;
extern uint32_t __ICFEDIT_region_RAM_start__;
extern uint32_t __ICFEDIT_region_RAM_end__;
extern uint32_t __ICFEDIT_region_CLASSB_start__;
extern uint32_t __ICFEDIT_region_CLASSB_end__;

#pragma section = ".intvec"
#pragma section = ".data_init"

/* Constants necessary for Flash CRC calculation  (last block - 64 bytes - separated for CRC) */
#define ROM_START             ((uint32_t)__section_begin(".intvec"))
#define ROM_CODE_END          ((uint32_t)__section_end(".data_init"))

#define ROM_END                                                                                                      \
                                ((uint32_t)(((ROM_CODE_END)&(CRC_BLOCK_BYTE - 1)) ? ((ROM_CODE_END + CRC_BLOCK_BYTE) & \
                                                               (~(CRC_BLOCK_BYTE - 1))) : ROM_CODE_END))

#define CODE_ALL_SIZE         (ROM_CODE_END - ROM_START)
#define ROM_SIZE              (ROM_END - ROM_START)

#define STEPS_NUMBER          ((uint32_t)ROM_SIZE / (CRC_BLOCK_BYTE / 16))
#define ROM_SIZEinWORDS       (uint32_t)(ROM_SIZE / (CRC_BLOCK_BYTE / 16))
#define FLASH_BLOCK_WORDS     (uint32_t)((ROM_SIZEinWORDS) / STEPS_NUMBER)

/* Constants necessary for Full March tests */
#define RAM_START             (uint32_t *)&__ICFEDIT_region_RAM_start__
#define RAM_END               (uint32_t *)&__ICFEDIT_region_RAM_end__

/* Constants necessary for Transparent March tests */
#define CLASS_B_START         ((uint32_t *)(&__ICFEDIT_region_CLASSB_start__))
#define CLASS_B_END           ((uint32_t *)aStackOverFlowPtrn - 4)
#define REF_CRC32  ROM_END


#endif  /* __IAR_SYSTEMS_ICC__ */

#if defined(__CC_ARM) || defined(__GNUC__)
extern uint32_t Load$$LR$$LR_IROM1$$Base,Load$$LR$$LR_IROM1$$Length,Load$$LR$$LR_IROM1$$Limit;
#define CRC_BLOCK_BYTE    64u
#define ROM_START         ((uint32_t)&Load$$LR$$LR_IROM1$$Base)
#define CODE_SIZE         ((uint32_t)&Load$$LR$$LR_IROM1$$Length)
#define CODE_END          ((uint32_t)&Load$$LR$$LR_IROM1$$Limit)
#define ROM_END           ((uint32_t)((CODE_END & (CRC_BLOCK_BYTE - 1))?((CODE_END + CRC_BLOCK_BYTE) &(~(CRC_BLOCK_BYTE - 1))):CODE_END))
#define ROM_SIZE          (ROM_END - ROM_START)
#define ROM_SIZEinWORDS (uint32_t) (ROM_SIZE/(CRC_BLOCK_BYTE/16))
#define STEPS_NUMBER          ((uint32_t)ROM_SIZE/CRC_BLOCK_BYTE)
#define FLASH_BLOCK_WORDS     (uint32_t)((ROM_SIZEinWORDS)/STEPS_NUMBER)
#define REF_CRC32  ROM_END
#endif  /* __CC_ARM */

#define MCU_RAM_BuffertestEnd_ADDR    (0x20000000UL + 32UL-1UL)
#define MCU_RAM_RunTimetestStart_ADDR (0x20000030UL)
#define MCU_RAM_RunTimetestEnd_ADDR   (0x2000007FUL)
#define MCU_RAM_End_ADDR   (0x20001FFFUL)
uint32_t RAMRunTimetest_start;
uint32_t RAMRunTimetest_end;
uint32_t RAMBuffertest_end;
uint32_t RAM_endadd_run;

#if defined(__CC_ARM) || defined(__GNUC__)
uint32_t App_ROM_START;
uint32_t App_CODE_SIZE;
uint32_t App_ROM_END;
uint32_t App_ROM_SIZE;
uint32_t App_ROM_SIZEinWORDS;
uint32_t App_STEPS_NUMBER;
uint32_t App_FLASH_BLOCK_WORDS;
uint32_t App_REF_CRC32;
#endif  /* __CC_ARM */

#ifdef __IAR_SYSTEMS_ICC__  /* IAR Compiler */
//uint8_t * App_ROM_START;
//uint8_t * App_ROM_END;
uint32_t App_ROM_START;
uint32_t App_ROM_END;
uint32_t App_ROM_SIZE;
uint32_t App_ROM_SIZEinWORDS;
uint32_t App_STEPS_NUMBER;
uint32_t App_FLASH_BLOCK_WORDS;
//uint32_t * App_REF_CRC32;
uint32_t  App_REF_CRC32;
#endif  /* __IAR_SYSTEMS_ICC__ */
/** @addtogroup
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void WWDG_Init(unsigned char ucTcnt, unsigned char ucWcnt);
void CBUart_Init(u32 bound);
void Iwdg_Init(unsigned short int IWDG_Prescaler, unsigned short int Reload);
void LSI_Calibration(void);
//void Flashtest_config(void);
ErrorStatus (*FlashFulltestcall)(void);

#ifdef __IAR_SYSTEMS_ICC__  /* IAR Compiler */
extern  void __iar_data_init3(void);
#endif /* __IAR_SYSTEMS_ICC__ */
/* Private functions ---------------------------------------------------------*/

#ifdef __IAR_SYSTEMS_ICC__  /* IAR Compiler */
#pragma optimize = none
#endif
void Flashtest_config(void)
{
#if defined(__CC_ARM) || defined(__GNUC__)
    App_ROM_START = ROM_START;
    App_ROM_END = ROM_END;
    App_ROM_SIZE = ROM_SIZE;
    App_ROM_SIZEinWORDS = ROM_SIZEinWORDS;
    App_STEPS_NUMBER = STEPS_NUMBER;
    App_FLASH_BLOCK_WORDS = FLASH_BLOCK_WORDS;
    App_REF_CRC32 = (uint32_t)REF_CRC32;
#endif  /* __CC_ARM */

#ifdef __IAR_SYSTEMS_ICC__  /* IAR Compiler */
    App_ROM_START = ROM_START;
    App_ROM_END = ROM_END;
    App_ROM_SIZE = ROM_SIZE;
    App_ROM_SIZEinWORDS = ROM_SIZEinWORDS;
    App_STEPS_NUMBER = STEPS_NUMBER;
    App_FLASH_BLOCK_WORDS = FLASH_BLOCK_WORDS;
    App_REF_CRC32 = REF_CRC32;

#endif  /* __IAR_SYSTEMS_ICC__ */

    FlashFulltestcall = 	IEC60730_B_FullFlash_test;
}

void Ramtest_config(void)
{
    RAMBuffertest_end = MCU_RAM_BuffertestEnd_ADDR;
    RAMRunTimetest_start = MCU_RAM_RunTimetestStart_ADDR;
    RAMRunTimetest_end = MCU_RAM_RunTimetestEnd_ADDR ;
    RAM_endadd_run = MCU_RAM_End_ADDR;
}
/******************************************************************************/
/**
  * @brief  FullCpu_test code
  * @param  : None
  * @retval : N
  */
ErrorStatus IEC60730_B_FullCpu_test(void)
{
    ErrorStatus result = SUCCESS;
    /*--------------------------------------------------------------------------*/
    /*------------------- CPU registers and flags self test --------------------*/
    /*--------------------------------------------------------------------------*/
    control_flow_call(CPU_TEST_CALLER);
    /* WARNING: all registers destroyed when exiting this function (including
    preserved registers R4 to R11) while excluding stack pointer R13) */
    if (IEC60730_B_StartUpCPUTest() != CPUTEST_SUCCESS)
    {
        result = ERROR;

    }
    else  /* CPU Test OK */
    {
        control_flow_resume(CPU_TEST_CALLER);
    }

    return (result);
}


void IEC60730_ClockTest_Startup(void)
{
    ClockStatus clk_sts;
    /*--------------------------------------------------------------------------*/
    /*----------------------- Clock Frequency Self Test ------------------------*/
    /*--------------------------------------------------------------------------*/
    control_flow_call(CLOCK_TEST_CALLER);

    /* test LSI & HSE clock systems */
    clk_sts = IEC60730_ClockStartUpTest();

    if (clk_sts != FREQ_OK)
    {
        FailSafePOR(CLOCK_TEST_CALLER);
    }

    control_flow_resume(CLOCK_TEST_CALLER);
}

#if defined (__CC_ARM) || defined (__GNUC__) /* KEIL Compiler */
extern int $Super$$main(void);
/******************************************************************************/
/**
  * @brief  Switch between startup and main code
  * @param  : None
  * @retval : None
  */
void $Sub$$main(void)
{
    //LSI_Calibration();
    if (CRC_FLAG != 0xAAu)
    {
        IEC60730_B_startup();                 /* trick to call StartUp before main entry */
    }

    CRC_FLAG = 0u;
    $Super$$main ();
}

#endif /* __CC_ARM */
/******************************************************************************/
/**
  * @brief  Contains the very first test routines executed right after
  *   the reset
  * @param  : None
  *   Flash interface initialized, Systick timer ON (2ms timebase)
  * @retval : None
  */

void IEC60730_B_startup(void)
{
    /* block run time tests performed at SysTick interrupt */
    TickCounter = TickCounterInv = 0;

#ifdef ClassB_VERBOSE_POR
    CBUart_Init(9600);
#endif
    /*--------------------------------------------------------------------------*/
    /*------Initialization of counters for control flow monitoring *------------*/
    /*--------------------------------------------------------------------------*/
    init_control_flow();

    if(IEC60730_B_FullCpu_test() != SUCCESS)
    {
        FailSafePOR(CPU_TEST_CALLEE);
    }

    /*--------------------------------------------------------------------------*/
    /*--------------------- WDOGs functionality self test ----------------------*/
    /*--------------------------------------------------------------------------*/

    control_flow_call(WDG_TEST_CALLER);

    /* two phases IWDG & WWDG test, system reset is performed here */
    if(IEC60730_B_WDGSelfTest() != SUCCESS)
		{
			FailSafePOR(WDG_TEST_CALLER);
		}

    control_flow_resume(WDG_TEST_CALLER);

    Flashtest_config();
    if(FlashFulltestcall() != SUCCESS)
    {
        FailSafePOR(CRC32_TEST_CALLER);
    }

    /*--------------------------------------------------------------------------*/
    /*   Verify Control flow before RAM init (which clears Ctrl flow counters)  */
    /*--------------------------------------------------------------------------*/
    if (control_flow_check_point(CHECKPOINT1) == ERROR)
    {
        FailSafePOR(CHECKPOINT1);
    }

#if 1
    /*--------------------------------------------------------------------------*/
    /* --------------------- Variable memory functional test -------------------*/
    /*--------------------------------------------------------------------------*/
    /* no stack operation can be performed during the test */

    __disable_irq();
    Ramtest_config();
    if((IEC60730_B_FullRamMarchC(RAM_START, (uint32_t *)RAM_endadd_run,BCKGRND_VA) != SUCCESS))
    {
        __enable_irq();
        FailSafePOR(RAM_TEST_CALLEE);
    }
    Ramtest_config();
    /* WARNING: Stack is zero-initialized when exiting from this routine */
    if (IEC60730_B_FullRamMarchC(RAM_START, (uint32_t *)RAM_endadd_run, BCKGRND) != SUCCESS)
    {
        /* restore interrupt capability */
        __enable_irq();
        FailSafePOR(RAM_TEST_CALLEE);
    }
    /* restore interrupt capability */
    __enable_irq();
#endif


#if 0 
    IEC60730_B_TranspMarchInit();
    __disable_irq();

    if(IEC60730_B_StartupMarch_VA5(RAMRunTimetest_start,RAMRunTimetest_end,RAM_endadd_run)!=SUCCESS)
    {
        __enable_irq();
        FailSafePOR(RAM_TEST_CALLEE);
    }

    if(IEC60730_B_StartupMarch(RAMRunTimetest_start,RAMRunTimetest_end,RAM_endadd_run)!=SUCCESS)
    {
        __enable_irq();
        FailSafePOR(RAM_TEST_CALLEE);
    }

    __enable_irq();
#endif
//    /* Initialization of counters for control flow monitoring
//       (destroyed during RAM test) */
    init_control_flow();

    IEC60730_B_CRC_StoreclassBData();

    IEC60730_ClockTest_Startup();
    /*--------------------------------------------------------------------------*/
    /* --------------- Initialize stack overflow pattern ---------------------- */
    /*--------------------------------------------------------------------------*/
    control_flow_call(STACK_OVERFLOW_TEST);

    aStackOverFlowPtrn[0] = 0xEEEEEEEEuL;
    aStackOverFlowPtrn[1] = 0xCCCCCCCCuL;
    aStackOverFlowPtrn[2] = 0xBBBBBBBBuL;
    aStackOverFlowPtrn[3] = 0xDDDDDDDDuL;

    control_flow_resume(STACK_OVERFLOW_TEST);
    /*--------------------------------------------------------------------------*/
    /* -----  Verify Control flow before Starting main program execution ------ */
    /*--------------------------------------------------------------------------*/

    if (control_flow_check_point(CHECKPOINT2) == ERROR)
    {
        FailSafePOR(CHECKPOINT2);
    }
    GotoCompilerStartUp();
}



/******************************************************************************/
/**
  * @brief  Verifies the consistency and value of control flow counters
  * @param  : check value of the positive counter
  * @retval : ErrorStatus (SUCCESS, ERROR)
  */
ErrorStatus control_flow_check_point(uint32_t chck)
{
    ErrorStatus Result = SUCCESS;

    if ((CtrlFlowCnt != (chck)) || ((CtrlFlowCnt ^ CtrlFlowCntInv) != 0xFFFFFFFFuL))
    {
        Result = ERROR;
    }
    return (Result);
}

void IEC60730_B_CRC_StoreclassBData(void)
{
	  /*--------------------------------------------------------------------------*/
    /*------------- Store reference 32-bit CRC in RAM after RAM test -----------*/
    /*--------------------------------------------------------------------------*/
    control_flow_call(CRC_STORE_CALLER);
    /* restore destroyed content of HAL structure for CRC */

    CRC_ResetDR();
    RefCrc32 = CRC_CalcBlockCRC(0u, 0u);
    RefCrc32Inv = ~RefCrc32;
    control_flow_resume(CRC_STORE_CALLER);
}


/**
  * @}
  */

/******************* (C) COPYRIGHT  Mindmotion *****END OF FILE****/
