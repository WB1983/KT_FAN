/**
  ******************************************************************************
  * @file    MM32fxx_IEC60730init.c
  * @author
  * @version V1.2.0
  * @date    19-Jun-2019
  * @brief   Contains the Self-test functions executed during main program execution.
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
#include "main.h"
#include "hal_conf.h"
#include "IEC60730_B_param.h"
#include "IEC60730_B_lib.h"
#include "led_classb.h"
#include <stdio.h>
#include "system_mm32spin0280.h"
extern uint32_t SystemCoreClock;
extern uint32_t App_STEPS_NUMBER;
uint32_t FULL_FLASH_CHECKED;
uint8_t   runtime_checkindx = 0;
uint8_t   runtime_flash_CtrlFlow_indx = 0;
uint32_t rom_test;
/** @addtogroup MM32FxxSelfTestLib_src
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
ErrorStatus IEC60730_CheckStack(void);
extern uint32_t HAL_GetTick(void);
ClassBTestStatus (*FlashRuntestcall)(void);
/******************************************************************************/
/**
  * @brief  Initializes the Class B variables and their inverted redundant
  *   counterparts. Init also the Systick for clock frequency monitoring.
  * @param  : None
  * @retval : None
  */
void IEC60730_InitRunTimeChecks(void)
{
//    uint32_t tout;

    /* Initialize variables for invariable memory check */
    IEC60730_B_TranspMarchInit();

    /* Init Class B variables required in main routine and SysTick interrupt
    service routine for timing purposes */
    TickCounter = 0u;
    TickCounterInv = 0xFFFFFFFFuL;

    TimeBaseFlag = 0u;
    TimeBaseFlagInv = 0xFFFFFFFFuL;

    LastCtrlFlowCnt = 0u;
    LastCtrlFlowCntInv = 0xFFFFFFFFuL;
    init_control_flow();

    /* Initialize variables for SysTick interrupt routine control flow monitoring */
    ISRCtrlFlowCnt = 0u;
    ISRCtrlFlowCntInv = 0xFFFFFFFFuL;

    aStackOverFlowPtrn[0] = 0xEEEEEEEEuL;
    aStackOverFlowPtrn[1] = 0xCCCCCCCCuL;
    aStackOverFlowPtrn[2] = 0xBBBBBBBBuL;
    aStackOverFlowPtrn[3] = 0xDDDDDDDDuL;

    /* Initialize SysTick to generate 1ms time base */
    if (SysTick_Config(SystemCoreClock/1000uL) != 0x00)
    {
#ifdef ClassB_VERBOSE_POR
        printf("Run time base init failure\n\r");
#endif /* ClassB_VERBOSE_POR */
        FailSafePOR(SYSTICK_INIT_CALLEE);
    }

    CRC_ResetDR();
    RefCrc32 = CRC_CalcBlockCRC(0u,0u);
    RefCrc32Inv = ~RefCrc32;
    /* Initialize variables for invariable memory check */
    Flashtest_config();
    IEC60730_FlashCrc32Init();
    printf("&pRunCrc32Chk:%x\n\r",(uint32_t)pRunCrc32Chk);
    FULL_FLASH_CHECKED = ((uint32_t)DELTA_MAIN * App_STEPS_NUMBER + LAST_DELTA_MAIN);
    FlashRuntestcall = IEC60730_crc32Run;
    /* wait for  incapture Value is valid */
    LSIPeriodFlag = 0u;
    while (LSIPeriodFlag == 0u)
    { }

    /* Initialize variables for main routine control flow monitoring */
    CtrlFlowCnt = 0u;
    CtrlFlowCntInv = 0xFFFFFFFFuL;

}

ErrorStatus IEC60730_B_CpuRunTime_test(void)
{

    ErrorStatus result = SUCCESS;
    /*----------------------------------------------------------------------*/
    /*---------------------------- CPU registers ----------------------------*/
    /*----------------------------------------------------------------------*/
    control_flow_call(CPU_TEST_CALLER);

    if (IEC60730_B_RunTimeCPUTest() != CPUTEST_SUCCESS)
    {
        result = ERROR;
    }
    else
    {
        control_flow_resume(CPU_TEST_CALLER);
    }
    return (result);
}
ErrorStatus IEC60730_Stack_Check(void)
{
    ErrorStatus result = SUCCESS;
    /*----------------------------------------------------------------------*/
    /*------------------------- Stack overflow -----------------------------*/
    /*----------------------------------------------------------------------*/
    control_flow_call(STACK_OVERFLOW_TEST);

    if (IEC60730_CheckStack() != SUCCESS)
    {
        result = ERROR;
    }
    else
    {
        control_flow_resume(STACK_OVERFLOW_TEST);
    }
    return (result);
}

void IEC60730_ClockTest_RunTime(void)
{
    /*----------------------------------------------------------------------*/
    /*------------------------- Clock monitoring ---------------------------*/
    /*----------------------------------------------------------------------*/
    control_flow_call(CLOCK_TEST_CALLER);
    switch ( IEC60730_MainClockTest() )
    {
    case FREQ_OK:
        control_flow_resume(CLOCK_TEST_CALLER);
        runtime_checkindx++;
        break;

    case EXT_SOURCE_FAIL:
#ifdef ClassB_VERBOSE
//            /* finish communication flow prior system clock change */

        printf("\n\r Clock Source failure (Run-time)\n\r");
#endif /* ClassB_VERBOSE */
        FailSafePOR(CLOCK_TEST_CALLER);
        break;

    case CLASS_B_VAR_FAIL:
#ifdef ClassB_VERBOSE
        printf("\n\r Class B variable error (clock test)\n\r");
#endif /* ClassB_VERBOSE */
        FailSafePOR(CLOCK_TEST_CALLER);
        break;

    case LSI_START_FAIL:
    case HSE_START_FAIL:
    case HSI_HSE_SWITCH_FAIL:
    case TEST_ONGOING:
    case CTRL_FLOW_ERROR:
    default:
#ifdef ClassB_VERBOSE
        printf("Abnormal Clock Test routine termination \n\r");
#endif  /* ClassB_VERBOSE */
        FailSafePOR(CLOCK_TEST_CALLER);
        break;
    }
}


/******************************************************************************/
/**
  * @brief  Perform set of generic tests at run time from main
  *         (can be divided into more separated segments if necessary)
  * @param  : None
  * @retval : None
  */
void IEC60730_DoRunTimeChecks(void)
{


    /* Is the time base duration elapsed? */
    if (TimeBaseFlag == 0xAAAAAAAAuL)

    {
        /* Verification of TimeBaseFlag integrity */
        /* The following pair of volatile variables is changed consistently at Systick ISR only */
        if ((TimeBaseFlag ^ TimeBaseFlagInv) == 0xFFFFFFFFuL)
        {
            TimeBaseFlag = 0u;

            if (runtime_checkindx == 0)
            {
                if(IEC60730_B_CpuRunTime_test() != SUCCESS )
                {
#ifdef ClassB_VERBOSE
                    printf("Run-time CPU Test Failure\n\r");
#endif /* ClassB_VERBOSE */
                    FailSafePOR(CPU_TEST_CALLER);
                }
                else
                {
                    runtime_checkindx++;
                }

            }
            else if (runtime_checkindx == 1)
            {
                if(IEC60730_Stack_Check()!=SUCCESS)
                {
#ifdef ClassB_VERBOSE
                    printf("Stack overflow\n\r");
#endif /* ClassB_VERBOSE */
                    FailSafePOR(STACK_OVERFLOW_TEST);
                }
                else
                {
                    runtime_checkindx++;
                }
            }
            else if (runtime_checkindx == 2)
            {
                IEC60730_ClockTest_RunTime();
            }
            else if (runtime_checkindx == 3)
            {
                IEC60730_B_FlashRunTime_test();
            }
        }
        else
        {
#ifdef ClassB_VERBOSE
            printf("Class B variable Error (main loop)\n\r");
#endif  /* ClassB_VERBOSE */
            FailSafePOR(DELTA_MAIN);
        }

    } /* End of periodic Self-test routine */

}

void IEC60730_B_FlashRunTime_test(void)
{

    /*----------------------------------------------------------------------*/
    /*------------------ Invariable memory CRC check -----------------------*/
    /*----------------------------------------------------------------------*/
    if(runtime_flash_CtrlFlow_indx == 0)
    {
        control_flow_call(FLASH_TEST_CALLER);
        rom_test = FlashRuntestcall();  /* Requires the control flow check to be modified */
//        printf("&pRunCrc32Chk:%x\n\r",(uint32_t)pRunCrc32Chk);
        switch ( rom_test )
        {
        case TEST_RUNNING:
            control_flow_resume(FLASH_TEST_CALLER);
            runtime_flash_CtrlFlow_indx++;
            break;

        case TEST_OK:
#ifdef ClassB_VERBOSE
            /* avoid any long string output here, '*' just marks the flash test completed ok */
#ifndef __GNUC__
//              putchar((int16_t)'*');
#else
//              __io_putchar((int16_t)'*');
#endif /* __GNUC__ */
#endif /* ClassB_VERBOSE */

            CtrlFlowCntInv -= FLASH_TEST_CALLER;
            runtime_flash_CtrlFlow_indx++;
            break;
        case TEST_FAILURE:
        case CLASS_B_DATA_FAIL:
        default:
#ifdef ClassB_VERBOSE
            printf(" Run-time FLASH CRC Error\n\r");
#endif  /* ClassB_VERBOSE */
            FailSafePOR(FLASH_TEST_CALLER);
            break;
        }
    }
    else if(runtime_flash_CtrlFlow_indx == 1)

    {
        /*----------------------------------------------------------------------*/
        /*---------------- Check Safety routines Control flow  -----------------*/
        /*------------- Refresh Window and independent watchdogs ---------------*/
        /*----------------------------------------------------------------------*/

        if (((CtrlFlowCnt ^ CtrlFlowCntInv) == 0xFFFFFFFFuL)
                &&((LastCtrlFlowCnt ^ LastCtrlFlowCntInv) == 0xFFFFFFFFuL))
        {
            if (rom_test == TEST_OK)
            {
                /* ==============================================================================*/
                /* MISRA violation of rule 11.4,17.4 - pointer arithmetic & casting has is used */
#ifdef __IAR_SYSTEMS_ICC__  /* IAR Compiler */
#pragma diag_suppress=Pm088,Pm141
#endif /* __IAR_SYSTEMS_ICC__ */
                if ((CtrlFlowCnt == FULL_FLASH_CHECKED)
                        && ((CtrlFlowCnt - LastCtrlFlowCnt) == (LAST_DELTA_MAIN)))
#ifdef __IAR_SYSTEMS_ICC__  /* IAR Compiler */
#pragma diag_default=Pm088,Pm141
#endif /* __IAR_SYSTEMS_ICC__ */
                    /* ==============================================================================*/
                {
                    CtrlFlowCnt = 0u;
                    CtrlFlowCntInv = 0xFFFFFFFFuL;

                }
                else  /* Return value form crc check was corrupted */
                {
#ifdef ClassB_VERBOSE
                    printf("Control Flow Error (main loop, Flash CRC)\n\r");
#endif  /* ClassB_VERBOSE */
                    FailSafePOR(LAST_DELTA_MAIN);
                }
            }
            else  /* Flash test not completed yet */
            {
                if ((CtrlFlowCnt - LastCtrlFlowCnt) != DELTA_MAIN)
                {
#ifdef ClassB_VERBOSE
                    printf("Control Flow Error (main loop, Flash CRC on-going)\n\r");
#endif  /* ClassB_VERBOSE */
                    FailSafePOR(DELTA_MAIN);
                }
            }

            LastCtrlFlowCnt = CtrlFlowCnt;
            LastCtrlFlowCntInv = CtrlFlowCntInv;
            runtime_flash_CtrlFlow_indx =0;
            runtime_checkindx =0;
        }
        else
        {
#ifdef ClassB_VERBOSE
            printf("Class B variable Error (main loop)\n\r");
#endif  /* ClassB_VERBOSE */
            FailSafePOR(DELTA_MAIN);
        }
    }


}
/******************************************************************************/
/**
  * @brief  This function verifies that Stack didn't overflow
  * @param  : None
  * @retval : ErrorStatus = (ERROR, SUCCESS)
  */
ErrorStatus IEC60730_CheckStack(void)
{
    ErrorStatus result = SUCCESS;

    CtrlFlowCnt += STACK_OVERFLOW_CALLEE;

    if ( aStackOverFlowPtrn[0] != 0xEEEEEEEEuL )
    {
        result = ERROR;
    }
    if ( aStackOverFlowPtrn[1] != 0xCCCCCCCCuL )
    {
        result = ERROR;
    }
    if ( aStackOverFlowPtrn[2] != 0xBBBBBBBBuL )
    {
        result = ERROR;
    }
    if ( aStackOverFlowPtrn[3] != 0xDDDDDDDDuL )
    {
        result = ERROR;
    }

    CtrlFlowCntInv -= STACK_OVERFLOW_CALLEE;

    return (result);
}

/**
  * @}
  */

/******************* (C) COPYRIGHT Mindmotion *****END OF FILE****/
