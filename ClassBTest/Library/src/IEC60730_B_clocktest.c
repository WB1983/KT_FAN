/**
  ******************************************************************************
  * @file    MM32fxx_IEC60730clockstart.c
  * @author
  * @version V1.2.0
  * @date    19-Jun-2017
  * @brief   This file contains the test function for clock circuitry
  *          and wrong frequency detection at start-up.
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
#include <stdio.h>
#include "main.h"
#include "hal_conf.h"
#include "IEC60730_B_param.h"
#include "IEC60730_B_lib.h"
#include "Interrupts_classb.h"
#include "system_mm32spin0280.h"

extern type_InterruptTest systick;
extern uint32_t Incap_Freq;
volatile static uint32_t tmpIC1;
volatile static uint32_t tmp0;
//volatile static uint16_t period = 0;
//volatile static uint16_t duty = 0;
volatile  uint16_t period = 0;
volatile  uint16_t duty = 0;
//#define HSE_Min       60937u
/** @addtogroup MM32FxxSelfTestLib_src
  * @{
  */


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Start up the internal and external oscillators and verifies
  *   that clock source is within the expected range
  * @param  : None
  * @retval : ClockStatus = (LSI_START_FAIL, HSE_START_FAIL,
  *   HSI_HSE_SWITCH_FAIL, XCROSS_CONFIG_FAIL, EXT_SOURCE_FAIL, FREQ_OK)
  */

#if !defined(CUSTOM_HSE_FREQ)
static void delay_x_cycle(u32 count)
{
    vu32 i = 0, j = 0;
    vu32 sDelayNopNumber = 100;
    for(i = 0; i < count; i++) {
        for(j = 0; j < sDelayNopNumber; j++) {
            __NOP(); //
        }
    }
}

static s32 HseIncapture_Init(void)
{
    u32 temp;
    vu32 StartUpCounter = 0;
    s32 HSEStatus = 0;

    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));

    //SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------
    //Enable HSE
    RCC->CR |= ((u32)RCC_CR_HSEON);

    delay_x_cycle(5);

    //Wait till HSE is ready and if Time out is reached exit
    while(1) {
        temp = RCC->CR & RCC_CR_HSERDY;
        StartUpCounter++;
        if(StartUpCounter < (36 * HSE_STARTUP_TIMEOUT))
            continue;
        if(temp != 0) {
            break;
        }
        if(StartUpCounter >= (120 * HSE_STARTUP_TIMEOUT)) {
            //timeout
//            SetSysClockToDefaultHSI();
            HSEStatus = 0x02;
            return HSEStatus;
        }
    }
    delay_x_cycle(5);
    if ((RCC->CR & RCC_CR_HSERDY) == RESET) {
        //If HSE fails to start-up, the application will have wrong clock
        //configuration. User can add here some code to deal with this error
        HSEStatus = (u32)0x03;
        return HSEStatus;
    }

    HSEStatus = 0x00;

    return HSEStatus;
}
#endif
/**
  * @brief  This function verifies the frequency from the last clock
  *   period measurement
  * @param  : None
  * @retval : ClockStatus = (LSI_START_FAIL, HSE_START_FAIL,
  *   HSI_HSE_SWITCH_FAIL, TEST_ONGOING, EXT_SOURCE_FAIL,
  *   CLASS_B_VAR_FAIL, CTRL_FLOW_ERROR, FREQ_OK)
  */
ClockStatus IEC60730_MainClockTest(void)
{

    ClockStatus result = TEST_ONGOING; /* In case of unexpected exit */

    CtrlFlowCnt += CLOCKPERIOD_TEST_CALLEE;
    /* ==============================================================================*/
    /* MISRA violation of rule 12.4 - "&&" operand can't contain side effects in normal operation
     - all the following pairs of volatile variables are changed consistently at timer ISR only */
#ifdef __IAR_SYSTEMS_ICC__  /* IAR Compiler */
#pragma diag_suppress=Pm026
#endif /* __IAR_SYSTEMS_ICC__ */
    /* checking result of HSE measurement done at TIM17 interrupt */
    if (((PeriodValue ^ PeriodValueInv) == 0xFFFFFFFFuL)\
            &&  ((LSIPeriodFlag ^ LSIPeriodFlagInv) == 0xFFFFFFFFuL)\
            &&   (LSIPeriodFlag != 0u) )
#ifdef __IAR_SYSTEMS_ICC__  /* IAR Compiler */
#pragma diag_default=Pm026
#endif /* __IAR_SYSTEMS_ICC__ */
        /* ==============================================================================*/

    {
        result = IEC60730_clockrun_valuecheck();
    } /* Control Flow error */
    else
    {
        result = CLASS_B_VAR_FAIL;
    }
//printf("PeriodValue = %d (%d,%d)\n\r", PeriodValue, HSE_LimitLow(SYSTCLK_AT_RUN_HSE), HSE_LimitHigh(SYSTCLK_AT_RUN_HSE));
    CtrlFlowCntInv -= CLOCKPERIOD_TEST_CALLEE;

    return (result);

}


ClockStatus IEC60730_ClockStartUpTest(void)
{


    ClockStatus clck_sts = TEST_ONGOING;

    CtrlFlowCnt += CLOCK_TEST_CALLEE;


#ifdef CUSTOM_HSE_FREQ

    if (clck_sts == TEST_ONGOING)
    {
        /* Configure dedicated timer to measure LSI period */
        if (IEC60730_InitClock_Xcross_Measurement() == ERROR)
        {
            clck_sts = XCROSS_CONFIG_FAIL;
        }
        else
        {
            clck_sts = IEC60730_HSI_LSIcheck();
        }
    }

#else

    if (clck_sts == TEST_ONGOING)
    {
        /* Configure dedicated timer to measure HSE period */
        if (IEC60730_InitClock_Xcross_Measurement() == ERROR)
        {
            clck_sts = XCROSS_CONFIG_FAIL;
        }
        else
        {
          /*if(!HseIncapture_Init())
            {
                clck_sts = IEC60730_HSE_LSIcheck();
            }
            else {
                clck_sts = XCROSS_CONFIG_FAIL;
            }
					*/
        }
    }
#endif

    /* Switch back HSI internal clock at any case */
    //SetClockTo48_HSI();

    if (clck_sts == TEST_ONGOING)
        clck_sts = FREQ_OK;

    CtrlFlowCntInv -= CLOCK_TEST_CALLEE;

    return (clck_sts);

}
////////////////////////////////////////////////////////////////////////////////
/// @brief  set MCO, use SYSCLK
/// @param  void
/// @retval None
////////////////////////////////////////////////////////////////////////////////
static void MCO_Config(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    RCC_GPIO_ClockCmd(GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2ENR_SYSCFG, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_0);
    //MCO use PA9

    GPIO_StructInit(&GPIO_InitStruct);
    //MCO use PA9
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_8;

    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));

    RCC_MCOConfig(RCC_MCO_HSI);

    RCC->CFGR2 &= ~(0x1F << 20);
    RCC->CFGR2 |= RCC_CFGR2_MCO_PRE_128;
}
////////////////////////////////////////////////////////////////////////////////
/// @brief  TIM3 Pin Config
/// @param  None
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void TIM3_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_GPIO_ClockCmd(GPIOA, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_1);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}
////////////////////////////////////////////////////////////////////////////////
/// @brief  TIM3 input capture period&duty Config
/// @param  None
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void TIM3_PWM_Input_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct;
    TIM_ICInitTypeDef  TIM3_ICInitStruct;
    NVIC_InitTypeDef        NVIC_InitStruct;

    TIM3_GPIO_Init();

    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM3, ENABLE);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
    TIM_TimeBaseStruct.TIM_Period    = 0xFFFF;
    TIM_TimeBaseStruct.TIM_Prescaler = 1 - 1;
    //Setting Clock Segmentation
    TIM_TimeBaseStruct.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
    ///TIM Upward Counting Mode
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);

    TIM_ICStructInit(&TIM3_ICInitStruct);
    TIM3_ICInitStruct.TIM_Channel     = TIM_Channel_2;
    TIM3_ICInitStruct.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    TIM3_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM3_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM3_ICInitStruct.TIM_ICFilter    = 0x0;
    //Configure PWM
    TIM_PWMIConfig(TIM3, &TIM3_ICInitStruct);
    //Select the valid input
    TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);
    //Configuration in master-slave reset mode
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);

    TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
#if 1
    NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x00;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
#endif
    TIM_Cmd(TIM3, ENABLE);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  TIM3 Handler Funtion
/// @param  None
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
#if defined TIM3_FOR_FS
void TIM3_IRQHandler(void)
{
    u32 tmp2;
    float f0=0;
    //Capture 1 Occurs Capture Event
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
    {
        duty= TIM_GetCapture1(TIM3);
        period   = TIM_GetCapture2(TIM3);
    }
    if( RCC->CFGR & RCC_CFGR_PPRE1)
    {
        tmp2 = RCC_GetPCLK1Freq()*2;
    }
    else
    {
        tmp2 = RCC_GetPCLK1Freq();
    }
    if (LSIPeriodFlag == 0u)
    {
        f0 = (float)tmp2 / (float)period ;
        Incap_Freq = (u32)f0;
        /* Set Flag tested at main loop */
        PeriodValueInv = ~PeriodValue;
        LSIPeriodFlag = 0xAAAAAAAAuL;
        LSIPeriodFlagInv = 0x55555555uL;
    }

    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}
#endif
uint32_t temp;
////////////////////////////////////////////////////////////////////////////////
/// @brief  TIM2 input capture  Config
/// @param  None
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void Tim2_Init(void)
{

    exNVIC_Init_TypeDef  NVIC_InitStruct;
    //-------------------------------------------------------------
    TIM_TimeBaseInitTypeDef TIM_StructInit;
    TIM_ICInitTypeDef TIM_ICStructInit;
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM2, ENABLE);               //TIM2 CLOCK ENALBE,Clock is pclk1=8mhz
    TIM_StructInit.TIM_Period=0xffffffff;
    TIM_StructInit.TIM_Prescaler=0;
    TIM_StructInit.TIM_ClockDivision=TIM_CKD_DIV1;                  //
    TIM_StructInit.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_StructInit.TIM_RepetitionCounter=0;
    TIM_TimeBaseInit(TIM2, &TIM_StructInit);

    TIM_ICStructInit.TIM_Channel=TIM_Channel_4;
    TIM_ICStructInit.TIM_ICPolarity=TIM_ICPolarity_Rising;
    TIM_ICStructInit.TIM_ICSelection=TIM_ICSelection_DirectTI;
    TIM_ICStructInit.TIM_ICPrescaler=TIM_ICPSC_DIV8;                // flsi/8
    TIM_ICStructInit.TIM_ICFilter=0;
    TIM_ICInit(TIM2, &TIM_ICStructInit);

    TIM_ITConfig(TIM2,TIM_IT_CC4,ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

    exNVIC_Init(&NVIC_InitStruct);

    TIM_Cmd(TIM2, DISABLE);
    TIM2->CNT=0;
#ifdef CUSTOM_HSE_FREQ
    TIM2->RESERVED0x50=(1<<6);
#else
#ifdef	 HSE_INCAP
    TIM2->RESERVED0x50=(3<<6);                                      //P323,lsi as timer2.ti4 capture input
#else
    TIM2->RESERVED0x50=(1<<6);
#endif
#endif
    //-------------------------------------------------------------
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);             //POWER reg clocked
    RCC_LSICmd(ENABLE);
    temp = RCC_GetPCLK1Freq() ;
}
/**
  * @brief This function handles tim2 interrupt.
  */
void TIM2_IRQHandler(void)
{
    u32 tmp0,tmp2;
    float f0=0;

    if(TIM_GetITStatus(TIM2,TIM_IT_CC4))
    {
        tmp0 = tmpIC1;
        tmpIC1 = TIM_GetCapture4(TIM2);

        if( RCC->CFGR & RCC_CFGR_PPRE1)
        {
            tmp2 = RCC_GetPCLK1Freq()*2;
        }
        else
        {
            tmp2 = RCC_GetPCLK1Freq();
        }

        if (LSIPeriodFlag == 0u)
        {

            if ( TIM_GetFlagStatus(TIM2,TIM_FLAG_CC4OF) == 0u)
            {
                f0 = (float)tmp2 / (float)(tmpIC1 - tmp0) * 8;
                Incap_Freq = (u32)f0;
                /* Set Flag tested at main loop */
                PeriodValue = (uint32_t)((tmpIC1 - tmp0)) & 0xFFFFuL;
                PeriodValueInv = ~PeriodValue;
                LSIPeriodFlag = 0xAAAAAAAAuL;
                LSIPeriodFlagInv = 0x55555555uL;
            }
            else
            {

                TIM_ClearFlag(TIM2,TIM_FLAG_CC4OF);
            }
        }

        TIM_ClearITPendingBit(TIM2,TIM_IT_CC4);
    }

}

////////////////////////////////////////////////////////////////////////////////
/// @brief  IEC60730 clock module test Config
/// @param  None
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
ErrorStatus IEC60730_InitClock_Xcross_Measurement(void)                  //clock domain��max pll
{
    ErrorStatus result = SUCCESS;
#if defined(CUSTOM_HSE_FREQ)&&defined(HSI_INCAP)
#if defined TIM3_FOR_FS
    MCO_Config();
    TIM3_PWM_Input_Init();
#endif
#else
    Tim2_Init();

    TIM_ClearFlag(TIM2, TIM_FLAG_CC4);
    TIM_Cmd(TIM2, ENABLE);
    TIM_CCxCmd(TIM2, TIM_Channel_4, TIM_CCx_Enable);
#endif
    return result;
}
////////////////////////////////////////////////////////////////////////////////
/// @brief  IEC60730 clock module HSE CLOCK Valuecheck at startup
/// @param  None
/// @retval ClockStatus = (LSI_START_FAIL, HSE_START_FAIL,
///  HSI_HSE_SWITCH_FAIL, XCROSS_CONFIG_FAIL, EXT_SOURCE_FAIL, FREQ_OK)
////////////////////////////////////////////////////////////////////////////////
ClockStatus IEC60730_HSE_LSIcheck(void)
{
    ClockStatus clck_sts = TEST_ONGOING;
    LSIPeriodFlag = 0u;
    while (LSIPeriodFlag == 0u)
    { }
    LSIPeriodFlag = 0u;
    while (LSIPeriodFlag == 0u)
    { }
    /*-------------------- HSE measurement check -------------------------*/
#ifdef	 HSE_INCAP

    if(Incap_Freq < HSE_Min )
    {
        clck_sts = EXT_SOURCE_FAIL;
    }
    else if(Incap_Freq > HSE_Max)
    {
        clck_sts = EXT_SOURCE_FAIL;
    }

#else
    if(Incap_Freq < LSI_Min )
    {
        clck_sts = EXT_SOURCE_FAIL;
    }
    else if(Incap_Freq > LSI_Max)
    {
        clck_sts = EXT_SOURCE_FAIL;
    }
#endif
//#ifdef ClassB_VERBOSE_POR
//			 printf("\r\nRead Incapture Frequency OK freq = %05dHz !\r\n", Incap_Freq);
//#endif
    return (clck_sts);
}
////////////////////////////////////////////////////////////////////////////////
/// @brief  IEC60730 clock module HSI CLOCK Valuecheck at startup
/// @param  None
/// @retval ClockStatus = (LSI_START_FAIL, HSE_START_FAIL,
///  HSI_HSE_SWITCH_FAIL, XCROSS_CONFIG_FAIL, EXT_SOURCE_FAIL, FREQ_OK)
////////////////////////////////////////////////////////////////////////////////
ClockStatus IEC60730_HSI_LSIcheck(void)
{
    ClockStatus clck_sts = TEST_ONGOING;
    LSIPeriodFlag = 0u;
    while (LSIPeriodFlag == 0u)
    { }
    LSIPeriodFlag = 0u;
    while (LSIPeriodFlag == 0u)
    { }
    /*-------------------- HSI measurement check -------------------------*/
#ifdef	 HSI_INCAP
    if(Incap_Freq < HSI_Min )
    {
        clck_sts = EXT_SOURCE_FAIL;
    }
    else if(Incap_Freq > HSI_Max)
    {
        clck_sts = EXT_SOURCE_FAIL;
    }
#else
    if(Incap_Freq < LSI_Min )
    {
        clck_sts = EXT_SOURCE_FAIL;
    }
    else if(Incap_Freq > LSI_Max)
    {
        clck_sts = EXT_SOURCE_FAIL;
    }
#endif
//#ifdef ClassB_VERBOSE_POR
//			 printf("\r\nRead Incapture Frequency OK freq = %05dHz !\r\n", Incap_Freq);
//#endif
    return (clck_sts);
}
////////////////////////////////////////////////////////////////////////////////
/// @brief  IEC60730 clock module  CLOCK Valuecheck at run time
/// @param  None
/// @retval ClockStatus = (LSI_START_FAIL, HSE_START_FAIL,
///  HSI_HSE_SWITCH_FAIL, XCROSS_CONFIG_FAIL, EXT_SOURCE_FAIL, FREQ_OK)
////////////////////////////////////////////////////////////////////////////////
ClockStatus IEC60730_clockrun_valuecheck(void)
{
    ClockStatus result = TEST_ONGOING;
#ifdef CUSTOM_HSE_FREQ
#ifdef HSI_INCAP
    if(Incap_Freq < HSI_Min )
#else
    if(Incap_Freq < LSI_Min )
#endif
#else
#ifdef HSE_INCAP
    if(Incap_Freq < HSE_Min )
#else
    if(Incap_Freq < LSI_Min )
#endif
#endif
    {
        result = EXT_SOURCE_FAIL;	/* Sub-harmonics: HSE -25% below expected */
    }
    else
    {
#ifdef CUSTOM_HSE_FREQ
#ifdef HSI_INCAP
        if (Incap_Freq > HSI_Max)
#else
        if (Incap_Freq > LSI_Max)
#endif
#else
#ifdef HSE_INCAP
        if (Incap_Freq > HSE_Max)
#else
        if (Incap_Freq > LSI_Max)
#endif

#endif
        {
            /* Switch back to internal clock */
            result = EXT_SOURCE_FAIL;	/* Harmonics: HSE +25% above expected */
            //result = FREQ_OK;	/* Harmonics: HSE +25% above expected */
        }
        else
        {
            result = FREQ_OK;         /* Crystal or Resonator started correctly */
            /* clear flag here to ensure refresh LSI measurement result will be taken at next check */
            LSIPeriodFlag = 0u;
        } /* No harmonics */
    } /* No sub-harmonics */

    return (result);
}


/**
  * @}
  */

/******************* (C) COPYRIGHT 2019 Mindmotion *****END OF FILE****/
