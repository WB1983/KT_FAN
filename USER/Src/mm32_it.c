/**
 * @file     mm32_it.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides the ITR functions and test samples.
 *
 * @attention
 *
 * THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
 * CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
 * TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
 * HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
 * CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
 *
 * <H2><CENTER>&COPY; COPYRIGHT MINDMOTION </CENTER></H2>
 */

/** Define to prevent recursive inclusion */
#define _MM32_IT_C_

/** Files includes */
#include "drv_inc.h"
#include "FOC_PMSM.h"
#include "StateMachine.h"
//#include "systick.h"
#include "Common.h"
#include "Compile.h"
#if(FS_MOTOR_FS_MONITOR == OPTION_ACTIVE)
#include "fsHandle.h"
#endif
#include "fsClock.h"
#include "fsMCU.h"
#include "Current.h"
#include "hal_misc.h"

/**
 * @addtogroup MM32_User_Layer
 * @{
 */
CallbackFunc2 STK_pfGeneralTimerCallback = NULL;
static CallbackFunc2 UART_pfReceivedCallback = NULL;

CallbackFunc2 STK_pfGeneralTimerCallbackUART3 = NULL;
static CallbackFunc2 UART_pfReceivedCallbackUART3 = NULL;


static CallbackFunc2 SCL_pfCaptureCallback = NULL;

static uint8_t MIT_TEST = 0;
static uint16_t MIT_Dure_Time;
static uint16_t MIT_Start_time;
static uint16_t MIT_Stop_time;

static uint32_t u16Test1;
static uint8_t MIT_TEST2 = 0;





static TInputCapture        MIT_tInputCapture = {0,0,0};

void MIT_vSetPeriodEvnetFlag(void);
void MIT_vReadNormalADCValue(void);

/**
 * @addtogroup User_Main
 * @{
 */

void NVIC_Configure(uint8_t ch, uint8_t pri)
{
    NVIC_InitTypeDef  NVIC_InitStruct;

    /** Initialization ADC interrupt */
    NVIC_InitStruct.NVIC_IRQChannel = ch;
    NVIC_InitStruct.NVIC_IRQChannelPriority = pri;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

void Interrupt_Init(void)
{
    /** Initialization Systick interrupt */
    NVIC_SetPriority(SysTick_IRQn, SYSTICK_INTERRUPT);
    /** Initialization ADC interrupt */
    NVIC_Configure(ADC2_IRQn, ADC2_INTERRUPT);
    /** ADC EOF interrupt enabled */
    ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);
    ADC_ClearITPendingBit(ADC2, ADC_IT_EOC);
    /** Initialization TIM interrupt */
    NVIC_Configure(TIM1_BRK_UP_TRG_COM_IRQn, TIM1_UPDATE_INTERRUPT);
    /** TIM Update interrupt disabled */
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
    /** TIM Break interrupt disabled */
    TIM_ClearFlag(TIM1, TIM_FLAG_Break);
    TIM_ITConfig(TIM1, TIM_FLAG_Break, DISABLE);
	
	  //Application
	  /** Initialization UART3 interrupt */
	//NVIC_SetPriority(UART3_IRQn, UART3_RX_INTERRUPT);
	//UART_ClearITPendingBit(UART1, 0x0FFF);
	//UART_ITConfig(UART1, UART_IT_RXIEN, ENABLE);//Enable interrupt
	
}


/*void SysTick_Handler(void)
{
    Clear_Over_Flag();
    Inc_Systicks();
	
	STK_pfGeneralTimerCallback();
	
}*/

void WWDG_IRQHandler(void) {}

void PVD_IRQHandler(void) {}

void PWM_IRQHandler(void) {}

void FLASH_IRQHandler(void) {}

void RCC_IRQHandler(void) {}

void EXTI0_1_IRQHandler(void) {}

void EXTI2_3_IRQHandler(void) {}

void EXTI4_15_IRQHandler(void) 
{
	if(EXTI_GetFlagStatus(EXTI_Line9) == SET)
		{
			//switch off the IGBT PWM output
			u16Test1 ++;
			EXTI_ClearFlag(EXTI_Line9);
		}

}

void HWDIV_IRQHandler(void) {}

void DMA1_Channel1_IRQHandler(void) {}

void DMA1_Channel2_3_IRQHandler(void) {}

void DMA1_Channel4_5_IRQHandler(void) {}

void ADC1_IRQHandler(void) {}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
    if(READ_TIM1_UPDATE_FLAG())
    {
        CLEAN_TIM1_UPDATE_FLAG();
    }
    if(READ_TIM1_BREAK_FLAG())
    {
        CLEAN_TIM1_BREAK_FLAG();
    }
}
void TIM1_CC_IRQHandler(void) {}

//void TIM2_IRQHandler(void) {}

void TIM3_IRQHandler(void) 
{
	    //Capture 1 Occurs Capture Event
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
    {
        MIT_tInputCapture.u16PeriodCount = TIM_GetCapture1(TIM3);
        MIT_tInputCapture.u16HighCount   = TIM_GetCapture2(TIM3);
    }

    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

			
}

void TIM8_BRK_UP_TRG_COM_IRQHandler(void) {}

void TIM8_CC_IRQHandler(void) {}

void TIM14_IRQHandler(void) {}
	
/**
  * @brief  This function handles ADC2 interrupt request.
  * @param  None
  * @retval None
  */

void ADC2_IRQHandler(void)
{
    static uint8_t u8ADCTimeCnt = 0;
    static uint8_t u8ADCTimeCnt2 = 0;
    static uint32_t u32IaSum = 0;
    static uint32_t u32IbSum = 0;
    static uint16_t u16Cnt = 0;
	/*if((ADC1->SREXT&ADC_SREXT_JEOSIF) == ADC_SREXT_JEOSIF)

		{
			MIT_TEST2 ++;
		}
	*/
    if(READ_ADC2_EOC_FLAG())
    {
        MIT_TEST ++;
        MIT_Start_time = FMC_u32GetSystickTimerCounter();
//#ifdef ADCTEST		
		STK_vTriggerSafeTime();
		
			/* Calculae the offset value of current*/
        if(u16Cnt <= 127)
        {
            u16Cnt++;
            u32IaSum += (int16_t)GET_ADC1_VALUE(IR_U_RANK);
            u32IbSum += (int16_t)GET_ADC2_VALUE(IR_V_RANK);
			CUR_vSetOffsetMeasState(CUR_OFFSET_MEAS_IS_ACTIVE);
        }
        else if(u16Cnt == 128)
        {
            u16Cnt++;
            u32IaSum = u32IaSum>>7;
            u32IbSum = u32IbSum>>7;
            Motor_1st.FOC.sIabc_offset.s16A = u32IbSum << 3;
            Motor_1st.FOC.sIabc_offset.s16B = u32IbSum << 3;
            u32IaSum = 0;
            u32IbSum = 0;
			CUR_vSetOffsetMeasState(CUR_OFFSET_MEAS_IS_ACTIVE);
        }
        else
        {
            
        	CUR_vSetOffsetMeasState(CUR_OFFSET_MEAS_NOT_ACTIVE);
            Get_ADC_Result(&Motor_1st);
            /* Fast Loop Statemachine */
            s_STATE_FAST[eM1_MainState]();
            
            if(++u8ADCTimeCnt >= Motor_1st.USER.u16SlowLoopDiv)   //For slow loop state machine,1ms
            {
                u8ADCTimeCnt = 0;
                Motor_1st.USER.bSlowLoopFlag = 1;
            }

            if(++u8ADCTimeCnt2 >= 10*Motor_1st.USER.u16SlowLoopDiv)   //10ms
            {
                u8ADCTimeCnt2 = 0;
                Motor_1st.USER.bSlowLoopFlag2 = 1;
                //MIT_vReadNormalADCValue();
            }
			
			//fs
			#if(FS_MOTOR_FS_MONITOR == OPTION_ACTIVE)
			
				FHE_vHandleTask();
			
			#endif
        }
		
		// Clear EOC Flag
        CLEAN_ADC2_EOC_FLAG();
        /* todo*/	
        MIT_Stop_time = FMC_u32GetSystickTimerCounter();
        if(MIT_Stop_time < MIT_Start_time)
        {
            MIT_Dure_Time = 65536 - MIT_Start_time;
            MIT_Dure_Time = MIT_Dure_Time + MIT_Stop_time;

        }
        else
        {
            MIT_Dure_Time = MIT_Stop_time - MIT_Start_time;
        }
				//#endif
    }

}

void TIM16_IRQHandler(void) {}

void TIM17_IRQHandler(void) {}

void I2C1_IRQHandler(void) {}

void COMP1_2_3_4_5_IRQHandler(void) {}

void SPI1_IRQHandler(void) {}

void SPI2_IRQHandler(void) {}

void UART1_IRQHandler(void) {}

void UART2_IRQHandler(void) 
{

	if(UART_GetITStatus(UART2, UART_IT_RXIEN) == SET)
	{
		UART_ClearITPendingBit(UART2, UART_IT_TXIEN);
		
        UART_pfReceivedCallback();
	}

}

void CSM_IRQn_IRQHan(void) {}

void UART3_IRQHandler(void) 
{
	
	if(UART_GetITStatus(UART3, UART_IT_RXIEN) == SET)
	{
		UART_ClearITPendingBit(UART3, UART_IT_TXIEN);
		
        UART_pfReceivedCallback();
	}
	
}
/**
  * @}
*/

void STK_vSetGenericTimerCallbackFunc(CallbackFunc2 pfCallbackfunction)
{
     STK_pfGeneralTimerCallback = pfCallbackfunction;
}


void UART_vSetUartReceiveCallbackFunc(CallbackFunc2 pfCallbackfunction)
{
     UART_pfReceivedCallback = pfCallbackfunction;
}


//for second uasrt. usart3
void STK_vSetGenericTimerCallbackFuncUART3(CallbackFunc2 pfCallbackfunction)
{
     STK_pfGeneralTimerCallbackUART3 = pfCallbackfunction;
}


void UART_vSetUartReceiveCallbackFuncUART3(CallbackFunc2 pfCallbackfunction)
{
     UART_pfReceivedCallbackUART3 = pfCallbackfunction;
}

TInputCapture * MIT_ptGetCaptureData(void)
{
	return &MIT_tInputCapture;
}


/**
  * @}
*/
