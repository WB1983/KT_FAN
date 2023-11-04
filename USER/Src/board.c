/**
 * @file     board.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides all the functions for the board level support package.
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
#define _BOARD_C_

/** Files includes */
#include "board.h"
#include "drv_inc.h"


/**
 * @addtogroup MM32_Hardware_Driver_Layer
 * @{
 */
void BOD_vExti9Config(void);
void BOD_vExti7Config(void);
/**
 * @addtogroup Bsp
 * @{
 */
 void BSP_vExti9Config(void);
 void BSP_vExti7Config(void);
 void BSP_vIdlePortInit(void);
 void BSP_vIICInit(void);
 void BSP_vUARTInit(void);


 /**
  * @brief OPAM  GPIO Initialization 
  * @param None
  * @retval None
  */
void Bsp_Op_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    /* GPIO Ports Clock Enable */
    RCC_AHBPeriphClockCmd(OPAMP_GPIO_CLK, ENABLE);

  	/*Configure GPIO pin : OPAM_Pin */
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;

    GPIO_InitStructure.GPIO_Pin     = OPAMP1_INM_PIN;
    GPIO_Init(OPAMP1_INM_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = OPAMP1_INP_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;

    GPIO_Init(OPAMP1_INP_PORT, &GPIO_InitStructure);

	/*
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;

    GPIO_InitStructure.GPIO_Pin     = OPAMP2_INM_PIN;
    GPIO_Init(OPAMP2_INM_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = OPAMP2_INP_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;

    GPIO_Init(OPAMP2_INP_PORT, &GPIO_InitStructure);
	*/
    GPIO_InitStructure.GPIO_Pin     = OPAMP3_INM_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;

    GPIO_Init(OPAMP3_INM_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = OPAMP3_INP_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;

    GPIO_Init(OPAMP3_INP_PORT, &GPIO_InitStructure);

}

/**
  * @brief ADC GPIO Initialization 
  * @param None
  * @retval None
  */
void Bsp_Adc_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    /* ADC Clock Enable */
    RCC_AHBPeriphClockCmd(ADC_GPIO_CLK, ENABLE);
    /*Configure ADC pin  */
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;

    GPIO_InitStructure.GPIO_Pin     = NTC_PIN;
    GPIO_Init(NTC_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = VBUS_PIN;
    GPIO_Init(VBUS_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = ISUM_PIN;
    GPIO_Init(ISUM_PORT, &GPIO_InitStructure);
#if THREE_SHUNT_R
    GPIO_InitStructure.GPIO_Pin     = IR_W_PIN;
    GPIO_Init(IR_W_PORT, &GPIO_InitStructure);
#endif
    GPIO_InitStructure.GPIO_Pin     = IR_V_PIN;
    GPIO_Init(IR_V_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = IR_U_PIN;
    GPIO_Init(IR_U_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = VOL_15_PIN;
    GPIO_Init(VOL_15_PORT, &GPIO_InitStructure);

	  GPIO_InitStructure.GPIO_Pin     = SO_C_PIN;
    GPIO_Init(SO_C_PORT, &GPIO_InitStructure);


	
}
void Bsp_Comp_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    /* COMP Clock Enable */
    RCC_AHBPeriphClockCmd((COMP_GPIO_CLK), ENABLE);
#if !USE_CRV//Question?? should Remove !
    GPIO_InitStructure.GPIO_Pin = COMP_INM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(COMP_INM_PORT, &GPIO_InitStructure);
#endif
	  /*Configure GPIO pin : COMP_Pin */
    GPIO_InitStructure.GPIO_Pin = COMP_INP_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(COMP_INP_PORT, &GPIO_InitStructure);
}

/**
  * @brief LED GPIO Initialization 
  * @param None
  * @retval None
  */

void Bsp_Led_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
	  /* LED Clock Enable */
    RCC_AHBPeriphClockCmd(LED_RCC_CLOCKGPIO, ENABLE);
    /*Configure GPIO pin : LED_Pin */
    /*GPIO_InitStructure.GPIO_Pin     =  LED1_PIN;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
    GPIO_Init(LED1_PORT, &GPIO_InitStructure);*/

    GPIO_InitStructure.GPIO_Pin     =  LED2_PIN;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
    GPIO_Init(LED2_PORT, &GPIO_InitStructure);
}


/**
  * @brief PWM GPIO Initialization 
  * @param None
  * @retval None
  */

void Bsp_Pwm_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    /* PWM Clock Enable */
    RCC_AHBPeriphClockCmd(BLDC1_GPIO_CLK, ENABLE);
    /*Configure GPIO pin : PWM_Pin */
    GPIO_InitStructure.GPIO_Pin     = BLDC1_UH_PIN;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF_PP;
    GPIO_Init(BLDC1_UH_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BLDC1_VH_PIN;
    GPIO_Init(BLDC1_VH_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BLDC1_WH_PIN;
    GPIO_Init(BLDC1_WH_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BLDC1_UL_PIN;
#if PWM_L_USE_IO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
#endif
    GPIO_Init(BLDC1_UL_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BLDC1_VL_PIN;
    GPIO_Init(BLDC1_VL_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BLDC1_WL_PIN;
    GPIO_Init(BLDC1_WL_PORT, &GPIO_InitStructure);
    /*selects the pin to used as Alternate function of PWM*/
    GPIO_PinAFConfig(BLDC1_UH_PORT, BLDC1_UH_PIN_SRC, BLDC1_UH_PIN_AF);
    GPIO_PinAFConfig(BLDC1_VH_PORT, BLDC1_VH_PIN_SRC, BLDC1_VH_PIN_AF);
    GPIO_PinAFConfig(BLDC1_WH_PORT, BLDC1_WH_PIN_SRC, BLDC1_WH_PIN_AF);

#if PWM_L_USE_TIM
    GPIO_PinAFConfig(BLDC1_UL_PORT, BLDC1_UL_PIN_SRC, BLDC1_UL_PIN_AF);
    GPIO_PinAFConfig(BLDC1_VL_PORT, BLDC1_VL_PIN_SRC, BLDC1_VL_PIN_AF);
    GPIO_PinAFConfig(BLDC1_WL_PORT, BLDC1_WL_PIN_SRC, BLDC1_WL_PIN_AF);
#endif
#if PWM_L_USE_IO
    GPIO_ResetBits(BLDC_UL_PORT, BLDC_UL_PIN);
    GPIO_ResetBits(BLDC_VL_PORT, BLDC_VL_PIN);
    GPIO_ResetBits(BLDC_WL_PORT, BLDC_WL_PIN);
#endif
}

void BSP_vIdlePortInit(void)
{
	//Initialize PORT A unused pin
	GPIO_InitTypeDef GPIO_InitStructure;


	//Initialize PORT B unused pin
	GPIO_StructInit(&GPIO_InitStructure);
	  /* LED Clock Enable */
    RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOB, ENABLE);
    /*Configure GPIO pin : LED_Pin */
    GPIO_InitStructure.GPIO_Pin     =  GPIO_Pin_14;	
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIOB->BRR = GPIO_InitStructure.GPIO_Pin;

	//Initialize PORT C unused pin
	GPIO_StructInit(&GPIO_InitStructure);
	  /* LED Clock Enable */
    RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOC, ENABLE);
    /*Configure GPIO pin : LED_Pin */
    GPIO_InitStructure.GPIO_Pin     =  GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;	
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIOC->BRR = GPIO_InitStructure.GPIO_Pin;

	//Initialize PORT D unused pin
	GPIO_StructInit(&GPIO_InitStructure);
	  /* LED Clock Enable */
    RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOD, ENABLE);
    /*Configure GPIO pin : LED_Pin */
    GPIO_InitStructure.GPIO_Pin     =  GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_7;	
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIOD->BRR = GPIO_InitStructure.GPIO_Pin;


}


void BSP_vIICInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOD, ENABLE);
    GPIO_StructInit(&GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin  =  GPIO_Pin_2|GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
}



/*
External interrupt port configuration

*/
void BSP_vExti9Config(void)
{
	//Initialize PORT A unused pin
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
	  /* LED Clock Enable */
    RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOB, ENABLE);
    /*Configure GPIO pin : LED_Pin */
    GPIO_InitStructure.GPIO_Pin     =  GPIO_Pin_9;	
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void BSP_vExti7Config(void)
{
	//Initialize PORT A unused pin
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
	  /* LED Clock Enable */
    RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOD, ENABLE);
    /*Configure GPIO pin : LED_Pin */
    GPIO_InitStructure.GPIO_Pin     =  GPIO_Pin_7;	
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IPU;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

}
////////////////////////////////////////////////////////////////////////////////
/// @brief  initialize exti interrupt Line
/// @note   if use jtag/swd interface GPIO PIN as KEY, need to be careful,
///         can not debug or program.
/// @param  None.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
static void EXTI9_NVIC_Config(void)
{
    EXTI_InitTypeDef EXTI_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2ENR_SYSCFG, ENABLE);

    //PB.9 use EXTI line 9
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource9);
    EXTI_InitStruct.EXTI_Line    = EXTI_Line9;
    EXTI_InitStruct.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStruct);

}

static void EXTI7_NVIC_Config(void)
{
    EXTI_InitTypeDef EXTI_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2ENR_SYSCFG, ENABLE);

    //PB.9 use EXTI line 7
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource7);
    EXTI_InitStruct.EXTI_Line    = EXTI_Line7;
    EXTI_InitStruct.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStruct);

}

////////////////////////////////////////////////////////////////////////////////
/// @brief  initialize exti nvic config
/// @note   if use jtag/swd interface GPIO PIN as KEY, need to be careful,
///         can not debug or program.
/// @param  None.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
static void EXTI_NVIC_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;

    NVIC_InitStruct.NVIC_IRQChannel = EXTI4_15_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}


void BSP_vUARTInit(void)
{
    //GPIO port set
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOA, ENABLE);  //

    //UART2_TX   GPIOA.2
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    //UART2_RX    GPIOA.3
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
/*
    //UART1_RX    GPIOC.7
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
*/    


}


void BSP_vUARTInitUART3(void)
{
    //GPIO port set
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOD, ENABLE);  //

    //UART2_TX   GPIOD.0
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    //UART2_RX    GPIOD.1
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

}



/**
 * @brief    : This function describes how to initialize I/O configurations related to the PWM output and input.
 * @param    : None
 * @retval   : None
 */
void Bsp_Capture_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);

    RCC_AHBPeriphClockCmd(TIM_CAPTURE_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin     = TIM3_CH1_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IPD;
    GPIO_Init(TIM3_CH1_PORT, &GPIO_InitStructure);

    GPIO_PinAFConfig(TIM3_CH1_PORT, TIM3_CH1_PIN_SRC, TIM3_CH1_PIN_AF);
}

/**
 * @brief    : This function describes how to initialize I/O configurations related to the PWM output and input.
 * @param    : None
 * @retval   : None
 */
void Bsp_PWM_Output_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);

    RCC_AHBPeriphClockCmd(TIM_OUT_PWM_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin     = TIM17_CH1_PIN;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF_PP;
    GPIO_Init(TIM17_CH1_PORT, &GPIO_InitStructure);

    GPIO_PinAFConfig(TIM17_CH1_PORT, TIM17_CH1_PIN_SRC, TIM17_CH1_PIN_AF);
}

void BSP_vDirectionDetect_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
	  /* LED Clock Enable */
    RCC_AHBPeriphClockCmd(DIR_RCC_CLOCKGPIO, ENABLE);


    GPIO_InitStructure.GPIO_Pin     =  DIRECTION_PIN;
    GPIO_InitStructure.GPIO_Speed   =  GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode    =  GPIO_Mode_IPD;
    GPIO_Init(DIRECTION_PORT, &GPIO_InitStructure);
}

/**
  * @brief Initialize all configured GPIO
  * @param None
  * @retval None
  */
void Bsp_Gpio_Init(void)
{
    Bsp_Led_Init();
    Bsp_Op_Init();
    Bsp_Adc_Init();
    Bsp_Comp_Init();
    Bsp_Pwm_Init();
	BSP_vExti9Config();
	BSP_vUARTInit();
	#if(SECOND_UART_USED == OPTION_ACTIVE)
	BSP_vUARTInitUART3();
	#endif
	//BSP_vIdlePortInit();
	//BSP_vIICInit();
	#if(SECOND_UART_USED == OPTION_PASSIVE)
	Bsp_Capture_Init();
	Bsp_PWM_Output_Init();
	#endif
    //BSP_vExti7Config();
	BSP_vDirectionDetect_Init();
}
/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
void Board_ADC_Init(void)
{
	  /*ADC1  RANK Array*/
    ADC_Channel_TypeDef sUserAdc1Channel[3];
	  /*ADC2  RANK Array*/
    ADC_Channel_TypeDef sUserAdc2Channel[4];
    /* Configure the ADC1 RANK Sequence*/
    sUserAdc1Channel[0].u8Rank = IR_U_RANK;
    sUserAdc1Channel[0].sAdcChannel = IR_U_CHANNEL;
    sUserAdc1Channel[0].pNext = &sUserAdc1Channel[1];
    sUserAdc1Channel[1].u8Rank = VBUS_RANK;
    sUserAdc1Channel[1].sAdcChannel = VBUS_CHANNEL;
    sUserAdc1Channel[1].pNext = &sUserAdc1Channel[2];
	sUserAdc1Channel[2].u8Rank = VF_RANK;
    sUserAdc1Channel[2].sAdcChannel = VL_15_CHANNEL;
    sUserAdc1Channel[2].pNext = NULL;
	/*sUserAdc1Channel[3].u8Rank = IV_RANK;
    sUserAdc1Channel[3].sAdcChannel = IN_VL_CHANNEL;
    sUserAdc1Channel[3].pNext = &sUserAdc1Channel[4];
	sUserAdc1Channel[4].u8Rank = IP_RANK;
    sUserAdc1Channel[4].sAdcChannel = IN_TP_CHANNEL;
    sUserAdc1Channel[4].pNext = NULL;*/

    /* Configure the ADC2 RANK Sequence*/
    sUserAdc2Channel[0].u8Rank = IR_V_RANK;
    sUserAdc2Channel[0].sAdcChannel = IR_V_CHANNEL;
    sUserAdc2Channel[0].pNext = &sUserAdc2Channel[1];
    sUserAdc2Channel[1].u8Rank = ISUM_RANK;
    sUserAdc2Channel[1].sAdcChannel = ISUM_CHANNEL;
    sUserAdc2Channel[1].pNext = &sUserAdc2Channel[2];
    sUserAdc2Channel[2].u8Rank = NTC_RANK;
    sUserAdc2Channel[2].sAdcChannel = NTC_CHANNEL;
    sUserAdc2Channel[2].pNext = &sUserAdc2Channel[3];
	 sUserAdc2Channel[3].u8Rank = SO_RANK;
    sUserAdc2Channel[3].sAdcChannel = SO_1R_CHANNEL;
    sUserAdc2Channel[3].pNext = NULL;
	

    /* Select the ADC external trigger source of the ADC is T1_CC4*/
    Drv_Adc_Basic_Init(ADC1, ADC1_ExternalTrigConv_T1_CC4);
    Drv_Adc_Basic_Init(ADC2, ADC2_ExternalTrigConv_T1_CC4);
    /* Select the ADC sample time*/
    Drv_Adc_Channel_Init(ADC1, sUserAdc1Channel, ADC_Samctl_3_5);
    Drv_Adc_Channel_Init(ADC2, sUserAdc2Channel, ADC_Samctl_3_5);

	/* Temperature and Vcc detection */
	Drv_Adc_Inject_Channel_Init();
	
    /*Enables the  ADC peripheral*/
    ADC_Cmd(ADC1, ENABLE);
    ADC_Cmd(ADC2, ENABLE);
}
/**
  * @brief COMP Initialization Function
  * @param None
  * @retval None
  */
void Board_Comp_Init(void)
{
    // COMP_POLL_InitTypeDef sPollInput;
    // sPollInput.COMP_Poll_Period = 4 << 4;
    // sPollInput.COMP_Poll_En = 1;//if just use one channel, need set to 1?
    // COMP_POLL_Init(COMP_NUMBER, &sPollInput);

    COMP_Input_TypeDef sUserCompInput;
	  /* Select the inverting input of the comparator */
    sUserCompInput.sCompInvertingInput = COMP_INVERTING;
	  /* Select the non inverting input of the comparator*/
    sUserCompInput.sCompNonInvertingInput = COMP_NON_INVERTING;
	  /* Select comparator external reference voltage */
    sUserCompInput.u8CompCrvSelect = COMP_CRV_VOLTAGE_SELECT;
	  /* Initializes the COMP according to the specified parameters in the COMP_Input_TypeDef */
    Drv_Comp_Init(COMP_NUMBER, &sUserCompInput);


}

void BOD_vI2CConfig(void)
{


}


/*
External interrupt port configuration

*/
void BOD_vExti9Config(void)
{

	EXTI9_NVIC_Config();
	EXTI_NVIC_Init();

}

void BOD_vExti7Config(void)
{

	EXTI7_NVIC_Config();
	EXTI_NVIC_Init();

}

/**
  * @brief Initialize all configured peripherals
  * @param None
  * @retval None
  */
void Peripheral_Init(void)
{
	
    Board_ADC_Init();
    Board_Comp_Init();
    Drv_Opamp_Init();
    Drv_Pwm_Init(TIM1, Motor_1st.USER.u16PWMPeriod, Motor_1st.USER.u16DeadTime);
    Drv_Iwdg_Init();
	  /*Initialize divider*/
    Drv_Hwdiv_Init();
	  /*Initialize sqrt*/
    Drv_Sqrt_Init();
	
   /*UART3 INIT*/
	UART_vCONSOLE_Init(9600);//configure uart
	#if(SECOND_UART_USED == OPTION_ACTIVE)
	UART_vCONSOLE_InitUART3(9600);//configure uart
	#endif
	BOD_vExti9Config();
	#if(SECOND_UART_USED == OPTION_PASSIVE)
	Drv_vCaptureInit();
	Drv_vPwmOutputInit();
	#endif
    //BOD_vExti7Config();
    
}

/**
  * @}
*/

/**
  * @}
*/
