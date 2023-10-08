#include <stdio.h>
#include "mm32_device.h"

#include "libtypes.h"

#include "libDefines.h"
#include "mapping.h"

#include "Timer2.h"

#include "Vofa.h"

#include <string.h>

#define JUST_FLOAT_TEST 1

Vofa_HandleTypedef vofa1;
static const uint8_t cmd_open[] = {0xAF,0xFA};
static const uint8_t cmd_close[] = {0xFA,0xAF};
static uint8_t cmdBuffer[10] = {0};	
bool Vofa_bDataFinished = FALSE;
bool Vofa_bTimerActionFinish = FALSE;

extern void Vofa_Test();
extern bool memCmp(uint8_t* a,uint8_t* b,uint32_t count);
void COM_vHandleTask(void)
{
		
		Vofa_Test();
}

void Vofa_Test()
{
#if JUST_FLOAT_TEST == 1
	if(Vofa_bDataFinished == TRUE)
	{
		if(Vofa_ReadCmd(&vofa1,cmdBuffer,10))
		{
			if(memCmp(cmdBuffer,(uint8_t*)cmd_open,sizeof(cmd_open)))
			{
				TIM2_CR1_CEN = 1;
			}
			
			if(memCmp(cmdBuffer,(uint8_t*)cmd_close,sizeof(cmd_close)))
			{
				TIM2_CR1_CEN = 0;
			}
			
			memset(cmdBuffer,0,10);
		}
		Vofa_bDataFinished = FALSE;
	}
#endif		

#if FIRE_WATER_TEST == 1
	if(Vofa_ReadLine(&vofa1,cmdBuffer,10))
	{			
		Vofa_Printf(&vofa1,"data:%d,%d\r\n",cmdBuffer[0],cmdBuffer[1]);
		memset(cmdBuffer,0,10);
	}
#endif
		
#if ROW_DATA_TEST == 1
	if(Vofa_ReadData(&vofa1,cmdBuffer,10))
	{
		Vofa_SendData(&vofa1,cmdBuffer,10);
		memset(cmdBuffer,0,10);
	}
#endif
}
/*
static void USART1_Init()
{
	usart_baudrate_set(USART1,2000000);
	usart_stop_bit_set(USART1, USART_STB_1BIT);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_parity_config(USART1, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART1, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART1, USART_CTS_DISABLE);
	
	usart_transmit_config(USART1,USART_TRANSMIT_ENABLE);
	usart_receive_config(USART1,USART_RECEIVE_ENABLE);
	usart_enable(USART1);
	usart_interrupt_enable(USART1, USART_INT_RBNE);	
}

static void TIMER0_Init()
{
	timer_parameter_struct params = {0};
	params.period = 300;
	params.prescaler = SystemCoreClock/1000000;
	timer_init(TIMER0,&params);
	timer_interrupt_enable(TIMER0,TIMER_INT_UP);	
}

static void GPIO_Config()
{
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
}

static void RCU_Config()
{
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_USART1);
	rcu_periph_clock_enable(RCU_TIMER0);
}

static void NVIC_Config()
{
	nvic_irq_enable(USART1_IRQn, 0, 0);
	nvic_irq_enable(TIMER0_UP_IRQn,1,1);
}
*/
bool memCmp(uint8_t* a,uint8_t* b,uint32_t count)
{
	volatile uint32_t i = 0;
	while(a[i] == b[i] && i < count)
	{
		i++;
	}
	return i==count;
}


void Vofa_SendDataCallBack(Vofa_HandleTypedef* handle,uint8_t* data,uint16_t length)
{
	uint16_t i;
	for(i = 0;i<length;i++)
	{

		USART1_TDR = data[i];
		while(USART1_ISR_TXE == 0);
	}	
}

uint8_t Vofa_GetDataCallBack(Vofa_HandleTypedef* handle)
{
	return USART1_RDR;
}


void COM_vInit(void)
{
	UART1_vInit();

	TIM2_vInit();

	Vofa_Init(&vofa1,VOFA_MODE_SKIP);

}

