////////////////////////////////////////////////////////////////////////////////
/// @file    uart.c
/// @author  AE TEAM
/// @brief   THIS FILE PROVIDES ALL THE SYSTEM FUNCTIONS.
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

// Define to prevent recursive inclusion
#define _UART_C_

// Files includes
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

#include "mm32_device.h"
#include "drv_sci.h"
#include "hal_conf.h"




////////////////////////////////////////////////////////////////////////////////
/// @addtogroup MM32_Hardware_Abstract_Layer
/// @{

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup UART
/// @{

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup UART_Exported_Functions
/// @{

#ifdef __GNUC__

#define PUTCHAR_PROTOTYPE s32 __io_putchar(s32 ch)
#else
//#define PUTCHAR_PROTOTYPE s32 fputc(s32 ch, FILE *f)

#endif

#ifdef USE_IAR
PUTCHAR_PROTOTYPE {
    while((UART1->CSR & UART_IT_TXIEN) == 0); //The loop is sent until it is finished
    UART1->TDR = (ch & (u16)0x00FF);
    return ch;
}

#else
void _sys_exit(s32 x)
{
    x = x;
}
//redefine fputcfunction
//s32 fputc(s32 ch, FILE* f)
//{
//    while((UART1->CSR & UART_IT_TXIEN) == 0); //The loop is sent until it is finished
 //   UART1->TDR = (ch & (u16)0x00FF);
 //   return ch;
//}

#endif

static void UART_vNVICConfigure(u8 ch, u8 pri, u8 sub)
{
    exNVIC_Init_TypeDef  NVIC_InitStruct;


    NVIC_InitStruct.NVIC_IRQChannel = ch;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = pri;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = sub;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

    exNVIC_Init(&NVIC_InitStruct);
}

void UART_vCONSOLE_Init(u32 baudrate)
{

    UART_InitTypeDef UART_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1ENR_UART2, ENABLE);   //enableUART1,GPIOAclock

    //UART initialset
    /*GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_0);*///rx
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);//rx
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);//tx

    UART_ClearITPendingBit(UART2, 0x0FFF);

    UART_StructInit(&UART_InitStruct);
    UART_InitStruct.UART_BaudRate = baudrate;
    UART_InitStruct.UART_WordLength = UART_WordLength_8b;
    UART_InitStruct.UART_StopBits = UART_StopBits_1;//one stopbit
    UART_InitStruct.UART_Parity = UART_Parity_No;//none odd-even  verify bit
    UART_InitStruct.UART_HardwareFlowControl = UART_HardwareFlowControl_None;//No hardware flow control
    UART_InitStruct.UART_Mode = UART_Mode_Rx; // receive and sent  mode

    UART_Init(UART2, &UART_InitStruct); //initial uart 1

    UART_vNVICConfigure(UART2_IRQn, 0, 2);

	UART_ITConfig(UART2, UART_IT_RXIEN, ENABLE);//Enable interrupt
		
    UART_Cmd(UART2, ENABLE);                    //enable uart 1

}


/**
 * @name: STK_vSetGenericTimerCallbackFunc
 * @description: set callback function for STK_pfGeneralTimerCallback
 * @parm: CallbackFunc2  pfCallbackfunction
 * @return {vlid}
 */


void UART_vWriteData(uint8_t u8Data)
{
    
    //Enable TX
		UART2->GCR = UART2->GCR|UART_GCR_TX;
    //UART_ITConfig(UART1, UART_IT_TXIEN, ENABLE);//Enable interrupt  
    UART_SendData(UART2, u8Data);


    while((RESET == UART_GetITStatus(UART2, UART_IT_TXIEN))/*||(UART_GetFlagStatus(UART1, UART_FLAG_TXEMPTY) == RESET)*/)
    {

    }
    UART_ClearITPendingBit(UART2, UART_IT_TXIEN);
		//UART_SetTransmitEnable(UART1, DISABLE);
}

uint8_t UART_vGetUARTData(void)
{
	uint8_t u8ReturnValue = 0;
	u8ReturnValue = UART_ReceiveData(UART2);
	return u8ReturnValue;

}

/**
 * description modbus receiving control
 * 
 * parameter:void
 * 
 * return: void
 * 
 * 
 * */
void UART_vModbusRxEnable(void)
{
	UART2->GCR = UART2->GCR|UART_GCR_RX;

}

/**
 * description modbus receiving control
 * 
 * parameter:void
 * 
 * return: void
 * 
 * 
 * */
void UART_vModbusRxDisable(void)
{
	UART2->GCR = UART2->GCR&(~UART_GCR_RX);

}

/**
 * description modbus receiving control
 * 
 * parameter:void
 * 
 * return: void
 * 
 * 
 * */
void UART_vModbusTxEnable(void)
{

	UART2->GCR = UART2->GCR|UART_GCR_TX;

}

/**
 * description modbus receiving control
 * 
 * parameter:void
 * 
 * return: void
 * 
 * 
 * */
void UART_vModbusTxDisable(void)
{

	UART2->GCR = UART2->GCR&(~UART_GCR_TX);

}

/**
input parameter:void

return value: true, not empty; false, empty

**/
FlagStatus UART_bUARTTXBUFEMPT(void)
{
  FlagStatus eReturnValue = RESET;
	eReturnValue = UART_GetFlagStatus(UART2, UART_FLAG_TXEPT);
	return eReturnValue;
}


/****************************************VOFA*************************************************/
void UART_vVOFAUARTInit(u32 baudrate)
{

    UART_InitTypeDef UART_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1ENR_UART2, ENABLE);   //enableUART1,GPIOAclock

    //UART initialset
    /*GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_0);*///rx
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);//rx
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);//tx

    UART_ClearITPendingBit(UART2, 0x0FFF);

    UART_StructInit(&UART_InitStruct);
    UART_InitStruct.UART_BaudRate = baudrate;
    UART_InitStruct.UART_WordLength = UART_WordLength_8b;
    UART_InitStruct.UART_StopBits = UART_StopBits_1;//one stopbit
    UART_InitStruct.UART_Parity = UART_Parity_No;//none odd-even  verify bit
    UART_InitStruct.UART_HardwareFlowControl = UART_HardwareFlowControl_None;//No hardware flow control
    UART_InitStruct.UART_Mode = UART_Mode_Rx|UART_Mode_Tx; // receive and sent  mode

    UART_Init(UART2, &UART_InitStruct); //initial uart 1

    UART_vNVICConfigure(UART2_IRQn, 0, 2);

	UART_ITConfig(UART2, UART_IT_RXIEN, ENABLE);//Enable interrupt
		
    UART_Cmd(UART2, ENABLE);                    //enable uart 1

}





/// @}

