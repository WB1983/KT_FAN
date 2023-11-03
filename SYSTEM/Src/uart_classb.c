////////////////////////////////////////////////////////////////////////////////
/// @file    uart.c
/// @author  NJ ClassB TEAM
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
#define _UART_CLASSB_C_

// Files includes
#include "IEC60730_B_param.h"

#include "uart_classb.h"

////////////////////////////////////////////////////////////////////////////////
/// @brief  Retargets the C library printf function to the UARTx.
/// @param  ch: character to send
/// @param  f: pointer to file (not used)
/// @retval The character transmitted
////////////////////////////////////////////////////////////////////////////////
//int fputc(int ch, FILE *f)
//{
//    while((UART1->CSR & UART_IT_TXIEN) == 0);
//    UART1->TDR = (ch & (uint16_t)0x00FF);
//    return ch;
//}
void Uart_init(u32 bound)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2ENR_UART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOA, ENABLE);



    UART_InitStructure.UART_BaudRate = bound;
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;
    UART_InitStructure.UART_StopBits = UART_StopBits_1;
    UART_InitStructure.UART_Parity = UART_Parity_No;
    UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
    UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;

    UART_Init(UART1, &UART_InitStructure);
    UART_Cmd(UART1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}
#if defined (ClassB_Uart)
#if defined(__CC_ARM)
int fputc(int ch, FILE* f)
{
    // Place your implementation of fputc here
    // e.g. write a character to the COM and Loop until the end of transmission
    while(1)
    {
        if(UART_GetFlagStatus(ClassB_Uart, UART_CSR_TXC) != RESET)
        {
            break;
        }
    }
    UART_SendData(ClassB_Uart, (u16)ch);
    return ch;
}
#elif defined(__ICCARM__) && (__VER__ < 9030001)
int fputc(int ch, FILE* f)
{
    // Place your implementation of fputc here
    // e.g. write a character to the COM and Loop until the end of transmission
    while(1)
    {
        if(UART_GetFlagStatus(ClassB_Uart, UART_CSR_TXC) != RESET)
        {
            break;
        }
    }
    UART_SendData(ClassB_Uart, (u16)ch);
    return ch;
}
#elif defined(__ICCARM__) && (__VER__ >= 9030001)
size_t __write(int handle, const unsigned char * buffer, size_t size)
{

    while(1)
    {
        if(UART_GetFlagStatus(ClassB_Uart, UART_CSR_TXEPT) != RESET)
        {
            break;
        }
    }

    if (buffer == 0)
    {
        return 0;
    }
    UART_SendData(ClassB_Uart, *buffer);
    return 1;
}

#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
int fputc(int ch, FILE* f)
{
    // Place your implementation of fputc here
    // e.g. write a character to the COM and Loop until the end of transmission
    while(1)
    {
        if(UART_GetFlagStatus(ClassB_Uart, UART_CSR_TXC) != RESET)
        {
            break;
        }
    }
    UART_SendData(ClassB_Uart, (u16)ch);
    return ch;
}
#elif defined(__GNUC__)
void _sys_exit(s32 x)
{
    x = x;
}
// With GCC, small printf (option LD Linker->Libraries->Small printf
// set to 'Yes') calls __io_putchar()
int __io_putchar(int ch)
{
    // Place your implementation of fputc here
    // e.g. write a character to the COM and Loop until the end of transmission
    while(1)
    {
        if(UART_GetFlagStatus(ClassB_Uart, UART_CSR_TXC) != RESET)
        {
            break;
        }
    }
    UART_SendData(ClassB_Uart, (u16)ch);
    return ch;
}
#else
#warning Not supported compiler type
#endif




////////////////////////////////////////////////////////////////////////////////
/// @brief  CONSOLE initialize for the C library printf function with UART for Class B
/// @param  baudrate: baud rate of the UART
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void CBUart_Init(u32 baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    UART_InitTypeDef UART_InitStruct;

    RCC_UART_ClockCmd(ClassB_Uart, ENABLE);                                     //enable UART2 clock
    RCC_GPIO_ClockCmd(GPIOA, ENABLE);                                           //enable GPIOA clock

    //UART initial
    //UART_StructInit(&UART_InitStruct);
    UART_InitStruct.UART_BaudRate = baudrate;
    UART_InitStruct.UART_WordLength = UART_WordLength_8b;
    UART_InitStruct.UART_StopBits = UART_StopBits_1;                            //one stopbit
    UART_InitStruct.UART_Parity = UART_Parity_No;                               //none odd-even  verify bit
    UART_InitStruct.UART_HardwareFlowControl = UART_HardwareFlowControl_None;   //No hardware flow control
    UART_InitStruct.UART_Mode = UART_Mode_Rx ;                    // receive and sent  mode

    UART_Init(ClassB_Uart, &UART_InitStruct);                                   //initial uart 1
    UART_Cmd(ClassB_Uart, ENABLE);

    //GPIO initial
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);                        //PA2:(AF1,UART1_TX)
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);                        //PA3:(AF1,UART1_RX)

    //UART1_TX   GPIOA.2
    //GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);


    //UART1_RX    GPIOA.3
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    //enable uart 1
}
#endif



