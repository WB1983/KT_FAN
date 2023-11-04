
////////////////////////////////////////////////////////////////////////////////
/// @file    Interrupts.c
/// @author  NJ ClassB TEAM
/// @brief
///
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

/* Includes ------------------------------------------------------------------------------------------------*/
#include "digitalio_classb.h"
#include "main.h"
#include "led_classb.h"
#include "key_classb.h"
/* Settings ------------------------------------------------------------------------------------------------*/
/* Private types -------------------------------------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------------------------------------*/
/* Global variables ----------------------------------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------------------------------------*/
/* Global functions ----------------------------------------------------------------------------------------*/
/*********************************************************************************************************//**
  * @brief  Compare two independent input values by the GPIO line connection.
  * @param  GPIOx_IN: where GPIOx_IN is the selected input port from the GPIO peripherals.
  * @param  GPIO_PIN_IN: The port pins.
  *   This parameter can be any combination of GPIO_PIN_x.
  * @param  GPIOx_CMP: where GPIOx_CMP is the selected input comparison from the GPIO peripherals.
  * @param  GPIO_PIN_CMP: The port pins.
  *   This parameter can be any combination of GPIO_PIN_x.
  * @retval SUCCESS or ERROR
  ***********************************************************************************************************/
ErrorStatus IEC60730_GPIO_InputCheck(GPIO_TypeDef* GPIOx_IN, u16 GPIO_PIN_IN,
                                     GPIO_TypeDef* GPIOx_CMP, u16 GPIO_PIN_CMP)
{
    bool InStatus, CmpStatus;

    InStatus = GPIO_ReadInputDataBit(GPIOx_IN, GPIO_PIN_IN);
    CmpStatus = GPIO_ReadInputDataBit(GPIOx_CMP, GPIO_PIN_CMP);
    if (InStatus == CmpStatus)
    {
        return SUCCESS;
    }
    else
    {
        return ERROR;
    }
}

/*********************************************************************************************************//**
 * @brief Check the expected value on input pin.
 * @param GPIOx_IN: where GPIOx is the selected GPIO from the GPIO peripherals.
 * @param GPIO_PIN_IN: Specify the port bits.
 *   This parameter can be any combination of GPIO_PIN_x.
 * @param Status: This parameter can be SET or RESET.
 * @retval SUCCESS or ERROR
 ************************************************************************************************************/
ErrorStatus IEC60730_GPIO_InputValueCheck(GPIO_TypeDef* GPIOx_IN, u16 GPIO_PIN_IN, BitAction Status)
{
    bool InStatus;

    InStatus = GPIO_ReadInputDataBit(GPIOx_IN, GPIO_PIN_IN);
    if (InStatus == Status)
    {
        return SUCCESS;
    }
    else
    {
        return ERROR;
    }
}



/*********************************************************************************************************//**
  * @brief  Check the expected value on output pin by reading the input value.
  * @param  GPIOx_OUT: where GPIOx_OUT is the selected output port from the GPIO peripherals.
  * @param  GPIO_PIN_OUT: The port pins.
  *   This parameter can be any combination of GPIO_PIN_x.
  * @retval SUCCESS or ERROR
  ***********************************************************************************************************/
ErrorStatus IEC60730_GPIO_OutputCheck(GPIO_TypeDef* GPIOx_OUT, u16 GPIO_PIN_OUT)
{
    bool OutStatus, CmpStatus;

    OutStatus = GPIO_ReadOutputDataBit(GPIOx_OUT, GPIO_PIN_OUT);
    CmpStatus = GPIO_ReadInputDataBit(GPIOx_OUT, GPIO_PIN_OUT);

    if (OutStatus == CmpStatus)
    {
        return SUCCESS;
    }
    else
    {
        return ERROR;
    }
}

/*********************************************************************************************************//**
 * @brief Set or Clear the selected port bits of data and compare expected input value.
 * @param GPIOx_OUT: where GPIOx is the selected GPIO from the GPIO peripherals.
 * @param GPIO_PIN_OUT: Specify the port bits.
 *   This parameter can be any combination of GPIO_PIN_x.
 * @param Status: This parameter can be SET or RESET.
 * @retval SUCCESS or ERROR
 ************************************************************************************************************/
ErrorStatus IEC60730_GPIO_WriteOutBits(GPIO_TypeDef* GPIOx_OUT, u16 GPIO_PIN_OUT, BitAction Status)
{
    bool CmpStatus;

    GPIO_WriteBit(GPIOx_OUT, GPIO_PIN_OUT, Status);
    CmpStatus = GPIO_ReadInputDataBit(GPIOx_OUT, GPIO_PIN_OUT);
    if (CmpStatus == Status)
    {
        return SUCCESS;
    }
    else
    {
        return ERROR;
    }
}

void IEC60730_DIOCheck(void)
{
    if(IEC60730_GPIO_WriteOutBits(LED1_Port,LED1_Pin,Bit_RESET))
    {
        printf("Dio  Test1  OK ....\r\n");
    }
    if(IEC60730_GPIO_OutputCheck(LED1_Port,LED1_Pin))
    {
        printf("Dio  Test2  OK ....\r\n");
    }
    if(IEC60730_GPIO_InputValueCheck(KEY1_GPIO_Port,KEY1_Pin,Bit_RESET))
    {
        printf("Dio  Test3  OK ....\r\n");
    }
    if(IEC60730_GPIO_InputCheck(KEY2_GPIO_Port,KEY2_Pin,KEY2_GPIO_Port,KEY3_Pin))
    {
        printf("Dio  Test4  OK ....\r\n");
    }
}
