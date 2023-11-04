#ifndef __LED_H
#define __LED_H	 
#include "HAL_conf.h"

//////////////////////////////////////////////////////////////////////////////////	 
//开发板
//LED驱动代码	   
////////////////////////////////////////////////////////////////////////////////// 
#define   LED_VLM   GPIO_Pin_3
#define   LED_NVM   GPIO_Pin_4
#define   LED_ERR   GPIO_Pin_5
#define  BSP_LED_On(gpio_pin)    GPIO_SetBits(GPIOB, gpio_pin);
#define  BSP_LED_Off(gpio_pin)   GPIO_ResetBits(GPIOB, gpio_pin);
#define  BSP_LED_Toggle(gpio_pin)  {GPIOB->ODR^gpio_pin;}

void LED_Init(void);//初始化

		 				    
#endif
