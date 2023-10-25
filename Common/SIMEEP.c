/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-03-13 23:23:15
 * @LastEditors: Wangbingbin bing.wang2@bshg.com
 * @LastEditTime: 2023-03-15 23:42:07
 * @FilePath: \Pressure_221206_16F18346.Xc:\Users\WangBingBin\AppData\Local\WangBing\Projects\MCU_MindMotion\Project\Meter_Keil\Meter_Keil\Application\Common\SIMEEP.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "drv_eep.h"


void SMP_vWriteWord(uint8_t * pu8Buffer)	
{
	__disable_irq();
	EEPROM_Write(pu8Buffer,2);
	__enable_irq();
}

void SMP_vReadWord(uint8_t * pu8Buffer)	
{
	__disable_irq();
	EEPROM_Read(pu8Buffer,2);
	__enable_irq();
}

//u8Length must be divided by 0x800.
void SMP_vWriteWordN(uint8_t * pu8Buffer, uint8_t u8Length)	
{
	__disable_irq();
	EEPROM_Write(pu8Buffer,u8Length);
	__enable_irq();
}

//u8Length must be divided by 0x800.
void SMP_vReadWordN(uint8_t * pu8Buffer, uint8_t u8Length)	
{
	__disable_irq();
	EEPROM_Read(pu8Buffer,u8Length);
	__enable_irq();
}