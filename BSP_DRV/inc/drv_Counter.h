#ifndef __DRV_COUNTER_H
#define __DRV_COUNTER_H

#include <stdio.h>
#include "mm32_device.h"
#include "hal_conf.h"
#include "parameter.h"

#define DRV_COUNTER_FRQ              SYSCLK_HSI_96MHz
//#define DRV_COUNTER_BASE_FRQ         SYSCLK_HSI_96MHz/0x00FF

#define DRV_PWM_PRESCALER            SYSCLK_HSI_96MHz/DRV_COUNTER_FRQ -1
#define DRV_PWM_PERIOD               0xFFFF

extern void Drv_Tim16_Counter_Init(uint16_t u16Prescaler, uint16_t u16Period);

extern void Drv_Tim16_EnDisable(bool benable);

extern uint32_t Drv_u32GetTim16CountVal(void);


#endif