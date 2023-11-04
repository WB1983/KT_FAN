#ifndef __DRV_OUTPUT_H
#define __DRV_OUTPUT_H

#include <stdio.h>
#include "mm32_device.h"
#include "hal_conf.h"
#include "parameter.h"

#define DRV_PWM_OUTPUT_FRQ              1000
#define DRV_PWM_BASE_FRQ                1000000
                                       

#define DRV_PWM_OUTPUT_PRESCALER        SYSCLK_HSI_96MHz/DRV_PWM_BASE_FRQ -1
#define DRV_PWM_OUTPUT_PERIOD           DRV_PWM_BASE_FRQ/DRV_PWM_OUTPUT_FRQ -1

#define DRV_PWM_PERIOD_OUT_LO           DRV_PWM_BASE_FRQ/DRV_PWM_BASE_FRQ -1

extern void Drv_vPwmOutputInit(void);

extern void Drv_vSetOutputFreq(uint16_t u16Frequency);

extern void Drv_vEnablePWMOutput(void);
#endif