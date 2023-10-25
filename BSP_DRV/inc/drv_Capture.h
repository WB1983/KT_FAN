#ifndef __DRV_CAPTURE_H
#define __DRV_CAPTURE_H

#define DRV_CAP_PWM_FRQ       		 1000
#define DRV_CAP_PWM_BASE_FRQ         48000000

#define DRV_CAP_PWM_PRESCALER        SYSCLK_HSI_96MHz/DRV_CAP_PWM_BASE_FRQ -1
#define DRV_CAP_PWM_PERIOD           DRV_CAP_PWM_BASE_FRQ/DRV_CAP_PWM_FRQ -1

extern void Drv_vCaptureInit(void);

#endif

