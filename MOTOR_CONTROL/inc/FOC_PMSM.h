#ifndef __FOC_PMSM_H
#define __FOC_PMSM_H
#include "main.h"


extern void Get_ADC_Result(Motor_TypeDef *Motor);
extern void DcBus_Ripple_suppress(MCFLIB_2_ALBE_T_S16 *sVAlBeIn,Q15_t s16Vbus, MCFLIB_2_ALBE_T_S16 *sVAlBe_Out);

void FOC_Current_Controller(Motor_TypeDef *Motor);
void PMSM_Obs(Motor_TypeDef *Motor);

extern uint16_t MOT_u16GetVariantResistorDigitalValue(void);

extern uint16_t * MOT_puGetOtherADCData(void);

#endif
