#ifndef __USER_SUB_H
#define __USER_SUB_H
#include "main.h"

void variable_reset(Motor_TypeDef *Motor);
void Init_Parameter(Motor_TypeDef *Motor);
void Var_Filt(Motor_TypeDef *Motor);

void UFN_vBackwardParamStartupInit(void);
void UFN_vForwardParamStartupInit(void);



#endif
