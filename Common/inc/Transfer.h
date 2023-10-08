#ifndef __TRANSFER_H
#define __TRANSFER_H

#include "Math.h"


extern void TRR_vClarkeTrans(TThreePhase* ptInput, TCartNum* ptOutput);

extern void TRR_vForwardTransformation(TCartNum* ptInput, TDQAxial* ptOutput, TAngle * pActualAngle);

extern void TRR_vBackwardTransformation(TDQAxial* ptInput, TCartNum* ptOutput, TAngle * pActualAngle);


#endif 