#ifndef __WINDDETECTION_H
#define __WINDDETECTION_H
#include "drv_inc.h"
#include "StateMachine.h"
#include "parameter.h"
#include "FOC_PMSM.h"
#include "Compile.h"
#include "Ramp.h"


#if (WDN_WIND_DETECTION_METHOD_1 == OPTION_ACTIVE)
extern bool WDN_bEstimateStartupScenario(void);
extern void WDN_vAnalyzeStartPower(void);

#elif (WDN_WIND_DETECTION_METHOD_2 == OPTION_ACTIVE)
extern void WDN_vInitWindDetection(void);
extern void WDN_vWindDetection(void);
extern void WDN_vCalculateBemfPeak(void);
extern void WDN_AlignInit(void);

#endif

extern void WDN_vConfigureStartupParam(float fAlignCur, float fStartupCur, float fAlignTime, float fStartupTime);

#endif
