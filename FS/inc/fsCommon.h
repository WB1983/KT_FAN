#ifndef __FS_COMMON_H
#define __FS_COMMON_H

#include "drv_inc.h"
#include "parameter.h"
#include "ParamMot.h"

#include "ParamMot.h"
#include "FOC_PMSM.h"
#include "Compile.h"

#include "main.h"
#include "Current.h"
#include "Voltage.h"
#include "SpeedMonitor.h"
#include "Filter.h"
#include "Transfer.h"

#define FCN_MONITOR_PERIOD             3//every 3 samples, implement one item monitor.

extern void FCN_vGetPhaseVoltage(void);

extern void FCN_vGetThreePhaseVol(TThreePhaseVolt* ptThreePha);

#endif