#ifndef __SPEED_CONTROL_H
#define __SPEED_CONTROL_H

#include "ramp.h"

extern TDirection SCL_bGetMotorDirection(void);

extern void SCL_vMotorDirectionCheck(void);

extern void SCL_vGetLowCount(void);

extern void SCL_vGetPeriodCount(void);

extern void SCL_vSpeedControlHandle(void);

#endif
