#ifndef __RAMP_H
#define __RAMP_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

#include "mlib.h"

/***************************const value(int16)*****************************/
//improve startup
#define Q11_MAX 				      2047
#define Q11_MIN  				      (-2048)
#define Q11_HALF_MAX 			      1024

#define QMATH_SINCOS_90DEGREE		  512
#define QMATH_SINCOS_180DEGREE		  1024
#define QMATH_SINCOS_270DEGREE		  1536
#define QMATH_SINCOS_360DEGREE		  2048

#define RMP_SPD_LOOP_FREQ             100//100Hz, one interrrupt every 10ms.

typedef enum _TDirection
{
	FORWARD,
	BACKWARD
}TDirection;

/**
    \brief Type definition of the modules data object type
*/
typedef struct _TRmpData {
		int32_t					tFinalRampValue;				/**< [fp] Input Interface: Final target speed value								*/
		int32_t                 tPreFinalRampValue;
		int32_t					tStepRefRampValue;				    /**< [fp] Actual target speed ramp output value								*/
		int32_t 				tBaseRampValue;
		int32_t                 tDeltaRampValue;
		int16_t                 tCurveAngle;
		int16_t                 tFactor;
		TDirection			    tDirection;						/**< Direction of rotation*/
		uint16_t                u16CurrentGradient;
		uint16_t                u16SpeedStartupGradient;
		uint16_t                u16SpeedRunGradient; 														
}TRmpData;

extern void RMP_vCalTrapezoidalCurve(uint8_t * u8Gradient, int16_t * s16CurrentValue, volatile int16_t * s16TargetValue);

extern void RMP_vCalcSmootherSinusoidalSpeedRampCurve(uint8_t * pu8Gradient, int16_t * ps16CurrentRefValue, volatile int16_t * ps16TargetValue);

extern void RMP_vUpdateSwitchSpeed(void);

extern void RMP_vConfigureStartupAlgorithm(TDirection tDirection);

extern TRmpData * RMP_ptGetRampDataPointer(void);

#endif

