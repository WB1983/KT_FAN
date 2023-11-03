#ifndef __COMMON_H
#define __COMMON_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

#include "Compile.h"		/* Include global compiler switches */
#include "Math.h"			/* Include fixed point math library */

/*****************************************************************************************************************
 * GLOBAL TYPES **************************************************************************************************
 *****************************************************************************************************************/
#define HIGH_BYTE(v)       (*((unsigned char*)&v))             /* Highbyte eines Integer bestimmen */
#define LOW_BYTE(v)        (*(((unsigned char*)&v)+1))         /* Lowbyte  eines Integer bestimmen */
#define HIGH_BYTE_LONG(v)  (*(((unsigned int*)&v)))
#define LOW_BYTE_LONG(v)   (*(((unsigned int*)&v)+2))

#define SET(VAR1,VAR2)  ((VAR1)|=(VAR2)) 
#define RES(VAR1,VAR2)  ((VAR1)&=~(VAR2))
#define TEST(VAR1,VAR2) ((VAR1)&(VAR2))

#define QMATH_ABS(A) 		((A < 0)?(-A):(A))

typedef void (* CallbackFunc1)(uint16_t u16Parameter);
typedef void (* CallbackFunc2)(void);
typedef void (* CallbackFunc3)(bool bParameter);

/** \brief PHASE CURRENT MEASUREMENT CHANNEL                                 
           - HW: Shunts and current amplifiers connected to ADC input pins   
           - SW: CUR Current module                                        */

typedef enum _TPhaCurMeasChannel {/**< Current measurement channels        */
  CUR_MEAS_PHASE_A,               /**< Current measurement channel phase A */
  CUR_MEAS_PHASE_B,               /**< Current measurement channel phase B */
  CUR_MEAS_PHASE_C                /**< Current measurement channel phase C */
}TPhaCurMeasChannel;

/** \brief BRIDGE CURRENT OFFSET MEASUREMENT SATAE                           
           - HW: Overall bridge measurement device                           
           - SW: CUR Current module                                        */

typedef enum _TBrdCurOffsetState {/**< Bridge offset measurement state     */
  CUR_OFFSET_MEAS_NOT_ACTIVE,     /**< Offset measurment not active */
  CUR_OFFSET_MEAS_IS_ACTIVE       /**< Offset measurment     active */
}TBrdCurOffsetState;
  

/**
 * \brief	Configuration structure of the output duty cycle for all channels
 *
 * 			The duty cycle is defined from 0% ... 100% => 0 ... 32768 where 0% means that
 * 			the low side power switch is on all the time and 100% means the high side switch
 * 			is on all the time.
 * */
typedef struct _TDutyCycleConfig {
    uint16_t uiDutyCyclePhaseU;      /**< Output duty cycle phase U */
    uint16_t uiDutyCyclePhaseV;      /**< Output duty cycle phase V */
    uint16_t uiDutyCyclePhaseW;      /**< Output duty cycle phase W */
}TDutyCycleConfig;

typedef struct _TCurrents {								/**< \brief All current values											*/
		TThreePhase		tIThree;						/**<  Orthogonal currents ia,ib,ic										*/
		TCartNum		tICart;							/**<  Cartesian  currents ialfa,ibeta                                   */
		TDQAxial        tIDQ;									
}TCurrents;

/** \brief    Parameter structure type definition for this unit.
 *
 *  \details  Includes all necessary parameters.
 */
typedef struct _TOverThresholdParm {
	int16_t		s16ThresholdValue;
	uint16_t	u16DelayCnt;
	int16_t		s16ConstBackThresholdValue;
} TOverThresholdParm;

/** \brief		Data structure type definition for this unit.
 *
 *  \details	Includes all necessary data.
 */
typedef struct _TOverThresholdData {
	int16_t		s16Input;
	int16_t		s16MaxValue;
	uint16_t	u16Count;
	bool		bStatus;
} TOverThresholdData;

/** \brief Three phase current and voltage system */
typedef TThreePhase TThreePhaseCur;
typedef TThreePhase TThreePhaseVolt;

/** \brief INTERFACE OVERCURRENT CALLBACK FUNCTION                           
           - Definition: CUR Current module                                  
           - Usage:      PWM Pwm module                                    */
typedef void (*vOverCurCallBack)(void);

extern  vOverCurCallBack* CUR_ptGetOverCurCallBackFunc(void);

extern void COM_vCheckOverThresholdLimit(const TOverThresholdParm *ptParaPtr, TOverThresholdData *ptDataPtr);

#endif
