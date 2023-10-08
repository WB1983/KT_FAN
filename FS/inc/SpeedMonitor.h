

#ifndef __FSVECTOR_H
#define __FSVECTOR_H

/* INCLUDES *********************************************************************************************/
#include "Math.h"			/* Include fixed point math interface */
#include "current.h"

/* GLOBAL VARIABLE DECLARATION **************************************************************************/
/** \brief	TSmrModelInputData structure holds the data needed for independent motor model sent from Vector.c */
typedef struct _TSmrModelInputData {	///< \brief Motor model input data object
	TFp		tIAlpha;		///< alpha phase current
	TFp		tIBeta;			///< beta phase current
	TFp		tUAlpha;		///< Motor alpha component
	TFp		tUBeta;			///< Motor beta component
	TAngle	tDeltaPsi;		///< Filtered delta psi
} TSmrModelInputData;

typedef struct _TPhaseVoltage {
    TFp a;    /**< Component of the A phase */
    TFp b;    /**< Component of the B phase */
    TFp c;    /**< Component of the C phase */
}TPhaseVoltage;

typedef struct _TVoltageCart {
	TFp tRe;    /**< Real part of the complex number */
    TFp tIm;    /**< Imag part of the complex number */
}TVoltageCart;


typedef struct _TBemfFluxVolt
{
	TFp tRe;    /**< Real part of the complex number */
    TFp tIm;    /**< Imag part of the complex number */
}TBemfFluxVolt;

typedef struct _TBemfCartVolt
{
	TFp tRe;    /**< Real part of the complex number */
    TFp tIm;    /**< Imag part of the complex number */
}TBemfCartVolt;


typedef struct _TConstructVoltage
{
	TThreePhase 	tVoltPhase;
	TCartNum     	tVoltCart;
	TFilterData  	tVoltAlpha;
	TFilterData  	tVolBelta;
	TBemfFluxVolt 	tFluxVolt;
	TPolNum     	tPolarVolt;
	TAngle          tLastAngle;   
	TAngle          tAngleDiff;  
}TConstructVoltage;

typedef struct _TSpeedObserver
{
	TFilterData tObserverSpeedFilter;
	int32_t     s32ObserverSpd;
}TSpeedObserver;

typedef struct _TSpdMonitorSpeedLimit
{
	TFp s32UpLimitSpeed;
	TFp s32DownLimitSpeed;
}TSpdMonitorSpeedLimit;

typedef struct _TSMRDataSecond
{
	TDutyCycleConfig       	tDutyCycle;
	TConstructVoltage     	tConstructVoltage;
	TCurrents 				tCurrent;
	TSpeedObserver          tSpeedObserver;
	TSafeTime				tTaskEntryTime;					/**< Task entry time stamp		*/
	TSafeTime				tLastTimeSliceTime;				/**< Task time base time stamp  */
	TSpdMonitorSpeedLimit   tSpdMonitorSpeedLimit;
	uint32_t                u32ErrorCount;
	uint32_t                tObserverSpdPU;
}TSMRDataSecond;

/* GLOBAL FUNCTION DECLARATION *************************************************************************/
/**
 * \brief	This function computes the second independent motor model
 * \param 	TSmrModelInputData holds the output data passed from normal motor model in Vector.c
 * \return	none
 * \remark	none
 * */
extern void SMR_vMotorModelCalculation(const TSmrModelInputData* ptModel);
extern void SMR_vInitModelCalculation(void);

extern void SMR_vSpeedEvaluation(void);

extern TSMRDataSecond * SMR_tGetFSVData(void);

extern void SMR_vCheckSpeedMonitor(void);

#endif // __FSVECTOR_H
