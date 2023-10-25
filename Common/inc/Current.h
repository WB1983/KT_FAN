/****************************************************************************************************************
 * INCLUDES *****************************************************************************************************
*****************************************************************************************************************/

#ifndef __CUR_H
#define __CUR_H

#include "common.h"
#include "LibDefines.h"
#include "Filter.h"
#include "main.h"
#include "fsclock.h"

/****************************************************************************************************************
 * GLOBAL TYPES *************************************************************************************************
*****************************************************************************************************************/
/**
 *
 */
typedef uint16_t (*TpfCurInterface)(void);

/** \brief    Enum of current quantities */
typedef enum _TCurQuantity {
  CUR_PHASE_A,             /**< Current phase A */
  CUR_PHASE_B,             /**< Current phase B */
  CUR_PHASE_C,             /**< Current phase C */
  CUR_MAXIMUM_Q,           /**< Current maximum */
  CUR_MAXIMUM_MA,          /**< Cur. max.[mA]   */
  CUR_IQ_VAL,              /**< Cur. iq value   */
  CUR_IQ_DAMPED,           /**< Cur. iq damped  */
  CUR_MAXIMUM_MA_DMPD      /**< Max.[mA] damped */
}TCurQuantity;


typedef struct _TThreePhaseRMS
{
	TThreePhaseCur tThreePhaseCurRaw;
	uint16_t       u16ThreePhaseRMSCur;
	TFilterData    tThreePhaseRMSCurFilter;
	uint16_t       u16ThreePhaseRMSCurFilter;
}TThreePhaseRMS;

typedef struct _TCurBufferCal
{
	int32_t buffer_IaN;
	int32_t buffer_IaP;
	int64_t buffer_Ia;

	int32_t buffer_IbN;
	int32_t buffer_IbP;
	int64_t buffer_Ib;

	int32_t buffer_IcN;
	int32_t buffer_IcP;
	int64_t buffer_Ic;

	int32_t buffer_IaN_Avg;
	int32_t buffer_IaP_Avg;
	int64_t buffer_Ia_Avg;

	int32_t buffer_IbN_Avg;
	int32_t buffer_IbP_Avg;
	int64_t buffer_Ib_Avg;

	int32_t buffer_IcN_Avg;
	int32_t buffer_IcP_Avg;
	int64_t buffer_Ic_Avg;
}TCurBufferCal;


/****************************************************************************************************************
 * GLOBAL FUNCTIONS *********************************************************************************************
*****************************************************************************************************************/

/**
 * \brief	Interface to get current values
 * \param	tBrd		requested hw device (bridge)
 * 			tQuantity	requested current quantity
 * \return	[qFp]		requested current value
 */
extern TFp CUR_tGetCurVal(TCurQuantity tQuantity);

#if 0
/**
 * \brief	Function sets the Iq current value to the passed parameter value
 *
 * 			This function is used by the vector control module which generates the Iq
 * 			current value. Because the current module should handle all current related
 * 			values, the Iq current is cyclically passed to this module.
 *
 * \param	tBrd		requested hw device (bridge)
 * 			siIq [q12]	Iq current
 * \return
 */
extern void CUR_vSetCurrentIq(THWChannel tBrd, TFp tIq);
#endif
/**
 * \brief	Motor current measurement offset voltage initialization
 *
 * 			This function implements the standard motor self-check initialization
 * 			interface. It should be registered in the self-check configuration
 * 			structure.\n
 * 			The function initializes the two offset voltages of the half bridge
 * 			current measurements A and B. The two values are initialized by
 * 			averaging the first N samples after reset.
 *
 * \param	pData Generic data pointer (not used by this function)
 *
 * \return	BOOL TRUE => Initialization is finished
 */
extern BOOL CUR_bInitOffsetCurrentVoltage(void);

/**
 * \brief	Function implements the motor current measurement supervision
 *
 * 			Function calls all current measurement related supervision functions.
 * 			These are:
 * 			\li Offset supervision
 * 			\li Peak phase current calculation
 * 			\li Over-current interrupt pin supervision
 *
 * \param	pData Required due to standard continuous self-check interface. Not used here
 *
 * \return	BOOL TRUE  Supervision is active
 */
extern BOOL CUR_bCheckMotCurrentMeasValues(const void* const pData);

/**
 * \brief	Set of alignment current
 * 			Interface function to set alignment current during positioning phase
 *
 * \return	void
 */
extern void CUR_vSetPositioningCurr(void);

/**
 * \brief	Interface to get VDD voltage
 * \param	void
 * \return	CUR_tRealVddVoltage_mV
 */
extern TFp CUR_tGetVddVoltage(void);
/* --------------------------------------------------------------------------------------------------------------------
 * Interface functions
 * -------------------------------------------------------------------------------------------------------------------- */

/**
 * \brief	Calculation of currents in both measured phases
 * \param	THWChannel tBrdg	Actual HW channel
 * \return	void
 *
 * !!! GLOBAL INTERFACE => DEKLARATION IN MotorInf.h !!!
 * This function has its declaration in  MotorInf.h
 * void CUR_vFetchAndCalcPhaseCurrents(THWChannel tBrdg)
 */

extern void CUR_vFetchAndCalcPhaseCurrents(void);
/**
 * \brief	Interface to get all three phases current values
 * \param	THWChannel tBrdg			Actual HW channel
 * 			TThreePhaseCur* ptThreePha	Pointer to phase currents
 * \return	void
 *
 * !!! GLOBAL INTERFACE => DEKLARATION IN MotorInf.h !!!
 * This function has its declaration in  MotorInf.h
 * void CUR_vGetThreePhaseCur(THWChannel tBrdg, TThreePhaseCur* ptThreePha)
 */
extern void CUR_vGetThreePhaseCur(TThreePhaseCur* ptThreePha);
/**
 * \brief	Interface to set offset state
 * \param	THWChannel tBrdg			Actual HW channel
 * 			TBrdCurOffsetState tState	Requested state
 * \return	void
 *
 * !!! GLOBAL INTERFACE => DEKLARATION IN MotorInf.h !!!
 * This function has its declaration in  MotorInf.h
 * void CUR_vSetOffsetMeasState(THWChannel tBrdg, TBrdCurOffsetState tState)
 */

extern void CUR_vSetOffsetMeasState(TBrdCurOffsetState tState);

/**
 * \brief	Interface to set callback function during OC event
 * \param	THWChannel tBrdg		Actual HW channel
 * \return	vOverCurCallBack* 		Address to call function
 *
 * !!! GLOBAL INTERFACE => DEKLARATION IN MotorInf.h !!!
 * This function has its declaration in  MotorInf.h
 * vOverCurCallBack* CUR_ptGetOverCurCallBackFunc(THWChannel tBrdg)
 */

extern void CUR_vGetThreePhaseRawValue(TThreePhaseCur* ptThreePhaRaw);


extern uint16_t CUR_u16GetThreePhaseRMS(void);

extern void CUR_vCurRMSFilterInit(void);

extern void CUR_vCalThreePhaseCurRMS(void);

extern void CUR_updatePhaseCurrentBuffer(void);

extern void CUR_vGetPhaseCurrentBufferSamples(void);

extern TCurBufferCal * CUR_tGetPhaseCurrentCalPointer(void);

#endif
