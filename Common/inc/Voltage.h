
#ifndef __VOL_H
#define __VOL_H

/*****************************************************************************************************************
 * INCLUDES ******************************************************************************************************
 *****************************************************************************************************************/
#include "Transfer.h"
#include "Math.h"
#include "Current.h"		/* Include own header				*/
#include "fsClock.h"
#include "LibDefines.h"
/*****************************************************************************************************************
 * GLOBAL TYPES **************************************************************************************************
 *****************************************************************************************************************/

typedef TFp		TVoltageQ;		/**< Voltage type in fixed point format */
typedef TFp		TVoltageV;		/**< Voltage type in unit [V] */

/*****************************************************************************************************************
 * GLOBAL MACROS *************************************************************************************************
 *****************************************************************************************************************/
#define M8(adr)  (*((vu8  *)(adr)))
#define M16(adr) (*((vu16 *)(adr)))
#define M32(adr) (*((vu32 *)(adr)))

/**
    @brief Voltage measurement init time stamp

     First voltage handle task is called after certain time
     to avoid FSF error report due first initialization of software
\n\n
     <b>PARAMETER DATA:</b>
     - <b>Valid value range:</b>  0-65000000
     - <b>Physical value:</b>     1000
     - <b>Physical unit:</b>      ms
     - <b>Internal SW unit:</b>   100ns
     - <b>Module file name:</b>   Voltage.c

     <b>CHANGE HISTORY:</b>
     - 24.06.2009 Initial Revision
*/
#define VOL_INIT_TIME_STAMP				STK_DEF_TIME_MS(1000)

#define VOL_REF_VOL                          25

// //Below 90 degree, it is one slop. mv/degree
// //90 degree 2.65
// #define FTE_SLOP_MIN_LOW         
// #define FTE_SLOP_MIN_HIGH

// //90 degree 2.95, 25 degree 
// #define FTE_SLOP_MAX_LOW         
// #define FTE_SLOP_MAX_HIGH

// // 90 Degree 2.8 25 degree 1.14-->25.54(MV/Degree)
// #define FTE_SLOP_TYPIC        4096*25.54
/*
*(4096*1.14/5)*5000 
*/
//#define FTE_BASE_TYPIC        934*5000   

typedef enum _ENormalADCID
{
	E_15V_VOL,
	E_ISUM_CUR,
	E_IPM_NTC_TEMP,
	E_ISM_WO_FIL,
	E_VREF,
	E_VTEMP,
	E_NOR_ADC_CH,
}ENormalADCID; 

/*
*Vref = 3.3*digital_old/4096
*Vref/digital = Vdd/4096
*Vdd = (Vref/digital)*4096 ==>3.3*digital_old/digital
*/
typedef struct _TREFVOLCAL
{
     uint16_t u16VoltageADCRef;
     uint16_t u16MCUVol;

}TREFVOLCAL;
 
typedef struct _TVOLERRORCHECK
{
     uint16_t u16MCUVoltageHighErrorCnt;
     uint16_t u16MCUVoltageLowErrorCnt;
     uint16_t u16IPMDriverVolHighErrorCnt;
	 uint16_t u16IPMDriverVolLowErrorCnt;

}TVOLERRORCHECK;

extern uint16_t   VOL_u16NormalAdcValueArray[E_NOR_ADC_CH];

/*****************************************************************************************************************
 * GLOBAL FUNCTIONS **********************************************************************************************
 *****************************************************************************************************************/

/**
 *	\brief	Function returns the result of the DC link voltage
 *
 *	\return	TRUE	A low voltage was detected
 *			FALSE	DC link voltage is present , normal operation
 */
extern BOOL VOL_bCheckLowVoltageCondition(void);

/**
 * \brief	Function reads the last samples AD values of the DC link voltage.
 *
 *			Additionally the function searches the max. value of the DCl voltage over
 *			one period and the min. value of the DC link voltage to be able to calculate
 *			the DC link ripple voltage. The DC link voltage samples are integrated over
 *			one line voltage period. The period value is used later to calculate the average
 *			DC link voltage.
 *
 * \return	void
 */
extern void VOL_vSampleDclVoltage(void);

/**
 *	\brief	Function Track the voltage values over one mains period
 *
 *	\note	This function needs to be called with a defined period which is
 *			much smaller than the mains period to ensure that the tracking
 *			works accurate. E.g. F track >= 10fmains
 *			An additional flag is set with each call to allow the main-loop
 *			handler to check whether the function has been called or not.
 *
 *	\return void
 */
extern void VOL_vTrackVoltageValues(void);

/**
 *	\brief	Function returns the last not filtered DC link voltage sample in fixed point format
 *
 *	\return	TFp	Last DC link voltage sample in unit [fixed point]
 */
extern TFp VOL_tGetDclVolRawSampleQ(void);

/**
 *	\brief	Function returns the value of the DCL average voltage
 *
 *			The voltage average is computed over the last mains period. This
 *			average value is not filtered.
 *
 *	\return	TFp	Period average of the DC link voltage in fixed point format
 */
extern TFp VOL_tGetDclVolQ(void);

/**
 * \brief	Function returns the value of the DCL average voltage
 *
 * 			The average DC link voltage is calculated over one line voltage
 * 			period.
 * \return	TFp [V] value of the DCL average voltage
 */
extern TFp VOL_tGetDclVolV(void);

/**
 * \brief	Function returns the value of the 13.5 average voltage
 * 			The average 13.5 voltage is calculated over one line voltage
 * 			period.
 *
 * 	\return	TFp [V] value of the 13.5V average voltage
 */
extern TFp VOL_tGet15VolmV(void);
/**
 * \brief	Function returns the value of the internal voltage reference voltage
 * 	\return	TFp [mV] value of the internal reference voltage 
 */

extern TFp VOL_tGetInternalVoltRefVolmV(void);
/**
 * \brief	Initialization of the internal data structures
 *
 *			Function waits until an initial delay time has elapsed. This is needed
 *			to be sure that the first AD samples have been taken and that the
 *			internal voltage values are initialized properly.
 *
 * \param	pData Pointer to a data structure (not used by this function)
 *
 * \return	BOOL TRUE => Initialization finished
*/
extern BOOL VOL_bInitDcLinkVoltageMeasurement(void);

/**
 * \brief	Function detects DC link under-voltage and critical over-voltage
 *
 * 			Under-voltage detection:\n
 * 			- If the average DC link voltage falls below the min. limit for loner than 20ms, the
 * 			  under-voltage error is set (motor system error and application error). Errors
 * 			  are only set if the line voltage watchdog has detected voltage pulses.
 * 			- If the average DC link voltage exceeds the min. limit plus hysteresis, the error
 * 			  is reset.
 *
 * 			Critical over-voltage detection:\n
 * 			- If the DC-link voltage value exceeds the critical limit for loner than 20ms,
 * 			  the error is set.
 * 			- If the average DC link voltage is below the max. limit minus hysteresis, the error
 * 			  is reset.
 *
 * \return	BOOL TRUE => always
*/
extern BOOL VOL_bVoltageErrorDetection(void);

/**
   \brief DC link voltage measurement handle

   DC link and line voltage measurement handler. Function
   calls all necesarry functions to calculate the actual DCL average
   and line voltage. Internally the function generates a time slice
   which is needed for the modules low-pass filters. Additionally the
   function checks if the AD sampling of the modules input values is
   triggered on a regular base.

   \param pData Needed due to the std selfchck interface but not used here

   \return   BOOL => Allways TRUE

*/
extern BOOL VOL_bHandleTask(void);

extern void VOL_tReadVoltageValue(void);

 /*
 * Initialize the const value for Vref and Temperature detection
 */
 extern void VOL_vConstValueInit(void);

#endif


