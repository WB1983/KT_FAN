
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
extern TFp VOL_tGet13_5VolmV(void);
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
extern BOOL VOL_bInitDcLinkVoltageMeasurement(const void* const pData);

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
extern BOOL VOL_bVoltageErrorDetection(const void* const pData);

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

#endif


