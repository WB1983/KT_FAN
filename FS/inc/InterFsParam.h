#ifndef __INTER_FS_PARAM_H
#define __INTER_FS_PARAM_H

#include "compile.h"				/* Include global compiler switches */

/* -------------------------------------------------------------------------------------------------------------------
 * VOLTAGE MEASUREMENT SUPERVISION
 * -----------------------------------------------------------------------------------------------------------------*/
/*- Low voltage limit definition -----------------------------------------------------------------------------------*/

/*standard voltage 220v DC, corresponding AC voltage is 220*1.414 = 310V, 

low voltage        120v  DC, corresponding AC voltage is 120*1.414 = 170,  hysteresis is 10V

High voltage       268v DC  corresonding AC voltage is 268*1.414 = 380,  hysteresis is 10V


*/
/**
    \defgroup VOL IF VOLTAGE MEASUREMENT SUPERVISION
    \{
*/
/**
 * \brief	Definition of AC value of low voltage limit
 */
#define IFP_LOW_VOLTAGE_LIMIT_V_AC			(120)

/**
 * 	\brief	Low voltage limit
 *
 * 			If the DC voltage value is below this limit,
 * 			the motor selftest and run of motor is denied.
 *
 * 			\li Physical unit [V]
 * 	*/
#define IFP_LOW_VOLTAGE_LIMIT_V						(TFp)(IFP_LOW_VOLTAGE_LIMIT_V_AC * FPM_SQRT2_Q15)

/**
 *	\brief	Hysteresis used for low voltage mode recovery
 *
 *			If the DC link voltage value is above
 *			the low voltage limit plus this
 *			hysteresis, the low voltage mode is left
 *			and the error is reset.
 *
 *			\li Physical unit [V]
 *	*/
#define IFP_LOW_VOLTAGE_HYSTERESIS_V				(10)

/*- Under voltage limit definition -----------------------------------------------------------------------------*/

/**
 * \brief	Definition of AC value of under voltage limit
 */
#define IFP_CRITICAL_UNDERVOLTAGE_V_AC					(80)

/**
 * 	\brief	Critical under voltage limit
 *
 * 			If the DC voltage value is below this limit certain time,
 * 			the low voltage error is set.
 *
 * 			\li Physical unit [V]
 * 	*/
#define IFP_CRITICAL_UNDERVOLTAGE_V					(TFp)(IFP_CRITICAL_UNDERVOLTAGE_V_AC * FPM_SQRT2_Q15)

/**
 *	\brief	Hysteresis used for under voltage mode recovery
 *
 *			If the DC link voltage value is above
 *			the low voltage limit plus this
 *			hysteresis, the low voltage mode is left
 *			and the error is reset.
 *
 *			\li Physical unit [V]
 */
#define IFP_UNDERVOLTAGE_HYSTERESIS_V				(10)

/*- Over voltage limit definition ------------------------------------------------------------------------------*/

/**
 * \brief	Definition of AC value of over voltage limit
 *
 * 			\li Physical unit [V]
 */
#define IFP_CRITICAL_OVERVOLTAGE_V_AC				(268)
/**
 *	\brief	Critical over voltage limit
 *
 *			If the DC link voltage value is above this limit certain time,
 *			the over-voltage error is set.
 *
 *			\li Physical unit [V]
 *	*/
#define IFP_CRITICAL_OVERVOLTAGE_V					(TFp) (IFP_CRITICAL_OVERVOLTAGE_V_AC * FPM_SQRT2_Q15)

/**
 *	\brief	Hysteresis used for over-voltage mode recovery
 *
 *			If the DC link voltage value is below the over-voltage limit minus this
 *			hysteresis, the over-voltage mode is left and the error is reset.
 *
 *			\li Physical unit [V]
 *	*/
#define IFP_OVERVOLTAGE_HYSTERESIS_V				(10)

/**
 *	\brief	Hysteresis used for over-voltage for internal voltage reference
 *
 *			If the interna voltage reference  value is below the  limit 
 *			, the error is set.
 *
 *			\li Physical unit [V]
 *	*/
#define IFP_INTERNAL_REFERENCE_VOLTAGE_HIGH_LIMIT_MV		(5300) //6.0V

#define IFP_INTERNAL_REFERENCE_VOLTAGE_LOW_LIMIT_MV		(4800) //6.0V


/**
 *	\brief	Hysteresis used for over-voltage for internal voltage reference
 *
 *			If the interna voltage reference  value is below the  limit 
 *			, the error is set.
 *
 *			\li Physical unit [V]
 *	*/
#define IFP_IPM_DRIVER_VOLTAGE_HIGH_LIMIT_MV		(15500) //15.5V
#define IFP_IPM_DRIVER_VOLTAGE_LOW_LIMIT_MV		    (14500) //14.5V

/*- Recovery timeout definition --------------------------------------------------------------------------------*/

/**
 *	\brief	Timeout for low line voltage error report
 *
 * 			If the DC link voltage value is below the low voltage limit for
 * 			longer than this time, a low voltage application/motor error is set.
 * 	*/
//todo: define right time delay value
#define IFP_LOW_VOLTAGE_TIMEOUT						STK_DEF_TIME_MS(20)

/**
 * 	\brief	Timeout for critical under-voltage detection
 *
 * 			If the peak value of the DC link voltage exceeds the max. value for
 * 			longer than this time, the power module is switched to power down mode.
 * 	*/
#define IFP_CRITICAL_UNDERVOLTAGE_TIMEOUT			STK_DEF_TIME_MS(20)

/**
 * 	\brief	Timeout for critical over-voltage detection
 *
 * 			If the peak value of the DC link voltage exceeds the max. value for
 * 			longer than this time, the power module is switched to power down mode.
 * 	*/
#define IFP_CRITICAL_OVERVOLTAGE_TIMEOUT			STK_DEF_TIME_MS(20)
/**\}*/ //end of group VOL CS VOLTAGE MEASUREMENT SUPERVISION
/**\}*/ //end of group CS_PARAMETERSET CS Parameterset (EU variant)

/*
* MCU voltage error confirm count
*/
#define VOL_MCU_VOL_ERROR_CNT                           200

/*
* IPM voltage error confirm count
*/
#define VOL_IPM_VOL_ERROR_CNT                           200

#endif
