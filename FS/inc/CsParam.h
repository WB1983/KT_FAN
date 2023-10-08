#ifndef __CSP_CH_H
#define __CSP_CH_H

#include "compile.h"				/* Include global compiler switches */

/**
    \defgroup CS_PARAMETERSET CS Parameterset (EU variant)
    \{
*/

/*****************************************************************************************************************
 * MOTOR 2 PARAMETERS ********************************************************************************************
 *****************************************************************************************************************/

#define CSP_MOT_MAX_RESTART_ATTEMPTS_M2		(3)

/*-------------------------------------------------------------------------------------------------
 * SPEED LIMIT PARAMTERS --------------------------------------------------------------------------
 *------------------------------------------------------------------------------------------------- */

/**
    \brief Max allowed target speed of the motor in [rpm]

    Target speed above this limit lead are limited to this value
*/
#define CSP_MAX_TARGET_SPEED_RPM_M2			(700)

/**
    \brief Min allowed target speed of the motor in [rpm]

    Target speeds below this limit lead to a target speed of 0rpm.
*/
#define CSP_MIN_TARGET_SPEED_RPM_M2			(600)

/**
 * \brief 	Corporate safety limit for i2t derived from the FS limit value
 * */
#if (USE_I2T_LIMIT_NOT_DIFFERENT_TIME_CONSTANT == OPTION_PASSIVE)
#define CSP_I2T_LIMIT_M2					(FSP_IIT_POWER_LIMIT_M2-500000UL)
#else
#define CSP_I2T_LIMIT_M2					(FSP_IIT_POWER_LIMIT_M2)
#endif

#define CSP_I2T_LIMIT_HYSTERESE				(CSP_I2T_LIMIT_M2-500000UL)
/**
 * \brief 	Corporate safety i2t filter coefficient
 * */
#define CSP_IITCS_EMA_COEFFICIENT_M2		(31500UL)

/**
 * \brief 	Corporate safety comparison time of the actual i2t value against limit
 * */
#define CSP_IITCS_POWER_CHECK_TIME_M2		STK_DEF_TIME_MS(40U)

/**
 * \brief 	Max accepted undershoot of the actual motor speed below the min speed limit in [rpm]
 * */
#define CSP_ACC_CTRL_UNDERSHOOT_RPM_M2		(100)


/*****************************************************************************************************************
 * MOTOR 1 PARAMETERS ********************************************************************************************
 *****************************************************************************************************************/

/**
     @brief Max. number of restart attempts of the motor

     If the motor is blocked, the PM tries to start the motor for this
     no. of times before a blocked error is set.
\n\n
     <b>PARAMETER DATA:</b>
     - <b>Valid value range:</b>  1-255
     - <b>Physical value:</b>     15
     - <b>Physical unit:</b>      n.a.
     - <b>Internal SW unit:</b>   n.a.
     - <b>Module file name:</b>   motor.c

     <b>CHANGE HISTORY:</b>
     - 19.04.2010 Moved from motor module to here; divided parameter into
                  two values, one for the motor 1 and one for the motor 2
*/
#define CSP_MOT_MAX_RESTART_ATTEMPTS_M1		(15)

/*-------------------------------------------------------------------------------------------------
 * SPEED LIMIT PARAMTERS --------------------------------------------------------------------------
 *------------------------------------------------------------------------------------------------- */

/**
    \brief Max allowed target speed of the motor in [rpm]

    Target speed above this limit lead are limited to this value.
*/
#define CSP_MAX_TARGET_SPEED_RPM_M1		(3800)

/**
    \brief Min allowed target speed of the motor [rpm]

    Target speeds below this limit lead to a target speed of 0rpm.
*/
#define CSP_MIN_TARGET_SPEED_RPM_M1		(1500)

/**
 * \brief 	Corporate safety limit for i2t derived from the FS limit value
 * */
#if (USE_I2T_LIMIT_NOT_DIFFERENT_TIME_CONSTANT == OPTION_PASSIVE)
#define CSP_I2T_LIMIT_M1					(FSP_IIT_POWER_LIMIT_M1-500000UL)
#else
#define CSP_I2T_LIMIT_M1					(FSP_IIT_POWER_LIMIT_M1)
#endif
/**
 * \brief 	Corporate safety i2t filter coefficient
 * */
#define CSP_IITCS_EMA_COEFFICIENT_M1		(31500UL)

/**
 * \brief 	Corporate safety comparison time of the actual i2t value against limit
 * */
#define CSP_IITCS_POWER_CHECK_TIME_M1		STK_DEF_TIME_MS(40U)

/**
 * \brief 	Max accepted undershoot of the actual motor speed below the min speed limit in [rpm]
 * */
#define CSP_ACC_CTRL_UNDERSHOOT_RPM_M1	(100)

/**\}*/ //end of group MOTOR Motor limit parameters

/* -------------------------------------------------------------------------------------------------------------------
 * VOLTAGE MEASUREMENT SUPERVISION
 * -----------------------------------------------------------------------------------------------------------------*/
/*- Low voltage limit definition -----------------------------------------------------------------------------------*/

/*standard voltage 12v DC, corresponding AC voltage is 12/1.414 = 8.5V, 

low voltage        8v  DC, corresponding AC voltage is 8/1.414 = 5.6,  hysteresis is 1V

High voltage       15v DC  corresonding AC voltage is 15/1.414 = 10.6,  hysteresis is 1V


*/
/**
    \defgroup VOL CS VOLTAGE MEASUREMENT SUPERVISION
    \{
*/
/**
 * \brief	Definition of AC value of low voltage limit
 */
#define CSP_LOW_VOLTAGE_LIMIT_V_AC			(5.6)

/**
 * 	\brief	Low voltage limit
 *
 * 			If the DC voltage value is below this limit,
 * 			the motor selftest and run of motor is denied.
 *
 * 			\li Physical unit [V]
 * 	*/
#define CSP_LOW_VOLTAGE_LIMIT_V						(TFp)(CSP_LOW_VOLTAGE_LIMIT_V_AC * FPM_SQRT2_Q15)

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
#define CSP_LOW_VOLTAGE_HYSTERESIS_V				(1)

/*- Under voltage limit definition -----------------------------------------------------------------------------*/

/**
 * \brief	Definition of AC value of under voltage limit
 */
#define CSP_CRITICAL_UNDERVOLTAGE_V_AC					(5)

/**
 * 	\brief	Critical under voltage limit
 *
 * 			If the DC voltage value is below this limit certain time,
 * 			the low voltage error is set.
 *
 * 			\li Physical unit [V]
 * 	*/
#define CSP_CRITICAL_UNDERVOLTAGE_V					(TFp)(CSP_CRITICAL_UNDERVOLTAGE_V_AC * FPM_SQRT2_Q15)

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
#define CSP_UNDERVOLTAGE_HYSTERESIS_V				(1)

/*- Over voltage limit definition ------------------------------------------------------------------------------*/

/**
 * \brief	Definition of AC value of over voltage limit
 *
 * 			\li Physical unit [V]
 */
#define CSP_CRITICAL_OVERVOLTAGE_V_AC				(10.6)
/**
 *	\brief	Critical over voltage limit
 *
 *			If the DC link voltage value is above this limit certain time,
 *			the over-voltage error is set.
 *
 *			\li Physical unit [V]
 *	*/
#define CSP_CRITICAL_OVERVOLTAGE_V					(TFp) (CSP_CRITICAL_OVERVOLTAGE_V_AC * FPM_SQRT2_Q15)

/**
 *	\brief	Hysteresis used for over-voltage mode recovery
 *
 *			If the DC link voltage value is below the over-voltage limit minus this
 *			hysteresis, the over-voltage mode is left and the error is reset.
 *
 *			\li Physical unit [V]
 *	*/
#define CSP_OVERVOLTAGE_HYSTERESIS_V				(1)

/**
 *	\brief	Hysteresis used for over-voltage for internal voltage reference
 *
 *			If the interna voltage reference  value is below the  limit 
 *			, the error is set.
 *
 *			\li Physical unit [V]
 *	*/
#define CSP_INTERNAL_REFERENCE_VOLTAGE_HIGH_LIMIT_MV		(1041) //3.8V

/*- Recovery timeout definition --------------------------------------------------------------------------------*/

/**
 *	\brief	Timeout for low line voltage error report
 *
 * 			If the DC link voltage value is below the low voltage limit for
 * 			longer than this time, a low voltage application/motor error is set.
 * 	*/
//todo: define right time delay value
#define CSP_LOW_VOLTAGE_TIMEOUT						STK_DEF_TIME_MS(20)

/**
 * 	\brief	Timeout for critical under-voltage detection
 *
 * 			If the peak value of the DC link voltage exceeds the max. value for
 * 			longer than this time, the power module is switched to power down mode.
 * 	*/
#define CSP_CRITICAL_UNDERVOLTAGE_TIMEOUT			STK_DEF_TIME_MS(20)

/**
 * 	\brief	Timeout for critical over-voltage detection
 *
 * 			If the peak value of the DC link voltage exceeds the max. value for
 * 			longer than this time, the power module is switched to power down mode.
 * 	*/
#define CSP_CRITICAL_OVERVOLTAGE_TIMEOUT			STK_DEF_TIME_MS(20)
/**\}*/ //end of group VOL CS VOLTAGE MEASUREMENT SUPERVISION
/**\}*/ //end of group CS_PARAMETERSET CS Parameterset (EU variant)

#endif
