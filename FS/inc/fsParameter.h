#ifndef __FSPARAMETER_H
#define __FSPARAMETER_H

#include "LibDefines.h"
#include "fsClock.h"
/*-------------------------------------------------------------------------------------------------
 * CURRENT MODULE PARAMETERS ----------------------------------------------------------------------
 *------------------------------------------------------------------------------------------------- */

#define FSP_CUR_OFFSET_ERROR_TIMEOUT		STK_DEF_TIME_MS(1000)



#define FSP_ENABLE						(1)						/**< Enable (BOOL) value							*/
																
#define FSP_DISABLE						(0)						/**< Disable (BOOL) value							*/
																
#define FSP_MILISECOND					(0.001)					/**< fraction of second for next calculation		*/


#define FSP_HSI_CLOCK 					(8000000)				/**< Frequency of internal high speed oscillator	*/

#define FSP_LSI_CLOCK					(40000)					/**< Frequency of internal low speed oscillator	*/

#define FSP_HSE_CLOCK					(8000000)				/**< Frequency of external high speed oscillator	*/

#define FSP_PLL_CLOCK					(CORE_CLOCK * 1000000)	/**< PLL frequency	*/



#define FSP_SYSTICK_RELOAD_TIME			(6.4)					/**< Systick reload value (ms)						*/

#define FSP_SYSTICK_RELOAD_VALUE_HSI_	(ulong)(FSP_HSI_CLOCK * FSP_SYSTICK_RELOAD_TIME *FSP_MILISECOND)
																/**< Systick reload value (HSI Clock source)		*/
																/**<( 8Mhz	* 6.4 * 0.001)		= 51200				*/
#define FSP_SYSTICK_RELOAD_VALUE_HSI	(ulong)(51200)			/**< Systick reload value (HSI Clock source)(QAC)	*/

#define FSP_SYSTICK_RELOAD_VALUE_HSE	(ulong)(FSP_HSE_CLOCK * FSP_SYSTICK_RELOAD_TIME *FSP_MILISECOND)
																/**< Systick reload value (HSE Clock source)		*/
																/**<( 8Mhz	* 6.4 * 0.001)		= 51200				*/
#define FSP_SYSTICK_RELOAD_VALUE_PLL	(ulong)(FSP_PLL_CLOCK * FSP_SYSTICK_RELOAD_TIME *FSP_MILISECOND)
																/**< Systick reload value (PLL Clock source)		*/
																/**<( 72Mhz	* 6.4 * 0.001)						*/


#define FSP_MAX_HSI_RATIO				(TFp)(320)				/**<	Macros are calculated, to avoid QAC errors	*/
																/**< (256 * 1.25) = 320 */
#define	FSP_MIN_HSI_RATIO				(TFp)(204)				/**<	Macros are calculated, to avoid QAC errors	*/
																/**< (256 / 1.25) = 204 */

#define FSP_MAX_PLL_RATIO				(ulong)(320) 			/**<	Macros are calculated, to avoid QAC errors	*/
																/**< (256 * 1.25) = 320								*/
#define	FSP_MIN_PLL_RATIO				(ulong)(204)			/**<	Macros are calculated, to avoid QAC errors	*/
																/**< (256 / 1.25) = \b 204								*/


#define FSP_MAX_HSE_RATIO				(TFp)5120				/**<	Macros are calculated, to avoid QAC errors	*/
																/**< (1024*(8MHz/8MHz) * 1.25)  = 5120 				*/
#define FSP_MIN_HSE_RATIO				(TFp)3276				/**<	Macros are calculated, to avoid QAC errors	*/
																/**< (1024*(8MHz/8MHz) / 1.25)  = 3276				*/


#define FSP_ERROR_CTR					(5)						/**< Define max error counter						*/

/*--------------------------------------------------------------------------------------------------------------*/
/**	Second motor model parameter definitions - fsVector.c														*/
/*--------------------------------------------------------------------------------------------------------------*/
/**
 * \defgroup MOT Second motor model parameter definitions - fsVector.c
 * \{*/
/**
 *	\brief	'FSP_FSV_MOTOR_MODEL_CYCLE_TIME_CHECK' macro defines cycle time of the models
 *			speed difference limit check.
 *	\note
 **/
#define FSP_SMR_MOTOR_MODEL_CYCLE_TIME_CHECK		STK_DEF_TIME_MS(1000U)

/**
 *	\brief	'FSP_FSV_MOTOR_MODEL_MAX_SPEED_ERRORS' macro defines maximum number of the
 *			speed error over 'FSP_FSV_MOTOR_MODEL_CYCLE_TIME_CHECK' time
 *	\note
 **/
#define FSP_SMR_MOTOR_MODEL_MAX_SPEED_ERRORS		(200UL)

/**
 *	\brief	'FSP_FSV_MOTOR_MODEL_MAX_SPEED_DIFF' macro defines maximum difference
 *			between 2 motor model speeds.
 *	\note	It is used for both positive and negative limit initialization.
 **/
#define FSP_SMR_MOTOR_MODEL_MAX_SPEED_DIFF			(60UL)			/*  */


/**
 * \defgroup IWWDG Independent internal window watchdog IWWDG - fsMcuSelfcheck.c
 * \{*/
/**
 * \brief	The time to trigger IWWDG is set at cca 150ms. As the internal RC oscillator that drives IWWDG can
 * 			have range 30-50kHz the 'FSP_IWWDG_RELOAD_VALUE' macro value is set for 50kHz (worst case).
 *			IWDG_TIME = 0.15s*50000Hz/4 = 1875  => 0x753
 */
#define FSP_IWWDG_RELOAD_VALUE				(0x0753U)			///< this macro sets the max.  time in which the IWWD must be triggered

#define FSM_SPD_EST_ERR_COUNT_MAX           (0x100)

/** \brief   _LROTOR_MIN_BEMF_PER_SPEED_RATIO_Q12_LS
*
*  \details  ratio between minimum bemf voltage per defined speed used for low speeds below _LROTOR_MIN_BEMF_PER_SPEED_SWITCH_SPEED
*/
#define FSF_LROTOR_MIN_BEMF_PER_SPEED_RATIO_Q12_LS	Q15(0.75*73/U_MAX)//(22089UL)


/** \brief   _LROTOR_MIN_BEMF_PER_SPEED_RATIO_Q12_HS
*
*  \details  ratio between minimum bemf voltage per defined speed used for high speeds above _LROTOR_MIN_BEMF_PER_SPEED_SWITCH_SPEED
*/
#define FSF_LROTOR_MIN_BEMF_PER_SPEED_RATIO_Q12_HS	Q15(0.8*73/U_MAX)//(22089UL)


/** \brief   _LROTOR_MIN_BEMF_PER_SPEED_SWITCH_SPEED
*
*  \details  [motor rpm]
*  			 If the actual speed is above this speed: for minimum BEMF limit evaluation will be used ratio _LROTOR_MIN_BEMF_PER_SPEED_RATIO_Q12_HS
*  			 If the actual speed is below this speed: for minimum BEMF limit evaluation will be used ratio _LROTOR_MIN_BEMF_PER_SPEED_RATIO_Q12_LS
*/
#define FSF_LROTOR_MIN_BEMF_PER_SPEED_SWITCH_SPEED   Q15(1000/SPEED_MAX)


/** \brief   _LROTOR_MIN_SUPPORTED_SPEED
*
*  \details  [motor rpm]
*  			 Threshold for minimum required speed (target and also actual) by LROTOR detection,
*  			 If the actual and target speed will be in range >0 and <MinSupportSpeed_rpm, increasing of error counter will be forced
*/
#define FSF_LROTOR_MIN_SUPPORTED_SPEED   			(300)



#define	MOTPAR_MIN_BEMF_PER_SPEED_LIMIT	(580)									/**< EMF limit for rotor lock detection			*/

/**********************For system tick**********************************************************************************/
#define FSP_FST_NOMINAL_SAFE_TIME_VALUE			(uint32)(GENPAR_CPU_CORE_CLOCK_MHZ * 100U)		///< for 96MHz and 16kHz PWM = 7200 Systick timer decrements
#define FSP_FST_MAX_TIME_DIFF_LIMIT				(0x00002D00UL)		///< 20% added to 9600 System timer reference - upper limit
#define FSP_FST_MIN_TIME_DIFF_LIMIT				(0x00001E00UL)		///< 20% subtracted from 7200 System timer reference - lower limit
#define FSP_FST_NBR_OF_TICKS_ERROR_CHECK		(100U)				///< check error every 10 mili seconds
#define FSP_FST_WRONG_TICK_TIME_NBR_LIMIT		(5U)				///< number of accepted too short/long ticks per 'FSP_FST_NBR_OF_TICKS_ERROR_CHECK' ticks
#define FSP_FST_NBR_OF_INIT_TICKS				(10U)				///< wait few tickes till system get stable after reset/waking up

#endif
