
#ifndef __PARAM_MOT_H
#define __PARAM_MOT_H

#include "Filter.h"
#include "ParamRef.h"
#include "Parameter.h"

#if(MOTOR_CURRENT_COMP == OPTION_ACTIVE)
#define MPM_MIN_MOTOR_CURRENT				(2500)				/**< Limit minimum value         */
#else
#define MPM_MIN_MOTOR_CURRENT				(0)					/**< Limit minimum value         */
#endif

#if(MOTOR_CURRENT_COMP_REV == OPTION_ACTIVE)
#define MPM_MIN_MOTOR_CURRENT_REV			(700)				/**< Limit minimum value         */
#else
#define MPM_MIN_MOTOR_CURRENT_REV			(0)					/**< Limit minimum value         */
#endif


#define MPM_OBR_FLX_FILCOE				FIR_tCalcFilterConst(0.2,3.0)			/**< Motor model PT1 filter flux calculation	*/
#define MOTPAR_DPSI_FLT_COEF			FIR_tCalcFilterConst(0.2,6.0)			/**< dPsi PT1 filter coefficient 				*/
#define MPM_CURVE_REG_FLT_COEF			FIR_tCalcFilterConst(0.2,13.8)			/**< IqUt filter coefficient 					*/
#define MPM_OBR_ID_ADJ_ANG_STPS			(1)		//2							/**< No.of.200us angle steep					*/
#define MPM_OBR_MAX_ANGLE_INC			(10)									/**< max angle steep during synchronization		*/
#define MPM_OBR_MIN_ANGLE_INC			(1)										/**< min angle steep during synchronization		*/
#define MPM_OBR_ANGLE_GRAD				(10)									/**< gradient for angle synchronization			*/
#define MPM_OBR_OBS_SPEED_TRES			(100)									/**< SMO running speed treshold					*/
#define MOTPAR_ID_RAMP_DOWN_DELAY		(500)									/**< Id delay during the synchronization		*/
#define MOTPAR_ID_RAMP_DOWN_STEP_SIZE	(1)										/**< Id steep down								*/
#define	MOTPAR_MIN_BEMF_PER_SPEED_LIMIT	(580)									/**< EMF limit for rotor lock detection			*/

#define MOTPAR_LD						(Ls*1000)
#define MOTPAR_LQ						(Ls*1000)
#define MOTPAR_LQ_LD					(MOTPAR_LD * MOTPAR_LQ)
#define MOTPAR_RS                       Rs//need update

#define MOTPAR_BASE_ElE_FREQ            SPEED_MAX*POLEPAIRS/60
#define MOTPAR_RS_Q                     Q15(MOTPAR_RS/REFPAR_REFERENCE_RESISTANCE_OHM)
#define MOTPAR_LS_Q                     Q15((2*3.14*MOTPAR_BASE_ElE_FREQ)*Ls/REFPAR_REFERENCE_RESISTANCE_OHM)
#define MOTPAR_Z_Q                      7798

#endif

