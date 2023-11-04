#ifndef __COMPILE_H
#define __COMPILE_H

/**
    \defgroup COMPILE Compile parameters
    \{
*/

/*****************************************************************************************************************
 * BASIC DEFINITIONS *********************************************************************************************
 *****************************************************************************************************************/

/**
 * \brief	Global macro to enable option
 */
#define OPTION_ACTIVE				(1)

/**
 * \brief	Global macro to disable option
 */
#define OPTION_PASSIVE				(0)
/**
 *  \brief Enables Test mode
 *  */
#ifndef MOTOR_START_WITHOUT_ALIGN
#define MOTOR_START_WITHOUT_ALIGN		OPTION_ACTIVE
#endif

/**
 *  \brief Enables ramp curve
 *  */
#ifndef MOTOR_START_HALF_SIN_CURVE
#define MOTOR_START_HALF_SIN_CURVE		OPTION_PASSIVE
#endif

/**
 *  \brief Alternative start up mode, Half since ramp up, call periode is 10ms
 *  */
#ifndef MOTOR_ALTERNATIVE_START_MODE
#define MOTOR_ALTERNATIVE_START_MODE	OPTION_PASSIVE
#endif


#ifndef FS_MOTOR_FS_MONITOR
#define FS_MOTOR_FS_MONITOR             OPTION_ACTIVE
#endif

#ifndef FS_COMMON_ENABLE
#define FS_COMMON_ENABLE                OPTION_ACTIVE
#endif

#ifndef FS_SPEED_MONITOR
#define FS_SPEED_MONITOR             	OPTION_ACTIVE
#endif

#ifndef FS_MOTOR_BLOCK
#define FS_MOTOR_BLOCK                  OPTION_ACTIVE
#endif

#ifndef FS_MOTOR_PHASE_LOSS
#define FS_MOTOR_PHASE_LOSS             OPTION_ACTIVE
#endif

//subdefinition
#ifndef FS_SPEED_MONITOR_SIMPLE
#define FS_SPEED_MONITOR_SIMPLE         OPTION_PASSIVE
#endif

#ifndef FS_MOTOR_BLOCK_SIMPLE
#define FS_MOTOR_BLOCK_SIMPLE           OPTION_PASSIVE
#endif

#ifndef FS_MOTOR_PHASE_LOSS_SIMPLE
#define FS_MOTOR_PHASE_LOSS_SIMPLE      OPTION_PASSIVE
#endif


#ifndef WDN_WIND_DETECTION_METHOD_1
#define WDN_WIND_DETECTION_METHOD_1     OPTION_ACTIVE
#endif

#ifndef WDN_WIND_DETECTION_METHOD_2
#define WDN_WIND_DETECTION_METHOD_2     OPTION_PASSIVE
#endif

#ifndef MPM_SYESTEM_DELAY
#define MPM_SYESTEM_DELAY				OPTION_ACTIVE
#endif

#ifndef ERROR_HANDLE_EN
#define ERROR_HANDLE_EN				    OPTION_ACTIVE
#endif

#ifndef DBUG_VOFA_EN
#define DBUG_VOFA_EN                    OPTION_PASSIVE
#endif

#ifndef SECOND_UART_USED
#define SECOND_UART_USED                OPTION_ACTIVE
#endif

#endif
