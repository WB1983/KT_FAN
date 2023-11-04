#ifndef __GENPAR_H
#define __GENPAR_H

/*****************************************************************************************************************
 * INCLUDES ******************************************************************************************************
 *****************************************************************************************************************/

/*-------------------------------------------------------------------------------------------------
 * SELF-CHECK BIT MASKS ---------------------------------------------------------------------------
 *------------------------------------------------------------------------------------------------- */
#define FSP_MSC_EXTERNAL_WATCHDOG_INIT	0x00000001      /**< Check of the external watch dog after reset */
#define FSP_MSC_EXTERNAL_WATCHDOG_RUN	0x00000002      /**< Check of the external watch dog before start of motor */
#define FSP_MSC_B6_POWER_STAGE_CH1		0x00000004      /**< Check of the B6 power stage output channel 1 */
#define FSP_MSC_B6_POWER_STAGE_CH2		0x00000008      /**< Check of the B6 power stage output channel 2 */
#define FSP_MSC_OVERCURRENT_CH1			0x00000010      /**< Check of the 3 over current traps output channel 1 */
#define FSP_MSC_OVERCURRENT_CH2			0x00000020      /**< Check of the 3 over current traps output channel 2 */
#define FSP_MSC_DCL_RELAY				0x00000040      /**< Check of the DCL relay */
#define FSP_MSC_LINETEST_MOTOR_1		0x00000080      /**< Check of the circulation pump phase resistances */
#define FSP_MSC_LINETEST_MOTOR_2		0x00000100      /**< Check of the drain pump phase resistances */
#define FSP_MSC_RESERVED_09				0x00000200      /**< Reserved */
#define FSP_MSC_RESERVED_10				0x00000400      /**< Reserved */
#define FSP_MSC_RESERVED_11				0x00000800      /**< Reserved */
#define FSP_MSC_RESERVED_12				0x00001000      /**< Reserved */
#define FSP_MSC_RESERVED_13				0x00002000      /**< Reserved */
#define FSP_MSC_RESERVED_14				0x00004000      /**< Reserved */
#define FSP_MSC_RESERVED_15				0x00008000      /**< Reserved */
#define FSP_MSC_RESERVED_16				0x00010000      /**< Reserved */
#define FSP_MSC_RESERVED_17				0x00020000      /**< Reserved */
#define FSP_MSC_RESERVED_18				0x00040000      /**< Reserved */
#define FSP_MSC_RESERVED_19				0x00080000      /**< Reserved */
#define FSP_MSC_RESERVED_20				0x00100000      /**< Reserved */
#define FSP_MSC_RESERVED_21				0x00200000      /**< Reserved */
#define FSP_MSC_RESERVED_22				0x00400000      /**< Reserved */
#define FSP_MSC_RESERVED_23				0x00800000      /**< Reserved */
#define FSP_MSC_RESERVED_24				0x01000000      /**< Reserved */
#define FSP_MSC_RESERVED_25				0x02000000      /**< Reserved */
#define FSP_MSC_RESERVED_26				0x04000000      /**< Reserved */
#define FSP_MSC_RESERVED_27				0x08000000      /**< Reserved */
#define FSP_MSC_RESERVED_28				0x10000000      /**< Reserved */
#define FSP_MSC_RESERVED_29				0x20000000      /**< Reserved */
#define FSP_MSC_RESERVED_30				0x40000000      /**< Reserved */
#define FSP_MSC_RESERVED_31				0x80000000      /**< Reserved */

/*****************************************************************************************************************
 * CIRCULATION PUMP PARAMETERS ***********************************************************************************
 *****************************************************************************************************************/

/*---------------------------------------------------------------------------------------------------------------------
 * Motor module configuration
 *--------------------------------------------------------------------------------------------------------------------- */

/**
    \brief Max. number of restart attempts of the CP
*/
#define GENPAR_MOT_MAX_RESTART_ATTEMPTS_M2       (5)

/**
 * \brief	Acknowledged system errors
 * 			Error list with all errors that are acknowledged by the motor
 * 			module before a motor ist started.
 * */
#define GENPAR_MOT_ACK_SYSTEM_ERRORS_M2			(ERM_SYS_OVERCURRENT_EVENT_BRIDGE1)

/**
 * \brief	Acknowledged motor errors
 * 			Error list with all errors that are acknowledged by the motor
 * 			module before a motor ist started.
 * */
#define GENPAR_MOT_ACK_MOTOR_ERRORS_M2			(ERM_MOT_CONTROL_LOST_SYNCH)

/*---------------------------------------------------------------------------------------------------------------------
 * Vector module configuration
 *--------------------------------------------------------------------------------------------------------------------- */

/*****************************************************************************************************************
 * DRAIN PUMP PARAMETERS *****************************************************************************************
 *****************************************************************************************************************/

/*---------------------------------------------------------------------------------------------------------------------
 * Motor module configuration
 *--------------------------------------------------------------------------------------------------------------------- */

/**
    \brief Max. number of restart attempts of the DP
*/
#define GENPAR_MOT_MAX_RESTART_ATTEMPTS_M1       (15)

/**
 * \brief	Acknowledged system errors
 * 			Error list with all errors that are acknowledged by the motor
 * 			module before a motor ist started.
 * */
#define GENPAR_MOT_ACK_SYSTEM_ERRORS_M1			(ERM_SYS_OVERCURRENT_EVENT_BRIDGE1)

/**
 * \brief	Acknowledged motor errors
 * 			Error list with all errors that are acknowledged by the motor
 * 			module before a motor ist started.
 * */
#define GENPAR_MOT_ACK_MOTOR_ERRORS_M1			(ERM_MOT_CONTROL_LOST_SYNCH)

/*---------------------------------------------------------------------------------------------------------------------
 * Vector module configuration
 *--------------------------------------------------------------------------------------------------------------------- */

/*****************************************************************************************************************
 * SELFCHECK PARAMETERS ******************************************************************************************
 *****************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------------------*/
/*	Line test parameters																						*/
/*--------------------------------------------------------------------------------------------------------------*/

/**
    @brief Maximum test time of the linetest

     If the linetest takes longer than this time, the motor selfcheck module
     generates a timeout error. This case only happens if there is a malfunction
     of the internal software, i.e. a MCU fault (e.g. RAM/ROM/STACK/...)
\n\n
     <b>PARAMETER DATA:</b>
     - <b>Valid value range:</b>  0-65000000
     - <b>Physical value:</b>     3
     - <b>Physical unit:</b>      s
     - <b>Internal SW unit:</b>   100ns
     - <b>Module file name:</b>   LineTest.c

     <b>CHANGE HISTORY:</b>
     - 24.06.2009 Initial Revision
*/
#define FSP_LNT_TEST_TIMEOUT			(STK_DEF_TIME_MS(3000))

/**
 * @brief Cycle test time of the linetest
 *
 * 	Self-check cycle time. If a self-check is triggers
 * 	while this time has not elapsed, the self-check
 * 	is not executed again. Instead the old results are
 * 	still valid.
\n\n
     <b>PARAMETER DATA:</b>
     - <b>Valid value range:</b>  0-65000000
     - <b>Physical value:</b>     3
     - <b>Physical unit:</b>      s
     - <b>Internal SW unit:</b>   100ns
     - <b>Module file name:</b>   LineTest.c

     <b>CHANGE HISTORY:</b>
     - 24.06.2009 Initial Revision
*/
#define FSP_LNT_TEST_CYCLE			(STK_DEF_TIME_MS(2000))

/**
     @brief Number of test repetitions in case of an error

     If the line test fails with each test attempt, an error is generated by
     the self check module. If a test succeeded, the test is assumed as passed
     and no more repetitions are triggered.
\n\n
     <b>PARAMETER DATA:</b>
     - <b>Valid value range:</b>  0-255
     - <b>Physical value:</b>     3
     - <b>Physical unit:</b>      n.a.
     - <b>Internal SW unit:</b>   n.a.
     - <b>Module file name:</b>   LineTest.c

     <b>CHANGE HISTORY:</b>
     - 24.06.2009 Initial Revision
*/
#define FSP_LNT_TEST_REPETITIONS         (4)

/**
	\brief	Max. Measurable phase resistance value in unit [Ohm]

	\note	A macro is used to define this value in unit [Ohm]. The internal
			working value might have a different format/unit.
	\note	Take care when setting this value because it depends on the measurable
			values of the DC link voltage and the phase current! Check the
			internal numbers to prevent overflows!

\n\n
	<b>PARAMETER DATA:</b>
	- <b>Valid value range:</b>
	- <b>Physical value:</b>
	- <b>Physical unit:</b>      n.a.
	- <b>Internal SW unit:</b>   n.a.
	- <b>Module file name:</b>   ResMeas.c

	 <b>CHANGE HISTORY:</b>
	- 31.01.2012 Initial Revision
*/
#define FSP_RSM_RESISTANCE_LIMIT      (100)  //\ todo: set max winding resistance

#if (FSP_LNT_TEST_REPETITIONS > 4)
#error "CHECK LINE TEST PARAMETERS: Number of test repetitions seems critical!"
#endif

/*@}*/

/**
 * \defgroup LNTCP Linetest Parameters CP
*/
/*@{*/

/**
	@brief Maximum line resistance in unit [Ohm]

	The measured resistance includes all resistances in series with the motor
	phase resistance during the measurement. These are:
	\li ESR of the DC link capacitor
	\li MOSFET Rdson
	\li Relay contact resistance
	\li Wire harness line resistance
	\li Motor connector resistance

	Therefore the measured resistance is typically too high and a reserve of
	about 8-10 Ohms should be included in the parameter to prevent spurious
	error detections.

	\note A macro is used to configure the resistance because the resistance
	 	 value might internally be handled in another format.
\n\n
	<b>PARAMETER DATA:</b>
	- <b>Valid value range:</b>  0-4095
	- <b>Physical value:</b>     4095
	- <b>Physical unit:</b>      n.a.
	- <b>Internal SW unit:</b>   n.a.
	- <b>Module file name:</b>   LineTest.c

	<b>CHANGE HISTORY:</b>
	- 31.01.2012 Initial Revision
*/
#define FSP_LNT_RMAX_M2       (RESISTANCE_OHM(185.0))

/**
	@brief Minimum line resistance

	The measured resistance includes all resistances in series with the motor
	phase resistance during the measurement. These are:
	\li ESR of the DC link capacitor
	\li MOSFET Rdson
	\li Relay contact resistance
	\li Wire harness line resistance
	\li Motor connector resistance

	Therefore the measured resistance is typically too high and a reserve of
	about 8-10 Ohms should be included in the parameter to prevent spurious
	error detections.

	\note A macro is used to configure the resistance because the resistance
	 	 value might internally be handled in another format.
\n\n
	<b>PARAMETER DATA:</b>
	- <b>Valid value range:</b>  0-4095
	- <b>Physical value:</b>     4095
	- <b>Physical unit:</b>      n.a.
	- <b>Internal SW unit:</b>   n.a.
	- <b>Module file name:</b>   LineTest.c

	<b>CHANGE HISTORY:</b>
	- 31.01.2012 Initial Revision
*/
#define FSP_LNT_RMIN_M2       (RESISTANCE_OHM(15.0))

/**
	@brief Maximum asymmerty resistance

	The measured resistance includes all resistances in series with the motor
	phase resistance during the measurement. These are:
	\li ESR of the DC link capacitor
	\li MOSFET Rdson
	\li Relay contact resistance
	\li Wire harness line resistance
	\li Motor connector resistance

	Therefore the measured resistance is typically too high and a reserve of
	about 8-10 Ohms should be included in the parameter to prevent spurious
	error detections.

	\note A macro is used to configure the resistance because the resistance
	 	 value might internally be handled in another format.
\n\n
	<b>PARAMETER DATA:</b>
	- <b>Valid value range:</b>  0-4095
	- <b>Physical value:</b>     4095
	- <b>Physical unit:</b>      n.a.
	- <b>Internal SW unit:</b>   n.a.
	- <b>Module file name:</b>   LineTest.c

	<b>CHANGE HISTORY:</b>
	- 31.01.2012 Initial Revision
*/
#define FSP_LNT_RASYM_M2       (RESISTANCE_OHM(10.0))

/*@}*/

/**
 * \defgroup LNTDP Linetest Parameters DP
*/
/*@{*/

/**
	@brief Maximum line resistance

	The measured resistance includes all resistances in series with the motor
	phase resistance during the measurement. These are:
	\li ESR of the DC link capacitor
	\li MOSFET Rdson
	\li Relay contact resistance
	\li Wire harness line resistance
	\li Motor connector resistance

	Therefore the measured resistance is typically too high and a reserve of
	about 8-10 Ohms should be included in the parameter to prevent spurious
	error detections.

	\note A macro is used to configure the resistance because the resistance
	 	 value might internally be handled in another format.
\n\n
	<b>PARAMETER DATA:</b>
	- <b>Valid value range:</b>  0-4095
	- <b>Physical value:</b>     4095
	- <b>Physical unit:</b>      n.a.
	- <b>Internal SW unit:</b>   n.a.
	- <b>Module file name:</b>   LineTest.c

	<b>CHANGE HISTORY:</b>
	- 31.01.2012 Initial Revision
*/
#define FSP_LNT_RMAX_M1       (RESISTANCE_OHM(185.0))

/**
	@brief Minimum line resistance

	The measured resistance includes all resistances in series with the motor
	phase resistance during the measurement. These are:
	\li ESR of the DC link capacitor
	\li MOSFET Rdson
	\li Relay contact resistance
	\li Wire harness line resistance
	\li Motor connector resistance

	Therefore the measured resistance is typically too high and a reserve of
	about 8-10 Ohms should be included in the parameter to prevent spurious
	error detections.

	\note A macro is used to configure the resistance because the resistance
	 	 value might internally be handled in another format.
\n\n
	<b>PARAMETER DATA:</b>
	- <b>Valid value range:</b>  0-4095
	- <b>Physical value:</b>     4095
	- <b>Physical unit:</b>      n.a.
	- <b>Internal SW unit:</b>   n.a.
	- <b>Module file name:</b>   LineTest.c

	<b>CHANGE HISTORY:</b>
	- 31.01.2012 Initial Revision
*/
#define FSP_LNT_RMIN_M1       (RESISTANCE_OHM(15.0))

/**
	@brief Maximum asymmerty resistance

	The measured resistance includes all resistances in series with the motor
	phase resistance during the measurement. These are:
	\li ESR of the DC link capacitor
	\li MOSFET Rdson
	\li Relay contact resistance
	\li Wire harness line resistance
	\li Motor connector resistance

	Therefore the measured resistance is typically too high and a reserve of
	about 8-10 Ohms should be included in the parameter to prevent spurious
	error detections.

	\note A macro is used to configure the resistance because the resistance
	 	 value might internally be handled in another format.
\n\n
	<b>PARAMETER DATA:</b>
	- <b>Valid value range:</b>  0-4095
	- <b>Physical value:</b>     4095
	- <b>Physical unit:</b>      n.a.
	- <b>Internal SW unit:</b>   n.a.
	- <b>Module file name:</b>   LineTest.c

	<b>CHANGE HISTORY:</b>
	- 31.01.2012 Initial Revision
*/
#define FSP_LNT_RASYM_M1       (RESISTANCE_OHM(10.0))

/*---------------------------------------------------------------------------------------------------------------------
 * Vector module configuration
 *--------------------------------------------------------------------------------------------------------------------- */

/*****************************************************************************************************************
 * GENERIC PARAMETERS (NOT RELATED TO A MOTOR) *******************************************************************
 *****************************************************************************************************************/

/*---------------------------------------------------------------------------------------------------------------------
 * HW Timer and modulator configuration -------------------------------------------------------------------------------
 *--------------------------------------------------------------------------------------------------------------------- */

/**
 * \brief	CPU core clock frequency in unit [MHz]
 * */
#define GENPAR_CPU_CORE_CLOCK_MHZ		(96.0)

/*---------------------------------------------------------------------------------------------------------------------
 * HW Timer and modulator configuration -------------------------------------------------------------------------------
 *--------------------------------------------------------------------------------------------------------------------- */

/**
 * \brief	Motor control PWM frequency in unit [Hz]
 * */
#define GENPAR_MOTOR_PWM_FREQUENCY_HZ	(16000.0)

/**
 * \brief	PWM switching dead time of IPM channel 1 in unit [us]
 * */
#define GENPAR_DEADTIME_IPM_CH1_US	(1.0)

/**
 * \brief	PWM switching dead time of IPM channel 2 in unit [us]
 * */
#define GENPAR_DEADTIME_IPM_CH2_US	(1.0)

/**
 * 	\brief	Minimum output duty cycle
 *
 * 	This value is used to limit the output duty cycle to some min. limit to prevent
 * 	PWM output pulses below a certain pulse width.
 * 	Background of this limitation is the occurrence of latch errors with some IPM types.
 * */
#define GENPAR_MIN_DUTY_CYCLE		(Q15(0.015))

/**
 * \brief	FOC control cycle time in unit [us]
 *
 * \note	The FOC control is executed every 2nd PWM cycle
 * */
#define GENPAR_FOC_CYCLE_TIME_US	((1e6 * 2.0) / GENPAR_MOTOR_PWM_FREQUENCY_HZ)

#define MPM_PWM_COMP_TIM           50  /*  50 * 13,89ns = 0,69us */
#define MPM_PWM_MIN_TIME           75  /*  75 * 13,89ns = 1,04us */

#define MPM_OBR_PWM_LIMIT_FACTOR   FP(0.95*0.866)             /* Limit pwm = safty_factor * cos(30) */
#define MPM_OBR_PWM_LIM_FAC_SQUARE FP(0.95*0.866*0.95*0.866)  /* square of limit factor             */

/*---------------------------------------------------------------------------------------------------------------------
 * Master configuration ----------------------------------------------------------------------------------------------
 *--------------------------------------------------------------------------------------------------------------------- */

/**
 *	\brief	CS error timeout after warning error occur
 *			Set: 15s
 */
#define GENPAR_MAS_CS_ERROR_TIMEOUT		STK_DEF_TIME_MS(12000)

/**
 * \brief
 */
#define	GENPAR_MAS_ERROR_TIMEOUT		STK_DEF_TIME_MS(20000)

/**
 * \brief
 */
#define	GENPAR_MAS_WDG_RESTART			GENPAR_MTA_WDG_RESTART

/**
 * \brief
 */
#define GENPAR_MAS_IDLE_TIME 			STK_DEF_TIME_MS(150)

/**
 * \brief	Init delay time before enter master functions
 */
#define GENPAR_MAS_INIT_TIME 			STK_DEF_TIME_MS(150)

/**
 *
 */
#define GENPAR_MAS_RUN_OUT				STK_DEF_TIME_MS(2500)

/*---------------------------------------------------------------------------------------------------------------------
 * MTA configuration ----------------------------------------------------------------------------------------------
 *--------------------------------------------------------------------------------------------------------------------- */
/**
 * \brief
 */
#define GENPAR_MTA_WDG_RESTART 			STK_DEF_TIME_MS(3000)

/**
 * \brief
 */
#define GENPAR_MTA_HWCHECK_TIMEOUT		STK_DEF_TIME_MS(6000)

/**
 * \brief
 */
#define GENPAR_MTA_INIT_TASK_TIME		STK_DEF_TIME_MS(200)

#if (MPM_SYESTEM_DELAY == OPTION_ACTIVE)
#define GENPAR_DELAY_TIME						STK_DEF_TIME_MS(1000)
#endif // MPM_SYESTEM_DELAY

/*---------------------------------------------------------------------------------------------------------------------
 * fsMotRelay configuration -------------------------------------------------------------------------------------------
 *--------------------------------------------------------------------------------------------------------------------- */
#define GENPAR_SWITCH_TIME_CH1			STK_DEF_TIME_MS(5)		/**< Motor selection relay switching time of channel 1 */
#define GENPAR_SWITCH_TIME_CH2			STK_DEF_TIME_MS(5)		/**< Motor selection relay switching time of channel 2 */

/**
 * \brief	Actor mapping of motor selection channel 1
 * */
#define GENPAR_ACTOR_MAPPING_CH1		(0)

/**
 * \brief	Actor mapping of motor selection channel 2
 * */
#define GENPAR_ACTOR_MAPPING_CH2		(LOG_MOTOR_RELAY_PHASE_U | LOG_MOTOR_RELAY_PHASE_V)

/*---------------------------------------------------------------------------------------------------------------------
 * ResMeas configuration ----------------------------------------------------------------------------------------------
 *--------------------------------------------------------------------------------------------------------------------- */

/**
 * \brief	Per unit to 0.1Ohm conversion value
 *
 * 			This value is used to convert the measured resistances from
 * 			per unit format to the real unit [0.1Ohm]
 * */
#define GENPAR_RSM_PU_TO_10OHM_CONV_VALUE		(Q0((REFPAR_REFERENCE_RESISTANCE_OHM*10.0) + 0.5))

/*---------------------------------------------------------------------------------------------------------------------
 * fsLineTest configuration -------------------------------------------------------------------------------------------
 *--------------------------------------------------------------------------------------------------------------------- */

/*
 * CIRCULATION PUMP VALUES ----------------------------------------------------
 * */

/**
 * \brief	Line test resistance measurement phase 1 M2
 * */
#define GENPAR_LNT_TEST_PHASE_1_M2		RSM_PATH_WH_UL

/**
 * \brief	Line test resistance measurement phase 2 CP
 * */
#define GENPAR_LNT_TEST_PHASE_2_M2		RSM_PATH_WH_VL

/**
 * \brief	Line test resistance measurement phase 3 M2
 * */
#define GENPAR_LNT_TEST_PHASE_3_M2		RSM_PATH_UH_VL

/**
 * \brief	Output voltage used for resistance measurement in unit [V] M2
 * */
#define GENPAR_LNT_MEAS_VOLTAGE_M2_V	(35.0)  //35

/**
 * \brief	Settling time before resistance measurement is started in unit [ms] M2
 * */
#define GENPAR_LNT_SETTLING_TIME_M2_MS	(150.0)

/**
 * \brief	No. of voltage and current samples taken for averaging CP
 * */
#define GENPAR_LNT_NO_OF_SAMPLES_M2		(400)

/*
 * DRAIN PUMP VALUES ----------------------------------------------------------
 * */

/**
 * \brief	Line test resistance measurement phase 1 M1
 * */
#define GENPAR_LNT_TEST_PHASE_1_M1		RSM_PATH_WH_UL

/**
 * \brief	Line test resistance measurement phase 2 DP
 * */
#define GENPAR_LNT_TEST_PHASE_2_M1		RSM_PATH_WH_VL

/**
 * \brief	Line test resistance measurement phase 3 DP
 * */
#define GENPAR_LNT_TEST_PHASE_3_M1		RSM_PATH_UH_VL

/**
 * \brief	Output voltage used for resistance measurement in unit [V] DP
 * */
#define GENPAR_LNT_MEAS_VOLTAGE_M1_V	(35.0)

/**
 * \brief	Settling time before resistance measurement is started in unit [ms] M1
 * */
#define GENPAR_LNT_SETTLING_TIME_M1_MS	(150.0)

/**
 * \brief	No. of voltage and current samples taken for averaging CP
 * */
#define GENPAR_LNT_NO_OF_SAMPLES_M1		(400)

/*---------------------------------------------------------------------------------------------------------------------
 * fsOCTest configuration ---------------------------------------------------------------------------------------------
 *--------------------------------------------------------------------------------------------------------------------- */

/**
 *	\brief	Test time of one over-current trap (half-bridge) => Output channel 1
 *
 *			The over-current event must occur within at least this time during the test.
 *			if it does not the self-check fails
 * */
#define GENPAR_OCT_TEST_TIME_CH1			(STK_DEF_TIME_MS(10))

/**
 *	\brief	Test time of one over-current trap (half-bridge) => Output channel 2
 *
 *			The over-current event must occur within at least this time during the test.
 *			if it does not the self-check fails
 * */
#define GENPAR_OCT_TEST_TIME_CH2			(STK_DEF_TIME_MS(10))

/**
 *	\brief	Current relaxation time	=> Output channel 1
 *
 * 			After each test step this time is inserted without applying a voltage to
 * 			the motor to allow the high motor phase current to decrease to zero. This
 * 			takes some time due to the inductivity of the motor coils.
 * */
#define GENPAR_OCT_RELAX_TIME_CH1			(STK_DEF_TIME_MS(2))

/**
 *	\brief	Current relaxation time	=> Output channel 2
 *
 * 			After each test step this time is inserted without applying a voltage to
 * 			the motor to allow the high motor phase current to decrease to zero. This
 * 			takes some time due to the inductivity of the motor coils.
 * */
#define GENPAR_OCT_RELAX_TIME_CH2			(STK_DEF_TIME_MS(4))    ////2


#define GENPAR_OCT_VOLTAGE_TIME_CH2			(STK_DEF_TIME_MS(1))    ////2

/**
 * 	\brief	Motor selector position => Output channel 1
 *
 * 			The configuration should be set to the motor with the lowest phase resistance
 * */
#define GENPAR_OCT_SEL_POS_CH1				MIF_SELECTOR_POS_1

/**
 * 	\brief	Motor selector position => Output channel 2
 *
 * 			The configuration should be set to the motor with the lowest phase resistance
 * */
#define GENPAR_OCT_SEL_POS_CH2				MIF_SELECTOR_POS_2

/**
 * 	\brief	Motor bridge to motor mapping
 *
 * 			Selection from MOB_tHwMapping<CP,DP>[] arrayes in MotorObj.c
 * */

#define BRIDGE_TO_MOTOR_MAPPING				(0)

#endif	/* __GENPAR_H */
