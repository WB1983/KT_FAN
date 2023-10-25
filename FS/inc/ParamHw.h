#ifndef __HWPAR_CH_H
#define __HWPAR_CH_H
/****************************************************************************************************************
 * INCLUDES *****************************************************************************************************
*****************************************************************************************************************/
#include "compile.h"
/****************************************************************************************************************
 * PARAMETER DEFINITIONS ****************************************************************************************
*****************************************************************************************************************/

/**
    \defgroup HW_PARAMETERSET HW Parameterset (EU variant)
    \{
*/
/*-------------------------------------------------------------------------------------------------
 * MAINS VOLTAGE PARAMETERS
 *------------------------------------------------------------------------------------------------- */
/**
    \defgroup VOL MAINS VOLTAGE PARAMETERS
    \{
*/
/**
     \brief Line voltage frequency


     Definition of the line voltage frequency. This value is the typical frequency
     of the line voltage. The value can be
     \li 50Hz => Europe
     \li 60Hz => US
\n\n
     <b>PARAMETER DATA:</b>
     - <b>Valid value range:</b>  50 or 60
     - <b>Physical value:</b>     50
     - <b>Physical unit:</b>      Hz
     - <b>Internal SW unit:</b>   Hz
     - <b>Module file name:</b>   Voltage.c

     <b>CHANGE HISTORY:</b>
     - 04.08.2009 Initial Revision

*/
#define PAR_LINE_VOLTAGE_FREQUENCY_HZ         (50.0)

/*-------------------------------------------------------------------------------------------------
 * AD CONVERTER PARAMETERS
 *------------------------------------------------------------------------------------------------- */

/**
     \brief AD converter reference voltage

     This voltage represents the theoretically measurable max. anolog voltage
     value. The value depends on the microcontroller and the HW design.
\n\n
     <b>PARAMETER DATA:</b>
     - <b>Valid value range:</b>  0.0 ... 5.0V
     - <b>Physical value:</b>     3.3
     - <b>Physical unit:</b>      V
     - <b>Internal SW unit:</b>   V
     - <b>Module file name:</b>   n.a.

     <b>CHANGE HISTORY:</b>
     - 06.04.2011 Initial Revision
*/
#define PAR_ADC_REF_VOLTAGE             (5.0)

/**
     \brief AD converter resolution in no. of bits
\n\n
     <b>PARAMETER DATA:</b>
     - <b>Valid value range:</b>  12
     - <b>Physical value:</b>     12
     - <b>Physical unit:</b>      n.a.
     - <b>Internal SW unit:</b>   n.a.
     - <b>Module file name:</b>   n.a.

     <b>CHANGE HISTORY:</b>
     - 06.04.2011 Initial Revision
*/
#define PAR_ADC_RESOLUTION_BITS         (12)

/*-------------------------------------------------------------------------------------------------
 * DC LINK VOLTAGE MEASUREMENT HW PARAMETERS
 *------------------------------------------------------------------------------------------------- */

/**
     \brief DC Link voltage measurement pullup resistor value.
\n\n
     <b>PARAMETER DATA:</b>
     - <b>Valid value range:</b>  2*220k
     - <b>Physical value:</b>     2*220k
     - <b>Physical unit:</b>      Ohm
     - <b>Internal SW unit:</b>   Ohm
     - <b>Module file name:</b>   n.a.

     <b>CHANGE HISTORY:</b>
     - 06.04.2011 Initial Revision
*/


#define PAR_DCL_VOLT_DIV_PU_RES         (800.0e3)

/**
     \brief 13.5 voltage measurement pullup resistor value.
\n\n
     <b>PARAMETER DATA:</b>
     - <b>Valid value range:</b>  39k
     - <b>Physical value:</b>     39k
     - <b>Physical unit:</b>      Ohm
     - <b>Internal SW unit:</b>   Ohm
     - <b>Module file name:</b>   n.a.

     <b>CHANGE HISTORY:</b>
     - 09.01.2013 Initial Revision
*/
#define PAR_15_VOLT_DIV_PU_RES		(10e3)

/**
     \brief DCL voltage measurement pulldown resistor value.

\n\n
     <b>PARAMETER DATA:</b>
     - <b>Valid value range:</b>  3.3k
     - <b>Physical value:</b>     3.3k
     - <b>Physical unit:</b>      Ohm
     - <b>Internal SW unit:</b>   Ohm
     - <b>Module file name:</b>   n.a.

     <b>CHANGE HISTORY:</b>
     - 06.04.2011 Initial Revision
*/
#define PAR_DCL_VOLT_DIV_PD_RES         (4.7e3)

/**
     \brief 13.5 voltage measurement pulldown resistor value.

\n\n
     <b>PARAMETER DATA:</b>
     - <b>Valid value range:</b>  10k
     - <b>Physical value:</b>     10k
     - <b>Physical unit:</b>      Ohm
     - <b>Internal SW unit:</b>   Ohm
     - <b>Module file name:</b>   n.a.

     <b>CHANGE HISTORY:</b>
     - 09.01.2013 Initial Revision
*/
#define PAR_15_VOLT_DIV_PD_RES		(2e3)

/**
     \brief DCL voltage measurement resistor divider ratio

     Ratio = Rpd / (Rpd + Rpu)
\n\n
     <b>PARAMETER DATA:</b>
     - <b>Valid value range:</b>  
     - <b>Physical value:</b>     
     - <b>Physical unit:</b>      none
     - <b>Internal SW unit:</b>   none
     - <b>Module file name:</b>   n.a.

     <b>CHANGE HISTORY:</b>
     - 06.04.2011 Initial Revision
*/
#define PAR_DCL_VOLT_DIV_RATIO          (PAR_DCL_VOLT_DIV_PD_RES/(PAR_DCL_VOLT_DIV_PD_RES + PAR_DCL_VOLT_DIV_PU_RES))

/**
     \brief 13.5 voltage measurement resistor divider ratio

     Ratio = Rpd / (Rpd + Rpu)
\n\n
     <b>PARAMETER DATA:</b>
     - <b>Valid value range:</b>
     - <b>Physical value:</b>
     - <b>Physical unit:</b>      none
     - <b>Internal SW unit:</b>   none
     - <b>Module file name:</b>   n.a.

     <b>CHANGE HISTORY:</b>
     - 06.04.2011 Initial Revision
*/
#define PAR_15_VOLT_DIV_RATIO          (PAR_15_VOLT_DIV_PD_RES/(PAR_15_VOLT_DIV_PD_RES + PAR_15_VOLT_DIV_PU_RES))

/**
     \brief Max. measurable DCL voltage value

     This voltage is defined by HW design, i.e. by the resistor divider and the
     AD reference voltage.
\n\n
     <b>PARAMETER DATA:</b>
     - <b>Valid value range:</b>  0-inf.
     - <b>Physical value:</b>     443
     - <b>Physical unit:</b>      V
     - <b>Internal SW unit:</b>   V
     - <b>Module file name:</b>   ParamRef.h

     <b>CHANGE HISTORY:</b>
     - 06.04.2011 Initial Revision
*/
#define PAR_DCL_VOLT_MAX_V				(((PAR_ADC_REF_VOLTAGE / PAR_DCL_VOLT_DIV_RATIO) + 0.5))

/**
     \brief Max. measurable 13.5 voltage value

     This voltage is defined by HW design, i.e. by the resistor divider and the
     AD reference voltage.
\n\n
     <b>PARAMETER DATA:</b>
     - <b>Valid value range:</b>  0-inf.
     - <b>Physical value:</b>     16669
     - <b>Physical unit:</b>      mV
     - <b>Internal SW unit:</b>   mV
     - <b>Module file name:</b>   ParamRef.h

     <b>CHANGE HISTORY:</b>
     - 06.04.2011 Initial Revision
*/
#define PAR_15_VOLT_MAX_MV			(((PAR_ADC_REF_VOLTAGE / PAR_15_VOLT_DIV_RATIO) + 0.5)*1000)

/**
     \brief internal voltage reference

     This voltage is defined by HW design, i.e. by the resistor divider and the
     AD reference voltage.
\n\n
     <b>PARAMETER DATA:</b>
     - <b>Valid value range:</b>  0-inf.
     - <b>Physical value:</b>     16669
     - <b>Physical unit:</b>      mV
     - <b>Internal SW unit:</b>   mV
     - <b>Module file name:</b>   ParamRef.h

     <b>CHANGE HISTORY:</b>
     - 06.04.2011 Initial Revision
*/
#define PAR_INTERNAL_VOL_REF_V			Q12(1.23)
/**\}*/ //end of group VOL MAINS VOLTAGE PARAMETERS

/*-------------------------------------------------------------------------------------------------
 * CURRENT MEASUREMENT PARAMETERS
 *------------------------------------------------------------------------------------------------- */
/**
    \defgroup CUR CURRENT MEASUREMENT PARAMETERS
    \{

*/
/**
    \defgroup CUR_DAC DAC value settings
    \{
*/

/** \brief Over current DAC value TIM 8 = Motor 2 = Comparators 1,2,3 \n
	Due to hw layout the DAC value used at inverted comparator input to set the over current threshold
	is empiric measured and determined by the following equation:\n

	DAC = [Uoff+Ulimit_run]/[(Vref/DACmax)]\n
	DAC = [(0.5+0.64)]/[(3.3/4096)]\n
	DAC = 1411.88 ~= 1412

	<b>Ferite:</b>\n\n

		- <b>PARAMETER DATA:</b> 1.5A
		- <b>Empiric DAC value:</b>
		- <b>CHANGE HISTORY:</b>
		- 23.05.2012 Initial Revision
*/

#define PAR_OC_DAC_VAL_TIM8_RUN_M2		1412

/** \brief Over current DAC value TIM 1 = Motor 1 = Comparators 4,5,6 \n
	Due to hw layout the DAC value used at inverted comparator input to set the over current threshold
	is empiric measured and determined by the following equation:\n

	DAC = [Uoff+Ulimit_run]/[(Vref/DACmax)]\n
	DAC = [(0.5+0.64)]/[(3.3/4096)]\n
	DAC = 1411.88 ~= 1412

	<b>Ferite:</b>\n\n

		- <b>PARAMETER DATA:</b> 1.5A
		- <b>Empiric DAC value:</b>
		- <b>CHANGE HISTORY:</b>
		- 23.05.2012 Initial Revision
*/

#define PAR_OC_DAC_VAL_TIM1_RUN_M1		1412

/** \brief Over current DAC value TIM 8 = Motor 2 = Comparators 4,5,6 \n
	Due to hw layout the DAC value used at inverted comparator input to set the over current threshold
	is empiric measured and determined by the following equation:\n

	DAC = [Uoff+Ulimit_stop]/[(Vref/DACmax)]\n
	DAC = [(0.5+0.212)]/[(3.3/4096)]\n
	DAC = 883.74 ~= 884

	DAC value is decreased during Over current test, to avoid noise produced by OC test.

     - <b>PARAMETER DATA:</b> 500mA
     - <b>Empiric DAC value:</b>
     - <b>CHANGE HISTORY:</b>
     - 23.05.2012 Initial Revision
*/

#define PAR_OC_DAC_VAL_TIM8_STOP_M2		400

/** \brief Over current DAC value TIM 8 = Motor 2 = Comparators 1,2,3 \n
	Due to hw layout the DAC value used at inverted comparator input to set the over current threshold
	is empiric measured and determined by the following equation:\n

	DAC = [Uoff+Ulimit_stop]/[(Vref/DACmax)]\n
	DAC = [(0.5+0.212)]/[(3.3/4096)]\n
	DAC = 883.74 ~= 884

	DAC value is decreased during Over current test, to avoid noise produced by OC test.

     - <b>PARAMETER DATA:</b> 500mA
     - <b>Empiric DAC value:</b>
     - <b>CHANGE HISTORY:</b>
     - 23.05.2012 Initial Revision
*/

#define PAR_OC_DAC_VAL_TIM1_STOP_M1		920

/** \brief Over current DAC value TIM 8 = Motor 2 = Comparators 4,5,6 for Overcurrent test\n
*/

#define PAR_OC_DAC_VAL_TIM8_STOP_M2_OC_TEST		400


/** \brief Over current DAC value TIM 8 = Motor 2 = Comparators 1,2,3 for Overcurrent test\n
*/

#define PAR_OC_DAC_VAL_TIM8_STOP_M1_OC_TEST		(420)

#define PAR_OC_DAC_VAL_TIM8_STOP_M2_OC_TEST_LOW		250

#define PAR_OC_DAC_VAL_TIM8_STOP_M2_OC_TEST_HIGH	500

#define PAR_OC_DAC_VAL_TIM8_COMP_M2_OC_TEST		320

#define PAR_OC_DAC_MAX_DUTY_COUNTER				(100)

#define PAR_OC_DAC_DUT_MULTIPLYER				(18)

#define PAR_OC_DAC_MAX_DUTY_DOWN				(750)
#define PAR_OC_DAC_MAX_DUTY_UP				(750)




/**\}*/ //end of group CUR_DAC DAC value settings

/*-------------------------------------------------------------------------------------------------
 * CURRENT MEASUREMENT HW PARAMETERS
 *------------------------------------------------------------------------------------------------- */

/**
    \brief Current measurement parameter and values
    - Base voltage      U = 3,3V
    - Amplifier:        v = (R3+R4)/R4
    - No load divider:  Uo = 3,3V*(R1/(R1+R2))
    - Shunt resistor:   Rsh
    - AD resulution     4096 = 12 bit
    - Offset resul.bits 9 bits (Offset in the middle !)
    - Volt.drop sec. :   Us = (Ad-OffsetAD)/4096*3,3V
    - Current prim.:    I  = v*Us/Rsh\n
CURRENT MEASURE. qFP:   I(QFP) = FPM_QxtoQy((AD-OFFSET_AD),9,FPM_GLOBAL_FP_FORMAT)\n
CURRENT CALCULATION mA: I(AD)  = FPM_QxtoQy(I(QFP),FPM_GLOBAL_FP_FORMAT,9) = (AD-OFFSET_AD)\n
                        I(A)   = (AD-OFFSET_AD)/4096*3,3V/v*1000/Rsh[mOhm]\n
                        I(mA)  = 1000 * I[A]
 */

#define PAR_CUR_MEAS_AD_BASE			11		/// Base bit position for cur. AD val.
#define PAR_CUR_MEAS_AD_RESOLUTION		4096	/// Base bit position for cur. AD val.
#define PAR_CUR_MEAS_BASE_VOLT			3.3		/// Base voltage at pull up resistor
#define PAR_CUR_MEAS_R1					5100.0	/// Amplifying resistor Amplifier
#define PAR_CUR_MEAS_R2					1000.0	/// Input resistor voltage divider
#define PAR_CUR_MEAS_R3					5100.0	/// Pull up resistor voltage divider
#define PAR_CUR_MEAS_R4					910.0	/// Pull down resistor voltage divider
#define PAR_CUR_MEAS_RSH				0.250	/// Shunt resistor (Ohm)
#define PAR_CUR_MEAS_MILIAMPS			1000.0	/// Conversion from A to mA
#define PAR_CUR_MEAS_ADC_ENH_RESOLUTION	8192	/// Enhanced ADC resolution

/* Redefine macros for simplicity equations */
#define R_1 PAR_CUR_MEAS_R1
#define R_2 PAR_CUR_MEAS_R2
#define R_3 PAR_CUR_MEAS_R3
#define R_4 PAR_CUR_MEAS_R4
#define RRS PAR_CUR_MEAS_RSH

#define CUR_OPAMP_GAIN		(1+(R_1/R_2))		/// Gain of operational amplifier 				TBD
												/// Overall gain of current measurement circuit	TBD
#define CUR_CIRCUIT_GAIN		((CUR_OPAMP_GAIN)*(R_3/(R_3+R_4)))

//need update
												/// Gain of current measurement circuit
#define PAR_CUR_MEAS_CONST_MA	Q12((PAR_CUR_MEAS_BASE_VOLT * PAR_CUR_MEAS_MILIAMPS)/(CUR_CIRCUIT_GAIN * PAR_CUR_MEAS_RSH * PAR_CUR_MEAS_ADC_ENH_RESOLUTION))
												/// Convert macro from AD value to current value

/**
 * \brief Minimum AD value for current offset\n
 * 1911 -> 1.54V
 * delta =(2048-1911) = 137
 * additional difference (20%) = 137* 1.2 = 165
 * ADC middle = 2048, => 2048 - 165 = 1883
 */
#define PAR_CUR_OFFSET_MIN_AD			1883

/**
 * \brief Maximum AD value for current offset\n
 * 2184 -> 1.75V
 * delta =(2048-1911) = 137
 * additional difference (20%) = 137* 1.2 = 165
 * ADC middle = 2048, => 2048 + 165 = 2213
 *
*/
#define PAR_CUR_OFFSET_MAX_AD			2213

/**\}*/ //end of group CUR CURRENT MEASUREMENT PARAMETERS
/**\}*/ //end of HW_PARAMETERSET HW Parameterset (EU variant)

/*
* NTC Tempeature limitation
*/
#define FTE_NTC_TEMPERATURE_THRESHOLD      100

/*
* NTC Temperature confirm count
*/
#define FTE_NTC_ERROR_CNT                  20

/*
* MCU Tempeature limitation
*/
#define FTE_INT_TEMPERATURE_HIGH_THRESHOLD      85

/*
* MCU Tempeature limitation
*/
#define FTE_INT_TEMPERATURE_LOW_THRESHOLD  -40

/*
* MCU Temperature confirm count
*/
#define FTE_INT_ERROR_CNT                  20


#endif
