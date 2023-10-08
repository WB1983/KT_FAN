/*********************************************************************************************************************
 * INCLUDES **********************************************************************************************************
 ********************************************************************************************************************/

#include "LibDefines.h"
#include "Transfer.h"
#include "Math.h"
#include "Current.h"		/* Include own header				*/
#include "Filter.h"
#include "fsClock.h"
#include "Voltage.h"
#include "board.h"
#include "ParamHw.h"
#include "CsParam.h"
#include "drv_inc.h"
#include "ParamRef.h"


/*********************************************************************************************************************
 * MODULE DEBUG, MODULE TEST AND INTERGARTION TEST INSTRUMENTATION ***************************************************
 ********************************************************************************************************************/
/* Compile switch for Voltage ripple calculation	*/
#define CALC_RIPPLE_VOLTAGE				OPTION_PASSIVE	//OPTION_ACTIVE	//

/*********************************************************************************************************************
 * LOCAL FUNCTION PROTOTYPES *****************************************************************************************
 ********************************************************************************************************************/
#if(CALC_RIPPLE_VOLTAGE == OPTION_ACTIVE)
static void VOL_vCalcDclVoltageRippleQ(void);
#endif
static void VOL_vCalcVolLowPassFilters(void);
static BOOL VOL_bCheckUnderVoltageCondition(void);
static BOOL VOL_bCheckOverVoltageCondition(void);
static void VOL_vCheckInternalReferenceVoltageCondition(void);

/*********************************************************************************************************************
 * LOCAL DEFINITIONS *************************************************************************************************
 ********************************************************************************************************************/

/**
 *	\brief	Module time base in safe timer tick units
 *	*/
#define VOL_TIMEBASE							STK_DEF_TIME_MS(100)

/**
 * 	\brief	Sampling frequency of the DC link and line voltage in unit [Hz]
 * 			Sampling time is set to 800	us -> f=1/t = 1250 Hz
 * 	*/
#define PAR_VOLTAGE_SAMPLING_FREQUENCY_HZ		(1250)

/**
 * 	\brief 	No. of voltage samples per line period
 * 	*/
#define PAR_VOLTAGE_SAMPLES_PER_LINE_PERIOD		(uint8_t)(((PAR_VOLTAGE_SAMPLING_FREQUENCY_HZ /		\
														  PAR_LINE_VOLTAGE_FREQUENCY_HZ) + 0.5))

/**
 *	\brief	Startup delay needed for module initialization
 *			After reset at least one complete line voltage period is needed to be
 *			sampled to be able to initialize the internal voltages
 */
#define VOL_INITIAL_ADC_DELAY					STK_DEF_TIME_MS(100)



/********************************************************************************************************************
 * LOCAL MACROS *****************************************************************************************************
 *******************************************************************************************************************/

/**
	\brief This macro converts a voltage in p.u. Qx format to a real voltage in unit [V]
	\param VolQx [fp_Qx] Voltage in p.u. format
	\param RefQ0 [fp_Q0] Reference voltage
*/
#define VOL_uiConvQxToRealVoltage(VolQx, RefQ0)			(((TFp)FPM_FpMul(VolQx, RefQ0)))

/********************************************************************************************************************
 * LOCAL CONSTANTS **************************************************************************************************
 *******************************************************************************************************************/

/* IIR_tCalcFilterConst(Tsample_ms, Tfilter_ms); */
												/**< Filter constant that is used to damp the DC link voltage		*/
static TFilterCoeff VOL_tVoltageFilterConst			= (TFilterCoeff)FIR_tCalcFilterConst(50.0, 200.0);
								 				/**< Filter constant that is used to damp the DC link peak voltage	*/
static TFilterCoeff VOL_tDclPeakVoltageFilterConst	= (TFilterCoeff)FIR_tCalcFilterConst(50.0, 1000.0);

/********************************************************************************************************************
 * LOCAL VARIABLES **************************************************************************************************
 *******************************************************************************************************************/

/* Input Samples */
static TFp VOL_uiDclVoltageAdValue;		/**< [AD-digits] Sample value of the DC link voltage AD conversion			*/
static TFp VOL_ui13_5VoltageValue;		/**< [AD-digits] Sample value of the 13.5 voltage AD conversion				*/
static TFp VOL_tVrefIntVolADValue;
/* Input samples in fixed point format */
static TFp	VOL_tDclVolSampleQ;			/**< [fp] Sample value of the DC link voltage AD conversion in Qx format	*/
static TFp	VOL_t13_5VolSampleQ;		/**< [fp] Sample value of the 13.5 link voltage AD conversion in Qx format	*/
static TFp  VOL_tVrefIntVolSampleQ;
static TFp	VOL_tDclVolPerSampleIntQ;	/**< [fp] Integrator used for DCL voltage integration over one mains period	*/
static TFp	VOL_t13_5VolPerSampleIntQ;	/**< [fp] Integrator used for 13.5 voltage integration over one mains period*/
static TFp	VOL_tVrefIntVolPerSampleIntQ;
static TFp	VOL_tDclVolPerSampleIntResultQ;		/**< [fp] DCL voltage integrated over one mains period				*/
static TFp	VOL_t13_5VolPerSampleIntResultQ;	/**< [fp] 13.5 voltage integrated over one mains period				*/
static TFp	VOL_tVrefIntVolPerSampleIntResultQ;
static TFp	VOL_tDclVoltageQ;					/**< [fp] DCL voltage averaged over one mains period				*/

static TFp	VOL_t13_5VoltageQ;					/**< [fp] 13.5 voltage averaged over one mains period				*/
static TFp  VOL_tVrefIntVolQ;
/* Min/max tracking values */
static TFp  VOL_tDclVolMaxTrackQ;			/**< [fp] Tracking variable: actual max. value of the DC link voltage	*/
static TFp  VOL_tDclVolMinTrackQ = (TFp)FP(1.0);/**< [fp] Tracking variable: actual min. value of the DC link voltage*/

/* Min/max values */
static TFp	VOL_tDclVolMaxQ;				/**< [fp] Max. value of the DC link voltage over the last period		*/
#if(CALC_RIPPLE_VOLTAGE == OPTION_ACTIVE)
static TFp	VOL_tDclVolMinQ;				/**< [fp] Min. value of the DC link voltage over the last period		*/
static TFp	VOL_tDclVolRippleQ;				/**< [fp] DC link voltage ripple over the last period					*/
#endif

/* Real voltage values */
static TFp	VOL_tDCLVoltageV;				/**< [V] Actually measured DC link voltage								*/
static TFp	VOL_t13_5VoltagemV;				/**< [V] Actually measured 13.5 voltage									*/
static TFp	VOL_tVrefIntVoltagemV;

static TFp VOL_uiDCLVoltagePeakValueV;		/**< [V] Actually measured DC link voltage peak value					*/
#if(CALC_RIPPLE_VOLTAGE == OPTION_ACTIVE)
static TFp VOL_uiDclVoltageRippleV;			/**< [V] DC link voltage ripple											*/
#endif

#if (MOTOR_VDD_OPTI == OPTION_ACTIVE)
static TFp VOL_tMaxVoltage;
static uint VOL_uiRealVddmV;
static BOOL VOL_bVddUpdated = FALSE;
static BOOL VOL_bVddRedOut = FALSE;
#endif
/* Low pass filters */
static TFilterData VOL_tDCLVoltageDmpdV;			/**< [V] Damped value of the DC link voltage					*/
static TFilterData VOL_tDCLPeakVoltageDmpdQ;		/**< [fp] Damped value of the DC link peak voltage				*/
static TFilterData VOL_tDCLPeakVoltageDmpdV;		/**< [V] Damped value of the DC link peak voltage				*/

/* State flags and other values */
									/**< Counter for tracking max. values of DCL and 13.5 voltages					*/
static uint32_t	VOL_ucPeriodSampleCtr = (uint32_t)PAR_VOLTAGE_SAMPLES_PER_LINE_PERIOD;
									/**< Flag is set when the control is in low-voltage mode and reset if not		*/
static BOOL		VOL_bLowVoltageMode = FALSE;
									/**< Flag is set when the control is in under-voltage mode and reset if not		*/
static BOOL		VOL_bUnderVoltageMode = FALSE;
									/**< Flag is set when the control is in overvoltage mode and reset if not		*/
static BOOL		VOL_bOverVoltageMode = FALSE;
									/**< Flag is set every time a new AD sample has been taken						*/
static BOOL		VOL_bVoltageSampleRefreshed = TRUE;
									/**< Flag is clear, when timer is loaded after blackout voltage occur			*/
static BOOL		VOL_bLowVoltageOK = FALSE;
									/**< Flag is clear, when timer is loaded after under voltage occur				*/
static BOOL		VOL_bUnderVoltageOK = FALSE;
									/**< Flag is clear, when timer is loaded after over voltage occur				*/
static BOOL		VOL_bOverVoltageOK = FALSE;

static TSafeTime VOL_tActualTimeLowVoltage;		/**< Actual time stamp for error release timer (Low voltage)		*/
static TSafeTime VOL_tActualTimeUnderVoltage;	/**< Actual time stamp for error release timer (Under-voltage)		*/
static TSafeTime VOL_tActualTimeOverVoltage;	/**< Actual time stamp for error release timer (Over-voltage)		*/

static TSafeTime VOL_tSafeTimeStamp = VOL_INIT_TIME_STAMP;	/**< Time stamp needed for time base generation 		*/
															/**< Need to init with higher value to avoid error		*/

/*********************************************************************************************************************
 * LOCAL FUNCTIONS ***************************************************************************************************
 ********************************************************************************************************************/

#if(CALC_RIPPLE_VOLTAGE == OPTION_ACTIVE)
/**
 *	\brief	Function calculates the DC link ripple voltage
 *
 *			The ripple voltage is calculated out of the locally found max and
 *			min DC link voltage values. The locally found max and min values
 *			are determined over one line voltage period.
 *	\return	void
 */
static void VOL_vCalcDclVoltageRippleQ(void)
	{
	TFp tDclVolMax;
	TFp tDclVolMin;

	/*
	 *	Sample max/min values in di() state to prevent interruption of the two copy instructions
	 */
	HSUP_vDisableInt();

	tDclVolMax = VOL_tDclVolMaxQ;
	tDclVolMin = VOL_tDclVolMinQ;

	HSUP_vEnableInt();

	if (tDclVolMax < tDclVolMin)
		{
		/*
		 * Max value is smaller than min value => ripple is about 0 => no load case
		 */
		VOL_tDclVolRippleQ = 0;
		}
	else
		{
		/*
		* Calculate ripple voltage
		* Ripple = max - min
		* */
		VOL_tDclVolRippleQ = tDclVolMax - tDclVolMin;
		}
	}
#endif

/**
 *	\brief	Function handles all low pass filters of the voltage module
 *
 *			This function is called in the modules time slice
 *	\return	void
 * */
static void VOL_vCalcVolLowPassFilters(void)
	{
	/* Low pass filter of the DC link voltage period AVERAGE value */
	VOL_tDCLVoltageDmpdV.tFilterInputValue = (TFp)VOL_tDCLVoltageV;
	FIR_vCalcFilter(&VOL_tDCLVoltageDmpdV, (TFilterCoeff*)(&VOL_tVoltageFilterConst));

	/* Low pass filter of the DC link voltage period PEAK value */
	VOL_tDCLPeakVoltageDmpdV.tFilterInputValue = (TFp)VOL_uiDCLVoltagePeakValueV;
	FIR_vCalcFilter(&VOL_tDCLPeakVoltageDmpdV, (TFilterCoeff*)(&VOL_tVoltageFilterConst));

	/* Low pass filter of the DC link voltage peak value */
	VOL_tDCLPeakVoltageDmpdQ.tFilterInputValue = VOL_tDclVolMaxQ;
	FIR_vCalcFilter(&VOL_tDCLPeakVoltageDmpdQ, (TFilterCoeff*)(&VOL_tDclPeakVoltageFilterConst));
	}

/**
 * 	\brief	Function checks whether there is a under voltage voltage condition present
 *
 * 	\return	BOOL Returns TRUE if error is detected, else FALSE
 */

static BOOL VOL_bCheckUnderVoltageCondition(void)
	{
	TFp			tVoltageValueV	=0;									/* Local store variable for voltage value		*/
	TSafeTime	tDeltaTime		=0;									/* Local store variable for time value			*/

	tDeltaTime = STK_tGetSafeTime();								/* Get actual time stamp						*/
	tDeltaTime-= VOL_tActualTimeUnderVoltage;						/* Calculate difference of time					*/
	tVoltageValueV = VOL_tGetDclVolV();								/* Store actual voltage level					*/

	if (tVoltageValueV > (CSP_CRITICAL_UNDERVOLTAGE_V + CSP_UNDERVOLTAGE_HYSTERESIS_V))
		{														/* Voltage mean value has risen above min. limit	*/
																/*+ hysteresis * => Recover from low voltage mode	*/
	
		VOL_bUnderVoltageMode = FALSE;								/* Reset return value							*/
		VOL_bUnderVoltageOK = TRUE;									/* Set auxiliary BOOL variable					*/
		}
	else if (tVoltageValueV < CSP_CRITICAL_UNDERVOLTAGE_V)			/* Voltage mean value is below min. limit		*/
		{
		if (VOL_bUnderVoltageOK == TRUE)							/* Voltage value is below limit first time		*/
			{
			VOL_tActualTimeUnderVoltage = STK_tGetSafeTime();		/* Store actual time stamp						*/
			VOL_bUnderVoltageOK = FALSE;							/* Reset auxiliary BOOL value					*/
			}
		else
			{
			if (tDeltaTime > CSP_CRITICAL_UNDERVOLTAGE_TIMEOUT)		/* Voltage mean value is below min. limit		*/
				{													/* after defined time => Enter low voltage mode	*/

				VOL_bUnderVoltageMode = TRUE;						/* Set return value								*/
				}
			}
		}
	else
		{/* Voltage level is in hysteresis range - do nothing	*/
		}
	return VOL_bUnderVoltageMode;
	}

/**
 *	\brief	Function checks whether there is a over voltage condition present or not
 *
 *	\return	BOOL Returns TRUE if error is detected, else FALSE
 */
static BOOL VOL_bCheckOverVoltageCondition(void)
	{
	TFp			tVoltageValueV	=0;									/* Local store variable for voltage value		*/
	TSafeTime	tDeltaTime		=0;									/* Local store variable for time value			*/

	tDeltaTime		 = STK_tGetSafeTime();							/* Get actual time stamp						*/
	tDeltaTime		-= VOL_tActualTimeOverVoltage;					/* Calculate difference of time					*/
	tVoltageValueV	 = VOL_tGetDclVolV();							/* Store actual voltage level					*/

	if (tVoltageValueV < (CSP_CRITICAL_OVERVOLTAGE_V - CSP_OVERVOLTAGE_HYSTERESIS_V))
		{														/* Voltage mean value has fall below max. limit		*/
																/*- hysteresis * => Recover from over voltage mode	*/
		
		VOL_bOverVoltageMode	= FALSE;							/* Reset return value							*/
		VOL_bOverVoltageOK		= TRUE;								/* Set auxiliary BOOL variable					*/
		}
	else if (tVoltageValueV > CSP_CRITICAL_OVERVOLTAGE_V)			/* Voltage mean value is over max. limit		*/
		{
		if (VOL_bOverVoltageOK == TRUE)								/* Voltage value is below limit first time		*/
			{
			VOL_tActualTimeOverVoltage = STK_tGetSafeTime();		/* Store actual time stamp						*/
			VOL_bOverVoltageOK 	= FALSE;							/* Reset auxiliary BOOL value					*/
			}
		else
			{
			if (tDeltaTime > CSP_CRITICAL_OVERVOLTAGE_TIMEOUT)		/* Voltage mean value is over max. limit		*/
				{													/* after defined time => Enter over voltage mode*/
				
				VOL_bOverVoltageMode = TRUE;						/* Set return value								*/
				}
			}
		}
	else
		{													/* Voltage level is in hysteresis range - do nothing	*/
		}
	return VOL_bOverVoltageMode;
	}
static void VOL_vCheckInternalReferenceVoltageCondition(void)
{
	TFp tVoltageValue = VOL_tGetInternalVoltRefVolmV();
	if(tVoltageValue <= CSP_INTERNAL_REFERENCE_VOLTAGE_HIGH_LIMIT_MV)
	{

	}
	else
	{

	}
}

/********************************************************************************************************************/
/* GLOBAL FUNCTIONS *************************************************************************************************/
/********************************************************************************************************************/

/**
 *	\brief	Function checks whether there is a low voltage condition present or not
 *
 *	\return	BOOL Returns TRUE if error is detected, else FALSE
 */
BOOL VOL_bCheckLowVoltageCondition(void)
	{
	TFp			tVoltageValueV	=0;									/* Local store variable for voltage value		*/
	TSafeTime	tDeltaTime		=0;									/* Local store variable for time value			*/

	tDeltaTime = STK_tGetSafeTime();								/* Get actual time stamp						*/
	tDeltaTime-= VOL_tActualTimeLowVoltage;							/* Calculate difference of time					*/
	tVoltageValueV = VOL_tGetDclVolV();								/* Store actual voltage level					*/

	if (tVoltageValueV > (CSP_LOW_VOLTAGE_LIMIT_V + CSP_LOW_VOLTAGE_HYSTERESIS_V))
		{														/* Voltage mean value has risen above min. limit	*/
																/*+ hysteresis * => Recover from low voltage mode	*/
		VOL_bLowVoltageMode	= FALSE;								/* Reset return value							*/
		VOL_bLowVoltageOK	= TRUE;									/* Set auxiliary BOOL variable					*/
		}
	else if (tVoltageValueV < CSP_LOW_VOLTAGE_LIMIT_V)				/* Voltage mean value is below min. limit		*/
		{
		if (VOL_bLowVoltageOK == TRUE)								/* Voltage value is below limit first time		*/
			{
			VOL_tActualTimeLowVoltage = STK_tGetSafeTime();			/* Store actual time stamp						*/
			VOL_bLowVoltageOK = FALSE;								/* Reset auxiliary BOOL value					*/
			}
		else
			{
			if (tDeltaTime > CSP_LOW_VOLTAGE_TIMEOUT)				/* Voltage mean value is below min. limit		*/
				{													/* after defined time => Enter low voltage mode	*/
				VOL_bLowVoltageMode = TRUE;							/* Set return value								*/
				}
			}
		}
	else
		{													/* Voltage level is in hysteresis range - do nothing	*/
		}
	return VOL_bLowVoltageMode;
	}

/**
 * 	\brief	Sampling of DCL voltage and 13.5 voltage values
 *			Function store actual ADC value of measured voltages
 *			Next are converted to Q-format
 * 	\return	void
 */
void VOL_vSampleDclVoltage(void)
	{
	/* Sample voltage values */
	VOL_uiDclVoltageAdValue	    = Motor_1st.FOC.s16VbusAvg;				/* Sample actual DCL voltage AD value	*/
	VOL_ui13_5VoltageValue	    = GET_ADC2_VALUE(VF_RANK);				/* Sample actual 13.5 voltage AD value	*/
	VOL_tVrefIntVolADValue	    = GET_ADC1_VALUE(IV_RANK);
	/* Fill voltage sample variables	*/
	VOL_tDclVolSampleQ		= VOL_uiDclVoltageAdValue;
	VOL_t13_5VolSampleQ		= VOL_ui13_5VoltageValue;
	VOL_tVrefIntVolSampleQ  = VOL_tVrefIntVolADValue;
	}
/**
 * 	\brief	Tracking of voltage values
 *
 *			Function store min/max value of DCL voltage
 *			After one mains period are calculated average voltages
 *			and converted to real Voltage value
 *
 * 	\return	void
 */
//static TFp VOL_temp;
#define PAR_DCL_VOLT_DIV_RATIOO (PAR_DCL_VOLT_DIV_RATIO * 1000)
void VOL_vTrackVoltageValues(void)
	{
	/* Integrate DCL voltage sample to build average value over one period */
	VOL_tDclVolPerSampleIntQ  += VOL_tDclVolSampleQ;
	//VOL_t13_5VolPerSampleIntQ += VOL_t13_5VolSampleQ;
	VOL_tVrefIntVolPerSampleIntQ += VOL_tVrefIntVolSampleQ;
	/* Track min/max values of line and DCL voltage */
	if (VOL_tDclVolSampleQ > VOL_tDclVolMaxTrackQ)
		{
		/* A new max. value has been found => there can't be a min. value too! */
		VOL_tDclVolMaxTrackQ = VOL_tDclVolSampleQ;
		}
	else
		{
		if (VOL_tDclVolSampleQ < VOL_tDclVolMinTrackQ)
			{
			VOL_tDclVolMinTrackQ = VOL_tDclVolSampleQ;
			}
		}

	/* Check sampling period*/
	VOL_ucPeriodSampleCtr--;
	if (0 == VOL_ucPeriodSampleCtr)
		{
		/* Store locally found period min./max. values */
		VOL_tDclVolMaxQ	= VOL_tDclVolMaxTrackQ;
#if(CALC_RIPPLE_VOLTAGE == OPTION_ACTIVE)
		VOL_tDclVolMinQ	= VOL_tDclVolMinTrackQ;
#endif
		/* One mains period is over => restart search of local max. and min. values */
		
		VOL_tDclVolPerSampleIntResultQ	= VOL_tDclVolPerSampleIntQ;
		//VOL_t13_5VolPerSampleIntResultQ = VOL_t13_5VolPerSampleIntQ;
		VOL_tVrefIntVolPerSampleIntResultQ = VOL_tVrefIntVolPerSampleIntQ;

		/* Calculated DC link average voltage over mains period in Q format */
		VOL_tDclVoltageQ  = (TFp)(VOL_tDclVolPerSampleIntResultQ   / (TFp)PAR_VOLTAGE_SAMPLES_PER_LINE_PERIOD);
		//VOL_t13_5VoltageQ = (TFp)(VOL_t13_5VolPerSampleIntResultQ  / (TFp)PAR_VOLTAGE_SAMPLES_PER_LINE_PERIOD);
		VOL_tVrefIntVolQ = (TFp)(VOL_tVrefIntVolPerSampleIntResultQ / (TFp)PAR_VOLTAGE_SAMPLES_PER_LINE_PERIOD);
		/* Calculated DC link average voltage over mains period in unit [V] */

	/*#if (MOTOR_VDD_OPTI == OPTION_ACTIVE)//need to update
		VOL_bVddUpdated = MAS_tEEPWriteFinished();
		if((VOL_bVddUpdated != FALSE) && (VOL_bVddRedOut != TRUE))
		{
			(void)NVMEM_bRead( _MMI_LAST_MEASURED_VDD_Q12_2_BYTE , (uchar *)&VOL_uiRealVddmV);
			VOL_bVddRedOut = TRUE;
		}
		if(VOL_uiRealVddmV != 0)
		{
			VOL_tMaxVoltage = Q0(((VOL_uiRealVddmV) / PAR_DCL_VOLT_DIV_RATIOO) + 0.5);
			VOL_tDCLVoltageV = (TFp)VOL_uiConvQxToRealVoltage(VOL_tDclVoltageQ, VOL_tMaxVoltage);
		}
		else
		{
		VOL_tDCLVoltageV = (TFp)VOL_uiConvQxToRealVoltage(VOL_tDclVoltageQ, Q0(REFPAR_REFERENCE_DCL_VOLTAGE_V));
		}
	#else*/
		VOL_tDCLVoltageV = (TFp)VOL_uiConvQxToRealVoltage(VOL_tDclVoltageQ, Q0(REFPAR_REFERENCE_DCL_VOLTAGE_V));
	//#endif


		//VOL_t13_5VoltagemV =(TFp)VOL_uiConvQxToRealVoltage(VOL_t13_5VoltageQ, Q0(REFPAR_REFERENCE_13_5_VOLTAGE_MV));
		VOL_tVrefIntVoltagemV = (TFp)VOL_uiConvQxToRealVoltage(VOL_tVrefIntVolQ,  Q0(REFPAR_REFERENCE_VREFINT_VOLTAGE_MV));
		
		/* Convert peak DCL voltage from Qx format to real voltage in unit [V]*/

		VOL_uiDCLVoltagePeakValueV = (TFp)VOL_uiConvQxToRealVoltage(VOL_tDclVolMaxQ, Q0(REFPAR_REFERENCE_DCL_VOLTAGE_V));


#if(CALC_RIPPLE_VOLTAGE == OPTION_ACTIVE)
		/* Convert DCL voltage ripple from Qx format to real voltage in unit [V]	*/
		VOL_uiDclVoltageRippleV = VOL_uiConvQxToRealVoltage(VOL_tDclVolRippleQ, Q0(REFPAR_REFERENCE_DCL_VOLTAGE_V));
#endif
		/* Restart period integration of the DC link voltage	*/
		VOL_tDclVolPerSampleIntQ	= 0;
		//VOL_t13_5VolPerSampleIntQ	= 0;
		VOL_tVrefIntVolPerSampleIntQ = 0;
		/* Restart min./max. tracking for the next period		*/
		VOL_tDclVolMaxTrackQ	= 0;
		VOL_tDclVolMinTrackQ	= (TFp)FP(1.0);
		VOL_ucPeriodSampleCtr	= (uint8_t)PAR_VOLTAGE_SAMPLES_PER_LINE_PERIOD;
		}
	/* Update handshaking flag between interrupt and mainloop layer */
	VOL_bVoltageSampleRefreshed = TRUE;
	}

/****************************************************************************************************************/
/* DATA INTERFACE FUNCTIONS *************************************************************************************/
/****************************************************************************************************************/
/*
 * 	\brief	Interface to return actual DCL voltage sample
 *
 * 	\return	TFp	[Q12]	VOL_tDclVolSampleQ
 */
TFp VOL_tGetDclVolRawSampleQ(void)
	{

	return VOL_tDclVolSampleQ;

	}

/**
 *	\brief	Interface to return averaged DC link voltage over one mains period in Q format
 *
 * 	\return	TFp	[Q12]	VOL_tDclVoltageQ
 */
TFp VOL_tGetDclVolQ(void)
	{
	return VOL_tDclVoltageQ;
	}

/**
 *	\brief	Interface to return averaged DC link voltage over one mains period in Voltage format
 *
 * 	\return	TFp	[V]	VOL_tDCLVoltageV
 */
TFp VOL_tGetDclVolV(void)
	{
	return VOL_tDCLVoltageV;
	}

/**
 * \brief	Interface to return averaged 13.5 voltage over one mains period in miliVoltage format
 *
 * \return	TFp	[mV]	VOL_t13_5VoltagemV
 */
TFp VOL_tGet13_5VolmV(void)
	{
	return VOL_t13_5VoltagemV;
	}
/**
 * \brief	Interface to return averaged internal voltage  reference over one mains period in miliVoltage format
 *
 * \return	TFp	[mV]	VOL_tVrefIntVoltagemV
 */
TFp VOL_tGetInternalVoltRefVolmV(void)
	{
	return VOL_tVrefIntVoltagemV;
	}

/****************************************************************************************************************/
/* INTERFACE FUNCTIONS ******************************************************************************************/
/****************************************************************************************************************/
/**
 *	\brief	Initialization of IIR filter output values
 *
 *	\return	BOOL
 */
//impr: waiting for fsmonitor.c
//todo: remove call parameterss
BOOL VOL_bInitDcLinkVoltageMeasurement(const void* const pData)
	{
	TSafeTime	tActualTime;
	tActualTime = STK_tGetSafeTime();
	if (tActualTime >= (TSafeTime)VOL_INITIAL_ADC_DELAY)
		{/* Call all functions needed to setup the internal data values	*/
#if(CALC_RIPPLE_VOLTAGE == OPTION_ACTIVE)
		VOL_vCalcDclVoltageRippleQ();
#endif
		/* Init low pass filter outputs	*/
		FIR_vInitFilterOutput(&VOL_tDCLVoltageDmpdV, VOL_tDCLVoltageV);
		FIR_vInitFilterOutput(&VOL_tDCLPeakVoltageDmpdQ, VOL_tDclVolMaxQ);
		return TRUE;
		}
	else
		{
		return FALSE;
		}

	}

/**
 * \brief	Voltage error detection function
 *			If DCL voltage is not in defined range, this functions return
 *			true, else returns false.
 *
 * \return	BOOL	Returns TRUE if error is detected, else FALSE
 */
//impr: waiting for fsmonitor.c
//todo: remove call parameters
BOOL VOL_bVoltageErrorDetection(const void* const pData)
	{
	BOOL bFuncReturnValue = FALSE;										/* Local return variable for function call	*/
#if(MPM_SYESTEM_DELAY == OPTION_ACTIVE)
	if(STK_tGetSafeTime() > GENPAR_DELAY_TIME)
	{
	VOL_vCheckInternalReferenceVoltageCondition();
	}
#else // MPM_SYESTEM_DELAY
	VOL_vCheckInternalReferenceVoltageCondition();
#endif // MPM_SYESTEM_DELAY
	bFuncReturnValue = VOL_bCheckUnderVoltageCondition();
	if(bFuncReturnValue != FALSE)
		{/*Under voltage mode is ACTIVE * => We don't need to check also the over voltage condition	*/
		return TRUE;
		}
	else
		{
			bFuncReturnValue = VOL_bCheckOverVoltageCondition();
		if (bFuncReturnValue != FALSE)
			{/* Overvoltage mode is ACTIVE			*/
			return TRUE;
			}
		else
			{/* Voltage is within acceptable range	*/
			return FALSE;
			}
		}

}

/**
 *	\brief	Main voltage task handler
 *			Function call in defined time period calculation of voltage values.
 *			If voltage values are not updated in ADC interrupt function,
 *			FSF error is reported.
 *	\return	BOOL
 */
//impr: waiting for fsmonitor.c
//todo: remove call parameters
BOOL VOL_bHandleTask(void)
	{
	TSafeTime tActualTime;
	tActualTime = STK_tGetSafeTime();
	if(((tActualTime - VOL_tSafeTimeStamp) >= (TSafeTime)VOL_TIMEBASE)&&(tActualTime > VOL_tSafeTimeStamp))
		{
		/*
		* Time slice active
		* Add time base to prevent missing a tick
		* => no incremental error
		* */
		VOL_tSafeTimeStamp += (TSafeTime)VOL_TIMEBASE;
#if(CALC_RIPPLE_VOLTAGE == OPTION_ACTIVE)
		VOL_vCalcDclVoltageRippleQ());
#endif
		VOL_vCalcVolLowPassFilters();
		if (VOL_bVoltageSampleRefreshed == TRUE)
			{
			/*
			* Voltage sampling mechanism is still working; value has been refreshed
			* => report success
			* */
			VOL_bVoltageSampleRefreshed = FALSE;  /* Reset flag for the next cycle */
			}
		else
			{
			/*
			* Voltage sampling function has not been triggered
			* since the last call of this function => there seems
			* to be a problem with the interrupt control
			* */

			}
		}
	return TRUE;

	}




