/****************************************************************************************************************
 * INCLUDES *****************************************************************************************************
*****************************************************************************************************************/
#include "common.h"
#include "LibDefines.h"
#include "Filter.h"
#include "main.h"
#include "fsclock.h"
#include "current.h"
#include "fsParameter.h"
#include "drv_inc.h"
#include "ParamRef.h"
#include "ParamHw.h"


/****************************************************************************************************************
 * LOCAL TYPES **************************************************************************************************
*****************************************************************************************************************/

/**
 *	\brief	Object type of the AD bridge currents
 */
typedef struct _TCurrentADValues {
	int16_t siAdPhA;		/**< Mesured AD value of phase A */
	int16_t siAdPhB;		/**< Mesured AD value of phase B */
}TCurrentADValues;

/**
 *	\brief	Object type of the motor currents
 */
typedef struct _TCurData {
	TBrdCurOffsetState	tOffsetMeasState;			/**< Current offset measurement state	*/
	TCurrentADValues	tAdValCur;					/**< AD bridge currents					*/
	TThreePhaseCur		tPhaseCur;					/**< Phase currents						*/
	TFp					tIq;						/**< Iq current							*/
	TFp					tIpeakSquaredQ;				/**< I peak value squared				*/
	TFp					tImaxQ;						/**< I maximum value					*/
	TFp					tImaxmA;					/**< I maximum val.[mA]					*/
	TFilterData			tOffsetChA;					/**< Offset current chA					*/
	TFilterData			tOffsetChB;					/**< Offset current chB					*/
	TFilterData			tIqDmpd;					/**< Iq current damped					*/
	TFilterData			tImaxmADmpd;				/**< I max.val.[mA] damp.				*/
	uint16_t			uiOCInputDebounceCounter;	/**< Overcurrent input debounce counter	*/
	uint16_t			uiOCCounter;				/**< Over-current event counter			*/
	TSafeTime			tOffsetErrTimeout;			/**< Time stamp used for offset voltage error handling	*/
	uint16_t			uiAlignmentCurrentPhA;		/**< Alignment current phase A			*/
	uint16_t			uiAlignmentCurrentPhB;		/**< Alignment current phase B			*/
}TCurData;

/**
 * \brief	Current measurement parameter object
 */
typedef const struct _TCurPara {
	TSafeTime			tOffsetErrTimeout;			/**< Timeout which is used for offset error handling			*/
}TCurPara;



/****************************************************************************************************************
 * LOCAL FUNCTION PROTOTYPES ************************************************************************************
*****************************************************************************************************************/
static void 	CUR_vHandleOverCurIntMeasChanM1(void);
static void 	CUR_vHandleOverCurIntMeasChanM2(void);
static void		CUR_vResetCurrentValues(void);
static void		CUR_vCalcCurrentQuantities(void);
static void		CUR_vCheckMotCurrentMeasOffsetVoltage(void);


/****************************************************************************************************************
 * LOCAL DEFINITIONS ********************************************************************************************
*****************************************************************************************************************/
#define CUR_OFFSET_RESOLUTION				11			/**< Current offset resolution in no. of bits				*/

/**
 * [ms] Task handler time base
 * */
#define CUR_TASK_TIME_BASE_MS				(50)

/**
 *  [SafeTime]	Module time base in 100ns units
 *  			This time is used as the reference tick of the module.
 *  			<b>Time Base:</b> 50ms
 * */
#define CUR_TASK_TIME_BASE					STK_DEF_TIME_MS(CUR_TASK_TIME_BASE_MS)


#define CUR_MAX_COMP_INT0_HIGH				6			/**< max. HIGH states of comparator output to set error		*/

/** \brief 	Safe time ticks needed after reset before offset initialization is started
			The ADC is triggered synchronous to the motor control PWM timer. To be sure that
			the ADC has been triggered at least once, this delay is introduced.*/

#define CUR_INITIAL_ADC_DELAY				10
/**
 * \brief	Count of max offset samples after reset
 */
#define CUR_OFFSET_SAMPLES_AFTER_RESET		10


/**
 * \brief Current object data initialization
 * */
#define CUR_DATA_OBJ_INIT_VALUE		{							\
		CUR_OFFSET_MEAS_NOT_ACTIVE,								\
		{0,0},													\
		{0,0,0},												\
		0,														\
		0,														\
		0,														\
		0,														\
		{0,0,0},												\
		{0,0,0},												\
		{0,0,0},												\
		{0,0,0},												\
		0,														\
		0,														\
		0,														\
		0,														\
		0														\
}

/**
 * \brief	Parameter initialization of bridge 1
 * */
#define CUR_PARA_OBJ_INIT_VALUE_BRIDGE_1		{				\
		FSP_CUR_OFFSET_ERROR_TIMEOUT							\
}

#define CUR_PHASE_CUR_SAMPLE_COUNT     8096

/****************************************************************************************************************
 * LOCAL VARIABLES **********************************************************************************************
*****************************************************************************************************************/
/* Major current object for two bridges, containing pairs (A,B) of Offsets, AD-values and phase values
 { {{{tFilIn.A,tFilOut.A},{tFilIn.B,tFilOut.B}},{tAd.A,tAd.B},{tPh.A,tPh.B}}   Bridge 1
   {{{tFilIn.A,tFilOut.A},{tFilIn.B,tFilOut.B}},{tAd.A,tAd.B},{tPh.A,tPh.B}} } Bridge 2 */

static TCurData	CUR_tDataBrd1				= CUR_DATA_OBJ_INIT_VALUE;
static TCurPara	CUR_tParaBridge1 		    = CUR_PARA_OBJ_INIT_VALUE_BRIDGE_1;

static uint8_t	CUR_ucOffsetSampleCounter	= CUR_OFFSET_SAMPLES_AFTER_RESET;		/**< Offset sample counter		*/
static TSafeTime CUR_tLastTimeStamp			= 0;									/**< Module system time			*/

/****************************************************************************************************************
 * LOCAL CONSTANTS **********************************************************************************************
*****************************************************************************************************************/

/**
 * \brief Imax current -> Tsampling = 50ms / Tfilter = 1500ms
 * */
static TFilterCoeff	CUR_tFilterCoefImax		= FIR_tCalcFilterConst(50.0,1500.0);

/**
 * \brief Iq current -> Tsampling = 50ms / Tfilter = 1500ms
 * */
static TFilterCoeff	CUR_tFilterCoefIq		= FIR_tCalcFilterConst(50.0,1500.0);

/**
 * \brief Offset current -> Tsampling = 50ms / Tfilter = 5000ms
 * */
static TFilterCoeff	CUR_tFilterCoefOffset	= FIR_tCalcFilterConst(50.0,5000.0);

static TFilterCoeff	CUR_tFilterCoefCurRMS   = FIR_tCalcFilterConst(1.0,3.2);//1/2*PI*F = 1/(2*3.14*50)


static TThreePhaseRMS CUR_tThreePhaseCurRms = {{0,0,0},0,{0,0,0},0};

static TCurBufferCal CUR_tCurBufferCal = {0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0};

static uint16_t testA_P;
static uint16_t testA_N;

static uint16_t testB_P;
static uint16_t testB_N;

static uint16_t testC_P;
static uint16_t testC_N;


/****************************************************************************************************************
 * LOCAL FUNCTIONS **********************************************************************************************
*****************************************************************************************************************/
/**
 * \brief	Function resets all current values of the passed object
 *
 * \param	ptObj	Pointer to the object to be handled
 *
 * \return	void
 * */
static void	CUR_vResetCurrentValues(void)
	{
	CUR_tDataBrd1.tImaxQ							= 0;
	CUR_tDataBrd1.tIq								= 0;
	CUR_tDataBrd1.tIqDmpd.tFilterInputValue			= 0;
	CUR_tDataBrd1.tIqDmpd.tFilterOutputValue		= 0;
	CUR_tDataBrd1.tImaxmA							= 0;
	CUR_tDataBrd1.tImaxmADmpd.tFilterInputValue		= 0;
	CUR_tDataBrd1.tImaxmADmpd.tFilterOutputValue	= 0;
	CUR_tDataBrd1.tIpeakSquaredQ					= 0;
	CUR_tDataBrd1.tPhaseCur.a						= 0;
	CUR_tDataBrd1.tPhaseCur.b						= 0;
	CUR_tDataBrd1.tPhaseCur.c						= 0;
	}

/**
 * \brief	Calculation of current quantities
 *
 * 			The peak current is calculated out of the alpha and beta components
 * 			of the current using the formula\n
 * 			Ipeak = sqrt(Ialpha^2 + Ibeta^2)\n
 * 			Using the current reference value the resulting peak current value
 * 			is transformed from p.u. to unit [mA].
 * 			Additionally the current in unit [mA] and the Iq current are low-pass filtered.
 *
 * \param	ptObj Pointer to the current object
 *
 * \return	void
 * */
static void CUR_vCalcCurrentQuantities(void)
	{
	TCartNum	tCartNumb;
	TFp			tCurIalpha;
	TFp			tCurIbeta;
	TFp			tTemp;

	if (CUR_tDataBrd1.tOffsetMeasState == CUR_OFFSET_MEAS_NOT_ACTIVE)
		{
		/* Offset measurement not active -> current calculations */
		CUR_tDataBrd1.tPhaseCur.a = Motor_1st.FOC.sIabc.s16A;
		CUR_tDataBrd1.tPhaseCur.b = Motor_1st.FOC.sIabc.s16B;
		
		//TRR_vClarkeTrans(&CUR_tDataBrd1.tPhaseCur, &tCartNumb);
		MCFLIB_Clark_Sat_S16((MCFLIB_3_ABC_T_S16 *)&(CUR_tDataBrd1.tPhaseCur), (MCFLIB_2_ALBE_T_S16 *)&tCartNumb);
		tCurIalpha = tCartNumb.tRe;
		tCurIbeta  = tCartNumb.tIm;

		/* Calculate Imax^2 = Ialpha^2 + Ibeta^2 where Ialpha and Ibeta are in q12 format */
		tTemp   = tCurIalpha	* tCurIalpha;
		tTemp  += tCurIbeta	* tCurIbeta;

		/* Imax in p.u. format */
		CUR_tDataBrd1.tImaxQ			= (TFp)FPM_ulSqrt((uint32_t)tTemp);

		/*
		* Note: tTemp contains the squared value of the peak phase current in 2x the p.u.
		* fixed point format because we did not use the FPmul function.
		* => It needs to be converted to the standard p.u. format.
		* =>	qx * qx = q2x
		* */
		CUR_tDataBrd1.tIpeakSquaredQ	= FPM_QxtoQy(tTemp,(2*FPM_GLOBAL_FP_FORMAT),FPM_GLOBAL_FP_FORMAT); // x>y !!!

		/* IqFp -> I[mA] */
		CUR_tDataBrd1.tImaxmA			= (int16_t)FPM_FpMul(CUR_tDataBrd1.tImaxQ, Q0(REFPAR_REFERENCE_CURRENT_MA_C));//need to update

		/* Filter Imax[mA] -> ImaxDmpd[mA] */
		CUR_tDataBrd1.tImaxmADmpd.tFilterInputValue = CUR_tDataBrd1.tImaxmA;						/* Actual value	*/
		FIR_vCalcFilter(&CUR_tDataBrd1.tImaxmADmpd,(TFilterCoeff*)(&CUR_tFilterCoefImax));	        /* Calc filter	*/

		/* Filter Iq -> IqDmpd */
		CUR_tDataBrd1.tIqDmpd.tFilterInputValue	= (TFp)CUR_tDataBrd1.tIq;							/* Actual value	*/
		FIR_vCalcFilter(&CUR_tDataBrd1.tIqDmpd,(TFilterCoeff*)(&CUR_tFilterCoefIq));			    /* Calc filter	*/
		}
	}

/**
 * \brief	Supervision of the current measurement offset voltage
 *			For each motor phase current measurement circuit the offset voltage
 * 			is measured while there is no current flowing through the shunt resistors
 * 			(motors are stopped and no self-test is running). The offset voltages
 * 			are low pass filtered to increase the robustness and then checked for
 * 			plausibility. If at least one signal is out of bounds a motor error is
 * 			set. If the signal is out of bounds for longer than a timeout time a
 * 			safety error is set => the software enters a safe state.
 * \param	ptObj	Pointer to a current measurement object
 * \return	void
 */
static void CUR_vCheckMotCurrentMeasOffsetVoltage(void)
	{
	TSafeTime	tActualTime;
	BOOL		bErrFlag = FALSE;

	tActualTime = STK_tGetSafeTime();

	if (CUR_tDataBrd1.tOffsetMeasState == CUR_OFFSET_MEAS_IS_ACTIVE)
		{
		/*
		 * Offset measurement is active
		 * => Reset current values
		 * => Sample and filter offset voltage
		 * => Check values against abs. maximum values
		 * */
		CUR_vResetCurrentValues();

		/* Filter Offset Channel A:  tAdValCur.siAdPhA -> tOffsetChA */
		CUR_tDataBrd1.tOffsetChA.tFilterInputValue     = (TFp)Motor_1st.FOC.sIabc.s16A; /* Actual value */
		FIR_vCalcFilter(&CUR_tDataBrd1.tOffsetChA,(TFilterCoeff*)(&CUR_tFilterCoefOffset));   /* Calc filter */

		/* Filter Offset Channel B:  tAdValCur.siAdPhA -> tOffsetChA */
		CUR_tDataBrd1.tOffsetChB.tFilterInputValue     = (TFp)Motor_1st.FOC.sIabc.s16B; /* Actual value */
		FIR_vCalcFilter(&CUR_tDataBrd1.tOffsetChB,(TFilterCoeff*)(&CUR_tFilterCoefOffset));   /* Calc filter */

		/* Check channel A limits */
		if (((int16_t)FIR_tGetFilterOutputx(&CUR_tDataBrd1.tOffsetChA) < PAR_CUR_OFFSET_MIN_AD)
		||  ((int16_t)FIR_tGetFilterOutputx(&CUR_tDataBrd1.tOffsetChA) > PAR_CUR_OFFSET_MAX_AD))
			{
				bErrFlag = TRUE;
			
				if((tActualTime - CUR_tDataBrd1.tOffsetErrTimeout) >= CUR_tParaBridge1.tOffsetErrTimeout)
					{
						//set error
					}
			}
			else
				{

				}

		/* Check channel B limits */
		if (((int16_t)FIR_tGetFilterOutputx(&CUR_tDataBrd1.tOffsetChB) < PAR_CUR_OFFSET_MIN_AD)
		||  ((int16_t)FIR_tGetFilterOutputx(&CUR_tDataBrd1.tOffsetChB) > PAR_CUR_OFFSET_MAX_AD))
			{
				bErrFlag = TRUE;
				if((tActualTime - CUR_tDataBrd1.tOffsetErrTimeout) >= CUR_tParaBridge1.tOffsetErrTimeout)
				{
					//set error
				}
			}
		else
			{

			}

		if(bErrFlag == FALSE)
			{
			/*
			 *	None of the two channels is out of range
			 *	=> Reload timeout
			 * */
			CUR_tDataBrd1.tOffsetErrTimeout = tActualTime;
			}
		}
	}


/****************************************************************************************************************
 * GLOBAL FUNCTIONS *********************************************************************************************
*****************************************************************************************************************/
/**
 * \brief	Interface function to get actual current value
 * 			Function returns requested current value according to
 * 			set of HW channel
 * \param	THWChannel		Actual HW channel
 * 			TCurQuantity	Actual requested value
 * \return	TFp
 */
TFp CUR_tGetCurVal( TCurQuantity tQuantity)
	{
	TFp tRetVal;
	switch (tQuantity)
		{
		case CUR_PHASE_A:
			{
			tRetVal = CUR_tDataBrd1.tPhaseCur.a;
			break;
			}
		case CUR_PHASE_B:
			{
			tRetVal = CUR_tDataBrd1.tPhaseCur.b;
			break;
			}
		case CUR_IQ_VAL:
			{
			tRetVal = CUR_tDataBrd1.tIq;
			break;
			}
		case CUR_IQ_DAMPED:
			{
			tRetVal = FIR_tGetFilterOutputx(&CUR_tDataBrd1.tIqDmpd);
			break;
			}
		case CUR_MAXIMUM_Q:
			{
			tRetVal = CUR_tDataBrd1.tImaxQ;
			break;
			}
		case CUR_MAXIMUM_MA:
			{
			tRetVal = CUR_tDataBrd1.tImaxmA;
			break;
			}
		case CUR_MAXIMUM_MA_DMPD:
			{
			tRetVal = FIR_tGetFilterOutputx(&CUR_tDataBrd1.tImaxmADmpd);
			break;
			}
		case CUR_PHASE_C:	//prevent lint error
		default:
			{
			tRetVal = 0;
			break;
			}
		}
	return tRetVal;
	}

#if 0
/**
 * \brief	Set target current Iq
 * \param	THWChannel	tBrd	HW channel
 * 			TFp			tIq		Iq value
 * \return	void
 */
void CUR_vSetCurrentIq(THWChannel tBrd, TFp tIq)
	{
	CUR_tDataBrd1.tIq = tIq;
	}
#endif
/**
 * \brief	Initialization function of current offset
 *
 * \param	pData	Pointer to a current measurement bridge object
 * \return	BOOL	TRUE if calculation is done, else FALSE
 */
BOOL CUR_bInitOffsetCurrentVoltage(void)
	{
	TSafeTime tActualTime;
	TFp tBrd1AdChA;
	TFp tBrd1AdChB;
	TFp tBrd2AdChA;
	TFp tBrd2AdChB;

	/* Ensure that there are already some AD values sampled via measuring the time after startup */
	tActualTime = STK_tGetSafeTime();
	if (tActualTime >= CUR_INITIAL_ADC_DELAY)
		{
		CUR_ucOffsetSampleCounter--;

		/* Add actual offset values for averaging */
		tBrd1AdChA = FPM_QytoQx(CUR_tDataBrd1.tAdValCur.siAdPhA, 0, FIR_FILTER_COEFF_FORMAT); // x < y !!!
		tBrd1AdChB = FPM_QytoQx(CUR_tDataBrd1.tAdValCur.siAdPhB, 0, FIR_FILTER_COEFF_FORMAT); // x < y !!!

		CUR_tDataBrd1.tOffsetChA.tFilterOutputValue += tBrd1AdChA;
		CUR_tDataBrd1.tOffsetChB.tFilterOutputValue += tBrd1AdChB;
		
		if (CUR_ucOffsetSampleCounter == 0)
			{
			/* Calculate the average value over the last input values */
			CUR_tDataBrd1.tOffsetChA.tFilterOutputValue /= CUR_OFFSET_SAMPLES_AFTER_RESET;
			CUR_tDataBrd1.tOffsetChB.tFilterOutputValue /= CUR_OFFSET_SAMPLES_AFTER_RESET;

			/* Current measurement offset value is valid after the first AD samples have been taken
			Average is calculated to reduce the effect of noise after reset.*/
			return TRUE;
			}
		else
			{
			return FALSE;				/* Wait until the number of AD samples have been taken for averaging	*/
			}
		}
	else
		{
		return FALSE;										/* Wait for the first AD samples 					*/
		}
	}/*lint !e715 Symbol 'pData' not referenced	*/

/**
 * \brief	Motor current measurement check function
 * 			Function run in defined time stamps tasks,
 * 			which checks current measurement circuitry.
 * 			Measurement is provide while motor is stopped.
 *
 * \param	pData	Pointer to a current measurement bridge object
 *
 * \return	BOOL
 */
BOOL CUR_bCheckMotCurrentMeasValues(const void* const pData)
	{
	TSafeTime tActualTime;
	tActualTime = STK_tGetSafeTime();
	if((tActualTime - CUR_tLastTimeStamp) >= (TSafeTime)CUR_TASK_TIME_BASE)
		{
		/*
		 * Add time base to prevent missing a tick => no incremental error
		 * */
		CUR_tLastTimeStamp += (TSafeTime)CUR_TASK_TIME_BASE;
		CUR_vCheckMotCurrentMeasOffsetVoltage();
	    CUR_vCalcCurrentQuantities();

		}
	else
		{														/* Wait until next activation of the time slice	*/
		}
	return TRUE;
	}/*lint !e715 Symbol 'pData' not referenced	*/

/**
 * \brief	Set of alignment current
 * 			Interface function to set alignment current during positioning phase
 *
 * \return	void
 */
void CUR_vSetPositioningCurr(void)
	{
	int Temp;

	Temp = CUR_tGetCurVal(CUR_PHASE_A);
	if (Temp < 0)
		{
		Temp *= (-1);
		}
	CUR_tDataBrd1.uiAlignmentCurrentPhA = (uint16_t)FPM_FpMul(Temp,REFPAR_REFERENCE_CURRENT_MA);

	Temp=CUR_tGetCurVal(CUR_PHASE_B);
	if (Temp < 0)
		{
		Temp *= (-1);
		}
	CUR_tDataBrd1.uiAlignmentCurrentPhB = (uint16_t)FPM_FpMul(Temp, REFPAR_REFERENCE_CURRENT_MA);

	}


/****************************************************************************************************************
 * INTERFACE FUNCTIONS - DEKLARATION IN MotorInf.h **************************************************************
*****************************************************************************************************************/
/**
 * \brief	Interface to get all three phases current values
 * \param	THWChannel tBrdg			Actual HW channel
 * 			TThreePhaseCur* ptThreePha	Pointer to phase currents
 * \return	void
 */
/* !!! GLOBAL INTERFACE => DEKLARATION IN MotorInf.h !!! */
void CUR_vGetThreePhaseCur(TThreePhaseCur* ptThreePha)
	{
	ptThreePha->a = Motor_1st.FOC.sIabc.s16A;
	ptThreePha->b = Motor_1st.FOC.sIabc.s16B;
	ptThreePha->c = Motor_1st.FOC.sIabc.s16C;
	}


/**
 * \brief	Interface to set offset state
 * \param	THWChannel tBrdg			Actual HW channel
 * 			TBrdCurOffsetState tState	Requested state
 * \return	void
 */
/* !!! GLOBAL INTERFACE => DEKLARATION IN MotorInf.h !!! */
void CUR_vSetOffsetMeasState(TBrdCurOffsetState tState)
	{
		CUR_tDataBrd1.tOffsetMeasState = tState;
	}
/*******************************************Interface function*****************************************/
void CUR_vGetThreePhaseRawValue(TThreePhaseCur* ptThreePhaRaw)
{
	
	ptThreePhaRaw->a = (int32_t)GET_ADC1_VALUE(IR_U_RANK);
	ptThreePhaRaw->b = (int32_t)GET_ADC2_VALUE(IR_V_RANK);
	ptThreePhaRaw->c = -ptThreePhaRaw->a-ptThreePhaRaw->b;

}

void CUR_vCurRMSFilterInit(void)
{
	//FIR_vInitFilterOutput(&(CUR_tThreePhaseCurRms.tThreePhaseRMSCurFilter,0);
}

/*
call it every 1ms.


*/
void CUR_vCalThreePhaseCurRMS(void)
{
	uint32_t u32CurSquare = 0;
	TThreePhaseCur* ptThreePhaRawCur = NULL;
	ptThreePhaRawCur = &(CUR_tThreePhaseCurRms.tThreePhaseCurRaw);
	CUR_vGetThreePhaseRawValue(ptThreePhaRawCur);
	//CUR_tThreePhaseCurRms.tThreePhaseCurRaw = *ptThreePhaRawCur;
	u32CurSquare = CUR_tThreePhaseCurRms.tThreePhaseCurRaw.a*CUR_tThreePhaseCurRms.tThreePhaseCurRaw.a;
	u32CurSquare += CUR_tThreePhaseCurRms.tThreePhaseCurRaw.b*CUR_tThreePhaseCurRms.tThreePhaseCurRaw.b;
	u32CurSquare += CUR_tThreePhaseCurRms.tThreePhaseCurRaw.c*CUR_tThreePhaseCurRms.tThreePhaseCurRaw.c;
	u32CurSquare = Division(u32CurSquare, 3);
	CUR_tThreePhaseCurRms.u16ThreePhaseRMSCur = Hw_Sqrt(u32CurSquare);

	//filter
	
    CUR_tThreePhaseCurRms.tThreePhaseRMSCurFilter.tFilterInputValue = CUR_tThreePhaseCurRms.u16ThreePhaseRMSCur;
    FIR_vCalcFilter(&(CUR_tThreePhaseCurRms.tThreePhaseRMSCurFilter), (TFilterCoeff*)(CUR_tFilterCoefCurRMS));
	CUR_tThreePhaseCurRms.u16ThreePhaseRMSCurFilter = CUR_tThreePhaseCurRms.tThreePhaseRMSCurFilter.tRealFilterOutputValue;
	
	
}


uint16_t CUR_u16GetThreePhaseRMS(void)
{
	return CUR_tThreePhaseCurRms.u16ThreePhaseRMSCur;
}

void CUR_updatePhaseCurrentBuffer(void)
{
	int32_t currentA;
	int32_t currentB;
	int32_t currentC;
	int64_t square;

	currentA = (int32_t)Motor_1st.FOC.sIabc.s16A;
	currentB = (int32_t)Motor_1st.FOC.sIabc.s16B;
	currentC = (int32_t)Motor_1st.FOC.sIabc.s16C;
	
	TCurBufferCal * ptBufferCur = NULL;
	ptBufferCur = &CUR_tCurBufferCal;

	if(currentA < 0)
	{
		/* erase one sample */
		ptBufferCur->buffer_IaN -= Division(ptBufferCur->buffer_IaN , CUR_PHASE_CUR_SAMPLE_COUNT);
		/* add new sample */
		ptBufferCur->buffer_IaN += currentA;

		/* erase one sample, discharge opposite buffer */
		ptBufferCur->buffer_IaP -= Division(ptBufferCur->buffer_IaP , CUR_PHASE_CUR_SAMPLE_COUNT);

		//ptBufferCur->buffer_Ia -= Division(ptBufferCur->buffer_Ia , CUR_PHASE_CUR_SAMPLE_COUNT);
		/* add new sample */
		//square = ((int64_t)currentA*(int64_t)currentA);
		//ptBufferCur->buffer_Ia += square;
		testA_N ++;
	}
	else
	{
		/* erase one sample */
		ptBufferCur->buffer_IaP -= Division(ptBufferCur->buffer_IaP , CUR_PHASE_CUR_SAMPLE_COUNT);
		/* add new sample */
		ptBufferCur->buffer_IaP += currentA;

		/* erase one sample, discharge opposite buffer */
		ptBufferCur->buffer_IaN -= Division(ptBufferCur->buffer_IaN , CUR_PHASE_CUR_SAMPLE_COUNT);

		//ptBufferCur->buffer_Ia -= Division(ptBufferCur->buffer_Ia , CUR_PHASE_CUR_SAMPLE_COUNT);
		/* add new sample */
		//square = ((int64_t)currentA*(int64_t)currentA);
		//ptBufferCur->buffer_Ia += square;
		testA_P ++;
	}

	if(currentB < 0)
	{
		/* erase one sample */
		ptBufferCur->buffer_IbN -= Division(ptBufferCur->buffer_IbN , CUR_PHASE_CUR_SAMPLE_COUNT);
		/* add new sample */
		ptBufferCur->buffer_IbN += currentB;

		/* erase one sample, discharge opposite buffer */
		ptBufferCur->buffer_IbP -= Division(ptBufferCur->buffer_IbP , CUR_PHASE_CUR_SAMPLE_COUNT);

		//ptBufferCur->buffer_Ib -= Division(ptBufferCur->buffer_Ib , CUR_PHASE_CUR_SAMPLE_COUNT);
		/* add new sample */
		//square = ((int64_t)currentB*(int64_t)currentB);
		//ptBufferCur->buffer_Ib += square;
		testB_N ++;
	}
	else
	{
		/* erase one sample */
		ptBufferCur->buffer_IbP -= Division(ptBufferCur->buffer_IbP , CUR_PHASE_CUR_SAMPLE_COUNT);
		/* add new sample */
		ptBufferCur->buffer_IbP += currentB;

		/* erase one sample, discharge opposite buffer */
		ptBufferCur->buffer_IbN -= Division(ptBufferCur->buffer_IbN , CUR_PHASE_CUR_SAMPLE_COUNT);

		//ptBufferCur->buffer_Ib -= Division(ptBufferCur->buffer_Ib , CUR_PHASE_CUR_SAMPLE_COUNT);
		/* add new sample */
		//square = ((int64_t)currentB*(int64_t)currentB);
		//ptBufferCur->buffer_Ib += square;
		testB_P ++;
	}

	if(currentC < 0)
	{
		/* erase one sample */
		ptBufferCur->buffer_IcN -= Division(ptBufferCur->buffer_IcN , CUR_PHASE_CUR_SAMPLE_COUNT);
		/* add new sample */
		ptBufferCur->buffer_IcN += currentC;

		/* erase one sample, discharge opposite buffer */
		ptBufferCur->buffer_IcP -= Division(ptBufferCur->buffer_IcP , CUR_PHASE_CUR_SAMPLE_COUNT);

		//ptBufferCur->buffer_Ic -= Division(ptBufferCur->buffer_Ic , CUR_PHASE_CUR_SAMPLE_COUNT);
		/* add new sample */
		//square = ((int64_t)currentC*(int64_t)currentC);
		//ptBufferCur->buffer_Ic += square;
		testC_N ++;
	}
	else
	{
		/* erase one sample */
		ptBufferCur->buffer_IcP -= Division(ptBufferCur->buffer_IcP , CUR_PHASE_CUR_SAMPLE_COUNT);
		/* add new sample */
		ptBufferCur->buffer_IcP += currentC;

		/* erase one sample, discharge opposite buffer */
		ptBufferCur->buffer_IcN -= Division(ptBufferCur->buffer_IcN , CUR_PHASE_CUR_SAMPLE_COUNT);

		//ptBufferCur->buffer_Ic -= Division(ptBufferCur->buffer_Ic , CUR_PHASE_CUR_SAMPLE_COUNT);
		/* add new sample */
		//square = ((int64_t)currentC*(int64_t)currentC);
		//ptBufferCur->buffer_Ic += square;
		testC_P ++;
	}
}



/** \brief    Refresh phase current buffer samples  in 15bit format
 *
 *  \details  Prepares the AVG values for the DCFSHDL and for DRVR component.
 */
void CUR_vGetPhaseCurrentBufferSamples(void)
{
		/* rms buffer sample size is CUR_PHASE_CUR_SAMPLE_COUNT */
		CUR_tCurBufferCal.buffer_IaP_Avg = (uint32_t)CUR_tCurBufferCal.buffer_IaP / (uint32_t)CUR_PHASE_CUR_SAMPLE_COUNT;
		CUR_tCurBufferCal.buffer_IbP_Avg = (uint32_t)CUR_tCurBufferCal.buffer_IbP / (uint32_t)CUR_PHASE_CUR_SAMPLE_COUNT;
		CUR_tCurBufferCal.buffer_IcP_Avg = (uint32_t)CUR_tCurBufferCal.buffer_IcP / (uint32_t)CUR_PHASE_CUR_SAMPLE_COUNT;

		/* rms is always positive ! */
		CUR_tCurBufferCal.buffer_IaN_Avg = (uint32_t)(-CUR_tCurBufferCal.buffer_IaN) / (uint32_t)CUR_PHASE_CUR_SAMPLE_COUNT;
		CUR_tCurBufferCal.buffer_IbN_Avg = (uint32_t)(-CUR_tCurBufferCal.buffer_IbN) / (uint32_t)CUR_PHASE_CUR_SAMPLE_COUNT;
		CUR_tCurBufferCal.buffer_IcN_Avg = (uint32_t)(-CUR_tCurBufferCal.buffer_IcN) / (uint32_t)CUR_PHASE_CUR_SAMPLE_COUNT;

		/* rms is for Ia Ib Ic ! */
		//CUR_tCurBufferCal.buffer_Ia_Avg = (uint32_t)(CUR_tCurBufferCal.buffer_Ia) / (uint32_t)CUR_PHASE_CUR_SAMPLE_COUNT;
		//CUR_tCurBufferCal.buffer_Ib_Avg = (uint32_t)(CUR_tCurBufferCal.buffer_Ib) / (uint32_t)CUR_PHASE_CUR_SAMPLE_COUNT;
		//CUR_tCurBufferCal.buffer_Ic_Avg = (uint32_t)(CUR_tCurBufferCal.buffer_Ic) / (uint32_t)CUR_PHASE_CUR_SAMPLE_COUNT;

}

TCurBufferCal * CUR_tGetPhaseCurrentCalPointer(void)
{
	return (TCurBufferCal *)&CUR_tCurBufferCal;
}


