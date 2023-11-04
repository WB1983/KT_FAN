#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

#include "mlib.h"
#include "ramp.h"
#include "compile.h"
#include "parameter.h"
#include "main.h"
#include "Common.h"

/**************************type define*********************************/
typedef struct _TOptimizeOpenSpeedData
{
	Q15_t   s16CurveAngle;
	int32_t s32LoopSWSpeed;
    int32_t s32Factor;
	int32_t s32CurveBaseSpeed;
	int32_t s32StepRampValue;

}TOptimizeOpenSpeedData;




typedef struct _TRAMPPARAM
{
	uint16_t u16CurrentMaxVal;
	uint16_t u16SpeedMaxVal;
	uint16_t u16CurrentMinVal;
	uint16_t u16SpeedMinVal;
	uint8_t  u8CurrentGradientMinVal;
	uint8_t  u8CurrentGradientMaxVal;
	uint8_t  u8SpeedGradientMinVal;
	uint8_t  u8SpeedGradientMaxVal;
}TRAMPPARAM;

typedef enum _EAlignCurStatus
{
	RMP_INCREASE,
	RMP_DECREASE,
	RMP_STABLE,

}EAlignCurStatus;

/***************************definition*********************************/

//Motor->USER.s16AlignIqCmd											= Q15(IQ_ALIGN / I_MAX);
//Motor->USER.u16AlignTimeCmd 										= ALIGN_TIME * RMP_SPD_LOOP_FREQ;

#define RAMP_CUR_GRAD_FORWARD_RUN                                   50 //(32768*(1/3.3)) / (2*100)  = 50

//Motor->USER.s16AlignIqCmd											= Q15(IQ_ALIGN / I_MAX);
//Motor->USER.u16AlignTimeCmd 										= ALIGN_TIME * RMP_SPD_LOOP_FREQ;

#define RAMP_CUR_GRAD_BACKWARD_RUN                                  50 //(32768*(1/3.3)) / (2*100)  = 50

//Motor->USER.u16OPToCLSwitchSpd                                    = Q15(STARTUP_TO_SPIN_SPEED / SPEED_MAX);
//#define STARTUP_TIME                                              2.0        //  unit:s
#define RAMP_SPD_GRAD_FORWARD_STARTUP                               12  //(32768*250/5000)/2*100
#define RAMP_SPD_GRAD_BACKWARD_STARTUP                              12  //(32768*250/5000)/2*100

#define RAMP_SPD_GRAD_FORWARD_RUN                                   24  //(32768*250/5000)/2*100
#define RAMP_SPD_GRAD_BACKWARD_RUN                                  24  //(32768*250/5000)/2*100

#define QMATH_SINCOS_INI_DEGREE       (-461)

#define QMATH_SINCOS_INI_DEGREE_DEF   (-510)

#define RMP_CLOSED_LOOP_SW_SPD_REV    (250)

#define RMP_OPTIMIZATION_DATA         {QMATH_SINCOS_INI_DEGREE_DEF,0,0,0}

#define RMP_REF_CAL_DIFF             2


const Q15_t QMATH_SineTable512[]=//sin0-sin90, 90/512 
{
		0,101,201,302,402,503,603,704,
		804,905,1005,1106,1206,1307,1407,1507,
		1608,1708,1809,1909,2009,2110,2210,2310,
		2411,2511,2611,2711,2811,2912,3012,3112,
		3212,3312,3412,3512,3612,3712,3812,3911,
		4011,4111,4211,4310,4410,4510,4609,4709,
		4808,4907,5007,5106,5205,5305,5404,5503,
		5602,5701,5800,5899,5998,6097,6195,6294,
		6393,6491,6590,6688,6787,6885,6983,7081,
		7180,7278,7376,7473,7571,7669,7767,7864,
		7962,8059,8157,8254,8351,8449,8546,8643,
		8740,8836,8933,9030,9127,9223,9319,9416,
		9512,9608,9704,9800,9896,9992,10088,10183,
		10279,10374,10469,10565,10660,10755,10850,10945,
		11039,11134,11228,11323,11417,11511,11605,11699,
		11793,11887,11980,12074,12167,12261,12354,12447,
		12540,12633,12725,12818,12910,13003,13095,13187,
		13279,13371,13463,13554,13646,13737,13828,13919,
		14010,14101,14192,14282,14373,14463,14553,14643,
		14733,14823,14912,15002,15091,15180,15269,15358,
		15447,15535,15624,15712,15800,15888,15976,16064,
		16151,16239,16326,16413,16500,16587,16673,16760,
		16846,16932,17018,17104,17190,17275,17361,17446,
		17531,17616,17700,17785,17869,17953,18037,18121,
		18205,18288,18372,18455,18538,18621,18703,18786,
		18868,18950,19032,19114,19195,19277,19358,19439,
		19520,19601,19681,19761,19841,19921,20001,20081,
		20160,20239,20318,20397,20475,20554,20632,20710,
		20788,20865,20943,21020,21097,21174,21251,21327,
		21403,21479,21555,21631,21706,21781,21856,21931,
		22006,22080,22154,22228,22302,22375,22449,22522,
		22595,22668,22740,22812,22884,22956,23028,23099,
		23170,23241,23312,23383,23453,23523,23593,23663,
		23732,23801,23870,23939,24008,24076,24144,24212,
		24279,24347,24414,24481,24548,24614,24680,24746,
		24812,24878,24943,25008,25073,25138,25202,25266,
		25330,25394,25457,25520,25583,25646,25708,25771,
		25833,25894,25956,26017,26078,26139,26199,26259,
		26320,26379,26439,26498,26557,26616,26674,26733,
		26791,26848,26906,26963,27020,27077,27133,27190,
		27246,27301,27357,27412,27467,27522,27576,27630,
		27684,27738,27791,27844,27897,27950,28002,28054,
		28106,28158,28209,28260,28311,28361,28411,28461,
		28511,28560,28610,28658,28707,28755,28803,28851,
		28899,28946,28993,29040,29086,29132,29178,29224,
		29269,29314,29359,29404,29448,29492,29535,29579,
		29622,29665,29707,29750,29792,29833,29875,29916,
		29957,29997,30038,30078,30118,30157,30196,30235,
		30274,30312,30350,30388,30425,30462,30499,30536,
		30572,30608,30644,30680,30715,30750,30784,30819,
		30853,30886,30920,30953,30986,31018,31050,31082,
		31114,31146,31177,31207,31238,31268,31298,31328,
		31357,31386,31415,31443,31471,31499,31527,31554,
		31581,31608,31634,31660,31686,31711,31737,31761,
		31786,31810,31834,31858,31881,31904,31927,31950,
		31972,31994,32015,32037,32058,32078,32099,32119,
		32138,32158,32177,32196,32214,32233,32251,32268,
		32286,32303,32319,32336,32352,32368,32383,32398,
		32413,32428,32442,32456,32470,32483,32496,32509,
		32522,32534,32546,32557,32568,32579,32590,32600,
		32610,32620,32629,32638,32647,32656,32664,32672,
		32679,32686,32693,32700,32706,32712,32718,32723,
		32729,32733,32738,32742,32746,32749,32753,32756,
		32758,32760,32762,32764,32766,32767,32767,32767,
		32767
};
		

/*	
 * Data INIT VALUES
 * */
/**
    \brief Definition of the data object init values
*/
#define RMP_DATA_OBJ_INIT_VALUES		{0,0,0,FORWARD,FALSE}
#define RMP_PARA_OBJ_INIT_VALUES        {0,0,0,0,0,0,0,0}


/**************************variable***********************************/



static const TOptimizeOpenSpeedData RMP_tOptimizeOpenSpeedDataInit =  RMP_OPTIMIZATION_DATA;
static TOptimizeOpenSpeedData RMP_tOptimizeOpenSpeedData           = RMP_OPTIMIZATION_DATA;


/* Ramp data objects */
static TRmpData RMP_tMotorData    = RMP_DATA_OBJ_INIT_VALUES;
static TRAMPPARAM  RMP_tMotorPara = RMP_PARA_OBJ_INIT_VALUES;
static bool RMP_bAirIntakePresent = false;



/******************************************LOCAL FUNCTION***************************************/

/** \brief		Output sin value.
 *
 *  \details	search the table and output the sin value.
 *
 *  \param
 */
int16_t QMATH_sin(int16_t angle)
{
	int16_t sinValue;

	if(angle >= Q11_MAX)
	{
		angle -= Q11_MAX;
	}
	else if (angle < 0)
	{
		angle += Q11_MAX;
	}
	else 
	{ 

	}

	if (angle <= QMATH_SINCOS_90DEGREE)
	{
		sinValue = QMATH_SineTable512[angle];
	}
	else
	{
		if (angle <= QMATH_SINCOS_180DEGREE)
		{
			sinValue = QMATH_SineTable512[QMATH_SINCOS_180DEGREE - angle];
		}
		else
		{
			if (angle <= QMATH_SINCOS_270DEGREE)
			{
				sinValue = -QMATH_SineTable512[angle - QMATH_SINCOS_180DEGREE];
			}
			else
			{
				sinValue = -QMATH_SineTable512[QMATH_SINCOS_360DEGREE - angle];
			}
		}
	}

	return sinValue;
}

/** \brief		Calculate Trapezoidal curve for current
 *
 *  \details	Calculate Trapezoidal curve.
 *
 *  \param
 */
void RMP_vCalTrapezoidalCurve(uint8_t * pu8Gradient, int16_t * ps16CurrentRefValue, volatile int16_t * ps16TargetValue)
{
	EAlignCurStatus eActionStatus = RMP_STABLE;
	int16_t s16DifferenceValue = 0;
	
	*ps16TargetValue = QMATH_limit(*ps16TargetValue, RMP_tMotorPara.u16CurrentMinVal, RMP_tMotorPara.u16CurrentMaxVal);
    *pu8Gradient     = QMATH_limit(*pu8Gradient, RMP_tMotorPara.u8CurrentGradientMinVal, RMP_tMotorPara.u8CurrentGradientMaxVal);

	s16DifferenceValue = *ps16TargetValue - *ps16CurrentRefValue;
	
	if(QMATH_ABS(s16DifferenceValue) > *pu8Gradient)
	{ 
		if(s16DifferenceValue < 0) 
			{

			eActionStatus = RMP_DECREASE;//negative
			*ps16CurrentRefValue -= *pu8Gradient;
			if(*ps16CurrentRefValue < *ps16TargetValue)
				{
					*ps16CurrentRefValue = *ps16TargetValue;
				}

			}
		else if(s16DifferenceValue > 0)
			{

				eActionStatus = RMP_INCREASE;
				*ps16CurrentRefValue += *pu8Gradient;
				if(*ps16CurrentRefValue > *ps16TargetValue)
				{
					*ps16CurrentRefValue = *ps16TargetValue;
				}

			}
		else
			{
			
			eActionStatus = RMP_STABLE;
				
			}
	}
	else
	{
		*ps16CurrentRefValue = *ps16TargetValue;
	}
	
}


void RMP_vCalcSmootherSinusoidalSpeedRampCurve(uint8_t * pu8Gradient, int16_t * ps16CurrentRefValue, volatile int16_t * ps16TargetValue)
{

	*ps16TargetValue = QMATH_limit(*ps16TargetValue, RMP_tMotorPara.u16SpeedMinVal, RMP_tMotorPara.u16SpeedMaxVal);
    *pu8Gradient      = QMATH_limit(*pu8Gradient, RMP_tMotorPara.u8SpeedGradientMinVal, RMP_tMotorPara.u8SpeedGradientMaxVal);
	
    if( *ps16TargetValue != RMP_tMotorData.tPreFinalRampValue)
    	{
			RMP_tMotorData.tDeltaRampValue = RMP_tMotorData.tFinalRampValue - *ps16CurrentRefValue;
			RMP_tMotorData.tBaseRampValue = *ps16CurrentRefValue;
			RMP_tMotorData.tCurveAngle = QMATH_SINCOS_INI_DEGREE_DEF;
    	}

	if (RMP_tMotorData.tCurveAngle <= QMATH_SINCOS_90DEGREE)
	{

		RMP_tMotorData.tCurveAngle += *pu8Gradient;

		RMP_tMotorData.tFactor = ((QMATH_sin(RMP_tMotorData.tCurveAngle)) / 2) + Q15(0.5f);

		*ps16CurrentRefValue = RMP_tMotorData.tBaseRampValue + MLIB_Mul_Q15(RMP_tMotorData.tFactor,RMP_tMotorData.tDeltaRampValue); 


	}
	else
	{
		*ps16CurrentRefValue = RMP_tMotorData.tBaseRampValue + RMP_tMotorData.tDeltaRampValue;
	}

	RMP_tMotorData.tPreFinalRampValue = RMP_tMotorData.tFinalRampValue;

}


/** \brief		update speed switch value
 *
 *  \details	
 *
 *  \param
 */
void RMP_vUpdateSwitchSpeed(void)
{

	if (RMP_tMotorData.tDirection != FORWARD)
	{
		Motor_1st.USER.u16OPToCLSwitchSpd = Q15(RMP_CLOSED_LOOP_SW_SPD_REV/SPEED_MAX);	//at backward closed loop speed up
	}	
	else
	{
		Motor_1st.USER.u16OPToCLSwitchSpd = Q15(STARTUP_TO_SPIN_SPEED / SPEED_MAX);	// forward go normal closed loop speed
	}

	
}


/** \brief		Inite motor startup algorithm parameters
 *
 *  \details	
 *
 *  \param
 */
void RMP_vConfigureStartupAlgorithm(TDirection tDirection)
{
	if(tDirection == FORWARD)
		{
			/* Alignment setting */
		    Motor_1st.USER.s16AlignIqCmd                                           = Q15(IQ_ALIGN / I_MAX);
		    Motor_1st.USER.u16AlignTimeCmd                                         = ALIGN_TIME * RMP_SPD_LOOP_FREQ;

			/* Starting setting */
		    Motor_1st.USER.s16StartupIqCmd                                         = Q15(IQ_STARTUP / I_MAX);
		    Motor_1st.USER.u16StartupTimeCmd                                       = STARTUP_TIME * RMP_SPD_LOOP_FREQ;


			RMP_tMotorData.u16CurrentGradient                                   = RAMP_CUR_GRAD_FORWARD_RUN;
			RMP_tMotorData.u16SpeedStartupGradient                              = RAMP_SPD_GRAD_FORWARD_STARTUP;
			RMP_tMotorData.u16SpeedRunGradient                                  = RAMP_SPD_GRAD_FORWARD_RUN;
			/* Startup Sine curve setting*/
		}
	else
		{

			/* Alignment setting */
		    Motor_1st.USER.s16AlignIqCmd                                           = -Q15(IQ_ALIGN / I_MAX);
		    Motor_1st.USER.u16AlignTimeCmd                                         = ALIGN_TIME * RMP_SPD_LOOP_FREQ;

			/* Starting setting */
		    Motor_1st.USER.s16StartupIqCmd                                         = -Q15(IQ_STARTUP / I_MAX);
		    Motor_1st.USER.u16StartupTimeCmd                                       = STARTUP_TIME * RMP_SPD_LOOP_FREQ;


			RMP_tMotorData.u16CurrentGradient                                   = RAMP_CUR_GRAD_BACKWARD_RUN;
			RMP_tMotorData.u16SpeedStartupGradient                              = RAMP_SPD_GRAD_BACKWARD_STARTUP;
			RMP_tMotorData.u16SpeedRunGradient                                  = RAMP_SPD_GRAD_BACKWARD_RUN;
			/* Startup Sine curve setting*/
		}
}

TRmpData * RMP_ptGetRampDataPointer(void)
{

	return &RMP_tMotorData;

}




