#include "drv_inc.h"
#include "StateMachine.h"
#include "parameter.h"
#include "FOC_PMSM.h"
#include "Compile.h"
#include "Ramp.h"
#include "main.h"
#include "WindDetection.h"
#include "Current.h"

#if (WDN_WIND_DETECTION_METHOD_1 == OPTION_ACTIVE)

typedef struct _TWindDetectionData
{
	int16_t u16TotalDetectTimeMs;
	int16_t u16ActualDetectTimeMs;
	bool    bIsLowSpeedStartup;
	bool    bIsHighSpeedStartup;
	bool    bIsDetectionFinish;

}TWindDetectionData;

typedef struct _TWindDetectionPara
{
	int16_t u16MinDetectTimeMs;
	int16_t u16BrakeTimeForLowSpeed;
	int16_t u16BrakeTimeForHighSpeed;

}TWindDetectionPara;

TOverThresholdParm WDN_tOverCurrentZeroPara = {2750, 20, 2400};//normal value
	
TOverThresholdData WDN_tOverCurrentZeroData = {0,0,0,0};

TOverThresholdParm WDN_tOverCurrentZeroHighSpdPara = {2850, 15, 2400};
	
TOverThresholdData WDN_tOverCurrentZeroHighSpdData = {0,0,0,0};


TWindDetectionData WDN_tWindDetectionData = {0,0,false,false,false};
TWindDetectionPara WDN_tWindDetectionPara = {50,100,200};//50 2000 4000

uint16_t u16ThreePhaseRMSCur = 0;//Three phase current RMS value

//For Debug
static uint8_t WDN_U8Test1 = 0;
static uint8_t WDN_U8Test2 = 0;
static uint8_t WDN_U8Test3 = 0;



#elif (WDN_WIND_DETECTION_METHOD_2 == OPTION_ACTIVE)

#define WDN_DEGREE_0_UP                    500
#define WDN_DEGREE_0_DO                      0

#define WDN_DEGREE_45_POS_UP              9000
#define WDN_DEGREE_45_POS_DO              8000

#define WDN_DEGREE_90_POS_UP             17000
#define WDN_DEGREE_90_POS_DO             16000

#define WDN_DEGREE_135_POS_UP            25000
#define WDN_DEGREE_135_POS_DO            24000

#define WDN_DEGREE_180_POS_UP            32767
#define WDN_DEGREE_180_POS_DO            32000

#define WDN_DEGREE_135_NEG_UP           -25000
#define WDN_DEGREE_135_NEG_DO           -24000

#define WDN_DEGREE_90_NEG_UP            -16384
#define WDN_DEGREE_90_NEG_DO            -16000

#define WDN_DEGREE_45_NEG_UP            -9000
#define WDN_DEGREE_45_NEG_DO            -8000

#define WDN_BEMF_PEAK_THRESHOLD_HI      1000
#define WDN_BEMF_PEAK_THRESHOLD_LO       300


#define WDN_WIND_MIN_DET_TIM        100//ms
#define WDN_WIND_MIN_DET_TIM_MS     WDN_WIND_MIN_DET_TIM*PWMFREQ/1000 // detect 100ms at least

typedef struct _TWindDetectionData
{
	int16_t u16DetectTimeMs;
	int16_t u16ActualDetectTimeMs;
	int16_t s16BemfPeak;
	int16_t s16BemfPeakFlag;

}TWindDetectionData;

typedef struct _TWindDetectionPara
{
	int16_t u16MinDetectTimeMs;


}TWindDetectionPara;

TWindDetectionData WDN_tWindDetectionData = {0,0,0,0};
TWindDetectionPara WDN_tWindDetectionPara = {WDN_WIND_MIN_DET_TIM_MS};


//for test
static uint8_t WDN_u8TestSec_1;
static uint8_t WDN_u8TestSec_2;
static uint8_t WDN_u8TestSec_3;
static uint8_t WDN_u8TestSec_4;
static uint8_t WDN_u8TestSec_5;




#endif

#if (WDN_WIND_DETECTION_METHOD_1 == OPTION_ACTIVE)

bool WDN_bEstimateStartupScenario(void)
{
	//bool bReturnValue = false;
	CUR_vCalThreePhaseCurRMS();
	u16ThreePhaseRMSCur = CUR_u16GetThreePhaseRMS();

	if (WDN_tWindDetectionData.u16TotalDetectTimeMs < UINT16_MAX)
		{
			WDN_tWindDetectionData.u16TotalDetectTimeMs ++;
		}
	

	WDN_tOverCurrentZeroData.s16Input = u16ThreePhaseRMSCur;
	COM_vCheckOverThresholdLimit(&WDN_tOverCurrentZeroPara, &WDN_tOverCurrentZeroData);

	WDN_tOverCurrentZeroHighSpdData.s16Input = u16ThreePhaseRMSCur;
	COM_vCheckOverThresholdLimit(&WDN_tOverCurrentZeroHighSpdPara, &WDN_tOverCurrentZeroHighSpdData);

	WDN_tWindDetectionData.u16ActualDetectTimeMs ++;
	
	if (true == WDN_tOverCurrentZeroData.bStatus)
		{
			WDN_tWindDetectionData.bIsLowSpeedStartup = true;
		}
	/*else
		{
			WDN_tWindDetectionData.bIsLowSpeedStartup = false;
		}
	*/
	if (true == WDN_tOverCurrentZeroHighSpdData.bStatus)
		{
			WDN_tWindDetectionData.bIsHighSpeedStartup = true;
		}
	/*else
		{
			WDN_tWindDetectionData.bIsHighSpeedStartup = false;
		}
	*/
	if (WDN_tWindDetectionData.u16ActualDetectTimeMs >= WDN_tWindDetectionPara.u16MinDetectTimeMs)
	{
		if (true == WDN_tWindDetectionData.bIsHighSpeedStartup)
		{
			if (WDN_tWindDetectionData.u16ActualDetectTimeMs >= WDN_tWindDetectionPara.u16BrakeTimeForHighSpeed)
			{
				WDN_tWindDetectionData.bIsDetectionFinish = true;
			}
		}
		else if (true == WDN_tWindDetectionData.bIsLowSpeedStartup)
		{
			if (WDN_tWindDetectionData.u16ActualDetectTimeMs >= WDN_tWindDetectionPara.u16BrakeTimeForLowSpeed)
			{
				WDN_tWindDetectionData.bIsDetectionFinish = true;
			}
		}
		else
		{
			WDN_tWindDetectionData.bIsDetectionFinish = true;
		}

	}

	/*if(WDN_tWindDetectionData.bIsDetectionFinish == true)
	{
		bReturnValue = true;
	}

	return bReturnValue;
	*/
	return WDN_tWindDetectionData.bIsDetectionFinish;
}

void WDN_vAnalyzeStartPower(void)
{

	if (true == WDN_tWindDetectionData.bIsDetectionFinish)
	{
		if (true == WDN_tWindDetectionData.bIsHighSpeedStartup)
		{
			//Align time
			//Align current
			//open loop current
			WDN_vConfigureStartupParam(1.0,1.0,3,3);
			WDN_U8Test1 ++;
		}
		else if (true == WDN_tWindDetectionData.bIsLowSpeedStartup)
		{
			//Align time
			//Align current
			//open loop current
			WDN_vConfigureStartupParam(1.0,1.0,3,2);
			WDN_U8Test2 ++;

		}
		else
		{
			// use normal value
			//Align time
			//Align current
			//open loop current
			//WDN_vConfigureStartupParam(0.1,0.1,2,2);
			//use default value
			WDN_U8Test3 ++;

		}
	}
}

#elif (WDN_WIND_DETECTION_METHOD_2 == OPTION_ACTIVE)
void WDN_vInitWindDetection(void)
{

	Motor_1st.FOC.sIdq_cmd.s16Q = 0;
	Motor_1st.FOC.sIdq_cmd.s16D = 0;

	Motor_1st.FOC.s16CmdTheta = 0;

	MCFLIB_Clark_Sat_S16(&Motor_1st.FOC.sIabc, &Motor_1st.FOC.sIAlBe);

	/* Calculate the sincos of  s16CmdTheta */
	Motor_1st.FOC.sAngle_cmd = MLIB_SinCos(Motor_1st.FOC.s16CmdTheta);
	
    /* PARK  transform */
    MCFLIB_Park_S16(&Motor_1st.FOC.sIAlBe, &Motor_1st.FOC.sAngle_cmd, &Motor_1st.FOC.sIdq);
	  

	Motor_1st.FOC.sVdq.s16D = 0;
	Motor_1st.FOC.sVdq.s16Q = 1;

	/* Update the sincos for invpark */
	Motor_1st.FOC.sAngle_cmd_update = Motor_1st.FOC.sAngle_cmd;
	
	/* Inv PARK transform*/
	MCFLIB_InvPark_S16(&Motor_1st.FOC.sVdq, &Motor_1st.FOC.sAngle_cmd_update, &Motor_1st.FOC.sVAlBe);
		
	/* Elimination of the DC-bus voltage ripple  */
    DcBus_Ripple_suppress(&Motor_1st.FOC.sVAlBe, Motor_1st.FOC.s16VbusAvg, &Motor_1st.FOC.sVAlBe_cmd);

	Motor_1st.FOC.SVM.s16U_Alpha = Motor_1st.FOC.sVAlBe_cmd.s16Alpha;
    Motor_1st.FOC.SVM.s16U_Beta = Motor_1st.FOC.sVAlBe_cmd.s16Beta;

    Motor_1st.FOC.SVM.u16sector = MCF_Svm_7(&Motor_1st.FOC.SVM);

    PWM_Update(&Motor_1st);
	
	WDN_tWindDetectionData.s16BemfPeak = 0;

}

void WDN_vWindDetection(void)
{
	if(WDN_tWindDetectionData.u16DetectTimeMs < WDN_tWindDetectionPara.u16MinDetectTimeMs)
		{
			WDN_tWindDetectionPara.u16MinDetectTimeMs ++;
		}
	else
		{

			WDN_tWindDetectionData.u16DetectTimeMs = 0;
			if(Motor_1st.FOC.SMO.s16Speed >= 0) 
			  {
					if(WDN_tWindDetectionData.s16BemfPeak >= WDN_BEMF_PEAK_THRESHOLD_HI) //startup directly for on the fly
					  {
						Motor_1st.FOC.sIdq_cmd.s16Q = 2000;//This value need negotiate.
						Motor_1st.FOC.sIdq_cmd.s16D = 0;
						WDN_u8TestSec_1 ++;
						//M1_SwitchRunWindDetSpin();	 
					  }
					  else if(WDN_tWindDetectionData.s16BemfPeak >= WDN_BEMF_PEAK_THRESHOLD_LO) //brake, startup
					  {
							//TIM_vBrakeInit(10000);
							WDN_u8TestSec_2 ++;
					  }
					  else
					  {
							WDN_AlignInit();
							WDN_u8TestSec_3 ++;
					  }
			  }
			  else 
			  {
					 if(WDN_tWindDetectionData.s16BemfPeak >= WDN_BEMF_PEAK_THRESHOLD_LO) //brake, startup
						 {
							TIM_vBrakeInit(15000);
							WDN_u8TestSec_4 ++;
						 }
						 else
						 {
							   WDN_AlignInit();
							   WDN_u8TestSec_5 ++;
						 }
			  }		

		}


	Motor_1st.FOC.sIdq_cmd.s16Q = 0;
	Motor_1st.FOC.sIdq_cmd.s16D = 0;

	MCFLIB_Clark_Sat_S16(&Motor_1st.FOC.sIabc, &Motor_1st.FOC.sIAlBe);

	PMSM_Obs(&Motor_1st);

	WDN_vCalculateBemfPeak();

	FOC_Current_Controller(&Motor_1st);
	
	Motor_1st.FOC.SVM.s16U_Alpha = Motor_1st.FOC.sVAlBe_cmd.s16Alpha;
	Motor_1st.FOC.SVM.s16U_Beta = Motor_1st.FOC.sVAlBe_cmd.s16Beta;
	
	Motor_1st.FOC.SVM.u16sector = MCF_Svm_7(&Motor_1st.FOC.SVM);
	
	PWM_Update(&Motor_1st);


}


void WDN_vCalculateBemfPeak(void)
{

	//record bemf peak value:90 degree，e_alpha max; 0 degree，e_beta max	  
		  if((Motor_1st.FOC.s16CmdTheta>=WDN_DEGREE_0_DO)&&(Motor_1st.FOC.s16CmdTheta<WDN_DEGREE_0_UP))
		  {
				WDN_tWindDetectionData.s16BemfPeak = Motor_1st.FOC.SMO.s16Ebeta_est;
				if(WDN_tWindDetectionData.s16BemfPeakFlag==0) 
					{
						WDN_tWindDetectionData.s16BemfPeakFlag = 1;
					}
		  }
	else if((Motor_1st.FOC.s16CmdTheta>WDN_DEGREE_45_POS_DO)&&(Motor_1st.FOC.s16CmdTheta<WDN_DEGREE_45_POS_UP))
		  {
				WDN_tWindDetectionData.s16BemfPeakFlag = 0;
		  } 			  
		  else if((Motor_1st.FOC.s16CmdTheta>WDN_DEGREE_90_POS_DO)&&(Motor_1st.FOC.s16CmdTheta<WDN_DEGREE_90_POS_UP))
		  {
				WDN_tWindDetectionData.s16BemfPeak = Motor_1st.FOC.SMO.s16Ealpha_est;
				if(WDN_tWindDetectionData.s16BemfPeakFlag==0) 
					{
						WDN_tWindDetectionData.s16BemfPeakFlag = 1;
					}
		  }
		  else if((Motor_1st.FOC.s16CmdTheta>WDN_DEGREE_135_POS_DO)&&(Motor_1st.FOC.s16CmdTheta<WDN_DEGREE_135_POS_UP))
		  {
				WDN_tWindDetectionData.s16BemfPeakFlag = 0;
		  }   
		  else if((Motor_1st.FOC.s16CmdTheta>WDN_DEGREE_180_POS_DO)&&(Motor_1st.FOC.s16CmdTheta<WDN_DEGREE_180_POS_UP))
		  {
				WDN_tWindDetectionData.s16BemfPeak = -Motor_1st.FOC.SMO.s16Ebeta_est;
				if(WDN_tWindDetectionData.s16BemfPeakFlag==0) 
					{
						WDN_tWindDetectionData.s16BemfPeakFlag = 1;
					}
		  }
		  else if((Motor_1st.FOC.s16CmdTheta>WDN_DEGREE_135_NEG_UP)&&(Motor_1st.FOC.s16CmdTheta<WDN_DEGREE_135_NEG_DO))
		  {
				WDN_tWindDetectionData.s16BemfPeakFlag = 0;
		  }   
		  else if((Motor_1st.FOC.s16CmdTheta>WDN_DEGREE_90_NEG_UP)&&(Motor_1st.FOC.s16CmdTheta<WDN_DEGREE_90_NEG_DO))
		  {
				WDN_tWindDetectionData.s16BemfPeak = -Motor_1st.FOC.SMO.s16Ealpha_est;
				if(WDN_tWindDetectionData.s16BemfPeakFlag==0) 
					{
						WDN_tWindDetectionData.s16BemfPeakFlag = 1;
					}
		  }
		  else if((Motor_1st.FOC.s16CmdTheta>WDN_DEGREE_45_NEG_UP)&&(Motor_1st.FOC.s16CmdTheta<WDN_DEGREE_45_NEG_DO))
		  {
				WDN_tWindDetectionData.s16BemfPeakFlag = 0;
		  }


}

/****************************************************/
void WDN_AlignInit(void)
{
	//Startup parameter update
	  M1_SwitchRunWindDetAlign();

}
#endif
void WDN_vConfigureStartupParam(float fAlignCur, float fStartupCur, float fAlignTime, float fStartupTime)
{

    Motor_1st.USER.s16AlignIqCmd                                           = Q15(fAlignCur / I_MAX);
    Motor_1st.USER.u16AlignTimeCmd                                         = fAlignTime * SLOWLOOP_FREQ;
    Motor_1st.USER.s16StartupIqCmd                                         = Q15(fStartupCur / I_MAX);
    Motor_1st.USER.u16StartupTimeCmd                                       = fStartupTime * SLOWLOOP_FREQ;


}

