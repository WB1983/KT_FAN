//NTC and Temperature
#include "fsmonitor.h"
#include "voltage.h"
#include "LibDefines.h"
#include "drv_inc.h"
#include "paramHw.h"
#include "ErrorReact.h"
/**********************************************definition************************************************/
#define FTE_NTC_TABLE_ELE                  50         /*50 element*/
#define FTE_NTC_CIRCUIT_RES                (10e3)     /*NTC Pull up resistor*/
#define FTE_SAMPLE_COUNT                   1024       /* ADC sample count */
#define FTE_TEMP_SLOP                      4.26*4095  /*mv/degree*/
/**********************************************typedef***************************************************/
/*Tempeature task ID*/
typedef enum _ETEMPTASKID
{
	E_TEMP_INIT,
	E_TEMP_RUN,
	E_TEMP_EVALUATION,
}ETEMPTASKID;

/*NTC talbe elements*/
typedef struct _TNTCTABLEELEMENT
{
	uint16_t u16Resistor;
	uint16_t u16Temperature;
}TNTCTABLEELEMENT;

/*NTC and Internal Temperature*/
typedef struct _TNTCTEMPCal
{
	ETEMPTASKID u8TaskState;
	uint16_t u16DigitalVol;
	uint16_t u16CalDeltaTemp;
	uint16_t u16CalDeltaRes;
	uint16_t u16EleDetalTemp;
	uint16_t u16CurTemp;
	uint16_t u16CurBaseTemp;

	uint32_t u32InternalTempSum;
	uint32_t u32NTCTempSum;
	uint16_t u16InternalTempAvg;
	uint16_t u16NTCTempAvg;
	uint16_t u16FilterCount;
	bool bFilterDone;

	uint16_t u16TemperatureADCRef;
	uint16_t u16IntTempVol;
	uint32_t u32Slop;
	int32_t s32CurrentTemperature;

	uint8_t u8NTCTempErrorCnt;
	uint8_t u8IntTempHighErrorCnt;
	uint8_t u8IntTempLowErrorCnt;
}TNTCTEMPCal;

/**********************************************const and variables*****************************************/
const TNTCTABLEELEMENT FTE_au16NTCTemperatureArray[FTE_NTC_TABLE_ELE] =
{
{3501,	20},
//{3478,	21},
{3453,	22},
//{3429,	23},
{3403,	24},
//{3377,	25},
{3350,	26},
//{3323,	27},
{3295,	28},
//{3266,	29},
{3237,	30},
//{3207,	31},
{3177,	32},
//{3146,	33},
{3114,	34},
//{3082,	35},
{3049,	36},
//{3016,	37},
{2982,	38},
//{2948,	39},
{2913,	40},
//{2878,	41},
{2843,	42},
//{2807,	43},
{2771,	44},
//{2734,	45},
//{2697,	46},
{2659,	47},
//{2622,	48},
{2584,	49},
//{2546,	50},
{2508,	51},
//{2470,	52},
{2431,	53},
//{2393,	54},
{2354,	55},
//{2316,	56},
{2277,	57},
//{2239,	58},
{2200,	59},
//{2162,	60},
{2123,	61},
//{2085,	62},
{2047,	63},
//{2009,	64},
{1972,	65},
//{1934,	66},
{1897,	67},
//{1860,	68},
{1823,	69},
//{1787,	70},
{1751,	71},
//{1715,	72},
{1680,	73},
//{1645,	74},
{1610,	75},
//{1576,	76},
{1543,	77},
//{1509,	78},
{1476,	79},
//{1444,	80},
{1412,	81},
//{1381,	82},
{1350,	83},
//{1320,	84},
{1290,	85},
//{1261,	86},
{1232,	87},
//{1204,	88},
{1176,	89},
//{1148,	90},
{1122,	91},
//{1095,	92},
{1069,	93},
//{1044,	94},
{1019,	95},
//{995,	    96},
{971,	    97},
//{948,	    98},
{925,	    99},
//{903,	   100},
{881,	   101},
//{859,	   102},
{838,	   103},
//{818,	   104},
{798,	   105},
//{778,	   106},
{759,	   107},
//{741,	   108},
{722,	   109},
//{704,	   110},
{687,	   111},
//{670,	   112},
{653,	   113},
//{637,	   114},
{621,	   115},
//{606,	   116},
{591,	   117},
//{576,	   118},
{562,	   119},
//{548,	   120},
};

static TNTCTEMPCal FTE_tNTCTempCal = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
/*****************************************************Local function***********************************************/
/*
*average value filter
*/
bool FTE_bDataFilter(void)
{
	
	//Average filter
	if(FTE_tNTCTempCal.u16FilterCount < FTE_SAMPLE_COUNT)
	{
		
		FTE_tNTCTempCal.u16FilterCount ++;
		FTE_tNTCTempCal.u32InternalTempSum   += VOL_u16NormalAdcValueArray[E_VTEMP];
		FTE_tNTCTempCal.u32NTCTempSum        += VOL_u16NormalAdcValueArray[E_IPM_NTC_TEMP];
	}
	else if(FTE_tNTCTempCal.u16FilterCount == FTE_SAMPLE_COUNT)
	{
		FTE_tNTCTempCal.u16FilterCount       = 0;
		FTE_tNTCTempCal.u16InternalTempAvg   =  FTE_tNTCTempCal.u32InternalTempSum	 >> 10;
		FTE_tNTCTempCal.u16NTCTempAvg        =  FTE_tNTCTempCal.u32NTCTempSum >> 10;

		FTE_tNTCTempCal.u32InternalTempSum   = 0;
		FTE_tNTCTempCal.u32NTCTempSum        = 0;
		
		FTE_tNTCTempCal.bFilterDone = true;
	
	}
	
	return FTE_tNTCTempCal.bFilterDone;

}

/*
*Init const value
*/
void FTE_vTemperatueInit(void)
{
	FTE_tNTCTempCal.u16TemperatureADCRef = 0x0fff & (M16(ADC_TEMPERATURE_BASE));//(Current vol -reference vol)*slope + 25==>(current dig*5000 - ref dig*3300)/(4096*slope') + 25
 	FTE_tNTCTempCal.u32Slop              = FTE_TEMP_SLOP;
}

 /*
 *Calculate current MCU temperature
 */
 void FTE_vCalculateTemperature(void)
 {
	int32_t s32TempVal = 0;
	FTE_tNTCTempCal.u16IntTempVol = FTE_tNTCTempCal.u16InternalTempAvg;
	s32TempVal = FTE_tNTCTempCal.u16IntTempVol*5000 - FTE_tNTCTempCal.u16TemperatureADCRef*3300;
	FTE_tNTCTempCal.s32CurrentTemperature = Division(s32TempVal, FTE_tNTCTempCal.u32Slop);
	FTE_tNTCTempCal.s32CurrentTemperature += VOL_REF_VOL;

 }

void FTE_vCalculateNTCTemperature(void)
{
	int32_t FTE_s32Temp = 0;
	uint8_t i = 0;
	//calculate Resistor
	FTE_tNTCTempCal.u16DigitalVol = FTE_tNTCTempCal.u16NTCTempAvg;

	if(FTE_tNTCTempCal.u16DigitalVol >= FTE_au16NTCTemperatureArray[0].u16Resistor)//lower than 40 degree.
		{
			return;
		}
	else if(FTE_tNTCTempCal.u16DigitalVol < FTE_au16NTCTemperatureArray[FTE_NTC_TABLE_ELE - 1].u16Resistor)
	{
		return;
	}
	else
	{
		for(i=0; i< FTE_NTC_TABLE_ELE; i++)
		{
			if(FTE_tNTCTempCal.u16DigitalVol < FTE_au16NTCTemperatureArray[i].u16Resistor)//lower than 40 degree.
			{
				
			}
			else
			{
				FTE_tNTCTempCal.u16CalDeltaTemp = FTE_au16NTCTemperatureArray[i].u16Temperature - FTE_au16NTCTemperatureArray[i-1].u16Temperature;
				FTE_tNTCTempCal.u16CalDeltaRes = FTE_au16NTCTemperatureArray[i-1].u16Resistor -FTE_au16NTCTemperatureArray[i].u16Resistor;
				
				if(FTE_tNTCTempCal.u16CalDeltaRes <= 0)
				{
					FTE_tNTCTempCal.u16CurTemp =  FTE_au16NTCTemperatureArray[i].u16Temperature;
					break;
				}
				else
				{
					FTE_s32Temp = (FTE_tNTCTempCal.u16DigitalVol - FTE_au16NTCTemperatureArray[i].u16Resistor)<<7;
					FTE_s32Temp = Division(FTE_s32Temp,FTE_tNTCTempCal.u16CalDeltaRes);
					FTE_s32Temp = (FTE_s32Temp*FTE_tNTCTempCal.u16CalDeltaTemp)>>7;
					FTE_tNTCTempCal.u16CurBaseTemp = FTE_au16NTCTemperatureArray[i].u16Temperature;
					FTE_tNTCTempCal.u16CurTemp = FTE_tNTCTempCal.u16CurBaseTemp + FTE_s32Temp;
					break;
				}

			}

		}
	}

}

/*
* Evaluate NTC and MCU internal temperature
*/
void FTE_vEvaluationTemperature(void)
{
//IPM NTC tempearture
	if(FTE_tNTCTempCal.u16CurTemp > FTE_NTC_TEMPERATURE_THRESHOLD)
	{
		if(FTE_tNTCTempCal.u8NTCTempErrorCnt > FTE_NTC_ERROR_CNT)
			{
			
			}
		else
			{
				FTE_tNTCTempCal.u8NTCTempErrorCnt ++;
			}
		
	}
	else
	{
		FTE_tNTCTempCal.u8NTCTempErrorCnt = 0;
		ERT_vResetErrorCode(ERT_FAUYLT_NTC_TEMP_HIGH);
	}

	if(FTE_tNTCTempCal.u8NTCTempErrorCnt > FTE_NTC_ERROR_CNT)
	{
		//Error Handling
		ERT_vErrorReport(ERT_FAUYLT_NTC_TEMP_HIGH);
	}

//internal temperature
	if(FTE_tNTCTempCal.s32CurrentTemperature > FTE_INT_TEMPERATURE_HIGH_THRESHOLD)
	{
		if(FTE_tNTCTempCal.u8IntTempHighErrorCnt > FTE_NTC_ERROR_CNT)
			{
			
			}
		else
			{
				FTE_tNTCTempCal.u8IntTempHighErrorCnt ++;
			}
	}
	else
	{
		FTE_tNTCTempCal.u8IntTempHighErrorCnt = 0;
		ERT_vResetErrorCode(ERT_FAUYLT_MCU_TEMP_HIGH);
	}

	if(FTE_tNTCTempCal.u8IntTempHighErrorCnt > FTE_NTC_ERROR_CNT)
	{
		//Error Handling
		ERT_vErrorReport(ERT_FAUYLT_MCU_TEMP_HIGH);
	}


	if(FTE_tNTCTempCal.s32CurrentTemperature < FTE_INT_TEMPERATURE_LOW_THRESHOLD)
	{
		if(FTE_tNTCTempCal.u8IntTempLowErrorCnt > FTE_NTC_ERROR_CNT)
			{
			}
		else
			{
				FTE_tNTCTempCal.u8IntTempLowErrorCnt ++;
			}
	}
	else
	{
		FTE_tNTCTempCal.u8IntTempLowErrorCnt = 0;
		ERT_vResetErrorCode(ERT_FAUYLT_MCU_TEMP_LOW);
	}

	if(FTE_tNTCTempCal.u8IntTempLowErrorCnt > FTE_NTC_ERROR_CNT)
	{
		//Error Handling
		ERT_vErrorReport(ERT_FAUYLT_MCU_TEMP_LOW);
	}

}

/* 
* Temperature task handler
*/
void FTE_vTemperatureCal(void)
{
	switch(FTE_tNTCTempCal.u8TaskState)
	{
		case E_TEMP_INIT:
		{
			FTE_vTemperatueInit();
			FTE_tNTCTempCal.u8TaskState = E_TEMP_RUN;
			break;

		}
		case E_TEMP_RUN:
		{
			/*******Temperature detection **********/
			if(FTE_bDataFilter() == true)
			{
				FTE_tNTCTempCal.bFilterDone = false;

				FTE_vCalculateTemperature();
				FTE_vCalculateNTCTemperature();
				FTE_tNTCTempCal.u8TaskState = E_TEMP_EVALUATION;
			}
			break;
		}
		case E_TEMP_EVALUATION:
		{
			FTE_vEvaluationTemperature();
			FTE_tNTCTempCal.u8TaskState = E_TEMP_RUN;
			break;
		}
		default:
		{
			FTE_tNTCTempCal.u8TaskState = E_TEMP_INIT;
			break;
		}

	}

	
}