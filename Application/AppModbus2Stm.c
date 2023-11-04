#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "fsclock.h"
#include "statemachineCom.h"
#include "AppModbus2Stm.h"
#include "Modbus_2.h"
#include "common.h"
/**
 * [ms] Task handler time base
 * */
#define AMS_TASK_TIME_BASE_MS                (250)

/* CONVERSION of the config values to working values */

/**
 *  [SafeTime] Task handler time base
 * */
#define AMS_TASK_TIME_BASE                  STK_DEF_TIME_MS(AMS_TASK_TIME_BASE_MS)

/**
 * [ms] Task handler time base
 * */
#define AMS_TIMOUT_TIME_BASE_MS                (100)

/* CONVERSION of the config values to working values */

/**
 *  [SafeTime] Task handler time base
 * */
#define AMS_TIMOUT_TIME_BASE                  STK_DEF_TIME_MS(AMS_TIMOUT_TIME_BASE_MS)

#define AMS_TIMEOUT_CNT_MAX                   5

typedef struct _T_APP_MOS2_DATA
{
	TSafeTime tTaskEntryTime;
	TSafeTime tLastTimeSliceTime;

	TSafeTime tRecTimeout;
	TSafeTime tLastTimeoutTime;

	uint8_t   u8TimeoutCnt;

	uint8_t   u8TransSwitch;//function code

}T_APP_MOS2_DATA;



void AMS_vInitModbus2Stm(void);
/*******************************state machine*******************************************/
T_EVENT_MAP MOS_EVENT_MAP[] = 
{
	{MOS2_EVENT_STARTUP,	      MOS2_INIT,	MOS2_IDLE  },
	{MOS2_EVENT_TRANS_MSG,	    MOS2_IDLE,	MOS2_TRANS },	
	{MOS2_EVENT_RECEP_MSG,	    MOS2_TRANS,	MOS2_RECEP },
	{MOS2_EVENT_RECEP_SUC,	    MOS2_RECEP,	MOS2_IDLE  },
	{MOS2_EVENT_RECEP_TIMOUT,   MOS2_RECEP, MOS2_TRANS },
  {MOS2_EVENT_RECEP_ERR,      MOS2_RECEP, MOS2_ERROR },
  {MOS2_EVENT_ERROR_FIN,      MOS2_ERROR, MOS2_IDLE  },
};


T_ACTION_MAP MOS_ACTION_MAP[] = 
{
	{MOS2_INIT,	    MOS2_vInit_entry,	MOS2_vInit_do,	MOS2_vInit_exit },
	{MOS2_TRANS,	MOS2_vTrans_entry,	MOS2_vTrans_do,	MOS2_vTrans_exit},
	{MOS2_RECEP,	MOS2_vRecep_entry,	MOS2_vRecep_do,	MOS2_vRecep_exit},
	{MOS2_ERROR,	MOS2_vError_entry,	MOS2_vError_do,	MOS2_vError_exit},
  {MOS2_IDLE,	    MOS2_vIdle_entry,	MOS2_vIdle_do,	MOS2_vIdle_exit },
};

static T_FSM                   AMS_tFSM = {0,0,0,0,0, NULL, NULL};	/* define state machine */
static T_APP_MOS2_DATA AMS_tModbus2Data = {0,0,0,0,0,E_REC_COR_4};
static CallbackFunc2 MOS2_pfStatemachine = AMS_vInitModbus2Stm;

/********************************statemachine******************************************/

/********************************************init**************************************/
void MOS2_vInit_entry(void)
{
    
}
 
void MOS2_vInit_do(void)
{
	FSM_vStateTransfer(&AMS_tFSM, MOS2_EVENT_STARTUP);
}

void MOS2_vInit_exit(void)
{
	
}

/********************************************transmission******************************/
void MOS2_vTrans_entry(void)
{
	
}
void MOS2_vTrans_do(void)
{   
    //Transmit message finish, jump to reception
    /* Get target speed */
	if(AMS_tModbus2Data.u8TransSwitch == E_REC_COR_4)
		{
    		MOS2_vGetTargetSpeed();
		}
	else if(AMS_tModbus2Data.u8TransSwitch == E_REC_COR_6)
		{
			/* write current speed */
			MOS2_vWriteCurrentSpeed();
		}
	FSM_vStateTransfer(&AMS_tFSM, MOS2_EVENT_RECEP_MSG);
}

void MOS2_vTrans_exit(void)
{
	
}

/********************************************Reception*********************************/
void MOS2_vRecep_entry(void)
{
	
}
void MOS2_vRecep_do(void)
{
	uint8_t u8ReturnCode = 0;
	u8ReturnCode =MOS2_u8ModBusRecHandle();
	if((AMS_tModbus2Data.tRecTimeout - AMS_tModbus2Data.tLastTimeoutTime) > AMS_TIMOUT_TIME_BASE)
		{
			AMS_tModbus2Data.u8TimeoutCnt ++;
			/* more than 5 times */
			if(AMS_tModbus2Data.u8TimeoutCnt > AMS_TIMEOUT_CNT_MAX)
				{
					//more than 5 times,
					FSM_vStateTransfer(&AMS_tFSM, MOS2_EVENT_RECEP_ERR);
				}
			else
				{
					//time out less than 5 time jump to transmit
					FSM_vStateTransfer(&AMS_tFSM, MOS2_EVENT_RECEP_TIMOUT);

				}


		}
	else
		{
			//SUCCESS,
			if(u8ReturnCode == E_REC_COR_4)   
				{
					FSM_vStateTransfer(&AMS_tFSM, MOS2_EVENT_RECEP_SUC);
				}
			if(u8ReturnCode == E_REC_COR_6)   
				{
					FSM_vStateTransfer(&AMS_tFSM, MOS2_EVENT_RECEP_SUC);

				}
    
		}



	
}
void MOS2_vRecep_exit(void)
{
	
}

/********************************************Error**************************************/
void MOS2_vError_entry(void)
{
	
}

void MOS2_vError_do(void)
{
    //Jump to error
		//Stop motor and record error
    FSM_vStateTransfer(&AMS_tFSM, MOS2_EVENT_ERROR_FIN);
	
}

void MOS2_vError_exit(void)
{
	
}

/********************************************Idle**************************************/

void MOS2_vIdle_entry(void)
{
	AMS_tModbus2Data.tLastTimeSliceTime = STK_tGetSafeTime();
}

void MOS2_vIdle_do(void)
{
    //if transmit event triggered
    AMS_tModbus2Data.tTaskEntryTime = STK_tGetSafeTime();
    if ((AMS_tModbus2Data.tTaskEntryTime - AMS_tModbus2Data.tLastTimeSliceTime) >= AMS_TASK_TIME_BASE_MS)
    	{
    		if(AMS_tModbus2Data.u8TransSwitch == E_REC_COR_4)
    			{
    				AMS_tModbus2Data.u8TransSwitch = E_REC_COR_6;
    			}
			else if(AMS_tModbus2Data.u8TransSwitch == E_REC_COR_6)
				{
					AMS_tModbus2Data.u8TransSwitch = E_REC_COR_4;
				}
			
			AMS_tModbus2Data.tLastTimeSliceTime = AMS_tModbus2Data.tTaskEntryTime;
			FSM_vStateTransfer(&AMS_tFSM, MOS2_EVENT_TRANS_MSG);
    	}
    
}

void MOS2_vIdle_exit(void)
{
	
}

/*****************************************statemachine function********************************/
/* get action table */
T_ACTION_MAP * MOS2_vGetActionMap(uint8_t *total)
{
	*total = sizeof(MOS_ACTION_MAP)/sizeof(T_ACTION_MAP);
	return MOS_ACTION_MAP;
}

/* get event table */
T_EVENT_MAP * MOS_vGetEventMap(uint8_t *total)
{
	*total = sizeof(MOS_EVENT_MAP)/sizeof(T_EVENT_MAP);
	return MOS_EVENT_MAP;
}

/* Init modbus statemachine */
void AMS_vInitModbus2Stm(void)
{
	int i = 0;		
	uint8_t u8ActNum = 0;
    uint8_t u8EventNum = 0;
	T_ACTION_MAP * stActMap;
	T_EVENT_MAP * stEventMap;
	
	stActMap = MOS2_vGetActionMap(&u8ActNum);
	stEventMap = MOS_vGetEventMap(&u8EventNum);
	
	FSM_vInit(&AMS_tFSM,stEventMap,stActMap,u8EventNum,u8ActNum,MOS2_INIT);


}

/* statemachine implementation */
void AMS_vModbus2StatemachineRun(void)
{
	FSM_vActionPerfrom(&AMS_tFSM);

}

void AMS_vModbus2Statemachine(void)
{
	MOS2_pfStatemachine();
	MOS2_pfStatemachine = AMS_vModbus2StatemachineRun;


}
