/*
* common part: write error code to FLASH
* flash LED lamp for different error code.
* error reaction
* different part: error recovery mechanism
*
*/
/*******************************Include*********************************************/
#include <stdlib.h>
#include <stdbool.h>

#include "ErrorReact.h"
#include "SIMEEP.h"
#include "drv_inc.h"
#include "ErrorReact.h"
#include "Statemachine.h"

/*******************************definition*****************************************/
#define ERT_BYTE_ERROR_CNT                8
#define ERT_BYTE_INFOR_CNT                8


#define FAULTRELEASE_TIME_MIDDLE    60 //2 minutes
#define FAULTRELEASE_TIME_LEVEL2    FAULTRELEASE_TIME_MIDDLE* SLOWLOOP_FREQ;

#define ERT_MAX_ERROR_CNT           10

typedef enum _E_ERRORTASKID
{
	E_FLASH_INIT,
	E_RECORD_RUN,	
	E_READ_RUN,
	E_IDLE,
}E_ERRORTASKID;

#define ERT_GENERIC_ERROR_LEVEL_1   				   (                               \
														MC_FAULT_I_DCBUS_OVER_MASK    |\
														MC_FAULT_U_DCBUS_UNDER_MASK   |\
														MC_FAULT_U_DCBUS_UNDER_MASK   |\
														ERT_15V_VOL_HIGH_MASK         |\
														ERT_INT_REF_VOL_HIGH_MASK     |\
														ERT_15V_VOL_LOW_MASK          |\
														ERT_INT_REF_VOL_LOW_MASK      |\
														ERT_NTC_TEMP_HIGH_MASK        |\
														ERT_MCU_TEMP_HIGH_MASK        |\
														ERT_MCU_TEMP_LOW_MASK)
					

#define ERT_GENERIC_ERROR_LEVEL_2                       (                               \
														ERT_SAFE_TIME_OUTOF_RANGE_MASK |\
														MC_FAULT_LOAD_OVER_MASK        |\
														MC_FAULT_SPEED_OVER_MASK       |\
														MC_FAULT_ROTOR_BLOCKED_MASK    |\
														MC_FAULT_PHASELESS_MASK        |\
														MC_FAULT_SPEED_INFOR_MASK      |\
														MC_FAULT_STARTUP_LOSS_SYN_MASK)

#define ERT_GENERIC_ERROR_LEVEL_3    	                (\
														MCU_ERT_FAUYLT_RAM_TEST_CALLEE_MASK       |\
														MCU_ERT_FAUYLT_RAM_TEST_COMPLETED_MASK    |\
														MCU_ERT_FAUYLT_SYSTICK_INIT_CALLEE_MASK   |\
														MCU_ERT_FAUYLT_CLOCK_TEST_CALLER_MASK     |\
														MCU_ERT_FAUYLT_CPU_TEST_CALLER_MASK       |\
														MCU_ERT_FAUYLT_STACK_OVERFLOW_TEST_MASK   |\
														MCU_ERT_FAUYLT_DELTA_MAIN_MASK            |\
														MCU_ERT_FAUYLT_FLASH_TEST_CALLER_MASK     |\
														MCU_ERT_FAUYLT_LAST_DELTA_MAIN_MASK       |\
														MCU_ERT_FAUYLT_WDG_TEST_CALLER_MASK       |\
														MCU_ERT_FAUYLT_CRC32_TEST_CALLER_MASK     |\
														MCU_ERT_FAUYLT_CHECKPOINT1_MASK           |\
														MCU_ERT_FAUYLT_CHECKPOINT2_MASK)		


/********************************variables****************************************/
TERRORDATA ERT_tErrorData           = {0,0,0,0,false,{0,0},{0,0}};

static uint8_t * ERT_pu8Buffer      = NULL;
static uint16_t ERT_u16FaultTimeCnt = 0;
static uint8_t  ERT_u8CurrentHighestProrityErrorId = 0;

static uint8_t  ERT_auFlashOperationBuffer[ERT_BYTE_ERROR_CNT + ERT_BYTE_INFOR_CNT];

const TERRORTABLEELEMENT ERT_tErrorTable[] =
{

	{ MC_FAULT_I_DCBUS_OVER_MASK                ,E_ERROR_LEVEL_1 },  /* OverCurrent fault flag */
	{ MC_FAULT_U_DCBUS_UNDER_MASK               ,E_ERROR_LEVEL_1 }, /* Undervoltage fault flag */
	{ MC_FAULT_U_DCBUS_OVER_MASK                ,E_ERROR_LEVEL_1 },/* Overvoltage fault flag */
		

	{ MC_FAULT_LOAD_OVER_MASK                   ,E_ERROR_LEVEL_2 },   /* Overload fault flag */
	{ MC_FAULT_SPEED_OVER_MASK                  ,E_ERROR_LEVEL_2 },   /* Over speed fault flag */
	{ MC_FAULT_ROTOR_BLOCKED_MASK               ,E_ERROR_LEVEL_2 },/* Blocked rotor fault flag */
	{ MC_FAULT_PHASELESS_MASK                   ,E_ERROR_LEVEL_2 },			/* Phasee less fault flag */
	{ MC_FAULT_SPEED_INFOR_MASK                 ,E_ERROR_LEVEL_2 },  /* Speed inforamtion error */
	{ MC_FAULT_STARTUP_LOSS_SYN_MASK            ,E_ERROR_LEVEL_2 },/* under startup Lost synchronization error */
		

	{ ERT_SAFE_TIME_OUTOF_RANGE_MASK            ,E_ERROR_LEVEL_2 },
	{ ERT_15V_VOL_HIGH_MASK                     ,E_ERROR_LEVEL_1 },
	{ ERT_INT_REF_VOL_HIGH_MASK                 ,E_ERROR_LEVEL_1 },
	{ ERT_15V_VOL_LOW_MASK               		,E_ERROR_LEVEL_1 },
	{ ERT_INT_REF_VOL_LOW_MASK           		,E_ERROR_LEVEL_1 },
	

	{ ERT_NTC_TEMP_HIGH_MASK                    ,E_ERROR_LEVEL_1 },
	{ ERT_MCU_TEMP_HIGH_MASK                    ,E_ERROR_LEVEL_1 },
	{ ERT_MCU_TEMP_LOW_MASK                     ,E_ERROR_LEVEL_1 },
		

	{ MCU_ERT_FAUYLT_RAM_TEST_CALLEE_MASK       ,E_ERROR_LEVEL_3 },          
	{ MCU_ERT_FAUYLT_RAM_TEST_COMPLETED_MASK    ,E_ERROR_LEVEL_3 },
	{ MCU_ERT_FAUYLT_SYSTICK_INIT_CALLEE_MASK   ,E_ERROR_LEVEL_3 },
	{ MCU_ERT_FAUYLT_CLOCK_TEST_CALLER_MASK     ,E_ERROR_LEVEL_3 },
	{ MCU_ERT_FAUYLT_CPU_TEST_CALLER_MASK       ,E_ERROR_LEVEL_3 },
	{ MCU_ERT_FAUYLT_STACK_OVERFLOW_TEST_MASK   ,E_ERROR_LEVEL_3 },
	{ MCU_ERT_FAUYLT_DELTA_MAIN_MASK            ,E_ERROR_LEVEL_3 },
	{ MCU_ERT_FAUYLT_FLASH_TEST_CALLER_MASK     ,E_ERROR_LEVEL_3 },
	{ MCU_ERT_FAUYLT_LAST_DELTA_MAIN_MASK       ,E_ERROR_LEVEL_3 },
		
		//Level 3
	{ MCU_ERT_FAUYLT_WDG_TEST_CALLER_MASK       ,E_ERROR_LEVEL_3 },                 
	{ MCU_ERT_FAUYLT_CRC32_TEST_CALLER_MASK     ,E_ERROR_LEVEL_3 },
	{ MCU_ERT_FAUYLT_CHECKPOINT1_MASK           ,E_ERROR_LEVEL_3 },
	{ MCU_ERT_FAUYLT_CHECKPOINT2_MASK           ,E_ERROR_LEVEL_3 },

};
/*************************************local function***********************************/
/* 
*System reaction after error happened 
*/

void ERT_vErrorReaction(void)
{
	DISABLE_PWMOUT();
	M1_SwitchForceFault();
} 

/********************************global function**********************************/
/*
* LED symbol display
*/
void ERT_vErrorLEDDisplay(void)
{
    static uint16_t u16FaultTimeCnt = 0;//timer counter
    static uint16_t u16FaultCnt     = 0;//error id variable
    static uint16_t u16FalultNumber = 0;//error id

    if(ERT_tErrorData.u8PriorityErrorID)
    {
      for (int i = 0;i<ERT_TOTAL_ERROR_NR;i++)
      {
         u16FalultNumber = ERT_tErrorData.tErrorRecord.u64ErrorIDMark &(0x01<<i)? i: u16FalultNumber;//get error id
      }     
        u16FaultTimeCnt++;
        if(u16FaultTimeCnt == 500)//0.5sec
        {
            LED2_ON();
        }
        else if(u16FaultTimeCnt == 1000)//0.5sec
        {
            u16FaultCnt++;
            LED2_OFF();
            if(u16FaultCnt == u16FalultNumber)
            {
                u16FaultTimeCnt = 2000;
            }
            else u16FaultTimeCnt = 0;
        }
        else if(u16FaultTimeCnt == 4000)//2000~4000, 2sec
        {
            u16FaultCnt = 0;
            u16FaultTimeCnt = 0;
        }
    }
    else
    {
        u16FaultCnt = 0;
        u16FaultTimeCnt = 0;
        LED2_OFF();
    }

}




/*
* Recovery process for motor relevant
*/
void ERT_vErrorReactionRecovery(void)
{
	if((ERT_tErrorData.u8PriorityErrorID != 0)&&(ERT_tErrorData.bRecoveryEnable == true))
		{
		
			if(ERT_tErrorTable[ERT_tErrorData.u8PriorityErrorID].u8ErrorType == E_ERROR_LEVEL_3)
				{

				}
			else if(ERT_tErrorTable[ERT_tErrorData.u8PriorityErrorID].u8ErrorType == E_ERROR_LEVEL_2)//error can't be cleaned automatically
				{
					ERT_u16FaultTimeCnt++;
					if(ERT_u16FaultTimeCnt > Motor_1st.USER.u16FaultReleaseTimeCmd)
					{
						/*clear error flag symbol*/
						ERT_tErrorData.u8PriorityErrorID = 0;
						/*clean error manually*/
						ERT_vResetErrorCode(ERT_GENERIC_ERROR_LEVEL_2);
						/*force to stop mode*/
						M1_SwitchForceToStop();
						ERT_u16FaultTimeCnt = 0;

					}
				}
			else if(ERT_tErrorTable[ERT_tErrorData.u8PriorityErrorID].u8ErrorType == E_ERROR_LEVEL_1)//error can clear automatically
				{
					//error must be cleaned automatically. if 
					if((ERT_tErrorData.tErrorRecord.u64ErrorIDMark&ERT_GENERIC_ERROR_LEVEL_1) == 0)
						{
							ERT_u16FaultTimeCnt++;
							if(ERT_u16FaultTimeCnt > Motor_1st.USER.u16FaultReleaseTimeCmd)
							{
								/*clear error flag symbol*/
								ERT_tErrorData.u8PriorityErrorID = 0;
								/*force to stop mode*/
								M1_SwitchForceToStop();
								ERT_u16FaultTimeCnt = 0;

							}
						}
				}

		}
	
}


/*
* Implement once， write error code, error reaction
*/
void ERT_vErrorReport(uint8_t u8ErrorCode)
{
	//symbolize to write
	ERT_vSetErrorCode(ERT_tErrorTable[u8ErrorCode].u8ErrorIDMASK);//prepare to write error code.
	//count from start
	ERT_u16FaultTimeCnt = 0;
	
	ERT_tErrorData.u8ErrorCnt ++;
	if(ERT_tErrorData.u8ErrorCnt > ERT_MAX_ERROR_CNT)
		{
			//stop motor, keep it
			ERT_vErrorReaction();
			//disable recovery
			ERT_tErrorData.bRecoveryEnable = false;

		}
	else
		{
		
			if(ERT_tErrorTable[u8ErrorCode].u8ErrorType > ERT_tErrorData.u8PriorityErrorID)
			{
				ERT_tErrorData.u8PriorityErrorID = u8ErrorCode;
				//error reaction
				ERT_vErrorReaction();
				//configure recovery time
				if(ERT_tErrorTable[u8ErrorCode].u8ErrorType == E_ERROR_LEVEL_1)
					{
						//Reaction to do something, set recovery time
						ERT_u16FaultTimeCnt = 0;
						Motor_1st.USER.u16FaultReleaseTimeCmd = FAULTRELEASE_TIME;
						
					}
				else if(ERT_tErrorTable[u8ErrorCode].u8ErrorType == E_ERROR_LEVEL_2)
					{
						ERT_u16FaultTimeCnt = 0;
						Motor_1st.USER.u16FaultReleaseTimeCmd = FAULTRELEASE_TIME_LEVEL2;
					
					}
				else if(ERT_tErrorTable[u8ErrorCode].u8ErrorType == E_ERROR_LEVEL_3)
					{
						//keep error status
					}

				}
			else
				{
				
				}
		}
}
/* 
*main task to implement write error code
*/
void ERT_vErrorCodeAccess(void)
{
	
	switch(ERT_tErrorData.u8TaskState)
		{
			case E_FLASH_INIT:
				{
					ERT_tErrorData.u8TaskState = E_RECORD_RUN;
					
					break;

				}
			case E_RECORD_RUN:
				{
					//Check if error is continuning 5 times. if yes, stop it. and launch MCU reset.
					//if motor is running, stop it.
					if(Motor_1st.FOC.s16SpdFilt == 0)
						{
							if(ERT_tErrorData.tErrorRecord.u64ErrorIDMark != ERT_tErrorData.u64PreErrorIDMark)
							{
								ERT_tErrorData.u64PreErrorIDMark = ERT_tErrorData.tErrorRecord.u64ErrorIDMark;

								//write error to Flash， write infor to flash
								ERT_pu8Buffer = (uint8_t *)&ERT_tErrorData.tErrorRecord;					
								SMP_vWriteWordN(ERT_pu8Buffer, ERT_BYTE_ERROR_CNT + ERT_BYTE_INFOR_CNT);

							}
						}
					break;

				}
			case E_READ_RUN:
				{
					//load data from flash.
					//only can implement when motor is stopped.
					ERT_pu8Buffer = (uint8_t *)&ERT_tErrorData.tErrorRead;
					SMP_vReadWordN(ERT_pu8Buffer,ERT_BYTE_ERROR_CNT + ERT_BYTE_INFOR_CNT);
					break;
				}

			case E_IDLE:
				{
					break;
				}
			
			default:
				{
					break;
				}


		}


}

/*
* read out error code from flash, 4 bytes. only can implement when motor is stopped.
*/
void ERT_vReadErrorCode(void)
{
	uint8_t * puBuffer = NULL;
	//load data from flash
	puBuffer = (uint8_t *)&ERT_tErrorData.tErrorRead.u64ReadOutErrorDebug;
	SMP_vReadWordN(puBuffer,ERT_BYTE_ERROR_CNT + ERT_BYTE_INFOR_CNT);

}

void ERT_vSetErrorCode(uint32_t u32ErrorCode)
{
	ERT_tErrorData.u64PreErrorIDMark |=u32ErrorCode;

}

void ERT_vResetErrorCode(uint32_t u32ErrorCode)
{
	ERT_tErrorData.u64PreErrorIDMark &=(~u32ErrorCode);

}

TERRORDATA * ERT_tGetErrorCode(void)
{

	return &ERT_tErrorData;
}

void ERT_vResetErrorCount(void)
{
	ERT_tErrorData.u8ErrorCnt = 0;

}


