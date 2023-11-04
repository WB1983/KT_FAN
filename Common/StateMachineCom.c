#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "StatemachineCom.h"

void FSM_vInit(T_FSM* ptFsm, T_EVENT_MAP* ptEventMap, T_ACTION_MAP *ptActionMap, uint8_t u8EventSum, uint8_t u8ActSum, uint8_t u8curState)
{
	ptFsm->u8Flag = 0;
	ptFsm->u8NextState = 0;
	ptFsm->u8EventSum = u8EventSum;
	ptFsm->u8ActSum = u8ActSum;
	ptFsm->u8CurState = u8curState;
	ptFsm->ptEventMap = ptEventMap;
	ptFsm->ptActionMap = ptActionMap;
}

void FSM_vStateTransfer(T_FSM* pFsm, uint8_t u8EventID)
{
	uint8_t i = 0;
	
	for(i=0; i<pFsm->u8EventSum; i++)
	{
		if((u8EventID == pFsm->ptEventMap[i].u8EventID) 
			&& (pFsm->u8CurState == pFsm->ptEventMap[i].u8CurState))
		{
			pFsm->u8NextState = pFsm->ptEventMap[i].u8NextState;
			pFsm->u8Flag = 1;
			
			return;
		}
	}	
}

uint8_t FSM_vGetAction(T_FSM* pFsm,uint8_t u8ID)
{
	int i = 0;
	
	for(i=0; i<pFsm->u8ActSum; i++)
	{
		if(u8ID == pFsm->ptActionMap[i].u8StateID)
			return i;
	}

	return 0xFF;
}

void FSM_vActionPerfrom(T_FSM* pFsm)
{
	uint8_t u8CurID  = 0xFF;
	uint8_t u8NextID = 0xFF;
	
	if(0 != pFsm->u8Flag)	/* not zero, implement state transfer */
	{
		u8CurID = FSM_vGetAction(pFsm,pFsm->u8CurState);
		u8NextID = FSM_vGetAction(pFsm,pFsm->u8NextState);
		
		if((0xFF != u8CurID) && (0xFF != u8NextID))
		{
			pFsm->ptActionMap[u8CurID].ptExitAct();	/* implement exit action under current state */
			pFsm->ptActionMap[u8NextID].ptEnterAct(); /* implement entry action to next state */
		}
		else
		{
			pFsm->u8Flag = 0;								/* clear flag bit */
			//printf("State transition failed! curState = %d, nextState = %d\n",pFsm->u8CurState,pFsm->u8NextState);
			return;
		}
	
		pFsm->u8CurState = pFsm->u8NextState;			/* current state switch */
		pFsm->u8NextState = 0xFF;
		pFsm->u8Flag = 0;								/* clear flag bit */
	}
	else
	{
		u8CurID = FSM_vGetAction(pFsm,pFsm->u8CurState);
		if(0xFF != u8CurID)
			pFsm->ptActionMap[u8CurID].ptRunningAct();/* flag is zero, stay current state and implemnt current action */
	}	
}

