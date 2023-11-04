#ifndef _STATEMACHINE_H
#define _STATEMACHINE_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>


typedef void (*pfStateAction)(void);	

typedef struct _T_ACTION_MAP
{
	uint8_t 		u8StateID;
	pfStateAction 	ptEnterAct;	
	pfStateAction 	ptRunningAct;	
	pfStateAction 	ptExitAct;
}T_ACTION_MAP; /* action table */

typedef struct _T_EVENT_MAP
{
	uint8_t	u8EventID;
	uint8_t u8CurState;
	uint8_t u8NextState;
}T_EVENT_MAP; /* event table */

typedef struct _T_FSM
{
	uint8_t u8Flag; 			/* state switch flag,1: to do state switch */
	uint8_t u8EventSum;			/* sum of event */
	uint8_t u8ActSum;			/* sum of action */
	uint8_t u8CurState;
	uint8_t u8NextState;
	T_ACTION_MAP *ptActionMap;
	T_EVENT_MAP *ptEventMap;
}T_FSM; /* statemaction control struct */


extern void FSM_vStateTransfer(T_FSM * pFsm, uint8_t u8EventID);

extern void FSM_vInit(T_FSM* ptFsm, T_EVENT_MAP* ptEventMap, T_ACTION_MAP *ptActionMap, uint8_t u8EventSum, uint8_t u8ActSum, uint8_t u8curState);

extern void FSM_vActionPerfrom(T_FSM* pFsm);
#endif

