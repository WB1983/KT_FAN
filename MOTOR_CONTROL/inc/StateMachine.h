#ifndef _STATEMACHINE_H_
#define _STATEMACHINE_H_

#include "main.h"
#include "stdlib.h"
#include "user_function.h"


/*! @brief Pointer to function */
typedef void (*pfcn_void_void)(void);

/*! @brief device fault typedef */
typedef uint16_t mcdef_fault_t;

/*! @brief States of machine enumeration */
typedef enum _m1_app_mainstate_t
{
    MainState_Fault    = 0,
    MainState_Init     = 1,
    MainState_Stop     = 2,
    MainState_Run      = 3,    
} m1_app_mainstate_t; /* Main States */

/*! @brief States of machine enumeration */
typedef enum _m1_run_substate_t
{
    RunState_Calib     = 0,
    RunState_Ready     = 1,
		RunState_WindDet   = 2,
		RunState_Brake     = 3,	
    RunState_Align     = 4,
    RunState_Startup   = 5,
    RunState_Spin      = 6,
    RunState_Freewheel = 7,
} m1_run_substate_t; /* Run sub-states */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
//#define MC_FAULT_I_DCBUS_OVER 1  /* OverCurrent fault flag */
//#define MC_FAULT_U_DCBUS_UNDER 2 /* Undervoltage fault flag */
//#define MC_FAULT_U_DCBUS_OVER 3  /* Overvoltage fault flag */
//#define MC_FAULT_LOAD_OVER 4     /* Overload fault flag */
//#define MC_FAULT_SPEED_OVER 5    /* Over speed fault flag */


/* Sets the fault bit defined by faultid in the faults variable */
#define MC_FAULT_SET(faults, faultid) (faults |= ((mcdef_fault_t)1 << faultid))

/* Clears the fault bit defined by faultid in the faults variable */
#define MC_FAULT_CLEAR(faults, faultid) (faults &= ~((mcdef_fault_t)1 << faultid))

/* Check the fault bit defined by faultid in the faults variable, returns 1 or 0 */
#define MC_FAULT_CHECK(faults, faultid) ((faults & ((mcdef_fault_t)1 << faultid)) >> faultid)

/* Clears all fault bits in the faults variable */
#define MC_FAULT_CLEAR_ALL(faults) (faults = 0)


extern m1_app_mainstate_t eM1_MainState;                 /* Main State */
/*! @brief Application sub-state function field - fast */
extern const pfcn_void_void s_STATE_FAST[4];

/*! @brief Application sub-state function field - slow */
extern const pfcn_void_void s_STATE_SLOW[4];

extern m1_run_substate_t eM1_RunSubState;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void M1_Fault_Fast(void);
void M1_Init_Fast(void);
void M1_Stop_Fast(void);
void M1_Run_Fast(void);

void M1_Fault_Slow(void);
void M1_Init_Slow(void);
void M1_Stop_Slow(void);
void M1_Run_Slow(void);




void M1_SwitchFaultStop(void);
static void M1_SwitchInitStop(void);
static void M1_SwitchStopFault(void);
static void M1_SwitchStopRun(void);
static void M1_SwitchRunFault(void);
//static void M1_SwitchRunStop(void);

static void M1_RunCalibFast(void);
static void M1_RunReadyFast(void);
static void M1_RunAlignFast(void);
static void M1_RunStartupFast(void);
static void M1_RunSpinFast(void);
static void M1_RunFreewheelFast(void);

static void M1_RunWindDetFast(void);
static void M1_RunBrakeFast(void);

static void M1_RunCalibSlow(void);
static void M1_RunReadySlow(void);
static void M1_RunAlignSlow(void);
static void M1_RunStartupSlow(void);
static void M1_RunSpinSlow(void);
static void M1_RunFreewheelSlow(void);

static void M1_RunWindDetSlow(void);
static void M1_RunBrakeSlow(void);


static void M1_SwitchRunCalibReady(void);
static void M1_SwitchRunReadyAlign(void);
static void M1_SwitchRunAlignStartup(void);
static void M1_SwitchRunAlignReady(void);
static void M1_SwitchRunStartupSpin(void);
static void M1_SwitchRunStartupFreewheel(void);
static void M1_SwitchRunSpinFreewheel(void);
static void M1_SwitchRunFreewheelReady(void);

/****************************************Add by WB*******************************/
extern void M1_SwitchRunReadyWindDet(void);
extern void M1_SwitchRunWindDetAlign(void);
extern void M1_SwitchRunWindDetBrake(void);
extern  void M1_SwitchRunBrakeReady(void);
extern void M1_SwitchRunWindDetSpin(void);

extern void M1_SwitchForceToStop(void);
extern void M1_SwitchForceFault(void);

static void M1_FaultDetection(void);
static void Fault_LED_Disp(void);


void PWM_Update(Motor_TypeDef *Motor);

#endif
