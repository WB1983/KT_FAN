#include "Voltage.h"		/* Include voltage measurement interface	*/
#include "Current.h"		/* Include current measurement interface	*/
#include "fsMonitor.h"		/* Include own header						*/
#include "compile.h"		/* Include global compiler switches */
#include "fsCommon.h"
#include "LibTypes.h"

/*****************************************************************************************************************
 * LOCAL TYPE DEFINTIONS *****************************************************************************************
 *****************************************************************************************************************/
/**
    \brief Type definition of the monitor init state flag register

    During the initialization phase of the registered monitor init
    functions this register stores a flag for each function. If an init
    function has returned TRUE, the corresponding flag is set and the
    function is not called anymore because the module is initialized.
*/
typedef uint32 TMonitorInitState;

/**
    \brief Type definition of the monitor init configuration table item

    Each monitor init is assigned to a handler function and a data item.
    That means the same init implementation can be executed on different
    data items.
*/
typedef struct _TMonitorInitConfigItem {
	TMonitorInitFct		pfMonitorInitFct;   /**< Pointer to the monitor init function */
	void*				pData;              /**< Pointer to the monitor init data item */
}TMonitorInitConfigItem;

/**
    \brief Type definition of the monitor configuration table item

    Each monitor is assigned to a handler function and a data item.
    That means the same check implementation can be executed on different
    data items.
*/
typedef struct _TMonitorConfigItem {
	TMonitorHandler  pfMonitorHandler;  /**< Pointer to the monitor handler function */
	void*            pData;             /**< Pointer to the monitor data item */
}TMonitorConfigItem;

/*****************************************************************************************************************
 * LOCAL FUNCTION PROTOTYPES *************************************************************************************
 *****************************************************************************************************************/
/**
	\brief Initialization of the monitored handle tasks
*/
static BOOL FSM_bInitMonitors(void);
/**
	\brief Handle tasks that are monitored(called) from fsMonitor.c
*/
static void FSM_vHandleMonitors(void);

/*****************************************************************************************************************
 * LOCAL VARIABLES ***********************************************************************************************
 *****************************************************************************************************************/
static uchar FSM_ucTaskState = TASK_NOT_INITIALISED;         /**< Actual state of the module task handler */

static TMonitorInitState FSM_tMonitorInitStateRegister = 0;  /**< Initialization state of the registered init functions */

static TMonitorStateReg  FSM_tMonitorStateRegister = 0;      /**< State register which contains the state of all monitoring functions */

/*****************************************************************************************************************
 * LOCAL CONSTANTS AND CONFIGURATION *****************************************************************************
 *****************************************************************************************************************/

/*
 * Monitor configuration constants
 * */
/**
    \brief Configuration of the monitoring initialization
*/

static const TMonitorInitConfigItem FSM_atMonitorInitFcts[] = {
/*    INIT FUNCTION							DATA POINTER */
	{(TMonitorInitFct)VOL_bInitDcLinkVoltageMeasurement,		(void*)0},
	//{(TMonitorInitFct)CUR_bInitOffsetCurrentVoltage,			(void*)0},
   {(TMonitorInitFct)VOL_vConstValueInit,	            		(void*)0},
};

/**
    \brief Computation of the no. of monitor init functions

    Do not change
*/
static const uchar FSM_ucNoOfMonitorInitFcts = (uchar)(sizeof(FSM_atMonitorInitFcts)/sizeof(TMonitorInitConfigItem));

/**
    \brief Computation of the all init mask

    This bitmask can be used to check whether all init functions
    have returned TRUE.
*/
static const TMonitorInitState FSM_tAllMonitorsInitializedMask = ((1 << (sizeof(FSM_atMonitorInitFcts)/sizeof(TMonitorInitConfigItem))) - 1);

/*
 * Monitor function configuration
 * */

/**
    \brief Configuration of the application monitoring

    \note Remember to define a bit mask in the header file!
*/

static const TMonitorConfigItem FSM_atMonitorConfig[] = {
/*							MONITOR FUNCTION				DATA POINTER */
	{	(TMonitorHandler)VOL_bHandleTask,					   (void*)0},
	{	(TMonitorHandler)VOL_bVoltageErrorDetection,		   (void*)0},
	{	(TMonitorHandler)CUR_bCheckMotCurrentMeasValues,	(void*)0},
};

/**
    \brief Computation of the no. of monitor functions

    Do not change
*/
static const uchar FSM_ucNoOfMonitors = (uchar)(sizeof(FSM_atMonitorConfig)/sizeof(TMonitorConfigItem));

/*****************************************************************************************************************
 * LOCAL MACROS **************************************************************************************************
 *****************************************************************************************************************/
#define FSM_INITIAL_DELAY    STK_DEF_TIME_MS(100)//100ms delay

/*****************************************************************************************************************
 * LOCAL FUNCTIONS ***********************************************************************************************
 *****************************************************************************************************************/
/**
     \brief Initialization of the monitors

     This function calls all the listed monitor init functions until their
     return value signals that their initialization has finished.
     All init functions are listed in the function pointer array
     MSC_apfMonitorInitFcts.

     \return BOOL   Current state of the initialization\n
             TRUE   => All monitors are initialized\n
             FALSE  => Initialization is ongoing\n
*/
static BOOL FSM_bInitMonitors(void) {
   uint32            u32Cnt         = 0;     /* Temp. loop counter */
   TMonitorInitState tInitMask      = 1;     /* Temp. mask to check which monitor is initialzed */
   BOOL              bInitFctReturn = FALSE; /* Return value of the last called init function */
   while (u32Cnt < FSM_ucNoOfMonitorInitFcts) 
      {
         if ((FSM_tMonitorInitStateRegister & tInitMask) != 0) 
            {
               /* Monitor is already initialized and thus is not called */
            }
         else 
            {
               /* Call monitor init function */
               bInitFctReturn = FSM_atMonitorInitFcts[u32Cnt].pfMonitorInitFct(FSM_atMonitorInitFcts[u32Cnt].pData);
            }

         if (bInitFctReturn != FALSE) 
            {
               /* Store that function is initialized now */
               FSM_tMonitorInitStateRegister |= tInitMask;
            }
         tInitMask <<= 1;
         u32Cnt++;
      }
   if ((FSM_tMonitorInitStateRegister & FSM_tAllMonitorsInitializedMask) == FSM_tAllMonitorsInitializedMask) 
      {
         /* All monitors have finished their initialization phase */
         return TRUE;
      }
      else 
      {
         return FALSE;
      }
}

/**
     \brief Monitor handler function

     This function calls all monitoring functions that are registered
     in the table FSM_atMonitorConfig[]. The current state of the
     monitors is stored in the local state register
     FSM_tMonitorStateRegister.

     \return void
*/
static void FSM_vHandleMonitors(void) {
   uint32           u32Cnt             = 0;   /* Temp. loop counter */
   BOOL             bHandlerFctReturn;        /* Return value of the last called handler function */
   TMonitorStateReg tMask              = 1;   /* Temp. mask for setting states */
   TMonitorStateReg tMonitorRegister   = 0;   /* Temp. monitor result register */
   while (u32Cnt < FSM_ucNoOfMonitors) {
      /* Call monitor handler function */
      bHandlerFctReturn = FSM_atMonitorConfig[u32Cnt].pfMonitorHandler(FSM_atMonitorConfig[u32Cnt].pData);
      if (bHandlerFctReturn != FALSE) {
         /* Current check is running without problems */
         tMonitorRegister |= tMask;
      }
      u32Cnt++;
      tMask <<= 1;
   }
   /* store current state of the monitoring functions */
   FSM_tMonitorStateRegister = tMonitorRegister;
}

/*****************************************************************************************************************
 * GLOBAL FUNCTIONS **********************************************************************************************
 *****************************************************************************************************************/

TMonitorStateReg FSM_tGetState(void) {
	return FSM_tMonitorStateRegister;
}

/*-------------------------------------------------------------------------------------------------
 *	uchar FSM_ucHandleTask(void) ------------------------------------------------------------------
 *------------------------------------------------------------------------------------------------- */
uchar FSM_ucHandleTask(void)
{
	BOOL bInitState = FALSE;
	switch (FSM_ucTaskState)
	{
		case TASK_INITIALISED:
		{				
			FSM_vHandleMonitors();
			break;
		}
		case TASK_NOT_INITIALISED:
		{
		    TSafeTime	tActualTime;
			tActualTime = STK_tGetSafeTime();
		   	if (tActualTime >= (TSafeTime)FSM_INITIAL_DELAY)
		   		{
		   			bInitState = FSM_bInitMonitors();
		   		}

			if (bInitState != FALSE)
			{
				FSM_ucTaskState = TASK_INITIALISED;
			}
			break;
		}
		default:
		{
			break;
		}
	}
	return FSM_ucTaskState;
}

