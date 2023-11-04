
#ifndef __FSM_H
#define __FSM_H
/* Includes ------------------------------------------------------------------------*/
#include <stdio.h>
#include "mm32_device.h"
#include "LibTypes.h"

/* Global definitions --------------------------------------------------------------*/

/* Global types ---------------------------------------------------------------------*/
/**
    \brief Function type of the monitor init function
//impr: actualize the comment
    Functions are called during the initialization phase of the software.
    The initialization phase is limited by a timeout. If the initialization
    takes longer than the configured timeout time, an error is reported.

    \param pData Pointer to a variable data object

    \return Intialization state of the module\n
            TRUE   =>   Module is initialized
            FALSE  =>   Module is not initialized
*/
typedef BOOL (*TMonitorInitFct)(void* pData);

/**
    \brief Function type of a monitor handler function

    All monitor handler functions are called on a regular base
    in each main loop.

    \param pData Pointer to a variable data object

    \return Internal state of the function\n
            TRUE   =>   Everything is ok, check is ongoing
            FALSE  =>   Module has detected a problem
*/
typedef BOOL (*TMonitorHandler)(void* pData);

/**
    \brief Typedefinition of the monitor state register.

    The state register contains one bit for each monitoring function.
    The bit is set when no error is pending, else the bit is cleared.
*/
typedef uint32_t TMonitorStateReg;


/**
    \brief Function returns the current value of the monitor state register

    \return TMonitorStateReg Actual value of the monitor state register
*/
extern TMonitorStateReg FSM_tGetState(void);

/**
    \brief Standard module task handler

    \return TASK_INITIALISED or TASK_NOT_INITIALISED
*/
extern uchar FSM_ucHandleTask(void);
#endif  // __FSM_H

