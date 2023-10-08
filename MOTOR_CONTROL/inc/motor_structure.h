#ifndef __PARAMETER_H
#define __PARAMETER_H

//#include "HAL_conf.h"
#include "HAL_device.h"
#include "mcflib.h"
#include "mlib.h"



/*Analog-to-Digital Converter struct*/
typedef struct
{
    ADC_TypeDef *ADC;
} ADCx_TypeDef;
//-----------------------------------------------------------------------------------------------

typedef struct
{
    uint32_t                u32sysclk;         /* system clock */
    uint16_t                u16period;         /* PWM period */
    uint16_t                u16halfperiod;     /* half system period */
    uint16_t                u16prescaler;      /* prescaler */
    uint32_t                u32freq;           /* PWM frequence */
    TIM_TypeDef             *Timer;            /* Timer */
} AV_PWM_TypeDef;
//-----------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------

typedef struct
{
    MCFLIB_2_DQ_T_S16   sVdq;                 /* dq axis voltage */
    MCFLIB_2_ALBE_T_S16 sVAlBe;               /* alpha beta axis voltage */
    MCFLIB_2_ALBE_T_S16 sVAlBe_cmd;           /* the commond of alpha beta axis voltage */

    MCFLIB_3_ABC_T_S16  sIabc;                /* dq axis voltage */
    MCFLIB_3_ABC_T_S16  sIabc_offset;         /* current offset */

    MCFLIB_2_ALBE_T_S16 sIAlBe;               /* alpha beta axis current */
    MCFLIB_2_DQ_T_S16   sIdq;                 /* dq axis current */
    MCFLIB_2_DQ_T_S16   sIdq_cmd;             /* the commond of dq axis current */
    MCFLIB_2_DQ_T_S16   sIdq_err;             /* the error of dq axis current */

    __IO int16_t    s16CmdTheta;              /* the commond of theta*/
    __IO int16_t    s16CmdTheta_Update;       /* the commond of theta update*/
    __IO int16_t    s16ThetaErr;              /* theta error*/
    sAngle_Trig     sAngle_cmd;               /* the commond of angle*/
    sAngle_Trig     sAngle_cmd_update;        /* the commond of angle update*/

    __IO int16_t    s16VMAX;                  /* V max value*/  

    __IO uint16_t   s16Vbus;                  /* DC bus voltage*/
    __IO uint16_t   s16VbusAvg;               /* the average of DC bus voltage*/

    __IO int16_t    s16VqLimit;               /* the limit value of Vq*/

    __IO int16_t    s16SpdCmd;                /* the commond of speed*/
    __IO int16_t    s16SpdFilt;               /* the filtered value of speed*/
    __IO int16_t    s16SpeedRamp;             /* the ramp value of speed*/
    __IO int32_t    s32SpeedRamp;             /* the ramp value of speed*/

    SVM_TypeDef     SVM;                      /* SVPWM*/

    SMOType         SMO;                      
    PLL_TypeDef   	sPLL;


} FOC_TypeDef;
//-----------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------
typedef struct
{
    uint32_t            u32sysclk;
    uint16_t            u16period;
    uint16_t            u16halfperiod;
    uint16_t            u16prescaler;
    uint32_t            u32freq;
    TIM_TypeDef         *Timer;
} CLK_TypeDef;

//-----------------------------------------------------------------------------------------------

typedef struct
{
    __IO uint16_t           u16PWMFreq;                 /* PWM frequency*/
    __IO uint16_t           u16DeadTime;                /* dead time */
    __IO int16_t            s16OVPCmd;                  /* the commond of over voltage protect  */
    __IO int16_t            s16UVPCmd;                  /* the commond of under voltage protect */
    __IO int16_t            s16DutyLimitCmd;            /* the commond of duty limit */
    __IO int16_t            s16IqLimitMax;              /* the max value of Iq limit  */


    __IO uint16_t           s16AlignCurAcc;             /* the acceleration value of align current*/
    __IO uint16_t           s16StartupSpdCmdMax;        /* the commond of startup speed max value */
    __IO uint16_t           u16AlignTimeCnt;            /* align time cnt */
    __IO uint16_t           u16StartupTimeCnt;          /* startup time cnt */

    __IO int16_t            s16AlignIqCmd;              /* the commond of align Iq */
    __IO uint16_t           u16AlignTimeCmd;            /* the commond of align time */
    __IO uint16_t           u16StartupTimeCmd;          /* the commond of startup time */
    __IO uint16_t           u16FreeWheelTimeCmd;        /* the commond of freewheel time */
    __IO uint16_t           u16FaultReleaseTimeCmd;     /* the commond of fault release time */
    __IO uint16_t           u16BlockTimeCmd;            /* the commond of block time */
    __IO uint16_t           u16BlockCounter;            /* block counter */

    __IO int16_t            s16StartupIqCmd;            /* the commond of startup Iq */
    __IO uint16_t           u16SyncRampSpdCmd;          /* the commond of sync ramp speed */
 
    __IO uint16_t           u16PreChargeTimeCmd;        /* the commond of precharge time */

    __IO uint16_t           u16OPToCLSwitchSpd;         /* Switch speed from open loop to close loop */

    //VS limit
    __IO int16_t            s16VqLimitMax;              /* Vq limit max value */
    __IO int16_t            s16VdLimitMax;              /* Vd limit max value */

    __IO int16_t            s16VspCmd;                  /* the speed commond of VSP */
    __IO uint16_t           s16VspCmdAvg;               /* the average speed commond of VSP */

    __IO uint16_t           u16PWMPeriod;               /* PWM period */
    __IO uint16_t           u16MaxDutyValue;            /* max duty value */
    __IO uint16_t           u16MinDutyValue;            /* min duty value */

    __IO uint16_t           u16SlowLoopDiv;             /* slow loop div */
    __IO bool               bSlowLoopFlag;              /* slow loop flag */
    __IO bool               bSlowLoopFlag2;             /* added by WB */
    __IO uint16_t           u16Align0Time;              /* align0 time */
    __IO uint16_t           u16Align1Time;              /* align1 time */
    __IO uint16_t           u16Ramp0Time;               /* ramp0 time */
    __IO uint16_t           u16Ramp1Time;               /* ramp1 time */
    __IO uint16_t           u16PreChargeTime;           /* precharge time */
    __IO uint16_t           u16SoftConectTime;          /* soft conect time */
    __IO uint8_t            u8MainState;                /* main state */

    MLIB_PIControl_T     		sSpeed_PI;
    MLIB_PIControl_T     		sId_PI;
    MLIB_PIControl_T     		sIq_PI;

    MLIB_FILTER_MA_T        sSpeed_Filt;
    MLIB_FILTER_MA_T        sVbus_Filt;
    MLIB_RAMP_T_Q31         sSpeedRamp;
} USER_TypeDef;


typedef struct
{
    AV_PWM_TypeDef  PWMx;
    ADCx_TypeDef    ADCx;
    FOC_TypeDef     FOC;
    USER_TypeDef    USER;

} Motor_TypeDef;


//-----------------------------------------------------------------------------------------------
#endif

