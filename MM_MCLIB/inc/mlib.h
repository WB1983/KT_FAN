/*******************************************************************************
*
* Copyright 
* 
*
****************************************************************************//*!
*
* @brief  Main MLIB header file for devices without FPU.
* 
*******************************************************************************/
#ifndef _MLIB_H_
#define _MLIB_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "mlib_types.h"
/*******************************************************************************
* Types
*******************************************************************************/
typedef struct
{
    int32_t   s32KP;        /* Proportional Gain */
    int32_t   s32KI;        /* Integral Gain */
    int32_t  	s32IAccK_1;      /* State variable output at step k-1 */
    Q15_t  		s16InErrK_1;     /* State variable input error at step k-1 */
    Q15_t  		s16UpperLim;     /* Upper Limit of the controller */
    Q15_t  		s16LowerLim;     /* Lower Limit of the controller */
    bool_t    bLimFlag;        /* Limitation flag, if set to 1, the controller 
                                  output has reached either the UpperLimit or LowerLimit */
}MLIB_PIControl_AW_T;

typedef struct
{
    int16_t   s16Kp;        	/* Proportional Gain */
	  int16_t   i16Kp_Shift;  	/* Proportional shift value */

    int16_t   s16Ki;       	 /* Integral Gain */
		int16_t   i16Ki_Shift;  	/* Integral shift value */

    int32_t  	s32IAccK_1;      /* State variable output at step k-1 */
    Q15_t  		s16UpperLim;     /* Upper Limit of the controller */
    Q15_t  		s16LowerLim;     /* Lower Limit of the controller */
    bool_t    bLimFlag;        /* Limitation flag, if set to 1, the controller 
                                  output has reached either the UpperLimit or LowerLimit */
}MLIB_PIControl_T;


typedef struct
{
    int16_t s16Fs;             /* Parameter: Motor dependent plant matrix */
    int16_t s16Gs;             /* Parameter: Motor dependent control gain */
    int16_t s16ErrMax;         /* Sliding control error max */
    int16_t s16Kslide;         /* Sliding control gain */
    int16_t	s16IerrGain;       /* Sliding current error gain */
    int16_t	i16IerrGainShift;  /* Sliding current error gain shift */
    int16_t s16Ialpha_est;		 /* Estimated stationary alfa-axis stator current */
    int16_t s16Ibeta_est;			 /*	Estimated stationary beta-axis stator current */
    int16_t s16Ialpha;		     /* Stationary alfa-axis stator current 		*/
    int16_t s16Ibeta;				   /*	Stationary beta-axis stator current */
    int16_t s16Ealpha_est;		 /*	Estimated stationary alfa-axis stator voltage */
    int16_t s16Ebeta_est;			 /*	Estimated stationary beta-axis stator voltage */
    int16_t s16Valpha;				 /* Stationary alfa--axis stator voltage */
    int16_t s16Vbeta;				   /*	Stationary beta-axis stator voltage */
    int16_t s16Z_alpha;			   /*	Z alfa-axis sliding control */
    int16_t s16Z_beta;			   /*	Z beta-axis sliding control */
    int16_t	s16Speed;			     /*	Estimated motor speed */
}SMOType;
   

typedef struct
{
    int32_t s32Theta;    /* Estimated position */
    int32_t s32Speed;    /* Estimated speed - first integrator output */  
    int32_t s32I_1;      /* State variable of observer controller part */
    int16_t s16IGain;    /* Observer integral gain */
    int16_t i16IGainSh;   /* Observer integral gain shift*/
    int16_t s16PGain;    /* Observer proportional gain */
    int16_t i16PGainSh;   /* Observer proportional gain shift*/
    int16_t s16ThGain;   /* Observer gain for output integrator of position */
    int16_t i16ThGainSh;  /* Observer gain shift for integrator of position */
}PLL_TypeDef;

typedef struct
{
    Q15_t s16RampUp;    /* Ramp-up increment */
    Q15_t s16RampDown;  /* Ramp-down increment */
    Q15_t s16State;     /* Previous ramp value */
	  bool_t bReachFlag;	/* Reach flag, if set to 1, the ramp output has reached the target input */
}MLIB_RAMP_T_Q15;

typedef struct
{
    Q31_t s32RampUp;    /* Ramp-up increment */
    Q31_t s32RampDown;  /* Ramp-down increment */
    Q31_t s32State;     /* Previous ramp value */
	  bool_t bReachFlag;	/* Reach flag, if set to 1, the ramp output has reached the target input */
}MLIB_RAMP_T_Q31;

typedef struct{
    int32_t  s32Acc;   /* Filter accumulator. */
    uint16_t u16Sh;    /* Number of samples for averaging, filter sample window [0,15].*/
}MLIB_FILTER_MA_T;

/*******************************************************************************
* Global function declaration 
*******************************************************************************/ 
extern Q15_t MLIB_Sin(Q15_t s16Angle);
extern Q15_t MLIB_Cos(Q15_t s16Angle);
extern sAngle_Trig MLIB_SinCos(Q15_t s16Angle);
extern Q15_t MLIB_Ramp_Q15(Q15_t s16Target, MLIB_RAMP_T_Q15 *psParam);
extern Q31_t MLIB_Ramp_Q31(Q31_t s32Target, MLIB_RAMP_T_Q31 *psParam);
extern Q15_t MLIB_PIControl_Q15(Q15_t s16InErr, MLIB_PIControl_T *psParam);
extern void MLIB_SMO(SMOType *sSMOObs);
extern Q15_t MLIB_PLL(Q15_t s16ThetaErr,PLL_TypeDef *sTO);



/*******************************************************************************
* INLINE FUNCTIONS
*******************************************************************************/ 

/***************************************************************************//*!
*
* s16Out = -s16Val
* Without saturation
*******************************************************************************/
static inline Q15_t MLIB_Neg_Q15(register Q15_t s16Val)
{
    return((Q15_t)(-s16Val)); 
}

/***************************************************************************//*!
*
* s32Out = -s32Val
* Without saturation
*******************************************************************************/ 
static inline Q31_t MLIB_Neg_Q31(register Q31_t s32Val)
{
    return((Q31_t)(-s32Val));
}
/***************************************************************************//*!
*
* s32Out = -s32Val
* With saturation
*******************************************************************************/  
static inline Q31_t MLIB_NegSat_Q31(register Q31_t s32Val)
{
    register int32_t i32Out;   
    
    i32Out = (int32_t)(MLIB_Neg_Q31(s32Val));
    i32Out = (i32Out == INT32_MIN) ? INT32_MAX : i32Out;
    return((Q31_t)(i32Out));   
}

/***************************************************************************//*!
*
* Takes the fractional part from number in accumulator format and
* saturate if the original number is greater than 1. 
* 
*******************************************************************************/ 
static inline Q15_t MLIB_Sat_Q15(register int32_t s32Accum)
{
    register int32_t i32z;
    i32z = (int32_t)(s32Accum);
  
    i32z = (i32z > ((int32_t)(INT16_MAX))) ? ((int32_t)(INT16_MAX)) : i32z;
    i32z = (i32z < ((int32_t)(INT16_MIN))) ? ((int32_t)(INT16_MIN)) : i32z;
    return ((Q15_t)(i32z));
}

/***************************************************************************//*!
*  Absolute value      
*  s16Out = |s16In|
*  not saturated
*******************************************************************************/
static inline Q15_t MLIB_Abs_Q15(register Q15_t s16Val)
{
    return((s16Val < (Q15_t)0) ? (-s16Val) : (s16Val));
}
/***************************************************************************//*!
*  Absolute value saturated
*  s16Out = |s16In|
*******************************************************************************/
static inline Q15_t MLIB_AbsSat_Q15(register Q15_t s16Val)
{
    register int16_t i16Temp;
  
    i16Temp = (int16_t)(MLIB_Abs_Q15(s16Val)); 
    i16Temp = (i16Temp == INT16_MIN) ? INT16_MAX : i16Temp; /* Correction for -1.0 * (-1.0) */
    return((Q15_t)(i16Temp));
}
/***************************************************************************//*!
*  Absolute value      
*  s32Out = |s32In|
*  not saturated
*******************************************************************************/
static inline Q31_t MLIB_Abs_Q31(register Q31_t s32Val)
{
    return((s32Val < (Q31_t)0) ? (-s32Val) : (s32Val));
}
/***************************************************************************//*!
*  Absolute value saturadet        
*  s32Out = |s32In|
*******************************************************************************/  
static inline Q31_t MLIB_AbsSat_Q31(register Q31_t s32Val)
{
    register int32_t i32Temp;
    
    i32Temp = (int32_t)(MLIB_Abs_Q31(s32Val));
    i32Temp = (i32Temp == INT32_MIN) ? INT32_MAX : i32Temp; /* Correction for -1.0 * (-1.0) */
    return((Q31_t)(i32Temp));
}
/***************************************************************************//*!
*  Addition        
*  s16Out = s16Add1 + s16Add2
*  function is not saturated
*******************************************************************************/
static inline Q15_t MLIB_Add_Q15(register Q15_t s16Add1, register Q15_t s16Add2)
{
    return((Q15_t)(s16Add1 + s16Add2));
}
/***************************************************************************//*!
*  Saturated Addition      
*  s16Out = s16Add1 + s16Add2
*******************************************************************************/
static inline Q15_t MLIB_AddSat_Q15(register Q15_t s16Add1, register Q15_t s16Add2)
{
    register int32_t i32z;
    
    i32z = (int32_t)((Q31_t)s16Add1 + (Q31_t)s16Add2); 
    i32z = (i32z > ((int32_t)(INT16_MAX))) ? ((int32_t)(INT16_MAX)) : i32z;
    i32z = (i32z < ((int32_t)(INT16_MIN))) ? ((int32_t)(INT16_MIN)) : i32z;
    return((Q15_t)(i32z)); 
}
/***************************************************************************//*!
*  Addition        
*  s32Out = s32Add1 + s32Add2
*  function is not saturated
*******************************************************************************/ 
static inline Q31_t MLIB_Add_Q31(register Q31_t s32Add1, register Q31_t s32Add2)
{
    return((Q31_t)(s32Add1 + s32Add2));
}
/***************************************************************************//*!
*  Saturated Addition      
*  s32Out = s32Add1 + s32Add2
*******************************************************************************/
static inline Q31_t MLIB_AddSat_Q31(register Q31_t s32Add1, register Q31_t s32Add2)
{
    register int32_t i32z;
    register int32_t i32SatMin;
    register int32_t i32SatMax;
 
    i32z = s32Add1 + s32Add2;    /* Addition without saturation to find saturation max. and min. */
    i32SatMax = ~(s32Add1 | s32Add2) & i32z;   /* Saturation max. from addends and addition without sat. */
    i32SatMin = (s32Add1 & s32Add2) & ~i32z;   /* Saturation min. from addends and addition without sat. */
    i32z = (i32SatMin < 0) ? INT32_MIN : i32z; /* Low limit restriction */
    i32z = (i32SatMax < 0) ? INT32_MAX : i32z; /* Hight limit restriction */
    return(i32z);       
}

/***************************************************************************//*!
*
* This function returns the s16Val input shifted by the number of u16Sh to the left. 
* The function does not saturate the output. 
*             
*******************************************************************************/ 
static inline Q15_t MLIB_ShL_Q15(register Q15_t s16Val, register uint16_t u16Sh)
{
    return((Q15_t)(s16Val << u16Sh));
}
/***************************************************************************//*!
*
* This function returns the s16Val input shifted by the number of u16Sh to the right.  
*             
*******************************************************************************/ 
static inline Q15_t MLIB_ShR_Q15(register Q15_t s16Val, register uint16_t u16Sh)
{
    return((Q15_t)(s16Val >> u16Sh));
}

/***************************************************************************//*!
*
* This function returns the s16Val input shifted by the number of i16Sh to the left. 
* If the i16Sh is negative, the input is shifted to the right. The function
* does not saturate the output.  
* 
*******************************************************************************/ 
static inline Q15_t MLIB_ShLBi_Q15(register Q15_t s16Val, register int16_t i16Sh)
{
    return (i16Sh<(int16_t)0) ? MLIB_ShR_Q15(s16Val, (uint16_t)(-i16Sh)) :
                                MLIB_ShL_Q15(s16Val, (uint16_t)(i16Sh));
}
/***************************************************************************//*!
*
* This function returns the s16Val input shifted by the number of i16Sh to the right. 
* If the i16Sh is negative, the input is shifted to the left. The function
* does not saturate the output.  
* 
*******************************************************************************/ 
static inline Q15_t MLIB_ShRBi_Q15(register Q15_t s16Val, register int16_t i16Sh)
{
    return (i16Sh<(int16_t)0) ? MLIB_ShL_Q15(s16Val, (uint16_t)(-i16Sh)) : 
                                MLIB_ShR_Q15(s16Val, (uint16_t)(i16Sh));
}
/***************************************************************************//*!
*
* s16Out = s16Min - s16Sub
* Without saturation
*******************************************************************************/
static inline Q15_t MLIB_Sub_Q15(register Q15_t s16Min, register Q15_t s16Sub)
{
    return(s16Min - s16Sub);
}

/***************************************************************************//*!
*  Saturated Subtraction       
*  s16Out = s16Min - s16Sub
*******************************************************************************/
static inline Q15_t MLIB_SubSat_Q15(register Q15_t s16Min, register Q15_t s16Sub)
{
    register int32_t i32z;
    
    i32z = (int32_t)((Q31_t)s16Min - (Q31_t)s16Sub); 
    i32z = (i32z > ((int32_t)(INT16_MAX))) ? ((int32_t)(INT16_MAX)) : i32z;
    i32z = (i32z < ((int32_t)(INT16_MIN))) ? ((int32_t)(INT16_MIN)) : i32z;
    return((Q15_t)(i32z)); 
}

/***************************************************************************//*!
*
* s32Out = s32Min - s32Sub
* Without saturation
*******************************************************************************/ 
static inline Q31_t MLIB_Sub_Q31(register Q31_t s32Min, register Q31_t s32Sub)
{
    return(s32Min - s32Sub);
}

/***************************************************************************//*!
*  Saturated Subtraction      
*  s32Out = s32Min - s32Sub
*******************************************************************************/
static inline Q31_t MLIB_SubSat_Q31(register Q31_t s32Min, register Q31_t s32Sub)
{
    register int32_t i32z;
    register int32_t i32SatMin;
    register int32_t i32SatMax;
 
    i32z = s32Min - s32Sub;    /* Subtraction without saturation to find saturation max. and min. */
    i32SatMax = ~(s32Min | (-s32Sub)) & i32z;   /* Saturation max. from addends and addition without sat. */
    i32SatMin = (s32Min & (-s32Sub)) & ~i32z;   /* Saturation min. from addends and addition without sat. */
    i32z = (i32SatMin < 0) ? INT32_MIN : i32z; /* Low limit restriction */
    i32z = (i32SatMax < 0) ? INT32_MAX : i32z; /* Hight limit restriction */
    return(i32z);       
}
/***************************************************************************//*!
*
* This function returns the s32Val input shifted by the number of u16Sh to the left. 
* The function does not saturate the output. 
*             
*******************************************************************************/
static inline Q31_t MLIB_ShL_Q31(register Q31_t s32Val, register uint16_t u16Sh)
{
    return((Q31_t)(s32Val << u16Sh));
}
/***************************************************************************//*!
*
* This function returns the s32Val input shifted by the number of u16Sh to the right.  
*             
*******************************************************************************/ 
static inline Q31_t MLIB_ShR_Q31(register Q31_t s32Val, register uint16_t u16Sh)
{
  return((Q31_t)(s32Val >> u16Sh));
}

/***************************************************************************//*!
*
* This function returns the s32Val input shifted by the number of i16Sh to the left. 
* If the i16Sh is negative, the input is shifted to the right. The function
* does not saturate the output.  
* 
*******************************************************************************/ 
static inline Q31_t MLIB_ShLBi_Q31(register Q31_t s32Val, register int16_t i16Sh)
{
    return (i16Sh<(int16_t)0) ? MLIB_ShR_Q31(s32Val, (uint16_t)(-i16Sh)) :
                                MLIB_ShL_Q31(s32Val, (uint16_t)(i16Sh));
}
/***************************************************************************//*!
*
* This function returns the s32Val input shifted by the number of i16Sh to the right. 
* If the i16Sh is negative, the input is shifted to the left. The function
* does not saturate the output.  
* 
*******************************************************************************/ 
static inline Q31_t MLIB_ShRBi_Q31(register Q31_t s32Val, register int16_t i16Sh)
{
    return (i16Sh<(int16_t)0) ? MLIB_ShL_Q31(s32Val, (uint16_t)(-i16Sh)) : 
                                MLIB_ShR_Q31(s32Val, (uint16_t)(i16Sh));
}

/***************************************************************************//*!
*
* This function returns the s32Val input shifted by the number of u16Sh to the left. 
* The function saturates the output. 
*             
*******************************************************************************/  
static inline Q31_t MLIB_ShLSat_Q31(register Q31_t s32Val, register uint16_t u16Sh)
{
    register Q31_t s32Temp, s32Out;
    
    s32Out = (Q31_t)(s32Val << u16Sh);
    s32Temp = (Q31_t)((Q31_t)(INT32_MAX) >> u16Sh);
    if(s32Val > s32Temp)
    {
        s32Out = (Q31_t)(INT32_MAX);
    }
    else
    {
        s32Temp = (Q31_t)((Q31_t)(INT32_MIN) >> u16Sh);
        if(s32Val <= s32Temp)
        {
            s32Out = (Q31_t)(INT32_MIN);
        }
    }
    return(s32Out);
}
/***************************************************************************//*!
*
* @brief  16-bit fractional to 32-bit fractional convert function
*
* @param  in  Q15_t s16Val - Argument in <0;1) in Q15_t
*
* @return This function returns - Q31_t value <0;1)
*       
* @remarks  This function convert the lower 16-bits of the 32-bit input and
*           returns the upper 16-bit. 
*
*******************************************************************************/
static inline Q31_t MLIB_Conv_Q31s(register Q15_t s16Val)
{
    return((Q31_t)((Q31_t)s16Val << ((uint16_t)16)));
}
/***************************************************************************//*!
*
* @brief  32-bit fractional to 16-bit fractional convert function
*
* @param  in  Q31_t s32Val - Argument in <-1;1) in Q31_t
*
* @return This function returns - Q15_t value <-1;1)
*       
* @remarks  This function convert the lower 16-bits of the 32-bit input and
*           returns the upper 16-bit. 
*
*******************************************************************************/
static inline Q15_t MLIB_Conv_Q15l(register Q31_t s32Val)
{
    return((Q15_t)(s32Val >> ((uint16_t)16)));
}
/***************************************************************************//*!
*
* s16Out = s16Mult1 * s16Mult2
* Without saturation
*******************************************************************************/ 
static inline Q15_t MLIB_Mul_Q15(register Q15_t s16Mult1, register Q15_t s16Mult2)
{
    return((Q15_t)(((int32_t)(s16Mult1)*(int32_t)(s16Mult2))>>15));
}
/***************************************************************************//*!
*
* s32Out = s32Mult1 * s32Mult2
* Without saturation
*******************************************************************************/   
static inline Q31_t MLIB_Mul_Q31(register Q31_t s32Mult1, register Q31_t s32Mult2)
{
  return((((Q31_t)(s32Mult1>>16)*(Q31_t)(s32Mult2>>16))<<1) + 
         (((Q31_t)(s32Mult1>>16)*(Q31_t)(s32Mult2&0xffff))>>15) + 
         (((Q31_t)(s32Mult1&0xffff)*(Q31_t)(s32Mult2>>16))>>15));   
}
/***************************************************************************//*!
*
* s32Out = s16Mult1 * s16Mult2
* Without saturation
*******************************************************************************/
static inline Q31_t MLIB_Mul_Q31ss(register Q15_t s16Mult1, register Q15_t s16Mult2)
{
    return((Q31_t)(((int32_t)(s16Mult1)*(int32_t)(s16Mult2))<<1)); 
}
/***************************************************************************//*!
*
* s32Out = s32Mult1 * s32Mult2
* With saturation
*******************************************************************************/  
static inline Q31_t MLIB_MulSat_Q31(register Q31_t s32Mult1, register Q31_t s32Mult2)
{
    register int32_t i32Temp;
    
    i32Temp = (int32_t)(MLIB_Mul_Q31(s32Mult1, s32Mult2));
    i32Temp = (i32Temp == INT32_MIN) ? INT32_MAX : i32Temp;
    return((Q31_t)(i32Temp)); 
}
/***************************************************************************//*!
*
* s32Out = s16Mult1 * s16Mult2
* With saturation
*******************************************************************************/
static inline Q31_t MLIB_MulSat_Q31ss(register Q15_t s16Mult1, register Q15_t s16Mult2)
{
    register int32_t i32Temp;
    
    i32Temp = (int32_t)(MLIB_Mul_Q31ss(s16Mult1, s16Mult2));
    i32Temp = (i32Temp == INT32_MIN) ? INT32_MAX : i32Temp;
    return((Q31_t)(i32Temp));
}

/***************************************************************************//*!
*
* s32Out = a32Accum * s16Mult
* Without saturation
*******************************************************************************/  
static inline Q31_t MLIB_Mul_Q31ls(register Q31_t s32Mult1, register Q15_t s16Mult2)
{
    return((Q31_t)(((((int32_t)(s32Mult1)&0xffff)*(int32_t)(s16Mult2))>>15)+
                      ((((int32_t)(s32Mult1)>>16)*(int32_t)(s16Mult2))<<1)));  
}

/***************************************************************************//*!
*
* @brief  The function calculates recursive form of moving average filter
*
* @param  ptr   GDFLIB_FILTER_MA_T_A32 * psParam
*               - Pointer to filter structure
*               
*               MLIB structure:                    
*               - a32Acc - accumulator of the filter
*               - u16Sh - 2 ^ u16Sh is the number of values
*
* @param  in    f16InX - input signal
*
* @return This function returns - frac16_t value
*        
* @remarks 
*
* Filter Equation:
* 
* sum(k)   = sum(k-1) + input
* out(k)   = sum(k)/N
* sum(k-1) = sum(k) - out(k)
*
****************************************************************************/
static inline Q15_t MLIB_FilterMA_Q15(Q15_t s16InX,MLIB_FILTER_MA_T *psParam)
{
    register int32_t s32Temp;
    register int32_t s32Acc;

    /* Input scaled such as to avoid saturation of the algorithm */
    s32Acc  = MLIB_AddSat_Q31(psParam->s32Acc, (Q31_t)s16InX);      
    s32Temp = MLIB_ShR_Q31(s32Acc,psParam->u16Sh);
    s32Acc  = MLIB_SubSat_Q31(s32Acc, s32Temp);

    /* Store new accumulator state */
    psParam->s32Acc = s32Acc;
    return((Q15_t)MLIB_Sat_Q15(s32Temp));
}

#endif /* _MLIB_H_ */
