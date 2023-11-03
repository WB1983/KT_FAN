/*******************************************************************************
*
* Copyright:
* 
*
****************************************************************************//*!
*
* @brief  MCFLIB Types  
* 
*******************************************************************************/
#ifndef _MCFLIB_TYPES_H_
#define _MCFLIB_TYPES_H_


/*******************************************************************************
* Includes
*******************************************************************************/ 
#include "mlib_types.h"

/******************************************************************************
* Macros 
******************************************************************************/  
  
/* MCFLIB_ONE_DIV_SQRT3_s16 define, the value is: 32768 / sqrt(3) = 18919 = 0x49E7 */
#define MCFLIB_ONE_DIV_SQRT3_S16  (Q15_t)18919
/* MCFLIB_SQRT3_DIV_2_s16 define, the value is: 32768 * sqrt(3) / 2 = 28378 = 0x6EDA */  
#define MCFLIB_SQRT3_DIV_2_S16    (Q15_t)28378


/*******************************************************************************
* Types 
********************************************************************************/ 
/* Integer data types */

/* The MCFLIB_3COOR_T_s16 structure type corresponds to the three-phase stationary
coordinate system based on the A, B, C components. */
typedef struct
{
    Q15_t s16A;
    Q15_t s16B;
    Q15_t s16C;
} MCFLIB_3_ABC_T_S16;

/* The MCFLIB_2COOR_ALBE_T_s16 structure type corresponds to the two-phase
stationary coordinate system based on the Alpha and Beta orthogonal components. */
typedef struct
{
    Q15_t s16Alpha;
    Q15_t s16Beta;
} MCFLIB_2_ALBE_T_S16;

/* The MCFLIB_2COOR_DQ_T_s16 structure type corresponds to the two-phase rotating
coordinate system based on the D and Q orthogonal components. */
typedef struct
{
    Q15_t s16D;
    Q15_t s16Q;
} MCFLIB_2_DQ_T_S16;

/* The MCFLIB_2COOR_DQ_T_s32 structure type corresponds to the two-phase rotating
coordinate system based on the D and Q orthogonal components. */
typedef struct
{
    Q31_t s32D;
    Q31_t s32Q;
} MCFLIB_2_DQ_T_S32;

/* The MCFLIB_2COOR_ALBE_T_s16 structure type corresponds to the two-phase
stationary coordinate system based on the Alpha and Beta orthogonal components. */
typedef struct
{
    Q31_t s32Alpha;
    Q31_t s32Beta;
} MCFLIB_2_ALBE_T_S32;


/*******************************************************************************
* Macros 
*******************************************************************************/ 
typedef struct
{
    uint8_t    u8Section;
		int16_t    s16U_Alpha;
    int16_t    s16U_Beta;
		int16_t    t_1;
    int16_t    t_2;
		uint16_t   u16sector;
    int16_t    s16DutyA;
    int16_t    s16DutyB;
    int16_t    s16DutyC;
        
}SVM_TypeDef;


/*******************************************************************************
* Exported function prototypes
*******************************************************************************/   
extern uint16_t MCF_Svm_7(SVM_TypeDef *sSVM);

/***************************************************************************//*!
*
* @brief       
*
* @param          
*
* @remarks     
*
*              d = alpha * cos(theta) + beta  * sin(theta)
*              q = beta  * cos(theta) - alpha * sin(theta)
*
*******************************************************************************/

static inline void MCFLIB_Park_Sat_S16(const MCFLIB_2_ALBE_T_S16 *psIn,
                                       const sAngle_Trig *psAnglePos,
                                       MCFLIB_2_DQ_T_S16 *psOut)
{
    register Q31_t s16A;
    register Q31_t s16B;
    register Q31_t s16Sin;
    register Q31_t s16Cos;
    register Q31_t s32Temp;
    
    /* Store values for calculation */
    s16A    = (psIn->s16Alpha);
    s16B    = (psIn->s16Beta);
    s16Sin  = (psAnglePos->s16Sin);
    s16Cos  = (psAnglePos->s16Cos);
    
    /* Calculate d = alpha * cos(theta) + beta * sin(theta) */
    s32Temp = (s16Cos * s16A + s16Sin * s16B) >> 15;
    /* Result limitation to 16-bit range*/     
    s32Temp = (s32Temp < -32768) ? INT16_MIN : s32Temp;
    s32Temp = (s32Temp >  32767) ? INT16_MAX : s32Temp;
    psOut->s16D = (Q15_t)s32Temp ;
    
    /* Calculate q = beta * cos(theta) - alpha * sin(theta) */
    s32Temp = ((s16Cos * s16B) - (s16Sin * s16A)) >> 15;
    /* Result limitation to 16-bit range */     
    s32Temp = (s32Temp < -32768) ? INT16_MIN : s32Temp;
    s32Temp = (s32Temp >  32767) ? INT16_MAX : s32Temp;
    psOut->s16Q = (Q15_t) s32Temp ;    
}

/***************************************************************************//*!
*
* @brief       
*
* @param          
*
* @remarks     
*
*              d = alpha * cos(theta) + beta  * sin(theta)
*              q = beta  * cos(theta) - alpha * sin(theta)
*
*******************************************************************************/

static inline void MCFLIB_Park_S16(const MCFLIB_2_ALBE_T_S16 *psIn,
                                       const sAngle_Trig *psAnglePos,
                                       MCFLIB_2_DQ_T_S16 *psOut)
{
    register Q31_t s16A;
    register Q31_t s16B;
    register Q31_t s16Sin;
    register Q31_t s16Cos;
    register Q31_t s32Temp;
    
    /* Store values for calculation */
    s16A    = (psIn->s16Alpha);
    s16B    = (psIn->s16Beta);
    s16Sin  = (psAnglePos->s16Sin);
    s16Cos  = (psAnglePos->s16Cos);    
    /* Calculate d = alpha * cos(theta) + beta * sin(theta) */
    s32Temp = (s16Cos * s16A + s16Sin * s16B) >> 15;
    psOut->s16D = (Q15_t)s32Temp ;
    
    /* Calculate q = beta * cos(theta) - alpha * sin(theta) */
    s32Temp = ((s16Cos * s16B) - (s16Sin * s16A)) >> 15;
    psOut->s16Q = (Q15_t) s32Temp ;    
}
/***************************************************************************//*!
*
* @brief        
*
* @param  
*                       
* @remarks      
*               alpha = d * cos(theta) - q * sin(theta)
*               beta  = d * sin(theta) + q * cos(theta)
*
*******************************************************************************/

static inline void MCFLIB_InvPark_Sat_S16(const MCFLIB_2_DQ_T_S16 *psIn,
                                          const sAngle_Trig *psAnglePos,
                                          MCFLIB_2_ALBE_T_S16 *psOut)
{
    register Q31_t s16D;
    register Q31_t s16Q;
    register Q31_t s16Sin;
    register Q31_t s16Cos;
    register Q31_t s32Temp;
    
    /* Store values for calculation */
    s16D    = (psIn->s16D);
    s16Q    = (psIn->s16Q);
    s16Sin  = (psAnglePos->s16Sin);
    s16Cos  = (psAnglePos->s16Cos);
    
    /* Calculate beta  = d * sin(theta) + q * cos(theta) */ 
    s32Temp = (s16Cos * s16Q + s16Sin * s16D) >> 15;
    /* Result limitation to 16-bit range*/    
    s32Temp = (s32Temp < -32768) ? INT16_MIN : s32Temp;
    s32Temp = (s32Temp >  32767) ? INT16_MAX : s32Temp;
    psOut->s16Beta = (Q15_t)s32Temp ;
    
    /* Calculate alpha = d * cos(theta) - q * sin(theta) */   
    s32Temp = ((s16Cos * s16D) - (s16Sin * s16Q)) >> 15;
    /* Result limitation to 16-bit range*/    
    s32Temp = (s32Temp < -32768) ? INT16_MIN : s32Temp;
    s32Temp = (s32Temp >  32767) ? INT16_MAX : s32Temp;
    psOut->s16Alpha = (Q15_t) s32Temp ;   
}
/***************************************************************************//*!
*
* @brief        
*
* @param  
*                       
* @remarks      
*               alpha = d * cos(theta) - q * sin(theta)
*               beta  = d * sin(theta) + q * cos(theta)
*
*******************************************************************************/

static inline void MCFLIB_InvPark_S16(const MCFLIB_2_DQ_T_S16 *psIn,
                                          const sAngle_Trig *psAnglePos,
                                          MCFLIB_2_ALBE_T_S16 *psOut)
{
    register Q31_t s16D;
    register Q31_t s16Q;
    register Q31_t s16Sin;
    register Q31_t s16Cos;
    register Q31_t s32Temp;
    
    /* Store values for calculation */
    s16D    = (psIn->s16D);
    s16Q    = (psIn->s16Q);
    s16Sin  = (psAnglePos->s16Sin);
    s16Cos  = (psAnglePos->s16Cos);
    
    /* Calculate beta  = d * sin(theta) + q * cos(theta) */ 
    s32Temp = (s16Cos * s16Q + s16Sin * s16D) >> 15;
    psOut->s16Beta = (Q15_t)s32Temp ;
    
    /* Calculate alpha = d * cos(theta) - q * sin(theta) */   
    s32Temp = ((s16Cos * s16D) - (s16Sin * s16Q)) >> 15;
    psOut->s16Alpha = (Q15_t) s32Temp ;   
}
/***************************************************************************//*!
*
* @brief   
* @remarks Modifies the structure pointed by psOut pointer 
*          according to the following equations:
*
*          alpha = a
*          beta  = b * 1 / sqrt(3)  - c * 1 / sqrt(3) 
* 
****************************************************************************/  
static inline void MCFLIB_Clark_Sat_S16(const MCFLIB_3_ABC_T_S16 *psIn,
                                        MCFLIB_2_ALBE_T_S16 *psOut)
{     
    register Q31_t s32Temp;
    
    /* Calculate Alpha orthogonal coordinate [alpha = a] */
    psOut->s16Alpha = psIn->s16A;
    
    /* Calculate Beta orthogonal coordinate [beta  = b * 1 / sqrt(3)  - c * 1 / sqrt(3] 
    and right shift to 16-bit range */
    s32Temp = ((psIn->s16B - psIn->s16C)*MCFLIB_ONE_DIV_SQRT3_S16)>>15;
    
    /* Result limitation to 16-bit range*/ 
    s32Temp = (s32Temp < INT16_MIN) ? INT16_MIN : s32Temp;
    s32Temp = (s32Temp > INT16_MAX) ? INT16_MAX : s32Temp;   
    psOut->s16Beta = s32Temp;
}

/****************************************************************************
*
* @brief        
*
*               a = alpha
*               b = -0.5 * alpha + sgrt(3) / 2 * beta
*               c = -0.5 * alpha - sgrt(3) / 2 * beta
*
****************************************************************************/ 
static inline void MCFLIB_InvClark_Sat_S16(const MCFLIB_2_ALBE_T_S16 *psIn,
                                           MCFLIB_3_ABC_T_S16 *psOut)
{                                                                                     
    register Q31_t s16M1;
    register Q31_t s16M2;
    register Q31_t s32Temp;
    
    /* Store constant -0.5*alpha to s16M1 */
    s16M1 = -(psIn->s16Alpha >> 1);
    /* Store constant beta * sqrt(3) / 2 shifted right to s16 range as s16M2 */
    s16M2 = (psIn->s16Beta * MCFLIB_SQRT3_DIV_2_S16) >> 15;
    
    /* Calculate a coordinate [a = alpha] */
    psOut->s16A = psIn->s16Alpha;   
    
    /* Calculate b coordinate [b = -alpha/2 + beta * sqrt(3) / 2  ] */    
    s32Temp = s16M1 + s16M2;
    /* Result limitation to 16-bit range*/  
    s32Temp = (s32Temp < INT16_MIN) ? INT16_MIN : s32Temp;
    s32Temp = (s32Temp > INT16_MAX) ? INT16_MAX : s32Temp;           
    psOut->s16B = s32Temp;
    
    /* Calculate c coordinate [c = -alpha/2 - beta * sqrt(3) / 2  ] */  
    s32Temp = s16M1 - s16M2;
    /* Result limitation to 16-bit range*/ 
    s32Temp = (s32Temp < INT16_MIN) ? INT16_MIN : s32Temp;
    s32Temp = (s32Temp > INT16_MAX) ? INT16_MAX : s32Temp;  
    psOut->s16C = s32Temp;     
}




#endif /* _MCFLIB_TYPES_H_ */



