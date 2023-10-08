
#ifndef __FPM_H
#define __FPM_H

/*****************************************************************************************************************
 * INCLUDES ******************************************************************************************************
 *****************************************************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>



/*****************************************************************************************************************
 * GLOBAL TYPES **************************************************************************************************
 *****************************************************************************************************************/

/**
    \brief Type definition of the fixed point data format
*/
typedef int32_t  TFp;

/**
  \brief Type definition of the used angle format
*/
typedef int16_t  TAngle;

/**
  \brief Type definition of a complex number in cartesian format
*/
typedef struct _TCartNum {
    TFp tRe;    /**< Real part of the complex number */
    TFp tIm;    /**< Imag part of the complex number */
}TCartNum;

/**
  \brief Type definition of a complex number in polar format
*/
typedef struct _TPolNum {
    TFp     tMag;       /**< Magnitude of the complex vector */
    TAngle  siAngle;    /**< Angle of the complex vector */
}TPolNum;

/**
   \brief Type definition of a complex number
*/
typedef struct _TCplxNum {
    TCartNum    tCartNum;     /**< Complex number in cartesian format */
    TPolNum     tPolNum;      /**< Complex number in polar format */
}TCplxNum;

/**
    \brief Type definition of a common three phase system value
*/
typedef struct _TThreePhase {
    TFp a;    /**< Component of the A phase */
    TFp b;    /**< Component of the B phase */
    TFp c;    /**< Component of the C phase */
}TThreePhase;

/**
    \brief Type definition of a common d,q phase system value
*/
typedef struct _TDQAxial{
     TFp tAxis_d;
     TFp tAxis_q;
}TDQAxial;


/**
 * \brief Type definition of a point
 *
 * A point is defined by a x coordinate and a y coordinate in a cartesian coordinate system
 * */
typedef struct _TPoint {
	TFp	tXValue;	/**< x-Coordinate */
	TFp	tYValue;	/**< y-Coordinate */
}TPoint;


/*****************************************************************************************************************
 * GLOBAL DEFINITIONS ********************************************************************************************
 *****************************************************************************************************************/

/**
  \brief Definition of the global fixed point format
 
  This value determines the number of bits used for the fractional part in
  all standard fixed point numbers.

  \remark CONFIGURE HERE THE DESIRED FP FORMAT
*/
#define FPM_GLOBAL_FP_FORMAT    12

#define FPM_SHIFT_360                  65536   /* 360 q16  */
#define FPM_TWO_PI            ((TAngle)65535)  /* 360 q15  */
#define FPM_PI                ((TAngle)32767)  /* 180 q15  */
#define FPM_NEG_PI            ((TAngle)32768)  /*-180 q15  */
#define FPM_PI_HALF           ((TAngle)16384)  /*  90 q15  */
#define FPM_NEG_PI_HALF       ((TAngle)-16384) /* -90 q15  */
#define FPM_PI_QUARTER        ((TAngle) 8192)  /*  45 q15  */
#define FPM_NEG_PI_QUARTER    ((TAngle)-8192)  /* -45 q15  */
#define FPM_PI_THIRD          ((TAngle)10923)  /*  60 q15  */

#define FPM_SQRT3_INV_Q15    Q15(0.577350279)  /* 1/sqrt(3)        q15 format */
#define FPM_SQRT3_INV_Q      FP(0.577350279)   /* 1/sqrt(3)     global format */
#define FPM_TWO_DIV_SQRT3_Q  FP(1.154700538)   /* 2/sqrt(3)     global format */
#define FPM_SQRT2_Q15		          (1.414213)	 /* sqrt2	q15 format		*/

#define FPM_SIN_MAX_Q15              Q15(1.0)  /* Q15 maximum value  (reference) */

/*****************************************************************************************************************
 * GLOBAL MACROS *************************************************************************************************
 *****************************************************************************************************************/

/**
  \brief Definition of the fixed point conversion macro
 
  The passed number is converted to the standard fixed point format as it is
  defined above (refer to FPM_GLOBAL_FP_FORMAT)
 
*/
#define Q28(x)    ((TFp)((x)*268435456.0))   /**< Q28 fixed point number definition macro */
#define Q27(x)    ((TFp)((x)*134217728.0))   /**< Q27 fixed point number definition macro */
#define Q26(x)    ((TFp)((x)*67108864.0))    /**< Q26 fixed point number definition macro */
#define Q25(x)    ((TFp)((x)*33554432.0))    /**< Q25 fixed point number definition macro */
#define Q24(x)    ((TFp)((x)*16777216.0))    /**< Q24 fixed point number definition macro */
#define Q23(x)    ((TFp)((x)*8388608.0))     /**< Q23 fixed point number definition macro */
#define Q22(x)    ((TFp)((x)*4194304.0))     /**< Q22 fixed point number definition macro */
#define Q21(x)    ((TFp)((x)*2097152.0))     /**< Q21 fixed point number definition macro */
#define Q20(x)    ((TFp)((x)*1048576.0))     /**< Q20 fixed point number definition macro */
#define Q19(x)    ((TFp)((x)*524288.0))      /**< Q19 fixed point number definition macro */
#define Q18(x)    ((TFp)((x)*262144.0))      /**< Q18 fixed point number definition macro */
#define Q17(x)    ((TFp)((x)*131072.0))      /**< Q17 fixed point number definition macro */
#define Q16(x)    ((TFp)((x)*65536.0))       /**< Q16 fixed point number definition macro */
//#define Q15(x)    ((TFp)((x)*32768.0))       /**< Q15 fixed point number definition macro */
#define Q14(x)    ((TFp)((x)*16384.0))       /**< Q14 fixed point number definition macro */
#define Q13(x)    ((TFp)((x)*8192.0))        /**< Q13 fixed point number definition macro */
#define Q12(x)    ((TFp)((x)*4096.0))        /**< Q12 fixed point number definition macro */
#define Q11(x)    ((TFp)((x)*2048.0))        /**< Q11 fixed point number definition macro */
#define Q10(x)    ((TFp)((x)*1024.0))        /**< Q10 fixed point number definition macro */
#define Q9(x)     ((TFp)((x)*512.0))         /**< Q9 fixed point number definition macro */
#define Q8(x)     ((TFp)((x)*256.0))         /**< Q8 fixed point number definition macro */
//#define Q7(x)     ((TFp)((x)*128.0))         /**< Q7 fixed point number definition macro */
#define Q6(x)     ((TFp)((x)*64.0))          /**< Q6 fixed point number definition macro */
#define Q5(x)     ((TFp)((x)*32.0))          /**< Q5 fixed point number definition macro */
#define Q4(x)     ((TFp)((x)*16.0))          /**< Q4 fixed point number definition macro */
#define Q3(x)     ((TFp)((x)*8.0))           /**< Q3 fixed point number definition macro */
#define Q2(x)     ((TFp)((x)*4.0))           /**< Q2 fixed point number definition macro */
#define Q1(x)     ((TFp)((x)*2.0))           /**< Q1 fixed point number definition macro */
#define Q0(x)     ((TFp)((x)*1.0))           /**< Q0 fixed point number definition macro */

/**
  \brief Definition of the fixed point conversion macro

  The passed number is converted to the standard fixed point format as it is
  defined above (refer to FPM_GLOBAL_FP_FORMAT)

*/
#define Qu28(x)    ((TFp)((x)*268435456))   /**< Q28 fixed point number definition macro */
#define Qu27(x)    ((TFp)((x)*134217728))   /**< Q27 fixed point number definition macro */
#define Qu26(x)    ((TFp)((x)*67108864))    /**< Q26 fixed point number definition macro */
#define Qu25(x)    ((TFp)((x)*33554432))    /**< Q25 fixed point number definition macro */
#define Qu24(x)    ((TFp)((x)*16777216))    /**< Q24 fixed point number definition macro */
#define Qu23(x)    ((TFp)((x)*8388608))     /**< Q23 fixed point number definition macro */
#define Qu22(x)    ((TFp)((x)*4194304))     /**< Q22 fixed point number definition macro */
#define Qu21(x)    ((TFp)((x)*2097152))     /**< Q21 fixed point number definition macro */
#define Qu20(x)    ((TFp)((x)*1048576))     /**< Q20 fixed point number definition macro */
#define Qu19(x)    ((TFp)((x)*524288))      /**< Q19 fixed point number definition macro */
#define Qu18(x)    ((TFp)((x)*262144))      /**< Q18 fixed point number definition macro */
#define Qu17(x)    ((TFp)((x)*131072))      /**< Q17 fixed point number definition macro */
#define Qu16(x)    ((TFp)((x)*65536))       /**< Q16 fixed point number definition macro */
#define Qu15(x)    ((TFp)((x)*32768))       /**< Q15 fixed point number definition macro */
#define Qu14(x)    ((TFp)((x)*16384))       /**< Q14 fixed point number definition macro */
#define Qu13(x)    ((TFp)((x)*8192))        /**< Q13 fixed point number definition macro */
#define Qu12(x)    ((TFp)((x)*4096))        /**< Q12 fixed point number definition macro */
#define Qu11(x)    ((TFp)((x)*2048))        /**< Q11 fixed point number definition macro */
#define Qu10(x)    ((TFp)((x)*1024))        /**< Q10 fixed point number definition macro */
#define Qu9(x)     ((TFp)((x)*512))         /**< Q9 fixed point number definition macro */
#define Qu8(x)     ((TFp)((x)*256))         /**< Q8 fixed point number definition macro */
#define Qu7(x)     ((TFp)((x)*128))         /**< Q7 fixed point number definition macro */
#define Qu6(x)     ((TFp)((x)*64))          /**< Q6 fixed point number definition macro */
#define Qu5(x)     ((TFp)((x)*32))          /**< Q5 fixed point number definition macro */
#define Qu4(x)     ((TFp)((x)*16))          /**< Q4 fixed point number definition macro */
#define Qu3(x)     ((TFp)((x)*8))           /**< Q3 fixed point number definition macro */
#define Qu2(x)     ((TFp)((x)*4))           /**< Q2 fixed point number definition macro */
#define Qu1(x)     ((TFp)((x)*2))           /**< Q1 fixed point number definition macro */
#define Qu0(x)     ((TFp)((x)*1))           /**< Q0 fixed point number definition macro */


#if (FPM_GLOBAL_FP_FORMAT == 0)
#define FP(x)     Qu0(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 1)
#define FP(x)     Qu1(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 2)
#define FP(x)     Qu2(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 3)
#define FP(x)     Qu3(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 4)
#define FP(x)     Qu4(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 5)
#define FP(x)     Qu5(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 6)
#define FP(x)     Qu6(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 7)
#define FP(x)     Qu7(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 8)
#define FP(x)     Qu8(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 9)
#define FP(x)     Qu9(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 10)
#define FP(x)     Qu10(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 11)
#define FP(x)     Qu11(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 12)
#define FP(x)     Qu12(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 13)
#define FP(x)     Qu13(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 14)
#define FP(x)     Qu14(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 15)
#define FP(x)     Qu15(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 16)
#define FP(x)     Qu16(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 17)
#define FP(x)     Qu17(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 18)
#define FP(x)     Qu18(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 19)
#define FP(x)     Qu19(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 20)
#define FP(x)     Qu20(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 21)
#define FP(x)     Qu21(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 22)
#define FP(x)     Qu22(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 23)
#define FP(x)     Qu23(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 24)
#define FP(x)     Qu24(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 25)
#define FP(x)     Qu25(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 26)
#define FP(x)     Qu26(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 27)
#define FP(x)     Qu27(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 28)
#define FP(x)     Qu28(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 29)
#define FP(x)     Qu29(x)
#endif
#if (FPM_GLOBAL_FP_FORMAT == 30)
#define FP(x)     Qu30(x)
#endif

/******************************************************************************
 * FORMAT CONVERSION MACROS ***************************************************
 ******************************************************************************/
#define   GFPF   FPM_GLOBAL_FP_FORMAT

#if(GFPF >= 30)
#define   FPM_GQtoQ30(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 30)))
#define   FPM_Q30toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 30)))
#else
#define   FPM_GQtoQ30(x)  ((TFp) (x) * (TFp)(1UL << (30 - GFPF)))
#define   FPM_Q30toGQ(x)  ((TFp) (x) / (TFp)(1UL << (30 - GFPF)))
#endif

#if(GFPF >= 29)
#define   FPM_GQtoQ29(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 29)))
#define   FPM_Q29toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 29)))
#else
#define   FPM_GQtoQ29(x)  ((TFp) (x) * (TFp)(1UL << (29 - GFPF)))
#define   FPM_Q29toGQ(x)  ((TFp) (x) / (TFp)(1UL << (29 - GFPF)))
#endif

#if(GFPF >= 28)
#define   FPM_GQtoQ28(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 28)))
#define   FPM_Q28toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 28)))
#else
#define   FPM_GQtoQ28(x)  ((TFp) (x) * (TFp)(1UL << (28 - GFPF)))
#define   FPM_Q28toGQ(x)  ((TFp) (x) / (TFp)(1UL << (28 - GFPF)))
#endif

#if(GFPF >= 27)
#define   FPM_GQtoQ27(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 27)))
#define   FPM_Q27toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 27)))
#else
#define   FPM_GQtoQ27(x)  ((TFp) (x) * (TFp)(1UL << (27 - GFPF)))
#define   FPM_Q27toGQ(x)  ((TFp) (x) / (TFp)(1UL << (27 - GFPF)))
#endif

#if(GFPF >= 27)
#define   FPM_GQtoQ26(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 26)))
#define   FPM_Q26toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 26)))
#else
#define   FPM_GQtoQ26(x)  ((TFp) (x) * (TFp)(1UL << (26 - GFPF)))
#define   FPM_Q26toGQ(x)  ((TFp) (x) / (TFp)(1UL << (26 - GFPF)))
#endif

#if(GFPF >= 25)
#define   FPM_GQtoQ25(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 25)))
#define   FPM_Q25toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 25)))
#else
#define   FPM_GQtoQ25(x)  ((TFp) (x) * (TFp)(1UL << (25 - GFPF)))
#define   FPM_Q25toGQ(x)  ((TFp) (x) / (TFp)(1UL << (25 - GFPF)))
#endif

#if(GFPF >= 24)
#define   FPM_GQtoQ24(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 24)))
#define   FPM_Q24toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 24)))
#else
#define   FPM_GQtoQ24(x)  ((TFp) (x) * (TFp)(1UL << (24 - GFPF)))
#define   FPM_Q24toGQ(x)  ((TFp) (x) / (TFp)(1UL << (24 - GFPF)))
#endif

#if(GFPF >= 23)
#define   FPM_GQtoQ23(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 23)))
#define   FPM_Q23toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 23)))
#else
#define   FPM_GQtoQ23(x)  ((TFp) (x) * (TFp)(1UL << (23 - GFPF)))
#define   FPM_Q23toGQ(x)  ((TFp) (x) / (TFp)(1UL << (23 - GFPF)))
#endif

#if(GFPF >= 22)
#define   FPM_GQtoQ22(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 22)))
#define   FPM_Q22toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 22)))
#else
#define   FPM_GQtoQ22(x)  ((TFp) (x) * (TFp)(1UL << (22 - GFPF)))
#define   FPM_Q22toGQ(x)  ((TFp) (x) / (TFp)(1UL << (22 - GFPF)))
#endif

#if(GFPF >= 21)
#define   FPM_GQtoQ21(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 21)))
#define   FPM_Q21toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 21)))
#else
#define   FPM_GQtoQ21(x)  ((TFp) (x) * (TFp)(1UL << (21 - GFPF)))
#define   FPM_Q21toGQ(x)  ((TFp) (x) / (TFp)(1UL << (21 - GFPF)))
#endif

#if(GFPF >= 20)
#define   FPM_GQtoQ20(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 20)))
#define   FPM_Q20toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 20)))
#else
#define   FPM_GQtoQ20(x)  ((TFp) (x) * (TFp)(1UL << (20 - GFPF)))
#define   FPM_Q20toGQ(x)  ((TFp) (x) / (TFp)(1UL << (20 - GFPF)))
#endif

#if(GFPF >= 19)
#define   FPM_GQtoQ19(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 19)))
#define   FPM_Q19toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 19)))
#else
#define   FPM_GQtoQ19(x)  ((TFp) (x) * (TFp)(1UL << (19 - GFPF)))
#define   FPM_Q19toGQ(x)  ((TFp) (x) / (TFp)(1UL << (19 - GFPF)))
#endif

#if(GFPF >= 18)
#define   FPM_GQtoQ18(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 18)))
#define   FPM_Q18toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 18)))
#else
#define   FPM_GQtoQ18(x)  ((TFp) (x) * (TFp)(1UL << (18 - GFPF)))
#define   FPM_Q18toGQ(x)  ((TFp) (x) / (TFp)(1UL << (18 - GFPF)))
#endif

#if(GFPF >= 17)
#define   FPM_GQtoQ17(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 17)))
#define   FPM_Q17toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 17)))
#else
#define   FPM_GQtoQ17(x)  ((TFp) (x) * (TFp)(1UL << (17 - GFPF)))
#define   FPM_Q17toGQ(x)  ((TFp) (x) / (TFp)(1UL << (17 - GFPF)))
#endif

#if(GFPF >= 16)
#define   FPM_GQtoQ16(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 16)))
#define   FPM_Q16toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 16)))
#else
#define   FPM_GQtoQ16(x)  ((TFp) (x) * (TFp)(1UL << (16 - GFPF)))
#define   FPM_Q16toGQ(x)  ((TFp) (x) / (TFp)(1UL << (16 - GFPF)))
#endif

#if(GFPF >= 15)
#define   FPM_GQtoQ15(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 15)))
#define   FPM_Q15toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 15)))
#else
#define   FPM_GQtoQ15(x)  ((TFp) (x) * (TFp)(1UL << (15 - GFPF)))
#define   FPM_Q15toGQ(x)  ((TFp) (x) / (TFp)(1UL << (15 - GFPF)))
#endif

#if(GFPF >= 14)
#define   FPM_GQtoQ14(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 14)))
#define   FPM_Q14toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 14)))
#else
#define   FPM_GQtoQ14(x)  ((TFp) (x) * (TFp)(1UL << (14 - GFPF)))
#define   FPM_Q14toGQ(x)  ((TFp) (x) / (TFp)(1UL << (14 - GFPF)))
#endif

#if(GFPF >= 13)
#define   FPM_GQtoQ13(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 13)))
#define   FPM_Q13toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 13)))
#else
#define   FPM_GQtoQ13(x)  ((TFp) (x) * (TFp)(1UL << (13 - GFPF)))
#define   FPM_Q13toGQ(x)  ((TFp) (x) / (TFp)(1UL << (13 - GFPF)))
#endif

#if(GFPF >= 12)
#define   FPM_GQtoQ12(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 12)))
#define   FPM_Q12toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 12)))
#else
#define   FPM_GQtoQ12(x)  ((TFp) (x) * (TFp)(1UL << (12 - GFPF)))
#define   FPM_Q12toGQ(x)  ((TFp) (x) / (TFp)(1UL << (12 - GFPF)))
#endif

#if(GFPF >= 11)
#define   FPM_GQtoQ11(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 11)))
#define   FPM_Q11toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 11)))
#else
#define   FPM_GQtoQ11(x)  ((TFp) (x) * (TFp)(1UL << (11 - GFPF)))
#define   FPM_Q11toGQ(x)  ((TFp) (x) / (TFp)(1UL << (11 - GFPF)))
#endif

#if(GFPF >= 10)
#define   FPM_GQtoQ10(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 10)))
#define   FPM_Q10toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 10)))
#else
#define   FPM_GQtoQ10(x)  ((TFp) (x) * (TFp)(1UL << (10 - GFPF)))
#define   FPM_Q10toGQ(x)  ((TFp) (x) / (TFp)(1UL << (10 - GFPF)))
#endif

#if(GFPF >= 9)
#define   FPM_GQtoQ9(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 9)))
#define   FPM_Q9toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 9)))
#else
#define   FPM_GQtoQ9(x)  ((TFp) (x) * (TFp)(1UL << (9 - GFPF)))
#define   FPM_Q9toGQ(x)  ((TFp) (x) / (TFp)(1UL << (9 - GFPF)))
#endif

#if(GFPF >= 8)
#define   FPM_GQtoQ8(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 8)))
#define   FPM_Q8toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 8)))
#else
#define   FPM_GQtoQ8(x)  ((TFp) (x) * (TFp)(1UL << (8 - GFPF)))
#define   FPM_Q8toGQ(x)  ((TFp) (x) / (TFp)(1UL << (8 - GFPF)))
#endif

#if(GFPF >= 7)
#define   FPM_GQtoQ7(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 7)))
#define   FPM_Q7toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 7)))
#else
#define   FPM_GQtoQ7(x)  ((TFp) (x) * (TFp)(1UL << (7 - GFPF)))
#define   FPM_Q7toGQ(x)  ((TFp) (x) / (TFp)(1UL << (7 - GFPF)))
#endif

#if(GFPF >= 6)
#define   FPM_GQtoQ6(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 6)))
#define   FPM_Q6toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 6)))
#else
#define   FPM_GQtoQ6(x)  ((TFp) (x) * (TFp)(1UL << (6 - GFPF)))
#define   FPM_Q6toGQ(x)  ((TFp) (x) / (TFp)(1UL << (6 - GFPF)))
#endif

#if(GFPF >= 5)
#define   FPM_GQtoQ5(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 5)))
#define   FPM_Q5toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 5)))
#else
#define   FPM_GQtoQ5(x)  ((TFp) (x) * (TFp)(1UL << (5 - GFPF)))
#define   FPM_Q5toGQ(x)  ((TFp) (x) / (TFp)(1UL << (5 - GFPF)))
#endif

#if(GFPF >= 4)
#define   FPM_GQtoQ4(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 4)))
#define   FPM_Q4toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 4)))
#else
#define   FPM_GQtoQ4(x)  ((TFp) (x) * (TFp)(1UL << (4 - GFPF)))
#define   FPM_Q4toGQ(x)  ((TFp) (x) / (TFp)(1UL << (4 - GFPF)))
#endif

#if(GFPF >= 3)
#define   FPM_GQtoQ3(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 3)))
#define   FPM_Q3toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 3)))
#else
#define   FPM_GQtoQ3(x)  ((TFp) (x) * (TFp)(1UL << (3 - GFPF)))
#define   FPM_Q3toGQ(x)  ((TFp) (x) / (TFp)(1UL << (3 - GFPF)))
#endif

#if(GFPF >= 2)
#define   FPM_GQtoQ2(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 2)))
#define   FPM_Q2toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 2)))
#else
#define   FPM_GQtoQ2(x)  ((TFp) (x) * (TFp)(1UL << (2 - GFPF)))
#define   FPM_Q2toGQ(x)  ((TFp) (x) / (TFp)(1UL << (2 - GFPF)))
#endif

#if(GFPF >= 1)
#define   FPM_GQtoQ1(x)  ((TFp) (x) / (TFp)(1UL << (GFPF - 1)))
#define   FPM_Q1toGQ(x)  ((TFp) (x) * (TFp)(1UL << (GFPF - 1)))
#else
#define   FPM_GQtoQ1(x)  ((TFp) (x) * (TFp)(1UL << (1 - GFPF)))
#define   FPM_Q1toGQ(x)  ((TFp) (x) / (TFp)(1UL << (1 - GFPF)))
#endif

/* Generic conversion macro */

//#define   FPM_QxtoQy123(val,x,y)  (((x)>=(y)) ? ((TFp)(val)/(TFp)(1UL << ((x) - (y)))):((TFp) (val) * (TFp)(1UL <<((y) - (x)))))
// Upper macro was redefined to next macros, due lint errors.
//	 now is needed to check, if x is > than y or vice versa.
#define   FPM_QxtoQy(val,x,y) ((TFp)(val)/(TFp)(1UL << ((x) - (y))))	// if x > y
#define   FPM_QytoQx(val,x,y)  ((TFp) (val) * (TFp)(1UL <<((y) - (x))))	// if x < y
#define   FPM_QxtoQx(val,x,y)  ((TFp)(val))								// if x = y

/******************************************************************************
 * FIXED POINT MULTIPLICATION MACROS ******************************************
 ******************************************************************************/
#define FPM_FpMul(x,y)     ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL << FPM_GLOBAL_FP_FORMAT)))

#define FPM_FpMulQ1(x,y)   ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 1)))
#define FPM_FpMulQ2(x,y)   ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 2)))
#define FPM_FpMulQ3(x,y)   ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 3)))
#define FPM_FpMulQ4(x,y)   ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 4)))
#define FPM_FpMulQ5(x,y)   ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 5)))
#define FPM_FpMulQ6(x,y)   ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 6)))
#define FPM_FpMulQ7(x,y)   ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 7)))
#define FPM_FpMulQ8(x,y)   ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 8)))
#define FPM_FpMulQ9(x,y)   ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 9)))
#define FPM_FpMulQ10(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 10)))
#define FPM_FpMulQ11(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 11)))
#define FPM_FpMulQ12(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 12)))
#define FPM_FpMulQ13(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 13)))
#define FPM_FpMulQ14(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 14)))
#define FPM_FpMulQ15(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 15)))
#define FPM_FpMulQ16(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 16)))
#define FPM_FpMulQ17(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 17)))
#define FPM_FpMulQ18(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 18)))
#define FPM_FpMulQ19(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 19)))
#define FPM_FpMulQ20(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 20)))
#define FPM_FpMulQ21(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 21)))
#define FPM_FpMulQ22(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 22)))
#define FPM_FpMulQ23(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 23)))
#define FPM_FpMulQ24(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 24)))
#define FPM_FpMulQ25(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 25)))
#define FPM_FpMulQ26(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 26)))
#define FPM_FpMulQ27(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 27)))
#define FPM_FpMulQ28(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 28)))
#define FPM_FpMulQ29(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 29)))
#define FPM_FpMulQ30(x,y)  ((TFp)(((TFp)(x)*(TFp)(y)) / (TFp)(1UL  << 30)))

/******************************************************************************
 * FIXED POINT DIVISION MACROS ************************************************
 ******************************************************************************/
#define FPM_FpDiv(x,y)     ((TFp)(((TFp)(x)* (TFp)(1UL  << FPM_GLOBAL_FP_FORMAT))/(TFp)(y)))

#define FPM_FpDivQ1(x,y)   ((TFp)(((TFp)(x)* (TFp)(1UL  << 1 ))/(TFp)(y)))
#define FPM_FpDivQ2(x,y)   ((TFp)(((TFp)(x)* (TFp)(1UL  << 2 ))/(TFp)(y)))
#define FPM_FpDivQ3(x,y)   ((TFp)(((TFp)(x)* (TFp)(1UL  << 3 ))/(TFp)(y)))
#define FPM_FpDivQ4(x,y)   ((TFp)(((TFp)(x)* (TFp)(1UL  << 4 ))/(TFp)(y)))
#define FPM_FpDivQ5(x,y)   ((TFp)(((TFp)(x)* (TFp)(1UL  << 5 ))/(TFp)(y)))
#define FPM_FpDivQ6(x,y)   ((TFp)(((TFp)(x)* (TFp)(1UL  << 6 ))/(TFp)(y)))
#define FPM_FpDivQ7(x,y)   ((TFp)(((TFp)(x)* (TFp)(1UL  << 7 ))/(TFp)(y)))
#define FPM_FpDivQ8(x,y)   ((TFp)(((TFp)(x)* (TFp)(1UL  << 8 ))/(TFp)(y)))
#define FPM_FpDivQ9(x,y)   ((TFp)(((TFp)(x)* (TFp)(1UL  << 9 ))/(TFp)(y)))
#define FPM_FpDivQ10(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 10))/(TFp)(y)))
#define FPM_FpDivQ11(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 11))/(TFp)(y)))
#define FPM_FpDivQ12(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 12))/(TFp)(y)))
#define FPM_FpDivQ13(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 13))/(TFp)(y)))
#define FPM_FpDivQ14(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 14))/(TFp)(y)))
#define FPM_FpDivQ15(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 15))/(TFp)(y)))
#define FPM_FpDivQ16(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 16))/(TFp)(y)))
#define FPM_FpDivQ17(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 17))/(TFp)(y)))
#define FPM_FpDivQ18(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 18))/(TFp)(y)))
#define FPM_FpDivQ19(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 19))/(TFp)(y)))
#define FPM_FpDivQ20(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 20))/(TFp)(y)))
#define FPM_FpDivQ21(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 21))/(TFp)(y)))
#define FPM_FpDivQ22(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 22))/(TFp)(y)))
#define FPM_FpDivQ23(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 23))/(TFp)(y)))
#define FPM_FpDivQ24(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 24))/(TFp)(y)))
#define FPM_FpDivQ25(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 25))/(TFp)(y)))
#define FPM_FpDivQ26(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 26))/(TFp)(y)))
#define FPM_FpDivQ27(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 27))/(TFp)(y)))
#define FPM_FpDivQ28(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 28))/(TFp)(y)))
#define FPM_FpDivQ29(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 29))/(TFp)(y)))
#define FPM_FpDivQ30(x,y)  ((TFp)(((TFp)(x)* (TFp)(1UL  << 30))/(TFp)(y)))

/******************************************************************************
 * FIXED POINT ABS MACRO ******************************************************
 ******************************************************************************/
#define FPM_FpAbs(x)		((((TFp)(x)) >= 0) ? ((TFp)(x)) : ((TFp)(-(x))))

/*****************************************************************************************************************
 * GLOBAL FUNCTIONS **********************************************************************************************
 *****************************************************************************************************************/


/**
 * \brief Conversion from cartesian to polar coordinates
 * */
void FPM_vCart2Pol(TCartNum* tCN, TPolNum* tPN);

/**
    \brief Integer square root implementation

    Function calculates the integer square root of the passed parameter.

    \param ulRadicand Radicand for which the sqrt is computed

    \return uint32 Square root of the passed parameter

    \remark The result would also fit in an uint16 type but the native type
            of the target processor is 32bit wide => this prevents a value
            extension with 0

    \remark RUNTIME: ~4-5�s

*/
uint32_t FPM_ulSqrt(uint32_t ulRadicand);

//for motor startup improvement
extern int16_t QMATH_sin(int16_t angle);

extern int16_t QMATH_max(int16_t value1, int16_t value2);

extern int16_t QMATH_min(int16_t value1, int16_t value2);

extern int16_t QMATH_limit(int16_t value, int16_t limitMin, int16_t limitMax);
/*****************************************************************************************************************
 * GLOBAL TRIGONOMETRIC FUNCTIONS ********************************************************************************
 *****************************************************************************************************************/

/**
    \brief Decalration of the sine look up table (LUT)
*/
extern const TFp FPM_tSineTabQ15[];

/**
    \brief Size of the sine LUT (no. of elements)
*/
#define FPM_SINE_TAB_SIZE           (sizeof(FPM_tSineTabQ15)/sizeof(TFp))

/**
    \brief No. of bits used for the angle argument within the sine tab
*/
#define FPM_SINE_TAB_ANGLE_BITS     (8)

/**
    \brief Mapping of the implemented sine table
*/
#define FPM_tSineTab                FPM_tSineTabQ15

/**
    \brief Sin function (Q15-Format)

    The macro reads the mapped sine table according to the passed angle argument.
    The angle is a 16Bit value which is mapped on a 8Bit table index.
*/
#define FPM_tSin(Phi)        (FPM_tSineTab[ (uint32_t)(Phi) >> (16-FPM_SINE_TAB_ANGLE_BITS)])

/**
    \brief Cos function (Q15-Format)
    cosx = sin(x+90)

    The macro reads the mapped sine table according to the passed angle argument.
    The angle is a 16Bit value which is mapped on a 8Bit table index.
*/
#define FPM_tCos(Phi)       (FPM_tSineTab[(((uint32_t)(Phi) + (uint32_t)FPM_PI_HALF) & 0x0000FFFF) >> (16-FPM_SINE_TAB_ANGLE_BITS)])


//improve startup
#define Q11_MAX 				      2047
#define Q11_MIN  				      (-2048)
#define Q11_HALF_MAX 			      1024

#define QMATH_SINCOS_90DEGREE		  512
#define QMATH_SINCOS_180DEGREE		  1024
#define QMATH_SINCOS_270DEGREE		  1536
#define QMATH_SINCOS_360DEGREE		  2048
#endif		/* __FPM_H */
