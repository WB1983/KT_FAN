#ifndef __FILTER_H
#define __FILTER_H

/*****************************************************************************************************************
 * INCLUDES ******************************************************************************************************
 *****************************************************************************************************************/

#include "compile.h"		/* Include global compiler switches */
#include "Math.h"			/* Include fixed point math library */

/*****************************************************************************************************************
 * GLOBAL TYPES **************************************************************************************************
 *****************************************************************************************************************/
typedef TFp  TFilterCoeff;      /**< \brief Type definition of a LPF constant
                                     \note  The filter coefficient has the value range 0.0<Cf<1.0. The coefficient
                                            needs to be set with respect to the defined filter fixed point format!
                                     \note  The filter constant is not included in the data structure because
                                            this value might be located in flash (const) memory */

/**
    \brief Type definition of the IIR low pass filter data structure
*/
typedef struct _TFilterData {
     TFp    tFilterInputValue;          /**< Filter input value */
     TFp    tFilterOutputValue;         /**< Filter output value */

     TFp	tRealFilterOutputValue;		/**< Filter output value without fractional part */

}TFilterData;

/*****************************************************************************************************************
 * LIBRARY CONFIGURATION *****************************************************************************************
 *****************************************************************************************************************/
#define FIR_FILTER_COEFF_FORMAT  16      /**< Configuration Value: Filter coefficient fixed point format
                                              This value must be smaller than 16 */

#if (FIR_FILTER_COEFF_FORMAT > 16)
#error "IIR_FILTER_COEFF_FORMAT must be smaller than 16!"
#endif

/*****************************************************************************************************************
 * GLOBAL MACROS *************************************************************************************************
 *****************************************************************************************************************/
#define FIR_FILTER_COEFF(Val)    (Q16(Val))   /**< Filter coefficient value definition macro
                                                   \note CHANGE THIS VALUE with respect to the IIR_FILTER_COEFF_FORMAT!
                                                         The transformation macro must match with the format */

/**
    \brief Macro converts a filter time constant into the filter coefficient in fp format

    \param Tsample_ms Sample time of the filter inputs
    \param Tfilter_ms Desired time constant of the low pass filter

    \note This macro should be used to define filter constants because this ensures that
          the format of the filter can be easily changed.
    \note The time constants can be defined in float format. The coefficient will be converted
          to fixed point format by the pre-processor.
    \note This macro can only be used for definition of coefficients BEFORE compile time!
*/
#define FIR_tCalcFilterConst(Tsample_ms, Tfilter_ms)     (FIR_FILTER_COEFF(((Tsample_ms)/(Tfilter_ms))))

/**
    \brief Interface to the integer filter output value

    \param ptFilter POINTER to the filter data structure
*/
#define FIR_tGetFilterOutput(ptFilter)	(FPM_QxtoQy((ptFilter)->tFilterOutputValue,FIR_FILTER_COEFF_FORMAT,0))// x>y !!!
#define FIR_tGetFilterOutputx(ptFilter)	(FPM_QxtoQy((ptFilter)->tFilterOutputValue,FIR_FILTER_COEFF_FORMAT,0))// x>y !!!

/*****************************************************************************************************************
 * GLOBAL FUNCTIONS **********************************************************************************************
 *****************************************************************************************************************/
/**
    \brief This macro initializes the filter output value according to the filter format

    \param ptFilter    Pointer to the filter data structure
    \param tInitValue  Initial filter output value
*/
void FIR_vInitFilterOutput(TFilterData* ptFilter, TFp tInitValue);

/**
    \brief IIR filter calculation function

    The filter function implements a simple single tap IIR low-pass-filter.
    The following formula is implemented:\n

    y(n) = (x(n)-int(y(n-1))) * Kf + y(n-1) \n

    Where\n

    y(n)   denotes the actual output sample
    y(n-1) denotes the last output sample
    x(n)   denotes the actual input sample
    Kf     denotes the filter coefficient

    \param ptData         Pointer to the filter data structure
    \param ptFilterCoeff  Pointer to the filter coefficients (constants)

    \return void

    \remark Only pointers to the required data structures are passed. This allows
            a future extension to more than one filter tap.
*/
void FIR_vCalcFilter(TFilterData* ptData, TFilterCoeff* tFilterCoeff);

#endif
