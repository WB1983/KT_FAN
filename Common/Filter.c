/*****************************************************************************************************************
 * INCLUDES ******************************************************************************************************
 *****************************************************************************************************************/

#include "Filter.h"     /* Include own header */

/*****************************************************************************************************************
 * GLOBAL FUNCTIONS **********************************************************************************************
 *****************************************************************************************************************/

/*****************************************************************************************************************
 * void FIR_vCalcIIRFilter(FIRRFilterData* ptData, TFilterCoeff* tFilterCoeff) ***********************************
 *****************************************************************************************************************/
void FIR_vCalcFilter(TFilterData* ptData, TFilterCoeff* tFilterCoeff)
{
   TFp  tDiff;

   /*
    * Compute value difference between new input sample and integer
    * part of the last output sample.
    * NOTE: Input value is interpreted to be formatted in Q0 format
    * */
   tDiff = ptData->tFilterInputValue - FIR_tGetFilterOutput(ptData);

   /* Integrate weighted difference */
   ptData->tFilterOutputValue  += tDiff * (*tFilterCoeff);

   /*
    * Compute integer part of the actual LPF filter output.
    * */
   ptData->tRealFilterOutputValue = FIR_tGetFilterOutput(ptData);

}

/*****************************************************************************************************************
 * void FIR_vInitIIRFilterOutput(FIRRFilterData* ptFilter, TFp tInitValue) ***************************************
 *****************************************************************************************************************/
void FIR_vInitFilterOutput(TFilterData* ptFilter, TFp tInitValue) {
	ptFilter->tRealFilterOutputValue	= FPM_QytoQx(tInitValue, 0, FIR_FILTER_COEFF_FORMAT); // x < y !!!
	ptFilter->tRealFilterOutputValue	= tInitValue;
}

