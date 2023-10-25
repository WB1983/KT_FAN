#include "Transfer.h"
#include "Math.h"
#include "mlib_types.h"

/******************************************************************************
 * void FPM_vClarkeTrans(TThreePhase* ptInput, TCartNum* ptOutput)
 ******************************************************************************/
void TRR_vClarkeTrans(TThreePhase* ptInput, TCartNum* ptOutput)
{
   TFp tTemp;

   /* Real part */
   ptOutput->tRe = ptInput->a;

   /* Imaginary part -> Imag = 1/sqrt(3)*(A + 2 * B) */
   tTemp  = ptInput->b * 2;
   tTemp += ptInput->a;
   tTemp  = FPM_FpMulQ15(tTemp,FPM_SQRT3_INV_Q15);

   ptOutput->tIm = tTemp;
}

/*-------------------------------------------------------------------------------------------------
 * static void VEC_vForwardTransformation(const TMotObj* ptMot)
 * Implementation of the forward park transformation (vector rotation)
 *------------------------------------------------------------------------------------------------- */
void TRR_vForwardTransformation(TCartNum* ptInput, TDQAxial* ptOutput, TAngle * pActualAngle)
{
	int32_t siCalcQ;   /* Calculation qG * q15  	Id = Ialpha * cos(psi) + Ibeta  * sin(psi)
      	  	  	  	  	  	  	  	  	  	  	Iq = Ibeta  * cos(psi) - Ialpha * sin(psi) */
	siCalcQ  = ptInput->tRe*FPM_tCos((uint16_t)(*pActualAngle)); /* Ialpha * cos(psi) */
	siCalcQ += ptInput->tIm*FPM_tSin((uint16_t)(*pActualAngle)); /* Ibeta  * sin(psi) */
	siCalcQ  = FPM_QxtoQy(siCalcQ,15,0);                                          /* qG * q15 -> qQ    */ // x>y !!!
	ptOutput->tAxis_d = siCalcQ;
	siCalcQ = ptInput->tIm * FPM_tCos((uint16_t)(*pActualAngle)); /* Ibeta  * cos(psi) */
	siCalcQ -= ptInput->tRe * FPM_tSin((uint16_t)(*pActualAngle)); /* Ialpha * sin(psi) */
	siCalcQ = FPM_QxtoQy(siCalcQ,15,0);                                          /* qG * q15 -> qQ    */ // x>y !!!
	ptOutput->tAxis_q = siCalcQ;

}

/*-------------------------------------------------------------------------------------------------
 * static void VEC_vBackwardTransformation(const TMotObj* ptMot)
 * Implementation of the backward park transformation (vector rotation)
 * This function rotates the rotor oriented voltage vector back to the
 * stator oriented coordinate system.
 *------------------------------------------------------------------------------------------------- */
void TRR_vBackwardTransformation(TDQAxial* ptInput, TCartNum* ptOutput, TAngle * pActualAngle)
{
	int32_t siCalcQ;   /* 	Ualpha = Ud * cos(psi) - Uq  * sin(psi)
      	  	  	  	    Ubeta  = Uq * cos(psi) + Ud  * sin(psi)  */

	siCalcQ  = ptInput->tAxis_d * FPM_tCos((uint16_t)(*pActualAngle)); /* Ud * cos(psi)     */
	siCalcQ -= ptInput->tAxis_q * FPM_tSin((uint16_t)(*pActualAngle)); /* Uq * sin(psi)     */
	siCalcQ  = FPM_QxtoQy(siCalcQ,15,0);                                           /* qG * q15 -> qQ    */ // x>y !!!
	ptOutput->tRe = siCalcQ;
	siCalcQ  = ptInput->tAxis_q * FPM_tCos((uint16_t)(*pActualAngle)); /* Uq * cos(psi)     */
	siCalcQ += ptInput->tAxis_d * FPM_tSin((uint16_t)(*pActualAngle)); /* Ud * sin(psi)     */
	siCalcQ  = FPM_QxtoQy(siCalcQ,15,0);                                           /* qG * q15 -> qQ    */ // x>y !!!
	ptOutput->tIm = siCalcQ;
}