
////////////////////////////////////////////////////////////////////////////////
/// @file    Interrupts.c
/// @author  NJ ClassB TEAM
/// @brief
///
////////////////////////////////////////////////////////////////////////////////
/// @attention
///
/// THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
/// CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
/// TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
/// CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
/// HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
/// CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
///
/// <H2><CENTER>&COPY; COPYRIGHT MINDMOTION </CENTER></H2>
////////////////////////////////////////////////////////////////////////////////
// Define to prevent recursive inclusion

#include "Interrupts_classb.h"

/*!
	The interrupt check has to be called in fixed equidistant times. You must estimate how many times the
	interrupt will occur within this known time slice.

	\param *pIRQ Pointer to the interrupt test structure.
	\param lowerBound Estimated minimum count of occured interrupts. (Don't take respect to the individualValue).
	\param upperBound Estimated maximum count of occured interrupts. (Don't take respect to the individualValue).
	\param individualValue The internally used individual up-counting value.

	\attention Ensure to place the function call #IEC60730_InterruptOcurred into the corresponding
	interrupt service handler!
*/
void IEC60730_InitInterruptTest(type_InterruptTest *pIRQ, uint32_t lowerBound, uint32_t upperBound, uint32_t individualValue)
{
    if(pIRQ != 0)
    {
        pIRQ->Count = 0;

        pIRQ->lower = lowerBound;
        pIRQ->upper = upperBound;

        pIRQ->individualValue = individualValue;

        pIRQ->CountOverflow = FALSE;
    }
}

/*!
	This function must be called from any interrupt service handler which has to be tested.

	\param *pIRQ Pointer to the interrupt test structure.
*/
void IEC60730_InterruptOcurred(type_InterruptTest *pIRQ)
{
    if(pIRQ != 0)
    {
        uint32_t cnt = pIRQ->Count;

        pIRQ->Count = pIRQ->Count + pIRQ->individualValue;

        if(cnt > pIRQ->Count)
            pIRQ->CountOverflow = TRUE;
    }
}

/*!
	The interrupt check routine must be called once in known equidistant times.

	\attention	Prevent this function to be disturbed by interrupts. You can either call it insde of an interrupt service handler,
	or disable interrupts globally, but this is not recommended.

	\param *pIRQ Pointer to the interrupt test structure.
	\return Tells if passed or failed. See
*/
ErrorStatus IEC60730_InterruptCheck(type_InterruptTest *pIRQ)
{
    ErrorStatus Result = ERROR;

    if(pIRQ != 0)
    {
//		printf("ADC CH1 Volt is %04d mV\r\n", pIRQ->individualValue);
//		printf("Interrupt count is %02d \r\n", pIRQ->Count);
        Result = ifIsInRange(pIRQ->lower*pIRQ->individualValue,pIRQ->Count,pIRQ->upper*pIRQ->individualValue) ?
                 (SUCCESS) :
                 (ERROR);

        if(pIRQ->CountOverflow == TRUE)
        {
            pIRQ->CountOverflow = FALSE;
            Result = ERROR;
        }

        pIRQ->Count = 0;
    }

    return Result;
}
