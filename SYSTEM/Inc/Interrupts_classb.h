
////////////////////////////////////////////////////////////////////////////////
/// @file    Interrupts.h
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

#ifndef __INTERRUPTS_CLASSB_H
#define __INTERRUPTS_CLASSB_H

#include <stdint.h>
#include <stdbool.h>
#include "types.h"


/* bool */
#ifndef FALSE
#define FALSE    ((bool_t)0)                       
#endif

#ifndef TRUE
#define TRUE     ((bool_t)1)                        
#endif

#define least				8
#define middle			10
#define most				12

#define CUT(a,b,c)				(MAX(a,MIN(b,c)))
#define ifIsInRange(a,b,c)		((CUT(a,b,c) == b)?TRUE:FALSE)
/*! \brief Structure for interrupt testing.

	For each interrupt, which has to be tested, there has to be one instance of this structure.
	To initialise the struct, you have to use the initialisation function
	\a IEC60730_InitInterruptTest(type_InterruptTest *pIRQ, UINT32 lowerBound, UINT32 upperBound, UINT32 individualValue).
*/
typedef struct tag_IntTest
{
	// general
	uint32_t Count;			/*!< The counter Variable*/

	uint32_t lower;			/*!< The estimated minimum count of interrupt occurencies*/
	uint32_t upper;			/*!< The estimated maximum count of interrupt occurencies*/
	
	uint32_t individualValue;	/*!< The individual up-counting value*/

	BOOL CountOverflow;

	BOOL cyclic;

	//injected Interrupt testing
	uint32_t	minTime;			/*!< The time count that has to be waited, before the check is done */
} type_InterruptTest;

/*! \brief Initialises an interrupt test. */
void IEC60730_InitInterruptTest(type_InterruptTest *pIRQ, uint32_t lowerBound, uint32_t upperBound, uint32_t individualValue);

/*! \brief To be called inside the interrupt. */
void IEC60730_InterruptOcurred(type_InterruptTest *pIRQ);
/*! \brief Checks the interrupt. */
ErrorStatus IEC60730_InterruptCheck(type_InterruptTest *pIRQ);

#endif
