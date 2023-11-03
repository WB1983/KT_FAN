////////////////////////////////////////////////////////////////////////////////
/// @file    main.c
/// @author  NJ ClassB TEAM
/// @brief
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

//#include "delay.h"
#include "main.h"
#include "adc_classb.h"
#include "digitalio_classb.h"
#include "beeper.h"
#include "key_classb.h"
#include "led_classb.h"
#include "IEC60730_B_param.h"
#include "IEC60730_B_var.h"
#include "system_mm32spin0280.h"
#include "Interrupts_classb.h"
#include "IEC60730_B_lib.h"

#include "drv_inc.h"
#include "board.h"
#include "user_function.h"
#include "StateMachine.h"
#include "Modbus.h"
#include "TimeSlice.h"
#include "Compile.h"
#include "FOC_PMSM.h"
#include "VariantResistor.h"
#include "SpeedControl.h"

#if(FS_MOTOR_FS_MONITOR == OPTION_ACTIVE)
#include "fsHandle.h"
#endif
#include "fsClock.h"
#include "fsMonitor.h"
#include "ErrorHandle.h"

//u32 temp1;
void IEC60730_InitRunTimeChecks(void);
void IEC60730_DoRunTimeChecks(void);
void SystemCoreClockUpdate (void);
void CBUart_Init(u32 baudrate);
extern void WWDG_Init(unsigned char ucTcnt,unsigned char ucWcnt);
extern void Iwdg_Init(unsigned short int IWDG_Prescaler, unsigned short int Reload);

Motor_TypeDef Motor_1st;
int16_t M1FaultID, M1FaultID_Record;

/*void PA0_Config()
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    GPIO_StructInit(&GPIO_InitStruct);
    RCC_GPIO_ClockCmd(GPIOA, ENABLE);


    GPIO_InitStruct.GPIO_Pin  =  GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_SetBits(GPIOA,GPIO_Pin_0);

}*/
////////////////////////////////////////////////////////////////////////////////

void WDG_Init()
{
#ifdef USE_INDEPENDENT_WDOG
    /* Initialize IWDG for run time if applicable */
#ifdef IWDG_FEATURES_BY_WINDOW_OPTION
#endif /* IWDG_FEATURES_BY_WINDOW_OPTION */
    Iwdg_Init(IWDG_Prescaler_4, 400);

#endif /* USE_INDEPENDENT_WDOG */

#ifdef USE_WINDOW_WDOG
    /* Initialize WWDG for run time if applicable */
    /* Setup period - to be adjusted in according w/ appli need */
    WWDG_Init(0xFF,0x7F);
#endif /* USE_WINDOW_WDOG */
}

////////////////////////////////////////////////////////////////////////////////
void WDG_Refresh()
{
    /*----------------------------------------------------------------------*/
    /*---------------- Check Safety routines Control flow  -----------------*/
    /*------------- Refresh Window and independent watchdogs ---------------*/
    /*----------------------------------------------------------------------*/

#ifdef USE_WINDOW_WDOG
    /* Update WWDG counter */
    WWDG_SetCounter(0xFF);
    //printf(" feed watch dog\n\r");
#endif /* USE_WINDOW_WDOG */

    /* Reload IWDG counter */
#ifdef USE_INDEPENDENT_WDOG
    IWDG_ReloadCounter();
#endif /* USE_INDEPENDENT_WDOG */
}
////////////////////////////////////////////////////////////////////////////////
type_InterruptTest systick;
////////////////////////////////////////////////////////////////////////////////
int main(void)
{
#ifdef ClassB_VERBOSE_POR
    CBUart_Init(9600);
#endif
    //before entering main(),execute
    // 1: startup_mm32spin0280_iar.s
    // 2: system_mm32spin0280.C                 //sysclk configure & class b uart configure
    //Class B Self Testing Code at Power on ---------------------------------------------------------------- don't modify (begin)
    SystemCoreClockUpdate(); //change to Custom
#ifdef ClassB_VERBOSE_POR
    printf("run main....\r\n");
#endif
    WDG_Init();
    IEC60730_InitRunTimeChecks();
    //PA0_Config();
#ifdef ClassB_VERBOSE_POR
    printf("init run system....\r\n");
#endif
    //Class B Self Testing Code at Power on ---------------------------------------------------------------- don't modify (end)
    FCK_vInitSafeTime();
    //user init code----------------------------------------------(begin)
	 /* Initialize  motor control parameters */
    Init_Parameter(&Motor_1st);
	 /* Initialize all GPIO */
    Bsp_Gpio_Init();
	 /* Initialize all configured peripherals */
    Peripheral_Init();
	 /* Initialize interrupts */
    Interrupt_Init();

   	/* Application initialization */
	  MOS_vModbusInit();
		/* Infinite loop */
    //user init code----------------------------------------------(end)

    while(1)
    {
        //Class B Self Testing code at running ------------------------------------------------------------- don't modify (begin)
        IEC60730_DoRunTimeChecks();
        //Class B Self Testing code at running ------------------------------------------------------------- don't modify (end)

        WDG_Refresh();

        //user code----------------------------------------------(begin)
		MOS_vModBusHandler();
		
		TSE_vSliceTaskHandle();
		
		#if(FS_MOTOR_FS_MONITOR == OPTION_ACTIVE)
		FHE_vFunctionSafetyCheck();
        FSM_ucHandleTask();
		#endif
		
		#if(DBUG_VOFA_EN == OPTION_ACTIVE)
		//COM_vHandleTask();
		#endif
		
		SCL_vSpeedControlHandle();
		
		//VRR_vAdaptSpdControl();
		
		#if(ERROR_HANDLE_EN == OPTION_ACTIVE)
		EHE_vErrorHandleTask();
		#endif
        //user code----------------------------------------------(end)
    }
}

