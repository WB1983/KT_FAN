#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "foc_pmsm.h"

#define VRR_REF_VOL 5000
#define VRR_MAX_VOL 2000        //2000MV
#define VRR_MAX_VOL_DIG         13107   //(VRR_MAX_VOL/VRR_REF_VOL)*4096*8
#define VRR_MAX_SPD_VOL_DIG     9175    //(1400/5000)*32768
#define VRR_VOL_TO_SPD_FACTOR   7       // VRR_MAX_SPD_VOL_DIG*10/VRR_MAX_VOL_DIG  

static uint32_t VRR_u32SpdCmd = 0;

void VRR_vAdaptSpdControl(void)
{
	uint16_t u16ResistorVoltage = 0;
	u16ResistorVoltage = MOT_u16GetVariantResistorDigitalValue();
	VRR_u32SpdCmd = u16ResistorVoltage*VRR_VOL_TO_SPD_FACTOR/10;
	if(VRR_u32SpdCmd > VRR_MAX_SPD_VOL_DIG)
		{
		   Motor_1st.USER.s16VspCmd = VRR_MAX_SPD_VOL_DIG;
		}
	else
		{
			Motor_1st.USER.s16VspCmd = VRR_u32SpdCmd;
		}

	if(VRR_u32SpdCmd < 0)
		{
		
			Motor_1st.USER.s16VspCmd = 0;

		}

}





