#include "common.h"
#include "Math.h"

void COM_vCheckOverThresholdLimit(const TOverThresholdParm *ptParaPtr, TOverThresholdData *ptDataPtr)
{
	if(ptDataPtr->s16Input >= ptParaPtr->s16ThresholdValue)
	{
		ptDataPtr->u16Count ++;
		if(ptDataPtr->u16Count >= ptParaPtr->u16DelayCnt)
		{
			ptDataPtr->u16Count = ptParaPtr->u16DelayCnt;
			ptDataPtr->bStatus = true;
		}
		ptDataPtr->s16MaxValue = QMATH_max(ptDataPtr->s16Input, ptDataPtr->s16MaxValue);
	}
	else if (ptDataPtr->s16Input < ptParaPtr->s16ConstBackThresholdValue)
	{
		ptDataPtr->u16Count    = 0;
		ptDataPtr->bStatus     = false;
		ptDataPtr->s16MaxValue = 0;
	}
	else
	{
		// do nothing
	}
}

