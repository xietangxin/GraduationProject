#include "watchdog.h"


//bool watchdogTest(void)
//{
//	bool wasNormalStart = true;

//	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST)) 
//	{
//		RCC_ClearFlag();
//		wasNormalStart = false;
//		rt_kprintf("The system resumed after watchdog timeout [WARNING]\n");
//		printAssertSnapshotData();
//	}
//	return wasNormalStart;
//}


void watchdogInit(u16 xms)
{
	LWDG_HandleTypeDef IWDG_Handler;
	IWDG_Handler.Instance = IWDG;
	IWDG_Handler.Init.Prescaler = IWDG_PRESCALER_32;
	IWDG_Handler.Init.Reload = (uint32_t)(xms * 1.47);
	HAL_IWDG_Init(&IWDG_Handler);
	HAL_IWDG_Refresh(&IWDG_Handler);
	HAL_IWDG_Start(&IWDG_Handler);
//	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
//	IWDG_SetPrescaler(IWDG_Prescaler_32);

//	/* 47000/32Hz => 1.47  1ms*/
//	IWDG_SetReload((u16)(1.47*xms));

//	watchdogReset();
//	IWDG_Enable();
}
