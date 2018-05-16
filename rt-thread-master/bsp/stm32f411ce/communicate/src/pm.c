#include <rtthread.h>
#include <string.h>
#include <stdbool.h>


#include "pm.h"
#include "drv_led.h"
#include "commander.h"
#include "radiolink.h"
#include "remoter_ctrl.h"
#include "stabilizer.h"




/* 电源管理驱动代码	*/

#if defined(__CC_ARM) 
	#pragma anon_unions
#endif

typedef __packed struct _PmSyslinkInfo
{
	__packed union
	{
		uint8_t flags;
		__packed struct
		{
			uint8_t pgood  : 1;
			uint8_t chg    : 1;
			uint8_t unused : 6;
		};
	};
	float vBat;
	
} PmSyslinkInfo;

static float    batteryVoltage;
static float    batteryVoltageMin = 6.0;
static float    batteryVoltageMax = 0.0;

static bool isInit;
static bool isLowpower;
static PMStates pmState;
static PmSyslinkInfo pmSyslinkInfo;
static uint32_t batteryLowTimeStamp;

static void pmSetBatteryVoltage(float voltage);


void pmInit(void)
{
	if(isInit) return;

	pmSyslinkInfo.vBat = 3.7f;
	pmSetBatteryVoltage(pmSyslinkInfo.vBat); 
	
	isInit = true;
}

bool pmTest(void)
{
	return isInit;
}

static void pmSetBatteryVoltage(float voltage)	/*设置电池电压最大最小值*/
{
	batteryVoltage = voltage;
	if (batteryVoltageMax < voltage)
	{
		batteryVoltageMax = voltage;
	}
	if (batteryVoltageMin > voltage)
	{
		batteryVoltageMin = voltage;
	}
}


float pmGetBatteryVoltage(void)
{
	return batteryVoltage;
}

void pmSyslinkUpdate(atkp_t *slp)
{
	memcpy(&pmSyslinkInfo, &slp->data[0], sizeof(pmSyslinkInfo));
	pmSetBatteryVoltage(pmSyslinkInfo.vBat);
}


PMStates pmUpdateState()	/* 更新电源状态 */
{
	PMStates state;
	bool isCharging = pmSyslinkInfo.chg;
	bool isPgood = pmSyslinkInfo.pgood;
	uint32_t batteryLowTime;

	batteryLowTime = rt_tick_get() - batteryLowTimeStamp;
	//batteryLowTime = getSysTickCnt() - batteryLowTimeStamp;

	if (isPgood && !isCharging)
	{
		state = charged;
	}else if (isPgood && isCharging)
	{
		state = charging;
	}else if (!isPgood && !isCharging && (batteryLowTime > PM_BAT_LOW_TIMEOUT))
	{
		state = lowPower;
	}else
	{
		state = battery;
	}

	return state;
}

void pmTask(void *param)	/* 电源管理任务 */
{
	PMStates pmStateOld = battery;
	uint32_t tickCount;

	tickCount = rt_tick_get();
	batteryLowTimeStamp = tickCount;

	rt_thread_delay(500);


	while(1)
	{
		rt_thread_delay(100);
		tickCount = rt_tick_get();
		

		if (pmGetBatteryVoltage() > PM_BAT_LOW_VOLTAGE)
		{
			batteryLowTimeStamp = tickCount;
		}

		pmState = pmUpdateState();

		if (pmState != pmStateOld)
		{
			switch (pmState)	/*电源状态切换*/
			{
				case charged:
					rt_kprintf("charged!\r\n");
					break;
				case charging:
					rt_kprintf("charging!\r\n");
					break;
				case lowPower:
					rt_kprintf("lowPower!\r\n");
					break;
				case battery:
					rt_kprintf("battery!\r\n");
					break;
				default:
					break;
			}
			pmStateOld = pmState;
		}
	}
}


bool getIsLowpower(void)
{
	return isLowpower;
}
