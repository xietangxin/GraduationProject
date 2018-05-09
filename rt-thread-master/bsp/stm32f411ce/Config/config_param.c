#include <stdbool.h>
#include <string.h>

#include "math.h"
#include "config_param.h"
#include "drv_flash.h"

#include <rtthread.h>

/* 配置参数驱动代码	*/



/* 11 表示V1.1 */
#define VERSION 11	

configParam_t configParam;

static configParam_t configParamDefault=
{
	.version = VERSION,		/*软件版本号*/

	.pidAngle=	/*角度PID*/
	{	
		.roll=
		{
			.kp=5.0,
			.ki=0.0,
			.kd=0.0,
		},
		.pitch=
		{
			.kp=5.0,
			.ki=0.0,
			.kd=0.0,
		},
		.yaw=
		{
			.kp=10.0,
			.ki=0.0,
			.kd=0.35,
		},
	},	
	.pidRate=	/*角速度PID*/
	{	
		.roll=
		{
			.kp=320.0,
			.ki=0.0,
			.kd=5.0,
		},
		.pitch=
		{
			.kp=320.0,
			.ki=0.0,
			.kd=5.0,
		},
		.yaw=
		{
			.kp=120.0,
			.ki=18.5,
			.kd=0.0,
		},
	},	
	.pidPos=	/*位置PID*/
	{	
		.vx=
		{
			.kp=0.0,
			.ki=0.0,
			.kd=0.0,
		},
		.vy=
		{
			.kp=0.0,
			.ki=0.0,
			.kd=0.0,
		},
		.vz=
		{
			.kp=21.0,
			.ki=0.0,
			.kd=60.0,
		},
	},
	.thrustBase=36000,	/*定高油门基础值*/
};

static uint32_t lenth = 0;
static bool isInit = false;
static bool isConfigParamOK = false;


static uint8_t configParamCksum(configParam_t* data)
{
	int i;
	uint8_t cksum=0;	
	uint8_t* c = (uint8_t*)data;  	
	size_t len=sizeof(configParam_t);

	for (i=0; i<len; i++)
		cksum += *(c++);
	cksum-=data->cksum;
	
	return cksum;
}

/* 参数配置初始化 */
void configParamInit(void)	
{
	if(isInit) return;
	
	lenth=sizeof(configParam);
	lenth=lenth/4+(lenth%4 ? 1:0);

	STMFLASH_Read(CONFIG_PARAM_ADDR, (uint32_t *)&configParam, lenth);
	
	if(configParam.version == VERSION)	/*版本正确*/
	{
		if(configParamCksum(&configParam) == configParam.cksum)	/*校验正确*/
		{
			rt_kprintf("Version V%1.1f check [OK]\r\n", configParam.version / 10.0f);
			isConfigParamOK = true;
		} else
		{
			rt_kprintf("Version check [FAIL]\r\n");
			isConfigParamOK = false;
		}
	}	
	else	/*版本更新*/
	{
		isConfigParamOK = false;
	}
	
	if(isConfigParamOK == false)	/*配置参数错误，写入默认参数*/
	{
		memcpy((uint8_t *)&configParam, (uint8_t *)&configParamDefault, sizeof(configParam));
		configParam.cksum = configParamCksum(&configParam);				/*计算校验值*/
		STMFLASH_Write(CONFIG_PARAM_ADDR,(uint32_t *)&configParam, lenth);	/*写入stm32 flash*/
		isConfigParamOK=true;
	}	
	
	
	// @todo 不知道这个信号量用来干什么的
	// 在降落之后释放信号量
	// xSemaphore = xSemaphoreCreateBinary();
	
	isInit=true;
}

void configParamTask(void* param)
{
	uint8_t cksum = 0;
	
	while(1) 
	{	
	//	xSemaphoreTake(xSemaphore, portMAX_DELAY);
		cksum = configParamCksum(&configParam);		/*数据校验*/
		
		if(configParam.cksum != cksum)	
		{
			configParam.cksum = cksum;	/*数据校验*/
			watchdogInit(1000);			/*擦除时间比较长，看门狗时间设置大一些*/					
			STMFLASH_Write(CONFIG_PARAM_ADDR,(uint32_t *)&configParam, lenth);	/*写入stm32 flash*/
			watchdogInit(WATCHDOG_RESET_MS);		/*重新设置看门狗*/
		}						
	}
}

bool configParamTest(void)
{
	return isInit;
}

void configParamGiveSemaphore(void)
{
	//xSemaphoreGive(xSemaphore);		
}

void resetConfigParamPID(void)
{
	configParam.pidAngle = configParamDefault.pidAngle;
	configParam.pidRate = configParamDefault.pidRate;
	configParam.pidPos = configParamDefault.pidPos;
}
