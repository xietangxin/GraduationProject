#include <stdbool.h>
#include <string.h>

#include "math.h"
#include "config_param.h"
#include "drv_flash.h"

#include <rtthread.h>

/* ���ò�����������	*/



/* 11 ��ʾV1.1 */
#define VERSION 11	

configParam_t configParam;

static configParam_t configParamDefault=
{
	.version = VERSION,		/*����汾��*/

	.pidAngle=	/*�Ƕ�PID*/
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
	.pidRate=	/*���ٶ�PID*/
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
	.pidPos=	/*λ��PID*/
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
	.thrustBase=36000,	/*�������Ż���ֵ*/
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

/* �������ó�ʼ�� */
void configParamInit(void)	
{
	if(isInit) return;
	
	lenth=sizeof(configParam);
	lenth=lenth/4+(lenth%4 ? 1:0);

	STMFLASH_Read(CONFIG_PARAM_ADDR, (uint32_t *)&configParam, lenth);
	
	if(configParam.version == VERSION)	/*�汾��ȷ*/
	{
		if(configParamCksum(&configParam) == configParam.cksum)	/*У����ȷ*/
		{
			rt_kprintf("Version V%1.1f check [OK]\r\n", configParam.version / 10.0f);
			isConfigParamOK = true;
		} else
		{
			rt_kprintf("Version check [FAIL]\r\n");
			isConfigParamOK = false;
		}
	}	
	else	/*�汾����*/
	{
		isConfigParamOK = false;
	}
	
	if(isConfigParamOK == false)	/*���ò�������д��Ĭ�ϲ���*/
	{
		memcpy((uint8_t *)&configParam, (uint8_t *)&configParamDefault, sizeof(configParam));
		configParam.cksum = configParamCksum(&configParam);				/*����У��ֵ*/
		STMFLASH_Write(CONFIG_PARAM_ADDR,(uint32_t *)&configParam, lenth);	/*д��stm32 flash*/
		isConfigParamOK=true;
	}	
	
	
	// @todo ��֪������ź���������ʲô��
	// �ڽ���֮���ͷ��ź���
	// xSemaphore = xSemaphoreCreateBinary();
	
	isInit=true;
}

void configParamTask(void* param)
{
	uint8_t cksum = 0;
	
	while(1) 
	{	
	//	xSemaphoreTake(xSemaphore, portMAX_DELAY);
		cksum = configParamCksum(&configParam);		/*����У��*/
		
		if(configParam.cksum != cksum)	
		{
			configParam.cksum = cksum;	/*����У��*/
			watchdogInit(1000);			/*����ʱ��Ƚϳ������Ź�ʱ�����ô�һЩ*/					
			STMFLASH_Write(CONFIG_PARAM_ADDR,(uint32_t *)&configParam, lenth);	/*д��stm32 flash*/
			watchdogInit(WATCHDOG_RESET_MS);		/*�������ÿ��Ź�*/
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
