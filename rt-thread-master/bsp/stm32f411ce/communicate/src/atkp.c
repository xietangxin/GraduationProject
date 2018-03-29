#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "atkp.h"
#include "radiolink.h"
#include "usblink.h"
#include "usbd_usr.h"
#include "stabilizer.h"
#include "motors.h"
#include "commander.h"
#include "flip.h"
#include "pm.h"
#include "pid.h"
#include "attitude_pid.h"
#include "sensors.h"
#include "position_pid.h"
#include "config_param.h"
#include "power_control.h"
#include "remoter_ctrl.h"

///*FreeRTOS相关头文件*/
//#include "FreeRTOS.h"
//#include "task.h"
//#include "semphr.h"
//#include "queue.h"

/* 无线通信驱动代码	
 * 说明：此文件程序基于于匿名科创地面站V4.34通信协议下位机示例代码修改。
********************************************************************************/

//数据拆分宏定义
#define  BYTE0(dwTemp)       ( *( (uint8_t *)(&dwTemp)	)  )
#define  BYTE1(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 1) )
#define  BYTE2(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 2) )
#define  BYTE3(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 3) )

//数据返回周期时间（单位ms）
#define  PERIOD_STATUS		30
#define  PERIOD_SENSOR 		10
#define  PERIOD_RCDATA 		40
#define  PERIOD_POWER 		100
#define  PERIOD_MOTOR			40
#define  PERIOD_SENSOR2 	40
#define  PERIOD_SPEED   	50

#define ATKP_RX_QUEUE_SIZE 	10 /*ATKP包接收队列消息个数*/

typedef struct  
{
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
	uint16_t thrust;
}joystickFlyui16_t;

bool isConnect = false;
bool isInit = false;
bool flyable = false;
static joystickFlyui16_t rcdata;

//static xQueueHandle rxQueue;
static struct rt_messagequeue rxMq;
static atkp_t mqBuff[ATKP_RX_QUEUE_SIZE];

extern PidObject pidAngleRoll;
extern PidObject pidAnglePitch;
extern PidObject pidAngleYaw;
extern PidObject pidRateRoll;
extern PidObject pidRatePitch;
extern PidObject pidRateYaw;

static void atkpSendPacket(atkp_t *p)
{
	radiolinkSendPacket(p);  // 无线发送数据包  @todo
	
	if(getusbConnectState()) // 测试四轴USB是否和上位机连接
	{
		usblinkSendPacket(p); // 连接到上位机 发送数据
	}	
}

/***************************发送至匿名上位机指令******************************/
static void sendStatus(float roll, float pitch, float yaw, uint32_t alt, uint8_t fly_model, uint8_t armed)
{
	uint8_t _cnt=0;
	atkp_t p;
	uint16_t _temp;
	uint32_t _temp2 = alt;
	
	p.msgID = UP_STATUS;
	
	_temp = (int)(roll*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = (int)(pitch*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = (int)(yaw*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	p.data[_cnt++]=BYTE3(_temp2);
	p.data[_cnt++]=BYTE2(_temp2);
	p.data[_cnt++]=BYTE1(_temp2);
	p.data[_cnt++]=BYTE0(_temp2);
	
	p.data[_cnt++] = fly_model;
	p.data[_cnt++] = armed;
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendSenser(uint16_t a_x,uint16_t a_y,uint16_t a_z,uint16_t g_x,uint16_t g_y,uint16_t g_z,uint16_t m_x,uint16_t m_y,uint16_t m_z)
{
	uint8_t _cnt=0;
	atkp_t p;
	uint16_t _temp;
	
	p.msgID = UP_SENSER;

	_temp = a_x;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = 0;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);	
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendRCData(uint16_t thrust,uint16_t yaw,uint16_t roll,uint16_t pitch,uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,uint16_t aux5,uint16_t aux6)
{
	uint8_t _cnt=0;
	atkp_t p;
	
	p.msgID = UP_RCDATA;
	p.data[_cnt++]=BYTE1(thrust);
	p.data[_cnt++]=BYTE0(thrust);
	p.data[_cnt++]=BYTE1(yaw);
	p.data[_cnt++]=BYTE0(yaw);
	p.data[_cnt++]=BYTE1(roll);
	p.data[_cnt++]=BYTE0(roll);
	p.data[_cnt++]=BYTE1(pitch);
	p.data[_cnt++]=BYTE0(pitch);
	p.data[_cnt++]=BYTE1(aux1);
	p.data[_cnt++]=BYTE0(aux1);
	p.data[_cnt++]=BYTE1(aux2);
	p.data[_cnt++]=BYTE0(aux2);
	p.data[_cnt++]=BYTE1(aux3);
	p.data[_cnt++]=BYTE0(aux3);
	p.data[_cnt++]=BYTE1(aux4);
	p.data[_cnt++]=BYTE0(aux4);
	p.data[_cnt++]=BYTE1(aux5);
	p.data[_cnt++]=BYTE0(aux5);
	p.data[_cnt++]=BYTE1(aux6);
	p.data[_cnt++]=BYTE0(aux6);

	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendPower(uint16_t votage, uint16_t current)
{
	uint8_t _cnt=0;
	atkp_t p;
	
	p.msgID = UP_POWER;
	
	p.data[_cnt++]=BYTE1(votage);
	p.data[_cnt++]=BYTE0(votage);
	p.data[_cnt++]=BYTE1(current);
	p.data[_cnt++]=BYTE0(current);
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendMotorPWM(uint16_t m_1, uint16_t m_2, uint16_t m_3, uint16_t m_4, uint16_t m_5, uint16_t m_6, uint16_t m_7, uint16_t m_8)
{
	uint8_t _cnt=0;
	atkp_t p;
	
	p.msgID = UP_MOTOR;
	
	p.data[_cnt++]=BYTE1(m_1);
	p.data[_cnt++]=BYTE0(m_1);
	p.data[_cnt++]=BYTE1(m_2);
	p.data[_cnt++]=BYTE0(m_2);
	p.data[_cnt++]=BYTE1(m_3);
	p.data[_cnt++]=BYTE0(m_3);
	p.data[_cnt++]=BYTE1(m_4);
	p.data[_cnt++]=BYTE0(m_4);
	p.data[_cnt++]=BYTE1(m_5);
	p.data[_cnt++]=BYTE0(m_5);
	p.data[_cnt++]=BYTE1(m_6);
	p.data[_cnt++]=BYTE0(m_6);
	p.data[_cnt++]=BYTE1(m_7);
	p.data[_cnt++]=BYTE0(m_7);
	p.data[_cnt++]=BYTE1(m_8);
	p.data[_cnt++]=BYTE0(m_8);
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendSenser2(uint32_t bar_alt,uint16_t csb_alt)
{
	uint8_t _cnt=0;
	atkp_t p;
	
	p.msgID = UP_SENSER2;
	
	p.data[_cnt++]=BYTE3(bar_alt);
	p.data[_cnt++]=BYTE2(bar_alt);
	p.data[_cnt++]=BYTE1(bar_alt);
	p.data[_cnt++]=BYTE0(bar_alt);
	
	p.data[_cnt++]=BYTE1(csb_alt);
	p.data[_cnt++]=BYTE0(csb_alt);
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendPid(uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	uint8_t _cnt=0;
	atkp_t p;
	uint16_t _temp;
	
	p.msgID = 0x10+group-1;

	_temp = p1_p * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p1_i * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p1_d * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p2_p * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p2_i * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p3_p * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p3_i * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendCheck(uint8_t head, uint8_t check_sum)
{
	atkp_t p;
	
	p.msgID = UP_CHECK;
	p.dataLen = 2;
	p.data[0] = head;
	p.data[1] = check_sum;
	atkpSendPacket(&p);
}

/****************************************************************************/

/*数据周期性发送给上位机，每1ms调用一次*/
static void atkpSendPeriod(void)
{
	static uint16_t count_ms = 1;

	if(!(count_ms % PERIOD_STATUS))
	{
		attitude_t attitude;
		getAttitudeData(&attitude);
		int positionZ = getPositionZ() * 100.f;
		sendStatus(attitude.roll, attitude.pitch, attitude.yaw, positionZ, 0, flyable);				
	}
	if(!(count_ms % PERIOD_SENSOR))
	{
		Axis3i16 acc;
		Axis3i16 gyro;
		Axis3i16 mag;
		getSensorRawData(&acc, &gyro, &mag);
		sendSenser(acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, mag.x, mag.y, mag.z);				
	}
	if(!(count_ms % PERIOD_RCDATA))
	{
		sendRCData(rcdata.thrust, rcdata.yaw, rcdata.roll,
					rcdata.pitch, 0, 0, 0, 0, 0, 0);
	}
	if(!(count_ms % PERIOD_POWER))
	{
		float bat = pmGetBatteryVoltage();
		sendPower(bat*100,500);
	}
	if(!(count_ms % PERIOD_MOTOR))
	{
		uint16_t m1,m2,m3,m4;
		motorPWM_t motorPWM;
		getMotorPWM(&motorPWM);
		m1 = (float)motorPWM.m1/65535*1000;
		m2 = (float)motorPWM.m2/65535*1000;
		m3 = (float)motorPWM.m3/65535*1000;
		m4 = (float)motorPWM.m4/65535*1000;
		sendMotorPWM(m1,m2,m3,m4,0,0,0,0);
	}
	if(!(count_ms % PERIOD_SENSOR2))
	{
		int baro = getBaroData() * 100.f;
		sendSenser2(baro,0);
	}
	if(++count_ms>=65535) 
		count_ms = 1;	
}

static uint8_t atkpCheckSum(atkp_t *packet)
{
	uint8_t sum;
	sum = DOWN_BYTE1;
	sum += DOWN_BYTE2;
	sum += packet->msgID;
	sum += packet->dataLen;
	for(int i=0; i<packet->dataLen; i++)
	{
		sum += packet->data[i];
	}
	return sum;
}

static void atkpReceiveAnl(atkp_t *anlPacket)
{
	if(anlPacket->msgID	== DOWN_COMMAND)
	{
		switch(anlPacket->data[0])
		{
			case D_COMMAND_ACC_CALIB:
				break;
			
			case D_COMMAND_GYRO_CALIB:
				break;
			
			case D_COMMAND_MAG_CALIB:
				break;
			
			case D_COMMAND_BARO_CALIB:
				break;
			
			case D_COMMAND_FLIGHT_LOCK:
				flyable = false;
				break;
			
			case D_COMMAND_FLIGHT_ULOCK:
				flyable = true;
		}
	}			
	else if(anlPacket->msgID == DOWN_ACK)
	{
		if(anlPacket->data[0] == D_ACK_READ_PID)/*读取PID参数*/
		{
			sendPid(1,pidRateRoll.kp, pidRateRoll.ki, pidRateRoll.kd,
					  pidRatePitch.kp, pidRatePitch.ki, pidRatePitch.kd,
					  pidRateYaw.kp, pidRateYaw.ki, pidRateYaw.kd 
				   );
			sendPid(2,pidAngleRoll.kp, pidAngleRoll.ki, pidAngleRoll.kd,
					  pidAnglePitch.kp, pidAnglePitch.ki, pidAnglePitch.kd,
					  pidAngleYaw.kp, pidAngleYaw.ki, pidAngleYaw.kd 
				   );
			float kp, ki, kd;
			getPositionPIDZ(&kp, &ki, &kd);
			sendPid(3,kp,ki,kd,0,0,0,0,0,0);
		}
		if(anlPacket->data[0] == D_ACK_RESET_PARAM)/*恢复默认参数*/
		{
			resetConfigParamPID();
			positionControlInit();
			attitudeControlInit();
			
			sendPid(1,pidRateRoll.kp, pidRateRoll.ki, pidRateRoll.kd,
					  pidRatePitch.kp, pidRatePitch.ki, pidRatePitch.kd,
					  pidRateYaw.kp, pidRateYaw.ki, pidRateYaw.kd 
				   );
			sendPid(2,pidAngleRoll.kp, pidAngleRoll.ki, pidAngleRoll.kd,
					  pidAnglePitch.kp, pidAnglePitch.ki, pidAnglePitch.kd,
					  pidAngleYaw.kp, pidAngleYaw.ki, pidAngleYaw.kd 
				   );
			float kp, ki, kd;
			getPositionPIDZ(&kp, &ki, &kd);
			sendPid(3,kp,ki,kd,0,0,0,0,0,0);
		}
	}
	else if(anlPacket->msgID == DOWN_RCDATA)
	{
		rcdata = *((joystickFlyui16_t*)anlPacket->data);
	}
	else if(anlPacket->msgID == DOWN_POWER)/*nrf51822*/
	{
		pmSyslinkUpdate(anlPacket);
	}
	else if(anlPacket->msgID == DOWN_REMOTER)/*遥控器*/	
	{
		remoterCtrlProcess(anlPacket);
	}
	else if(anlPacket->msgID == DOWN_PID1)
	{
		pidRateRoll.kp  = 0.1*((uint16_t)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		pidRateRoll.ki  = 0.1*((uint16_t)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		pidRateRoll.kd  = 0.1*((uint16_t)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		pidRatePitch.kp = 0.1*((uint16_t)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		pidRatePitch.ki = 0.1*((uint16_t)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		pidRatePitch.kd = 0.1*((uint16_t)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		pidRateYaw.kp   = 0.1*((uint16_t)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		pidRateYaw.ki   = 0.1*((uint16_t)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		pidRateYaw.kd   = 0.1*((uint16_t)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
		uint8_t cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}
	else if(anlPacket->msgID == DOWN_PID2)
	{
		pidAngleRoll.kp  = 0.1*((uint16_t)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		pidAngleRoll.ki  = 0.1*((uint16_t)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		pidAngleRoll.kd  = 0.1*((uint16_t)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		pidAnglePitch.kp = 0.1*((uint16_t)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		pidAnglePitch.ki = 0.1*((uint16_t)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		pidAnglePitch.kd = 0.1*((uint16_t)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		pidAngleYaw.kp   = 0.1*((uint16_t)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		pidAngleYaw.ki   = 0.1*((uint16_t)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		pidAngleYaw.kd   = 0.1*((uint16_t)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
		uint8_t cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}		
	else if(anlPacket->msgID == DOWN_PID3)
	{
		float kp  = 0.1*((uint16_t)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		float ki  = 0.1*((uint16_t)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		float kd  = 0.1*((uint16_t)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		setPositionPIDZ(kp, ki, kd);
		uint8_t cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}
	else if(anlPacket->msgID == DOWN_PID4)
	{
		uint8_t cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}
	else if(anlPacket->msgID == DOWN_PID5)
	{
		uint8_t cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}
	else if(anlPacket->msgID == DOWN_PID6)
	{
//		s16 temp1  = ((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
//		s16 temp2  = ((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
//		s16 temp3  = ((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		uint16_t enable = ((uint16_t)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		uint16_t m1_set = ((uint16_t)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		uint16_t m2_set = ((uint16_t)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		uint16_t m3_set = ((uint16_t)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		uint16_t m4_set = ((uint16_t)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		setMotorPWM(enable,m1_set,m2_set,m3_set,m4_set);
		attitudePIDwriteToConfigParam();
		uint8_t cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}
} 

void atkpTxTask(void *param)
{
	sendMsgACK();
	while(1)
	{
		atkpSendPeriod();
		rt_thread_delay(1);
		//vTaskDelay(1);
	}
}

void atkpRxAnlTask(void *param)
{
	atkp_t p;
	while(1)
	{
		//  @todo  FreeRTOS 相关的代码
		rt_mq_recv(&rxMq, &p, sizeof(p), MAX_DELAY);
		//xQueueReceive(rxQueue, &p, portMAX_DELAY);
		atkpReceiveAnl(&p);
	}
}

void atkpInit(void)
{
	if(isInit) return;
	//  @todo  FreeRTOS 相关的代码
	rt_mq_init(&rxMq, "rxMq", mqBuff, sizeof(atkp_t), sizeof(mqBuff), RT_IPC_FLAG_FIFO);
	//rxQueue = xQueueCreate(ATKP_RX_QUEUE_SIZE, sizeof(atkp_t));
	//ASSERT(rxQueue);
	isInit = true;
}

bool atkpReceivePacketBlocking(atkp_t *p)
{
	RT_ASSERT(p != RT_NULL);
	RT_ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
	return rt_mq_send(&rxMq, p, sizeof(*p));
//	ASSERT(p);
//	ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
//	// @todo FreeRTOS 相关的代码
//	return xQueueSend(rxQueue, p, portMAX_DELAY);	
}
