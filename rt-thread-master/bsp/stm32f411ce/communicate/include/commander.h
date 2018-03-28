#ifndef __COMMANDER_H
#define __COMMANDER_H

#include "atkp.h"
#include "stabilizer_types.h"

/* 获取遥控数据驱动代码 */

#define COMMANDER_WDT_TIMEOUT_STABILIZE  500
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   1000

typedef struct
{
	uint8_t altHoldMode	: 1;	/*bit0 模式 1=定高 0=手动*/
	uint8_t keyFlight 	: 1;	/*bit1 一键起飞*/
	uint8_t keyLand 		: 1;	/*bit2 一键降落*/
	uint8_t emerStop 		: 1;	/*bit3 紧急停机*/
	uint8_t flightMode 	: 1;	/*bit4 飞行模式 1=无头 0=有头*/
	uint8_t reserved		: 3;	/*bit5~7 保留*/
}commanderBits_t;

/*控制数据结构体*/
typedef __packed struct
{
	float roll;       // deg
	float pitch;      // deg
	float yaw;        // deg
	float trimPitch;
	float trimRoll;
	uint16_t thrust;
} ctrlVal_t;

/*数据缓存结构体*/
typedef struct
{
	ctrlVal_t  tarVal[2];
	bool activeSide;
	uint32_t timestamp; 		/* FreeRTOS 时钟节拍*/
} ctrlValCache_t;

typedef enum
{
	RATE    = 0,
	ANGLE   = 1,
} RPYType;

typedef enum
{
	XMODE     = 0, /*X模式*/
	CAREFREE  = 1, /*无头模式*/
} YawModeType;

typedef enum
{
	ATK_REMOTER = 0,
	WIFI		= 1,
}ctrlSrc_e;
	
void commanderInit(void);
bool commanderTest(void);
void flightCtrldataCache(ctrlSrc_e ctrlSrc, ctrlVal_t pk);
void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state);
void flyerAutoLand(setpoint_t *setpoint,const state_t *state);

void getCommanderTrim(float* pitch, float* roll);

void setCommanderAltholdMode(bool set);
bool getCommanderAltholdMode(void);

void setCommanderKeyFlight(bool set);
bool getCommanderKeyFlight(void);

void setCommanderKeyland(bool set);
bool getCommanderKeyland(void);

void setCommanderFlightmode(bool set);
void setCommanderEmerStop(bool set);

#endif /* __COMMANDER_H */
