#ifndef __ATTITUDE_PID_H
#define __ATTITUDE_PID_H

#include <stdbool.h>
#include "commander.h"

/*  姿态PID控制代码	 */

#define ATTITUDE_UPDATE_RATE 	RATE_500_HZ
#define ATTITUDE_UPDATE_DT 		(1.0f / ATTITUDE_UPDATE_RATE)

void attitudeControlInit(void);
bool attitudeControlTest(void);

void attitudeRatePID(Axis3f *actualRate,attitude_t *desiredRate,control_t *output);	/* 角速度环PID */
void attitudeAnglePID(attitude_t *actualAngle,attitude_t *desiredAngle,attitude_t *outDesiredRate);	/* 角度环PID */
void attitudeResetAllPID(void);		/*复位PID*/
void attitudePIDwriteToConfigParam(void);

#endif /* __ATTITUDE_PID_H */
