#include <stdbool.h>
#include "pid.h"
#include "sensors.h"
#include "attitude_pid.h"

/* 姿态PID控制代码 */


PidObject pidAngleRoll;
PidObject pidAnglePitch;
PidObject pidAngleYaw;
PidObject pidRateRoll;
PidObject pidRatePitch;
PidObject pidRateYaw;

static inline int16_t limitPid(float in)
{
	if (in > INT16_MAX)
		return INT16_MAX;
	else if (in < -INT16_MAX)
		return -INT16_MAX;
	else
		return (int16_t)in;
}

void attitudeControlInit()
{
	pidInit(&pidAngleRoll, 0, configParam.pidAngle.roll, SENSOR9_UPDATE_DT);	/*roll  角度PID初始化*/
	pidInit(&pidAnglePitch, 0, configParam.pidAngle.pitch, SENSOR9_UPDATE_DT);	/*pitch 角度PID初始化*/
	pidInit(&pidAngleYaw, 0, configParam.pidAngle.yaw, SENSOR9_UPDATE_DT);		/*yaw   角度PID初始化*/
	pidSetIntegralLimit(&pidAngleRoll, PID_ANGLE_ROLL_INTEGRATION_LIMIT);	/*roll  角度积分限幅设置*/
	pidSetIntegralLimit(&pidAnglePitch, PID_ANGLE_PITCH_INTEGRATION_LIMIT);	/*pitch 角度积分限幅设置*/
	pidSetIntegralLimit(&pidAngleYaw, PID_ANGLE_YAW_INTEGRATION_LIMIT);		/*yaw   角度积分限幅设置*/
	
	pidInit(&pidRateRoll, 0, configParam.pidRate.roll, SENSOR9_UPDATE_DT);		/*roll  角速度PID初始化*/
	pidInit(&pidRatePitch, 0, configParam.pidRate.pitch, SENSOR9_UPDATE_DT);	/*pitch 角速度PID初始化*/
	pidInit(&pidRateYaw, 0, configParam.pidRate.yaw, SENSOR9_UPDATE_DT);		/*yaw   角速度PID初始化*/
	pidSetIntegralLimit(&pidRateRoll, PID_RATE_ROLL_INTEGRATION_LIMIT);		/*roll  角速度积分限幅设置*/
	pidSetIntegralLimit(&pidRatePitch, PID_RATE_PITCH_INTEGRATION_LIMIT);	/*pitch 角速度积分限幅设置*/
	pidSetIntegralLimit(&pidRateYaw, PID_RATE_YAW_INTEGRATION_LIMIT);		/*yaw   角速度积分限幅设置*/
}

bool attitudeControlTest()
{
	return true;
}

/* 角速度环PID */
void attitudeRatePID(Axis3f *actualRate,attitude_t *desiredRate,control_t *output)	
{
	output->roll = limitPid(pidUpdate(&pidRateRoll, desiredRate->roll - actualRate->x));
	output->pitch = limitPid(pidUpdate(&pidRatePitch, desiredRate->pitch - actualRate->y));
	output->yaw = limitPid(pidUpdate(&pidRateYaw, desiredRate->yaw - actualRate->z));
}

/* 角度环PID */
void attitudeAnglePID(attitude_t *actualAngle,attitude_t *desiredAngle,attitude_t *outDesiredRate)	
{
	outDesiredRate->roll = pidUpdate(&pidAngleRoll, desiredAngle->roll - actualAngle->roll);
	outDesiredRate->pitch = pidUpdate(&pidAnglePitch, desiredAngle->pitch - actualAngle->pitch);

	float yawError = desiredAngle->yaw - actualAngle->yaw ;
	if (yawError > 180.0f) 
		yawError -= 360.0f;
	else if (yawError < -180.0) 
		yawError += 360.0f;
	outDesiredRate->yaw = pidUpdate(&pidAngleYaw, yawError);
}

/* 复位PID */
void attitudeResetAllPID(void)	
{
	pidReset(&pidAngleRoll);
	pidReset(&pidAnglePitch);
	pidReset(&pidAngleYaw);
	pidReset(&pidRateRoll);
	pidReset(&pidRatePitch);
	pidReset(&pidRateYaw);
}

/* 将姿态PID写入到配置文件中 */
void attitudePIDwriteToConfigParam(void)
{
	configParam.pidAngle.roll.kp = pidAngleRoll.kp;
	configParam.pidAngle.roll.ki = pidAngleRoll.ki;
	configParam.pidAngle.roll.kd = pidAngleRoll.kd;
	
	configParam.pidAngle.pitch.kp = pidAnglePitch.kp;
	configParam.pidAngle.pitch.ki = pidAnglePitch.ki;
	configParam.pidAngle.pitch.kd = pidAnglePitch.kd;
	
	configParam.pidAngle.yaw.kp = pidAngleYaw.kp;
	configParam.pidAngle.yaw.ki = pidAngleYaw.ki;
	configParam.pidAngle.yaw.kd = pidAngleYaw.kd;
	
	configParam.pidRate.roll.kp = pidRateRoll.kp;
	configParam.pidRate.roll.ki = pidRateRoll.ki;
	configParam.pidRate.roll.kd = pidRateRoll.kd;
	
	configParam.pidRate.pitch.kp = pidRatePitch.kp;
	configParam.pidRate.pitch.ki = pidRatePitch.ki;
	configParam.pidRate.pitch.kd = pidRatePitch.kd;
	
	configParam.pidRate.yaw.kp = pidRateYaw.kp;
	configParam.pidRate.yaw.ki = pidRateYaw.ki;
	configParam.pidRate.yaw.kd = pidRateYaw.kd;
}

