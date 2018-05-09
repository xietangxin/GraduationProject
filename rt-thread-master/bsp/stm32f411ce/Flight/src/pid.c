#include "pid.h"

/* PID驱动代码 */

/* 初始化PidObject */
void pidInit(PidObject* pid, const float desired, const pidInit_t pidParam, const float dt)
{
	pid->error     = 0;
	pid->prevError = 0;
	pid->integ     = 0;
	pid->deriv     = 0;
	pid->desired = desired;
	pid->kp = pidParam.kp;
	pid->ki = pidParam.ki;
	pid->kd = pidParam.kd;
	pid->iLimit    = DEFAULT_PID_INTEGRATION_LIMIT;
	pid->iLimitLow = -DEFAULT_PID_INTEGRATION_LIMIT;
	pid->dt        = dt;
}


float pidUpdate(PidObject* pid, const float error)
{
	float output;

	pid->error = error;   

	pid->integ += pid->error * pid->dt;
	if (pid->integ > pid->iLimit)
	{
		pid->integ = pid->iLimit;
	}
	else if (pid->integ < pid->iLimitLow)
	{
		pid->integ = pid->iLimitLow;
	}

	pid->deriv = (pid->error - pid->prevError) / pid->dt;

	pid->outP = pid->kp * pid->error;
	pid->outI = pid->ki * pid->integ;
	pid->outD = pid->kd * pid->deriv;

	output = pid->outP + pid->outI + pid->outD;

	pid->prevError = pid->error;

	return output;
}

/* 设置PID积分限幅 */
void pidSetIntegralLimit(PidObject* pid, const float limit) 
{
    pid->iLimit = limit;
}

/* 设置PID最低积分限幅 */
void pidSetIntegralLimitLow(PidObject* pid, const float limitLow) 
{
    pid->iLimitLow = limitLow;
}

/* 重置PID */
void pidReset(PidObject* pid)
{
	pid->error     = 0;
	pid->prevError = 0;
	pid->integ     = 0;
	pid->deriv     = 0;
}

void pidSetError(PidObject* pid, const float error)
{
	pid->error = error;
}

void pidSetDesired(PidObject* pid, const float desired)
{
	pid->desired = desired;
}

float pidGetDesired(PidObject* pid)
{
	return pid->desired;
}

bool pidIsActive(PidObject* pid)
{
	bool isActive = true;

	if (pid->kp < 0.0001f && pid->ki < 0.0001f && pid->kd < 0.0001f)
	{
		isActive = false;
	}

	return isActive;
}

void pidSetKp(PidObject* pid, const float kp)
{
	pid->kp = kp;
}

void pidSetKi(PidObject* pid, const float ki)
{
	pid->ki = ki;
}

void pidSetKd(PidObject* pid, const float kd)
{
	pid->kd = kd;
}

void pidSetDt(PidObject* pid, const float dt) 
{
    pid->dt = dt;
}
