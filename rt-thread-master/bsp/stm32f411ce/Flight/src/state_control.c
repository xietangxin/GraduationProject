#include "state_control.h"
#include "stabilizer.h"
#include "sensfusion6.h"
#include "attitude_pid.h"
#include "position_pid.h"
#include "config_param.h"

/* 四轴姿态控制代码 */

static float actualThrust;
static attitude_t attitudeDesired;
static attitude_t rateDesired;


void stateControlInit(void)
{
	/* 初始化姿态PID */
	attitudeControlInit();
	/* 初始化位置PID */
	positionControlInit();	
}

bool stateControlTest(void)
{
	bool pass = true;
	pass &= attitudeControlTest();
	return pass;
} 

void stateControl(control_t *control, sensorData_t *sensors, state_t *state, setpoint_t *setpoint, const uint32_t tick)
{
	static uint16_t cnt = 0;
	
	if (RATE_DO_EXECUTE(POS_UPDATE_RATE, tick))		/* 250Hz */ 
	{
		/* 定高PID(速度模式) */
		altholdPID(&actualThrust, state, setpoint);		
	}

	if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick))	/* 500Hz */  
	{
		if(setpoint->isAltHold == false) 
		{
			actualThrust = setpoint->thrust;
		}
		if(control->flipDir == CENTER)
		{
			/* 期望YAW 速率模式 */
			attitudeDesired.yaw -= setpoint->attitude.yaw/ATTITUDE_UPDATE_RATE;	
			while(attitudeDesired.yaw > 180.0f) 
				attitudeDesired.yaw -= 360.0f;
			while(attitudeDesired.yaw < -180.0f) 
				attitudeDesired.yaw += 360.0f;
		}
		/* 期望ROLL 角度(自稳)模式 */
		attitudeDesired.roll = setpoint->attitude.roll;
		/* 期望PITCH 角度(自稳)模式 */		
		attitudeDesired.pitch = setpoint->attitude.pitch;	
		/* 角度(外环)PID */
		attitudeAnglePID(&state->attitude, &attitudeDesired, &rateDesired);	
		
		if(control->flipDir != CENTER)	/*空翻的时候只使用内环PID*/
		{
			rateDesired.pitch = setpoint->attitude.pitch;
			rateDesired.roll = setpoint->attitude.roll;
		}
		/* 角速度(内环)PID */
		attitudeRatePID(&sensors->gyro, &rateDesired, control);				

	#ifdef ENABLE_PID_TUNING	/* 使能PID调节 yaw值不更新 */
		control->yaw = 0;	
	#endif
	}

	control->thrust = (uint16_t)actualThrust;	

	if (control->thrust == 0)
	{			
		control->thrust = 0;
		control->roll = 0;
		control->pitch = 0;
		control->yaw = 0;
		
		/* 复位姿态PID */
		attitudeResetAllPID();	
		/* 复位位置PID */
		positionResetAllPID();
		/*复位计算的期望yaw值*/		
		attitudeDesired.yaw = state->attitude.yaw;		
		
		float velZ = state->velocity.z;
		if(velZ<0.001f && velZ>-0.001f && cnt++>1000)	/*降落完成后保存参数*/
		{
			cnt = 0;
			configParamGiveSemaphore();
		}
	}

}


