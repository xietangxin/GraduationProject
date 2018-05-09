#include "state_control.h"
#include "stabilizer.h"
#include "sensfusion6.h"
#include "attitude_pid.h"
#include "position_pid.h"
#include "config_param.h"

/* ������̬���ƴ��� */

static float actualThrust;
static attitude_t attitudeDesired;
static attitude_t rateDesired;


void stateControlInit(void)
{
	/* ��ʼ����̬PID */
	attitudeControlInit();
	/* ��ʼ��λ��PID */
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
		/* ����PID(�ٶ�ģʽ) */
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
			/* ����YAW ����ģʽ */
			attitudeDesired.yaw -= setpoint->attitude.yaw/ATTITUDE_UPDATE_RATE;	
			while(attitudeDesired.yaw > 180.0f) 
				attitudeDesired.yaw -= 360.0f;
			while(attitudeDesired.yaw < -180.0f) 
				attitudeDesired.yaw += 360.0f;
		}
		/* ����ROLL �Ƕ�(����)ģʽ */
		attitudeDesired.roll = setpoint->attitude.roll;
		/* ����PITCH �Ƕ�(����)ģʽ */		
		attitudeDesired.pitch = setpoint->attitude.pitch;	
		/* �Ƕ�(�⻷)PID */
		attitudeAnglePID(&state->attitude, &attitudeDesired, &rateDesired);	
		
		if(control->flipDir != CENTER)	/*�շ���ʱ��ֻʹ���ڻ�PID*/
		{
			rateDesired.pitch = setpoint->attitude.pitch;
			rateDesired.roll = setpoint->attitude.roll;
		}
		/* ���ٶ�(�ڻ�)PID */
		attitudeRatePID(&sensors->gyro, &rateDesired, control);				

	#ifdef ENABLE_PID_TUNING	/* ʹ��PID���� yawֵ������ */
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
		
		/* ��λ��̬PID */
		attitudeResetAllPID();	
		/* ��λλ��PID */
		positionResetAllPID();
		/*��λ���������yawֵ*/		
		attitudeDesired.yaw = state->attitude.yaw;		
		
		float velZ = state->velocity.z;
		if(velZ<0.001f && velZ>-0.001f && cnt++>1000)	/*������ɺ󱣴����*/
		{
			cnt = 0;
			configParamGiveSemaphore();
		}
	}

}


