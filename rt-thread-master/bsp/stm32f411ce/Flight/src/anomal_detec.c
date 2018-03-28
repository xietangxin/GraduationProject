#include <math.h>

#include "commander.h"
#include "anomal_detec.h"
#include "remoter_ctrl.h"


/* �쳣�����������	*/

#if defined(DETEC_ENABLED)

static bool detecFreeFall(float accZ, float accMAG)	/*����������*/
{
	static uint16_t cnt;
	static bool status;
	
	if(fabs(accMAG) > DETEC_FF_THRESHOLD)	/*������������*/
	{
		cnt=0;
		return false;
	}
	
	if(fabs(accZ + 1) < DETEC_FF_THRESHOLD)
	{
		if(cnt < DETEC_FF_COUNT)
			cnt++;
	}else
	{
		cnt=0;
		status = false;
	}
	
	if(cnt >= DETEC_FF_COUNT)
	{
		status = true;
	}	
	
	return status;
}

static bool detecTumbled(const state_t *state)	/*��ײ���*/
{
	static uint16_t cnt;
	static bool status;
	
	float fAbsRoll  = fabs(state->attitude.roll);
	float fAbsPitch = fabs(state->attitude.pitch);
	float fMax = (fAbsRoll >= fAbsPitch) ? fAbsRoll : fAbsPitch;
	
	if(fMax > DETEC_TU_THRESHOLD)
	{
		if(cnt < DETEC_TU_COUNT)
			cnt++;
	}else 
	{
		cnt=0;
		status = false;
	}
	
	if(cnt >= DETEC_TU_COUNT)
	{
		status = true;
	}
	
	return status;
}
#endif

/*�쳣���*/
void anomalDetec(const sensorData_t *sensorData, const state_t *state, const control_t *control)
{
	if(control->flipDir != CENTER) return;
	if(getCommanderAltholdMode() != true) return;
	
#if defined(DETEC_ENABLED)
	float accMAG = (sensorData->acc.x*sensorData->acc.x) +
					(sensorData->acc.y*sensorData->acc.y) +
					(sensorData->acc.z*sensorData->acc.z);
	
	if(detecFreeFall(state->acc.z, accMAG) ==true)/*����������*/
	{	
		setCommanderKeyFlight(true);
	}
	else if(detecTumbled(state)==true)/*��ײ���*/
	{
		setCommanderKeyFlight(false);
		setCommanderKeyland(false);
	}

#endif
}


