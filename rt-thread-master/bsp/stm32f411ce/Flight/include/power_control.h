#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H

#include "stabilizer_types.h"

/* 功率输出控制代码	*/

typedef struct 
{
	uint32_t m1;
	uint32_t m2;
	uint32_t m3;
	uint32_t m4;
	
}motorPWM_t;

void powerControlInit(void);
bool powerControlTest(void);
void powerControl(control_t *control);

void getMotorPWM(motorPWM_t* get);
void setMotorPWM(bool enable, uint32_t m1_set, uint32_t m2_set, uint32_t m3_set, uint32_t m4_set);
#endif 
