#ifndef __FLIP_H
#define __FLIP_H 
#include "stabilizer_types.h"

/* ����շ����ƴ���	*/


void flyerFlipCheck(setpoint_t* setpoint,control_t* control,state_t* state, const uint32_t tick);	/* Flyer �������*/

void setFlipDir(uint8_t dir);

#endif 

