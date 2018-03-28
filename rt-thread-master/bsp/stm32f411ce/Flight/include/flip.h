#ifndef __FLIP_H
#define __FLIP_H 
#include "stabilizer_types.h"

/* ËÄÖá¿Õ·­¿ØÖÆ´úÂë	*/


void flyerFlipCheck(setpoint_t* setpoint,control_t* control,state_t* state, const uint32_t tick);	/* Flyer ·­¹ö¼ì²â*/

void setFlipDir(uint8_t dir);

#endif 

