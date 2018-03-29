#ifndef __SENSFUSION6_H
#define __SENSFUSION6_H

#include "stabilizer_types.h"

/* 6轴数据融合代码	*/

#define DEG2RAD		0.017453293f	/* 度转弧度 π/180 */
#define RAD2DEG		57.29578f		/* 弧度转度 180/π */


void imuUpdate(Axis3f acc, Axis3f gyro, state_t *state , float dt);	/*数据融合 互补滤波*/

#endif

