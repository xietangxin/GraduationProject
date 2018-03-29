#ifndef __SENSFUSION6_H
#define __SENSFUSION6_H

#include "stabilizer_types.h"

/* 6�������ںϴ���	*/

#define DEG2RAD		0.017453293f	/* ��ת���� ��/180 */
#define RAD2DEG		57.29578f		/* ����ת�� 180/�� */


void imuUpdate(Axis3f acc, Axis3f gyro, state_t *state , float dt);	/*�����ں� �����˲�*/

#endif

