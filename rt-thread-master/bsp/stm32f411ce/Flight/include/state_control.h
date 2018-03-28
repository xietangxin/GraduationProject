#ifndef __STATE_CONTROL_H
#define __STATE_CONTROL_H

#include "stabilizer_types.h"

/* ������̬���ƴ���	*/

//#define ENABLE_PID_TUNING	/* ʹ��PID���� yawֵ������ */

void stateControlInit(void);
bool stateControlTest(void);
void stateControl(control_t *control, sensorData_t *sensors, state_t *state, setpoint_t *setpoint, const uint32_t tick);

#endif /*__STATE_CONTROL_H */
