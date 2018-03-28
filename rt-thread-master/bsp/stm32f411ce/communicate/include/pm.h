#ifndef __PM_H
#define __PM_H

#include <stdbool.h>
#include "atkp.h"

/* 电源管理驱动代码	*/

#define PM_BAT_LOW_VOLTAGE   			3.3f
#define PM_BAT_LOW_TIMEOUT   			(1000 * 5) 	/* 5s */

typedef enum
{
	battery,
	charging,
	charged,
	lowPower,
	shutDown,
	
} PMStates;


void pmInit(void);
bool pmTest(void);
void pmTask(void *param);
void pmSyslinkUpdate(atkp_t *slp);
float pmGetBatteryVoltage(void);
bool getIsLowpower(void);


#endif /* __PM_H */
