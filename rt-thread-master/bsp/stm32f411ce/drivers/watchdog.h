#ifndef __WATCHDOG_H
#define __WATCHDOG_H

#include <stdbool.h>

/* 看门狗程序 */

#define WATCHDOG_RESET_MS 100	/*看门狗复位时间*/
//#define watchdogReset() (IWDG_ReloadCounter())


void watchdogInit(uint16_t xms)
bool watchdogTest(void);


#endif 

