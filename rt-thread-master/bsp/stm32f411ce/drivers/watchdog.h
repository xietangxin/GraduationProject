#ifndef __WATCHDOG_H
#define __WATCHDOG_H

#include <stdbool.h>

/* ���Ź����� */

#define WATCHDOG_RESET_MS 100	/*���Ź���λʱ��*/
//#define watchdogReset() (IWDG_ReloadCounter())


void watchdogInit(uint16_t xms)
bool watchdogTest(void);


#endif 

