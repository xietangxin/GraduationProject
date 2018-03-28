/*
 * File      : main.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-07-29     Arda.Fu      first implementation
 */
#include <rtthread.h>
#include <board.h>
#include <pin.h>
#include "drv_led.h"
//#include "usbd_cdc_vcp.h"

#define	LEDB	25

void ledb_thread_entry(void *paramter)
{
	rt_kprintf("start ledb test\n");
	//ledSetAll();
	ledClearAll();
	//ledTest();
}

int main(void)
{
  /* user app entry */
	
	//usbd_cdc_vcp_Init();
	rt_thread_t  tid;
	tid = rt_thread_create("led", ledb_thread_entry, RT_NULL, 1024, 2, 10);
	if (tid)
			rt_thread_startup(tid);
	rt_kprintf("rt-thread start\n");

	return 0;
}





