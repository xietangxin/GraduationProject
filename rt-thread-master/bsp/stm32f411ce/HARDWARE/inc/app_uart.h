#ifndef __APP_UART_H
#define __APP_UART_h

#include <rthw.h>
#include <rtthread.h>

rt_err_t uart_open(const char *name);
rt_uint8_t uart_getchar(void);
void uart_putchar(char rt_uint8_t c);
void uart_putstring(char rt_uint8_t *s);

#endif
