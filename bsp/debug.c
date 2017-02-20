/**
 * @brief       : 
 *
 * @file        : debug.c
 * @author      : xujing
 * @version     : v0.0.1
 * @date        : 2016/9/23
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/9/23    v0.0.1      xujing    first version
 */  
#include "debug.h"

void nrf_printf(char *fmt,...)
{
    va_list ap;
    char string[256];
    uint32_t length;
    
    va_start(ap, fmt);
    vsprintf(string, fmt, ap);
    length = strlen(string);
    nrf_drv_uart_tx((uint8_t const * )string, length);
    va_end(ap);
}