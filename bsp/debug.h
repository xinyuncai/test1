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

#ifndef __DEBUG_H__
#define __DEBUG_H__
   
#include <stdarg.h>  
#include <string.h>    
#include <stdio.h>  
#include <nrf_drv_uart.h> 

void nrf_printf(char *fmt,...);
#endif