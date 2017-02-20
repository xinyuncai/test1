/**
 * @brief       : 
 *
 * @file        : app_http.h
 * @author      : cuihongpeng
 * @version     : v0.0.1
 * @date        : 2016/7/11
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/7/11    v0.0.1      cuihongpeng    first version
 */
#ifndef __APP_UART_H
#define __APP_UART_H

#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "boards.h"
#include "gu620.h"
#include "semphr.h"
#include "queue.h"
#include "app_util_platform.h"
#include "nrf_drv_uart.h"
#include "softdevice_handler.h"

#define HTTP_REQUEST "GET /test/ HTTP/1.1\r\nHost:123.59.83.91\r\n\r\n"

#endif
