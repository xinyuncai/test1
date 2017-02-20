/**
 * @brief       : 
 *
 * @file        : app_coap.h
 * @author      : cuihongpeng
 * @version     : v0.0.1
 * @date        : 2016/7/11
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/7/11    v0.0.1      cuihongpeng    first version
 */
#ifndef __APP_COAP_H
#define __APP_COAP_H

#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "boards.h"
#include "app_uart.h"
#include "nrf_drv_uart.h"
#include "softdevice_handler.h"
#include "gu620.h"
#include "coap.h"

#define COAP_REV_MAX	(128u)			/**< 接收最大长度 */

/**
 * @breaf coap 接收数据结构体
 */
typedef struct coap_data_type_
{
	uint8_t coap_rev_buff[COAP_REV_MAX];/**< 接收缓存 */
	uint8_t coap_rev_len;				/**< 当前接收长度 */
	uint8_t coap_count;					/**< 当前接收个数 */
} coap_data_type_t;

/**
 * @breaf coap 报文头结构体
 */
typedef struct coap_head_
{
    uint8_t version:2;                  /**< 版本号 */
    uint8_t type:2;                     /**< 类型 */
    uint8_t token_len:4;                /**< token长度 */
	uint8_t code;                       /**< 功能码 */
	uint16_t message_id;                /**< 信息ID */
} coap_head_t;

void app_coap_init(void);

#endif
