/**
 * @brief       : 
 * @file        : app.h
 * @author      : cuihongpeng
 * @version     : v0.0.1
 * @date        : 2016/7/11
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/7/11    v0.0.1      cuihongpeng    first version
 */
#ifndef __APP_H__
#define __APP_H__

#include "boards.h"
#include "app_comm.h"
#include "app_state.h"
#include "app_dec.h"
#include "app_ble.h"

#ifdef PCBA_TEST
#include "app_pcba.h"
#define APP_PCBA_PRIORITY       		(5u)  
#endif

#define SOFT_VERSION    "6.0.5"         /**< 固件版本编号 */
#define HARD_VERSION    "2.1.0"         /**< 硬件设备版本号 */
#define CFS_LOG			(1u)

/**< 任务优先级，FREERTOS数字越大，优先级越高 */
#define APP_BLE_PRIORITY                (3u)
#define APP_DEC_PRIORITY                (4u)
#define APP_STATE_PRIORITY      		(5u) 
#define APP_COMM_PRIORITY           	(6u)
#define APP_REPEAT1HOUR_PRIORITY        (4u)

#define DEBUG_TIME      				(0u)

#endif