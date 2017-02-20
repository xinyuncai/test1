/**
 * @brief       : 
 *
 * @file        : app_state.h
 * @author      : cuihongpeng
 * @version     : v0.0.1
 * @date        : 2016/7/11
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/7/11    v0.0.1      cuihongpeng    first version
 */
#ifndef __APP_STATE_H
#define __APP_STATE_H

#include "boards.h"

#define TRAN_ERR_INTERVAL       	(300)
#define TRAN_LED_INTERVAL       	(1500)
#define STATE_LED_INTERVAL       	(10000)
#define LOCATION_INTERVAL       	(10u * 1000)
#define BAT_INTERVAL            	(900 * 1000)
#define HEART_INTERVAL          	(600 * 1000)

#define RTC_HEART_INTERVAL      	(3*3600*COMPARE_COUNTERTIME)                   /* 3hours*/
#define RTC_BAT_INTERVAL        	(15*60*COMPARE_COUNTERTIME)                 /* 15mins*/
#define RTC_LOCATION_INTERVAL   	(10*COMPARE_COUNTERTIME)                    /* 10s */

#define STATUSRTC_LENGTH			10			//length of que_status_rtc
/**
 * 设备状态定义
 */
typedef enum state_
{
    IDLE = 0,
    BINDED,
    STOP,
    MOVE,
    REMOVED,
    WAY,
    STATION,
} state_t;

typedef struct fsm_
{
    uint8_t state;
	uint8_t action;
    void (*tran)(uint8_t state);
    void (*exectue)(uint8_t state);
    uint8_t (*current)(void);  
} fsm_t;
 /**< 外部变量 */
extern fsm_t fsm;

void app_state_init(void);
extern void tran_time_status(uint8_t status);

#endif //__APP_STATE_H