/**
 * @brief       : 
 *
 * @file        : app_comm.h
 * @author      : xujing
 * @version     : v0.0.1
 * @date        : 2016/10/12
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/10/12    v0.0.1      xujing    first version
 *
 */

#ifndef __APP_COMM_H__
#define __APP_COMM_H__

#include "frame.h"

//definition
#define QUECOAP_LENGTH              (10u)               //the length of quecoap
#define COMMCOAP_TIMEOUT            (1000u)             // the timeout of quecoap
#define MSG_BUF_LEN                 (256u)                /**< 接收和发送缓冲区最大长度 */
#define TIMEOUT_PERIOD              (60000u)               //1MIN
#define STATIONARY_TIMEOUT          (5u)
#define REPEAT_BUF_LEN              (10u)                  //重发buf长度
#define QUESETCOAP_LENGTH           (QUECOAP_LENGTH + QUECOAP_LENGTH * 2)
#define RTC_REPEAT1HOUR_INTERVAL    (3600*COMPARE_COUNTERTIME)                   /* 1hour*/

typedef enum cmd_
{
    BIND = 1,
    UNBIND,
    MOVEMENT,
    STATIONARY,
    LOCATION,
    REMOVE,
    LOW_POWER,
    HEARTBEAT,
    ONWAY,
    ONSTATION,
    UNKNOWN             // 位置状态
} cmd_e;

typedef enum status_
{
    EMPTY = 0,
    BUSY,
    DONE
} status_e;

typedef struct comm_coap_para_
{
	cmd_e command;
	coap_code code;
	void(*func)(bool success);
} comm_coap_para_t;

typedef struct repeat_buf_
{
    uint8_t data[MSG_BUF_LEN];
    uint8_t len;
    status_e status;     //指示buf是否可用：EMPTY;BUSY
} repeat_buf_t;

typedef struct repeat_coap_data_
{
    repeat_buf_t* pdata;        /* 指向repeat buf首地址*/
    uint32_t  time;        /* next time to wake up */
    uint8_t   count;      //1min or 1hour
} repeat_que_data_t;



extern void active_repeat_1hour_task(void);
extern uint8_t coap_request(comm_coap_para_t comm_coap_para);
extern void app_comm_init(void);
extern gprs_net_config_t server;

#endif