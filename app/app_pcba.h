/**
 * @brief       : 
 * @file        : app_pcba.h
 * @author      : xukai
 * @version     : v0.0.2
 * @date        : 2016/10/17
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/7/11    v0.0.1      cuihongpeng    first version
 */
#ifndef __APP_PCBA_H__
#define __APP_PCBA_H__

#include "boards.h"
#include "app_comm.h"
#include "app_state.h"
#include "app_dec.h"
#include "app_ble.h"

#define GPRS_SEND_COUNT         (100u)
void bsp_pcba_init(void);
void app_escort_pcba_init(void);

/*
 *@brief 单元测试函数 函数地图
 */
typedef struct test_map_ {
    void (*func)(int);
    char* name;
    int repeat;
    bool run;
} test_map_t;
extern SemaphoreHandle_t sem_escort_pcba;
extern uint8_t m_service_data[4];

#endif