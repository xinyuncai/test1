/**
 * @brief       : 
 *
 * @file        : app_dec.h
 * @author      : wangyaoting
 * @version     : v0.0.1
 * @date        : 2016/10/12
 *
 * Change Logs  :
 *
 * Date        Version      Author      	Notes
 * 2016/10/12  v0.0.1       wangyaoting     first version
 */

#ifndef __APP_DEC_H
#define __APP_DEC_H

#include "boards.h"

typedef enum dec_type_
{
    DEC_MOVE = 1,
    DEC_REMOVE,
} dec_type_e;

extern SemaphoreHandle_t sem_escort_tap;
extern SemaphoreHandle_t sem_escort_dec;
extern dec_type_e escort_dec_cmd;
extern TimerHandle_t m_acc_timer;
extern TimerHandle_t m_remove_timer;

void app_escort_dec_init(void);
void app_dec_init(void);
bool get_remove_bar_state(void);
void adxl345_move_operate(void);

#endif