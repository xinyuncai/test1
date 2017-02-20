/**
* @brief       : 
*
* @file        : acc_algorithm.h
* @author      : wangyaoting
* @version     : v0.0.1
* @date        : 2016/10/8
*
* Change Logs  :
*
* Date        Version      Author      Notes
* 2016/10/8    v0.0.1     wangyaoting    first version
*/

#ifndef __MOTION_ALGORITHM_H
#define __MOTION_ALGORITHM_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "boards.h"

#define SAMPLING_NUM 10

typedef enum acc_test_type_
{
    VARIANCE_MOVE = 1,
    VELOCITY_MOVE,
} acc_test_type_e;

/**
 * @brief 加速度结果结构体
 */
typedef struct acc_value_
{
    int16_t x;
    int16_t y;
    int16_t z;
} acc_value_t;

/**
* @breaf 加速度运动检测算法参数设置结构体
*/
typedef struct algo_acc_control_
{
    acc_test_type_e test_type;
    double upper_limit;
    double lower_limit;
    int16_t angle_mount_x;
    int16_t angle_mount_y;
    int16_t angle_mount_z;
} algo_acc_control_t;

/**
* @breaf 加速度运动检测算法接口
*/
typedef struct algorithm_acc_
{
    void (*setting)(algo_acc_control_t);
    double (*analysis)(acc_value_t *, int len, acc_test_type_e);
} algorithm_acc_t;

extern algorithm_acc_t algorithm_acc;
#endif