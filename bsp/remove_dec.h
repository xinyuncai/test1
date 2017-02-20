/**
* @brief       : 
*
* @file        : remove_dec.h
* @author      : wangyaoting
* @version     : v0.0.1
* @date        : 2016/9/5
*
* Change Logs  :
*
* Date        Version      Author      Notes
* 2016/9/5    v0.0.1     wangyaoting    first version
*/
#ifndef __REMOVE_DEC_H
#define __REMOVE_DEC_H

#include "nrf_drv_gpiote.h"

#define RMV_DET_PRESS               0       /**< 防拆杆松开 */
#define RMV_DET_RELEASE             1       /**< 防拆杆按下 */

#define RMV_DET_SWIO_1              8
#define RMV_DET_SWIO_2              7

void remove_detect_init(void);
void remove_detect_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
#endif