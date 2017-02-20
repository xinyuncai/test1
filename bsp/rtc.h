/**
 * @brief       : 
 *
 * @file        : rtc.h
 * @author      : xujing
 * @version     : v0.0.1
 * @date        : 2016/9/23
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/9/23    v0.0.1      xujing    first version
 */

#ifndef __RTC_H__
#define __RTC_H__

#include <stdint.h>
#include <time.h>
#include "nrf_drv_rtc.h"
#include "nrf_rtc.h"
#include "nrf_drv_clock.h"
#include "boards.h"

#define ENABLE_RTC_INT          1
#define DISABLE_RTC_INT         0
#define COMPARE_COUNTERTIME  (8UL)                                        /**< Get Compare event COMPARE_TIME seconds after the counter starts from 0. */
#define RTC_ALARM_INTERVAL      1*COMPARE_COUNTERTIME                     /* 1 s*/
#define TIME_DIFF               8*60*60
#define ADJUST_TIME             1475031119 + TIME_DIFF

typedef struct inner_rtc_cfgpara_
{
    uint32_t channel;
    nrf_drv_rtc_t const * const p_instance;
    
} inner_rtc_cfgpara_t;

typedef struct inner_rtc_
{
    void (*init)(inner_rtc_cfgpara_t m_cfgpara);

    void (*start)(inner_rtc_cfgpara_t m_cfgpara);

    uint32_t (*getcounter)(inner_rtc_cfgpara_t m_cfgpara);
    
    void (*setcounter)(inner_rtc_cfgpara_t m_cfgpara,
                              uint32_t value);

    void (*setrtccc)(inner_rtc_cfgpara_t m_cfgpara,
                              uint32_t value, bool enable);

    void (*close)(inner_rtc_cfgpara_t m_cfgpara);

} inner_rtc_t;

typedef struct rtc_alarm_
{
    void (*update)(inner_rtc_cfgpara_t m_cfgpara,
                   uint32_t val);

    void (*start)(inner_rtc_cfgpara_t m_cfgpara, uint32_t val);
    void (*start_moment)(inner_rtc_cfgpara_t m_cfgpara, uint32_t moment);
    uint32_t (*wakeup_moment)(inner_rtc_cfgpara_t m_cfgpara, uint32_t interval);
    void (*close)(inner_rtc_cfgpara_t m_cfgpara);

} rtc_alarm_t;

extern inner_rtc_t inner_rtc;
extern rtc_alarm_t rtc_alarm;
extern inner_rtc_cfgpara_t inner_rtc_cfgpara;
extern inner_rtc_cfgpara_t inner_rtc_heart;
extern inner_rtc_cfgpara_t inner_rtc_location;
extern inner_rtc_cfgpara_t inner_rtc_bat;

extern uint32_t inner_rtc_alarm_interval;
extern uint32_t inner_rtc_heart_interval;
extern uint32_t inner_rtc_location_interval;
extern uint32_t inner_rtc_bat_interval;

extern time_t (__time32)(time_t *t);

#endif