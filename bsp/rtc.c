/**
 * @brief       : 
 *
 * @file        : rtc.c
 * @author      : xujing
 * @version     : v0.0.1
 * @date        : 2016/9/23
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/9/23    v0.0.1      xujing    first version
 * status: getcounter is ok.
 *         setcounter needs to prove.
 *         alarm function needs to prove.
 */

#include <stdio.h>
#include <stdbool.h>
#include "app.h"
#include "app_state.h"
#include "nrf_log.h"
#include "rtc.h"


#define DEBUG   0

#define RTC_VALUEOUT    16777215                                    //2的24次方-1

uint32_t nextalarm_diff = 0;                                        //距离下一次alarm时间
uint32_t rtc_alarm_enable = 0;                                      //0: disabled; 1: enable
uint32_t inner_rtc_alarm_interval = RTC_ALARM_INTERVAL;             //for alarm interval
uint32_t standard_time = 1486429375 + TIME_DIFF;                    //unix时间
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);                  //使用RTC2

void inner_rtc_close(inner_rtc_cfgpara_t m_cfgpara);
void inner_rtc_init(inner_rtc_cfgpara_t m_cfgpara);
void inner_rtc_start(inner_rtc_cfgpara_t m_cfgpara);
void rtc_alarm_close(inner_rtc_cfgpara_t m_cfgpara);
uint32_t inner_rtc_getrtccc(inner_rtc_cfgpara_t m_cfgpara);
uint32_t inner_rtc_getcounter(inner_rtc_cfgpara_t m_cfgpara);
void rtc_alarm_update(inner_rtc_cfgpara_t m_cfgpara, uint32_t val);
void rtc_alarm_start(inner_rtc_cfgpara_t m_cfgpara, uint32_t val);
void inner_rtc_setcounter(inner_rtc_cfgpara_t m_cfgpara, uint32_t value);
void inner_rtc_setrtccc(inner_rtc_cfgpara_t m_cfgpara, uint32_t value, bool enable);
void rtc_alarm_start_moment(inner_rtc_cfgpara_t m_cfgpara, uint32_t moment);
uint32_t rtc_alarm_wakeup_moment(inner_rtc_cfgpara_t m_cfgpara, uint32_t interval);

inner_rtc_t inner_rtc = 
{
    .init = inner_rtc_init,
    .start = inner_rtc_start,
    .getcounter = inner_rtc_getcounter,
    .setcounter = inner_rtc_setcounter,
    .setrtccc = inner_rtc_setrtccc,
    .close = inner_rtc_close,
};

inner_rtc_cfgpara_t inner_rtc_cfgpara =
{
    .channel = 0,
    .p_instance = &rtc,
};

inner_rtc_cfgpara_t inner_rtc_location =
{
    .channel = 1,
    .p_instance = &rtc,
};

inner_rtc_cfgpara_t inner_rtc_heart =
{
    .channel = 2,
    .p_instance = &rtc,
};

inner_rtc_cfgpara_t inner_rtc_bat =
{
    .channel = 3,
    .p_instance = &rtc,
};
rtc_alarm_t rtc_alarm = 
{
    .update = rtc_alarm_update,
    .start = rtc_alarm_start,
    .start_moment = rtc_alarm_start_moment,
    .wakeup_moment = rtc_alarm_wakeup_moment,
    .close = rtc_alarm_close,
};

#if defined(__ICCARM__)
time_t (__time32)(time_t *t)
{
    *t = inner_rtc_getcounter(inner_rtc_cfgpara);
    return *t;
}
#endif

static void show_time(void)
{
    time_t timep;
    time(&timep);
    //nrf_printf("%s", asctime(localtime(&timep)));
    NRF_LOG_PRINTF("RTC :%s", asctime(localtime(&timep)));
}

/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
#if DEBUG_TIME
    static uint32_t tmp = 0;
    NRF_LOG_PRINTF("rtc isr is : %d.\n", inner_rtc_getcounter(inner_rtc_heart) - tmp);
    tmp = inner_rtc_getcounter(inner_rtc_heart);
#endif

    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
#ifdef PCBA_TEST
        rtc_alarm_start(inner_rtc_cfgpara, inner_rtc_alarm_interval);
        show_time();
#if DEBUG
        LEDS_INVERT(WORK_LED_BLUE_MASK);
#endif
#else
        active_repeat_1hour_task();
#endif
    }
#ifndef PCBA_TEST
    else if (int_type == NRF_DRV_RTC_INT_COMPARE1)
    {
        //location
        tran_time_status(LOCATION);
    }

    else if (int_type == NRF_DRV_RTC_INT_COMPARE2)
    {
        //heart beat
        rtc_alarm_start(inner_rtc_heart, inner_rtc_heart_interval);//repeat heart frame timer.
        tran_time_status(HEARTBEAT);
    }    

    else if (int_type == NRF_DRV_RTC_INT_COMPARE3)
    {
        //low power
        rtc_alarm_start(inner_rtc_bat, inner_rtc_bat_interval);//repeat low power frame timer.
        tran_time_status(LOW_POWER);
    }
#endif
}

/**
*       rtc_config
*       @input: m_cfgpara:
*               enable_irq:
*/
void rtc_config(inner_rtc_cfgpara_t m_cfgpara)
{
    uint32_t err_code = nrf_drv_rtc_init(m_cfgpara.p_instance, NULL, rtc_handler);
    APP_ERROR_CHECK(err_code);
}

/**
*       rtc0_config
*       @input: m_cfgpara:
*               enable_irq:
*/
void rtc_clear(inner_rtc_cfgpara_t m_cfgpara)
{
    nrf_drv_rtc_counter_clear(m_cfgpara.p_instance);
}

/**
*       inner_rtc_init
*       @input: m_cfgpara:
*               enable_irq:
*/
void inner_rtc_init(inner_rtc_cfgpara_t m_cfgpara)
{    
    nrf_drv_clock_lfclk_request(NULL);
    rtc_config(m_cfgpara);
}

/**
*       inner_rtc_start
*       @input:
*/
void inner_rtc_start(inner_rtc_cfgpara_t m_cfgpara)
{
    //Power on RTC instance
    nrf_drv_rtc_enable(m_cfgpara.p_instance);
}

/**
*       inner_rtc_getcounter
*       @result: uint: s
*/
uint32_t inner_rtc_getcounter(inner_rtc_cfgpara_t m_cfgpara)
{
    uint32_t value = 0;

    value = nrf_rtc_counter_get(m_cfgpara.p_instance->p_reg);
    value = value / COMPARE_COUNTERTIME + standard_time;    
    
    return value;
}

/**
*       inner_rtc_setcounter
*       @input: 
*               value: 校准值
*/
void inner_rtc_setcounter(inner_rtc_cfgpara_t m_cfgpara,
                              uint32_t value)
{
    uint32_t nextalarm_time = 0, rtc_time = 0;
    
    standard_time = value;
    
    if (rtc_alarm_enable > 0)
    {
        nextalarm_time = nrf_rtc_cc_get(m_cfgpara.p_instance->p_reg, m_cfgpara.channel);
        rtc_time = nrf_rtc_counter_get(m_cfgpara.p_instance->p_reg);
        if (nextalarm_time > rtc_time)
        {
            nextalarm_diff = (nextalarm_time - rtc_time) / COMPARE_COUNTERTIME;
        }
        else
        {
            ;//error
        }    
    }

    rtc_clear(m_cfgpara);//clear counter    
    
    if (rtc_alarm_enable > 0)
    {
        rtc_alarm_start(m_cfgpara, nextalarm_time);    
    }
}

/**
*       inner_rtc_setrtccc
*       @input：
*/
void inner_rtc_setrtccc(inner_rtc_cfgpara_t m_cfgpara,
                              uint32_t value, bool enable)
{
    nrf_drv_rtc_cc_set(m_cfgpara.p_instance, m_cfgpara.channel, value, enable);
}

/**
*       inner_rtc_getrtccc
*
*/
uint32_t inner_rtc_getrtccc(inner_rtc_cfgpara_t m_cfgpara)
{
    return nrf_rtc_cc_get(m_cfgpara.p_instance->p_reg, m_cfgpara.channel);
}

/**
*       inner_rtc_close
*
*/
void inner_rtc_close(inner_rtc_cfgpara_t m_cfgpara)
{
    ;
}

////////////////////////***alarm***/////////////////////////////////////////////

/**
*       alarm_update
*       @input: m_cfgpara:
*               val: alarm interval
*               
*/
void rtc_alarm_update(inner_rtc_cfgpara_t m_cfgpara,
                  uint32_t val)
{
    //todo copy the value to flash
    inner_rtc_alarm_interval = val;
    inner_rtc_setrtccc(m_cfgpara, val, true);

}

/**
*       rtc_alarm_start
*       @input: m_cfgpara:
*               val: alarm interval
*               
*/
void rtc_alarm_start(inner_rtc_cfgpara_t m_cfgpara, uint32_t val)
{    
    uint32_t moment;

    rtc_alarm_enable = 1;       //todo
    moment = rtc_alarm_wakeup_moment(m_cfgpara, val);

    inner_rtc_setrtccc(m_cfgpara, moment, true);
}

/**
*       rtc_alarm_start_moment
*       @input: m_cfgpara:
*               val: alarm moment
*               
*/
void rtc_alarm_start_moment(inner_rtc_cfgpara_t m_cfgpara, uint32_t moment)
{
    inner_rtc_setrtccc(m_cfgpara, moment, true);
}

/**
*       rtc_alarm_wakeup_moment
*       @input: interval
*               
*/
uint32_t rtc_alarm_wakeup_moment(inner_rtc_cfgpara_t m_cfgpara, uint32_t interval)
{
    uint32_t moment;

    moment = nrf_rtc_counter_get(m_cfgpara.p_instance->p_reg) + interval; 
    if (moment >= RTC_VALUEOUT)
    {
        moment -= RTC_VALUEOUT;
    }

    return moment;
}

/**
*       alarm_update
*       @input: m_cfgpara:
*               val: alarm interval
*               
*/
void rtc_alarm_close(inner_rtc_cfgpara_t m_cfgpara)
{
  //todo
    rtc_alarm_enable = 0;
    nrf_drv_rtc_cc_disable(m_cfgpara.p_instance, m_cfgpara.channel);     
}