/**
* @brief       : 
*
* @file        : remove_dec.c
* @author      : wangyaoting
* @version     : v0.0.1
* @date        : 2016/9/5
*
* Change Logs  :
*
* Date        Version      Author      Notes
* 2016/9/5    v0.0.1     wangyaoting    first version
*/
#include "remove_dec.h"
#include "app_util_platform.h"
#include "nrf_drv_gpiote.h"
#include "sdk_common.h"
#include "app.h"

/**
 * @brief 初始化防拆杆，上拉输入，检测上升沿和下降沿
 */
void remove_detect_init(void)
{
    ret_code_t err_code;
    
    if(nrf_drv_gpiote_is_init() != true)
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }
    
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(RMV_DET_SWIO_1, &config, remove_detect_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(RMV_DET_SWIO_1, true);

    err_code = nrf_drv_gpiote_in_init(RMV_DET_SWIO_2, &config, remove_detect_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(RMV_DET_SWIO_2, true);
}

void remove_detect_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if(xTimerIsTimerActive(m_remove_timer) == pdFALSE)
    {
        BaseType_t yieldReq = pdFALSE;
        if(pdPASS != xTimerStartFromISR(m_remove_timer, &yieldReq))
        {
            APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }
        portYIELD_FROM_ISR(yieldReq);
    }
}
