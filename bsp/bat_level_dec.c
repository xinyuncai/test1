/**
* @brief       : 
*
* @file        : power_dec.c
* @author      : wangyaoting
* @version     : v0.0.2
* @date        : 2016/10/17
*
* Change Logs  :
*
* Date        Version      Author      Notes
* 2016/9/22   v0.0.1     wangyaoting   first version
* 2016/10/17  v0.0.2     wangyaoting   second version
*/
#include "bat_level_dec.h"
#include "app_util_platform.h"
#include "sdk_common.h"
#include "nrf_drv_saadc.h"
#include "gu620.h"

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS       600                                          /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION        6                                            /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS      60                                          /**< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com. */
#define ADC_RES_10BIT                       1024                                         /**< Maximum digital value for 10-bit ADC conversion. */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

static nrf_saadc_value_t adc_buf[2];
static bool volts_adc_is_done = false;
static double now_dec_volts = 0;

void bat_level_init(void);
void updata_bat_volts(void);
void bat_level_uninit(void);

/**
* @breaf bat_level接口
*/
bat_level_t bat_level =
{
    .init = bat_level_init,
    .updata = updata_bat_volts,
    .volts = 0,
	.uninit = bat_level_uninit,
};

void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    static uint16_t power_dec_milli_volts;
	static uint16_t power_dec_end = 0;
	static uint16_t power_dec_start = 0;
    
    power_dec_start++;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        static nrf_saadc_value_t adc_result;
        uint32_t          err_code;

        adc_result = p_event->data.done.p_buffer[0];

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
        APP_ERROR_CHECK(err_code);

        power_dec_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;
        
        now_dec_volts = (double)power_dec_milli_volts/500;
		
		power_dec_end++;
		
		volts_adc_is_done = true;
    }
}

void bat_level_init(void)
{
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
    err_code = nrf_drv_saadc_channel_init(0,&config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], 1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[1], 1);
    APP_ERROR_CHECK(err_code);
}

void bat_level_uninit(void)
{
  	nrf_drv_saadc_uninit();
}

void updata_bat_volts(void)
{
  	static double volts_sum, volts_max, volts_min, volts_result;
	
	volts_sum = 0;
	volts_max = 0;
	volts_min = 100;
	volts_adc_is_done = false;
	
  	for(int i = 0; i < 3; i++)
	{
	  	bat_level_init();
		
		nrf_drv_saadc_sample();
		
		while(!volts_adc_is_done);
		volts_adc_is_done = false;
		
		bat_level_uninit();
				
		volts_sum += now_dec_volts;
		
		if(volts_min > now_dec_volts)
		  	volts_min = now_dec_volts;
		
		if(volts_max < now_dec_volts)
		  	volts_max = now_dec_volts;
	}
	
	volts_result = volts_sum - volts_min - volts_max;
	
	bat_level.volts = volts_result;
}