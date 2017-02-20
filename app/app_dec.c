/**
 * @brief       : 
 *
 * @file        : app_dec.c
 * @author      : wangyaoting
 * @version     : v0.0.1
 * @date        : 2016/9/28
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/9/28    v0.0.1      wangyaoting    first version
 */
#include "app.h"
#include "nrf_log.h"
#include "acc_algorithm.h"
#include "ble_advertising.h"

SemaphoreHandle_t sem_escort_dec;
SemaphoreHandle_t sem_escort_tap;
dec_type_e escort_dec_cmd;

void adxl345_move_operate(void);

TimerHandle_t m_acc_timer;
static double acc_SD = 0;
static adxl345_value_t acc_sampling_buf[10];
static uint32_t acc_sampling_num = 0;
static uint32_t acc_move_state = 0;         // 设备运动状态，0：静止，1：运动
static uint32_t acc_movement_num = 0;
static uint32_t acc_stationary_num = 0;

TimerHandle_t m_remove_timer;
static uint32_t remove_bar1_state = 0;      // 防拆杆1状态，指示RMV_DET_SWIO_1引脚电平
static uint32_t remove_bar2_state = 0;      // 防拆杆2状态，指示RMV_DET_SWIO_2引脚电平

bool get_remove_bar_state(void)
{
    if(remove_bar1_state == 0 && remove_bar2_state == 0)
    {
        return true;
    }
    return false;
}
static void acc_detect_timeout_handler(TimerHandle_t xTimer)
{
    UNUSED_PARAMETER(xTimer);
//    if(pdPASS != xTimerStop(m_acc_timer, 100))
//    {
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }
    escort_dec_cmd = DEC_MOVE;
    xSemaphoreGive(sem_escort_dec);
}

static void remove_detect_timeout_handler(TimerHandle_t xTimer)
{
    UNUSED_PARAMETER(xTimer);
    escort_dec_cmd = DEC_REMOVE;
    xSemaphoreGive(sem_escort_dec);
}

void adxl345_int_operate(void)
{
  	if(true == get_remove_bar_state())
	{
		advertising_start();
		LEDS_ON(WORK_LED_RED_MASK);
		os_delay(500);
		LEDS_OFF(WORK_LED_RED_MASK);
		LEDS_ON(WORK_LED_BLUE_MASK);
		os_delay(500);
		LEDS_OFF(WORK_LED_BLUE_MASK);
	}
}
void adxl345_move_operate(void)
{
	while(1)
	{
		acc_sampling_buf[acc_sampling_num] = adxl345.value();
		acc_sampling_num++;

		if(acc_sampling_num == SAMPLING_NUM)
		{
			acc_sampling_num = 0;
//			adxl345.read_int();
			acc_SD = algorithm_acc.analysis((acc_value_t *)acc_sampling_buf, SAMPLING_NUM, VARIANCE_MOVE);
			if(acc_SD <= ACC_STATIONARY_ACCURACY)
			{
				acc_stationary_num++;
				if(acc_stationary_num >= 5)
				{
					acc_movement_num = 0;
					if(acc_move_state != 0)
					{
						//设备运动变为静止
						acc_move_state = 0;
	#ifdef PCBA_TEST
						LEDS_INVERT(WORK_LED_RED_MASK);
	#else
						if(fsm.current() == MOVE)
						{
							fsm.tran(STOP);
						}
	#endif
					}
					acc_stationary_num = 5;
				}
			}
			if(acc_SD > ACC_MOVEMENT_ACCURACY)
			{
				acc_movement_num++;
				acc_stationary_num = 0;
				if(acc_movement_num >= 5)
				{
					if(acc_move_state != 1)
					{
						//设备静止变为运动
	#ifdef PCBA_TEST
						acc_move_state = 1;
						LEDS_INVERT(WORK_LED_BLUE_MASK);
	#else
						if(fsm.current() == BINDED || fsm.current() == STOP)
						{
						  	acc_move_state = 1;
							fsm.tran(MOVE);
						}
	#endif
					}
					acc_movement_num = 5;
				}
			}
			if(acc_move_state != 0 && acc_move_state != 1)
			{
				APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
			}
		}
		
		uint8_t retvalue;
		retvalue = adxl345.read_int();
		if((retvalue & 0x02) == 0x00)
		{
			static uint8_t break_num = 0;
		  	break_num++;
			break;
		}
//		if(pdPASS != xTimerChangePeriod(m_acc_timer, 100, 100))
//		{
//			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//		}
	}
}

void remove_int_operate(void)
{
    if(nrf_gpio_pin_read(RMV_DET_SWIO_1) != remove_bar1_state)
    {
        remove_bar1_state = nrf_gpio_pin_read(RMV_DET_SWIO_1);
        if(remove_bar1_state == RMV_DET_RELEASE)
        {
            // 按键1抬起处理程序
            NRF_LOG_PRINTF("remove bar(1) release\n");
#ifdef PCBA_TEST
            LEDS_INVERT(WORK_LED_RED_MASK);
#else
            if(fsm.current() != IDLE && fsm.current() != REMOVED)
			{
                if(pdPASS != xTimerStop(m_acc_timer, 100))
                {
                    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
                }
			  	adxl345.uninit();
                fsm.tran(REMOVED);
			}
#endif
        }
        else if(remove_bar1_state == RMV_DET_PRESS)
        {
            // 按键1按下处理程序
            // LEDS_INVERT(WORK_LED_BLUE_MASK);
        }
        else
        {
            APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }
    }
    
    if(nrf_gpio_pin_read(RMV_DET_SWIO_2) != remove_bar2_state)
    {
        remove_bar2_state = nrf_gpio_pin_read(RMV_DET_SWIO_2);
        if(remove_bar2_state == RMV_DET_RELEASE)
        {
            //按键2抬起处理程序
            NRF_LOG_PRINTF("remove bar(2) release\n");
#ifdef PCBA_TEST
            LEDS_INVERT(WORK_LED_BLUE_MASK);
#else
            if(fsm.current() != IDLE && fsm.current() != REMOVED)
			{
                if(pdPASS != xTimerStop(m_acc_timer, 100))
                {
                    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
                }
			  	adxl345.uninit();
                fsm.tran(REMOVED);
			}
#endif
        }
        else if(remove_bar2_state == RMV_DET_PRESS)
        {
            // 按键2按下处理程序
            // LEDS_INVERT(WORK_LED_BLUE_MASK);
        }
        else
        {
            APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }
    }
	
	if(get_remove_bar_state() == true && adxl345.is_init() == false)
	{
		NRF_LOG_PRINTF("adxl345 init start!\r\n");
		
		LEDS_ON(WORK_LED_BLUE_MASK);
        LEDS_ON(WORK_LED_RED_MASK);
		adxl345.init();
		
		if(pdPASS != xTimerStart(m_acc_timer, 100))
		{
			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
		}
		
		os_delay(1000);
		LEDS_OFF(WORK_LED_BLUE_MASK);
        LEDS_OFF(WORK_LED_RED_MASK);
		
		NRF_LOG_PRINTF("adxl345 init success!\r\n");
	}
}

void dec_device_init(void)
{
    // Create timers. 
    m_acc_timer = xTimerCreate("ACC", 3000, pdTRUE, NULL, acc_detect_timeout_handler);
    if(NULL == m_acc_timer)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#ifndef PCBA_TEST
    // 防拆开关 GPIO 初始化
    remove_detect_init();
#endif
    // m_remove_timer 50ms用于消除抖动
    m_remove_timer = xTimerCreate("REMO", 50, pdFALSE, NULL, remove_detect_timeout_handler);
    if(NULL == m_remove_timer)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    remove_bar1_state = nrf_gpio_pin_read(RMV_DET_SWIO_1);
    remove_bar2_state = nrf_gpio_pin_read(RMV_DET_SWIO_2);
}

/**
 * @brief app status task.
*/
static void app_escort_dec(void *pvParameters)
{  
    for(;;)
    {
        while(FALSE == xSemaphoreTake(sem_escort_dec, portMAX_DELAY));
        switch (escort_dec_cmd)
        {           
            case DEC_REMOVE:
                remove_int_operate();
                break;

            case DEC_MOVE:
                adxl345_move_operate();
                break;
        
            default:
                break;
        }
    }
}

/**
 * @brief app status task.
*/
static void app_escort_tap(void *pvParameters)
{  
    for(;;)
    {
        while(FALSE == xSemaphoreTake(sem_escort_tap, portMAX_DELAY));
//#ifndef PCBA_TEST
  		adxl345_int_operate();
//#endif
    }
}

void app_dec_init(void)
{
    dec_device_init();
    
    sem_escort_dec = xSemaphoreCreateBinary();
    if(NULL == sem_escort_dec)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }   

    if(pdPASS != xTaskCreate(app_escort_dec, "app_escort_dec", 256, NULL, APP_DEC_PRIORITY, NULL))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    sem_escort_tap = xSemaphoreCreateBinary();
    if(NULL == sem_escort_tap)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }   

    if(pdPASS != xTaskCreate(app_escort_tap, "app_escort_tap", 128, NULL, APP_DEC_PRIORITY, NULL))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}