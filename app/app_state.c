/**
 * @brief       : 
 *
 * @file        : app_state.c
 * @author      : cuihongpeng
 * @version     : v0.0.1
 * @date        : 2016/7/11
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/7/11    v0.0.1      cuihongpeng    first version
 */
#include "app.h"

static uint8_t current_state(void);
static void tran_state(uint8_t state);
static void exectue_state(uint8_t state);
static void location_cb(bool result);
static void low_power_cb(bool result);

 /**< 有限状态机初始化 */
fsm_t fsm = 
{
    .state = IDLE,
	.action = IDLE,
    .tran = tran_state,
    .exectue = exectue_state,
    .current = current_state,
};

static bool allow_remove_flag = FALSE;
static SemaphoreHandle_t sem_state;
static xQueueHandle que_time_status;
static TimerHandle_t app_heart_timer;
static TimerHandle_t app_bat_timer;
static TimerHandle_t app_location_timer;
static TimerHandle_t app_tran_led_timer;
static TimerHandle_t app_state_led_timer;
static TimerHandle_t app_tran_err_timer;
static comm_coap_para_t comm_coap_para;

uint32_t inner_rtc_location_interval = RTC_LOCATION_INTERVAL;       //for location interval
uint32_t inner_rtc_heart_interval = RTC_HEART_INTERVAL;             //for heart interval
uint32_t inner_rtc_bat_interval = RTC_BAT_INTERVAL;                 //for bat interval


void system_start_indecate(void)
{ 
    LEDS_ON(WORK_LED_BLUE_MASK);
    os_delay(1000);
    LEDS_OFF(WORK_LED_BLUE_MASK);    
}


static void heart_beat_cb(bool result)
{
    if(result != TRUE)
    {
        // resend
        NRF_LOG_PRINTF("heart beat frame send failed!\r\n");
    }
    else
    {
        NRF_LOG_PRINTF("heart beat frame send success!\r\n");
    }
}

/* Be called by RTC ISR*/
void tran_time_status(uint8_t status)
{
    BaseType_t yield_req = pdFALSE;
    
#if DEBUG_TIME
    NRF_LOG_PRINTF("app_time_status time out is %d . unit:1/8s. xTaskGetTickCount is %d . unit:ms. status:%d .\n", \
        inner_rtc.getcounter(inner_rtc_cfgpara), xTaskGetTickCount(), status);
#endif

    xQueueSendFromISR(que_time_status, (const void *const)&status, &yield_req);
}

static void app_time_status(void *pvParameters)
{
    uint8_t status;

    for(;;)
    {
        while(FALSE == xQueueReceive(que_time_status, (void* const)&status, portMAX_DELAY));   

        switch(status)
        {
            case LOCATION:
                comm_coap_para.command = LOCATION;
                comm_coap_para.code = CC_CHANGED;
                comm_coap_para.func = location_cb;
                coap_request(comm_coap_para);
                break;
                
            case HEARTBEAT:
                bat_level.updata();
                comm_coap_para.command = HEARTBEAT;
                comm_coap_para.code = CC_CHANGED;
                comm_coap_para.func = heart_beat_cb;
                coap_request(comm_coap_para);
                break;
                
            case LOW_POWER:
                bat_level.updata();
                if(bat_level.volts <= 3.8)
                {
                    comm_coap_para.command = LOW_POWER;
                    comm_coap_para.code = CC_CHANGED;
                    comm_coap_para.func = low_power_cb;
                    coap_request(comm_coap_para);
                }
                break;

            default:
                break;
        } 
    }
}

static void low_power_cb(bool result)
{
    if(result != TRUE)
    {
        // resend
        NRF_LOG_PRINTF("low power frame send failed!\r\n");
    }
    else
    {
        NRF_LOG_PRINTF("low power frame send success!\r\n");
    }    
}

static void location_cb(bool result)
{
    if(result != TRUE)
    {
        // resend
        NRF_LOG_PRINTF("location frame send failed!\r\n");
    }
    else
    {
        NRF_LOG_PRINTF("location frame send success!\r\n");
    }  

    if(fsm.current() == MOVE)
    {
        rtc_alarm.start(inner_rtc_location, inner_rtc_location_interval);
    }

    system_start_indecate();
}

static void app_tran_err_timeout_handler(TimerHandle_t xTimer)
{
    static uint8_t err_time_cnt = 0;
    LEDS_INVERT(WORK_LED_RED_MASK);
    if(err_time_cnt++ >= 20)
    {
	  	err_time_cnt = 0;
        LEDS_OFF(WORK_LED_RED_MASK);
        if(pdPASS != xTimerStop(app_tran_err_timer, 10))
        {
            APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }
    }
}

static void app_tran_err_timer_start(void)
{
    if(pdPASS != xTimerStart(app_tran_err_timer, 100))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }   
}

static void app_tran_led_timeout_handler(TimerHandle_t xTimer)
{
    LEDS_INVERT(WORK_LED_BLUE_MASK);
}

static void app_tran_led_timer_start(void)
{
    if(pdPASS != xTimerStart(app_tran_led_timer, 100))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }  
}

static void app_tran_led_timer_stop(void)
{
    if(pdPASS != xTimerStop(app_tran_led_timer, 10))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    LEDS_OFF(WORK_LED_BLUE_MASK);    
}

static void app_state_led_timeout_handler(TimerHandle_t xTimer)
{
    LEDS_ON(WORK_LED_BLUE_MASK);
    os_delay(500);
    LEDS_OFF(WORK_LED_BLUE_MASK);
}

static void app_state_led_timer_start(void)
{
    LEDS_ON(WORK_LED_BLUE_MASK);
    os_delay(500);
    LEDS_OFF(WORK_LED_BLUE_MASK);

    if(pdPASS != xTimerStart(app_state_led_timer, 100))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }    
}

static void app_state_led_timer_stop(void)
{
    if(pdPASS != xTimerStop(app_state_led_timer, 10))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    LEDS_OFF(WORK_LED_BLUE_MASK);    
}

static void idle(void)
{
  	fsm.state = IDLE;
    NRF_LOG_PRINTF("enter idle state!\r\n");
}

static void bind_cb(bool result)
{
    app_tran_led_timer_stop();
    if(result == TRUE)
    {
        NRF_LOG_PRINTF("enter bind state!\r\n");
		fsm.state = BINDED;
		
#if DEBUG_TIME
        NRF_LOG_PRINTF("app_heart_rtc start time is %d . unit:1/8s. xTaskGetTickCount is %d . unit:ms. \n", inner_rtc.getcounter(inner_rtc_cfgpara), xTaskGetTickCount());
#endif		
        //heart
        rtc_alarm.start(inner_rtc_heart, inner_rtc_heart_interval);
        // low power
        rtc_alarm.start(inner_rtc_bat, inner_rtc_bat_interval);

        NRF_LOG_PRINTF("start heart(3hours) | bat(15min) timer!\r\n");

        allow_remove_flag = TRUE;
    }
    else
    {
        app_tran_err_timer_start();
        // resend
        NRF_LOG_PRINTF("bind frame send failed!\r\n");
    }
}

static void bind(void)
{
    app_tran_led_timer_start();

    // bind frame
    comm_coap_para.command = BIND;
    comm_coap_para.code = CC_CHANGED;
    comm_coap_para.func = bind_cb;
    coap_request(comm_coap_para);    
}

static void stop_cb(bool result)
{   
    app_state_led_timer_stop();
    if(result == TRUE)
    {
        NRF_LOG_PRINTF("enter stop state!\r\n");         
    }
    else
    {
        // resend
        NRF_LOG_PRINTF("stop frame send failed!\r\n");
    }  
}

static void stop(void)
{
  	fsm.state = STOP;
	
    NRF_LOG_PRINTF("location timer stop!\r\n");   

    rtc_alarm.close(inner_rtc_location);

    // stop frame
    comm_coap_para.command = STATIONARY;
    comm_coap_para.code = CC_CHANGED;
    comm_coap_para.func = stop_cb;
    coap_request(comm_coap_para);  

    // location frame
    comm_coap_para.command = LOCATION;
    comm_coap_para.code = CC_CHANGED;
    comm_coap_para.func = location_cb;
    coap_request(comm_coap_para);    
}

static void move_cb(bool result)
{
    if(result == TRUE)
    {
        NRF_LOG_PRINTF("enter move state!\r\n"); 
    }
    else
    {
        // resend
        NRF_LOG_PRINTF("move frame send failed!\r\n");
    }  
}

static void move(void)
{
  	fsm.state = MOVE;
	
    app_state_led_timer_start();

    rtc_alarm.start(inner_rtc_location, inner_rtc_location_interval);
    NRF_LOG_PRINTF("location timer start! -- 10s\r\n");  
    
    // move frame
    comm_coap_para.command = MOVEMENT;
    comm_coap_para.code = CC_CHANGED;
    comm_coap_para.func = move_cb;
    coap_request(comm_coap_para);   
}

static void removed_cb(bool result)
{
    app_tran_led_timer_stop();
    if(result == TRUE)
    {
        NVIC_SystemReset();
    }
    else
    {
        app_tran_err_timer_start();
        // resend
        NRF_LOG_PRINTF("remove frame send failed!\r\n");
    }
}

static void removed(void)
{
    if(allow_remove_flag == TRUE)
    {
	  	fsm.state = REMOVED;
        app_tran_led_timer_start();
        // remove frame
        comm_coap_para.command = REMOVE;
        comm_coap_para.code = CC_CHANGED;
        comm_coap_para.func = removed_cb;
        coap_request(comm_coap_para);
    }
}

static void way_cb(bool result)
{
    app_tran_led_timer_stop();
    if(result == TRUE)
    {
        NRF_LOG_PRINTF("enter onway state!\r\n");
		fsm.state = WAY;
    }
    else
    {
        app_tran_err_timer_start();
        // resend
        NRF_LOG_PRINTF("onway frame send failed!\r\n");
    }
}

static void way(void)
{
    app_tran_led_timer_start();

    // bind frame
    comm_coap_para.command = ONWAY;
    comm_coap_para.code = CC_CHANGED;
    comm_coap_para.func = way_cb;
    coap_request(comm_coap_para);    
}

static void station_cb(bool result)
{
    app_tran_led_timer_stop();
    if(result == TRUE)
    {
        NRF_LOG_PRINTF("enter onstation state!\r\n");
		fsm.state = STATION;
    }
    else
    {
        app_tran_err_timer_start();
        // resend
        NRF_LOG_PRINTF("onstation frame send failed!\r\n");
    }
}

static void station(void)
{
    app_tran_led_timer_start();

    // bind frame
    comm_coap_para.command = ONSTATION;
    comm_coap_para.code = CC_CHANGED;
    comm_coap_para.func = station_cb;
    coap_request(comm_coap_para);    
}

static uint8_t current_state(void)
{
    return fsm.state;
}

static void tran_state(uint8_t action)
{
    fsm.action = action;
    xSemaphoreGive(sem_state);
}

static void exectue_state(uint8_t action)
{
    switch(action)
    {
        case IDLE:
            idle();
            break;

        case BINDED:
            bind();
            break;

        case STOP:
            stop();
            break;

        case MOVE:
            move();
            break;

        case REMOVED:
            removed();
            break;

        case WAY:
            way();
            break;

        case STATION:
            station();
            break;

	}
}

/**
 * @brief app state task.
*/
static void app_state(void *pvParameters)
{
    system_start_indecate();
    fsm.tran(IDLE);
	
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
	
    for(;;)
    {
        while(FALSE == xSemaphoreTake(sem_state, portMAX_DELAY));        
        fsm.exectue(fsm.action);
    }
}

void app_state_init(void)
{   
    // Create timers.   
    app_tran_err_timer = xTimerCreate("TRAN ERR", TRAN_ERR_INTERVAL, pdTRUE, NULL, app_tran_err_timeout_handler);
    if(NULL == app_tran_err_timer)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }    

    app_tran_led_timer = xTimerCreate("TRAN LED", TRAN_LED_INTERVAL, pdTRUE, NULL, app_tran_led_timeout_handler);
    if(NULL == app_tran_led_timer)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    } 

    app_state_led_timer = xTimerCreate("STATE LED", STATE_LED_INTERVAL, pdTRUE, NULL, app_state_led_timeout_handler);
    if(NULL == app_state_led_timer)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }     
    
    sem_state = xSemaphoreCreateBinary();
    if(NULL == sem_state)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    } 

    que_time_status = xQueueCreate(STATUSRTC_LENGTH, 1);;
    if(NULL == que_time_status)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    } 

    if(pdPASS != xTaskCreate(app_state, "app_state", 256, NULL, APP_STATE_PRIORITY, NULL))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    if(pdPASS != xTaskCreate(app_time_status, "app_time_status", 128, NULL, APP_STATE_PRIORITY, NULL))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}

/**@brief Function for application main entry.
 */
int main(void)
{
#if CFS_LOG  
    cfs_coffee_format();
#endif
    
    bsp_init();         /**< 初始化硬件 */
    
    NRF_LOG_INIT();     /**< 初始化Log系统 */
    NRF_LOG_PRINTF("System Start!\n");
    
    app_ble_init();
    app_state_init();
    app_comm_init();
    app_dec_init();
	
    show_device_info();
    
    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    while (true)
    {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}

void sleep(void)
{
    /* Clear Event Register */
    __SEV();
    /* Wait for event */
    __WFE();
    /* Wait for event */
    __WFE();
}

void vApplicationIdleHook( void )
{
     sleep();
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    NRF_LOG_PRINTF("freertos malloc failed!\n");
    for(;;);
}

/*-----------------------------------------------------------*/
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void) pxTask;

    NRF_LOG_PRINTF("application stack overflow!\n");
    NRF_LOG_PRINTF("application name: %s\n", pcTaskName);
    taskDISABLE_INTERRUPTS();
    for(;;);
}
