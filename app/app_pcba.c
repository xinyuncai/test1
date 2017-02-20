/**
 * @brief       : 
 * @file        : app_escort_status.c
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

// 所有测试函数声明
void adxl345_test(int repeat);
void cjson_test(int repeat);
void cjson_location_test(int repeat);
void freertos_malloc_test(int repeat);
void timer_fun_test(int repeat);
void bat_level_test(int repeat);
void low_power_test(int repeat);
void rtc_test(int repeat);
void gu620_test(int repeat);
void gprs_test(int repeat);
void gu620_power_test(int repeat);
void reliability_test(int repeat);
void remove_test(int repeat);

// 测试函数地图
test_map_t test_map[] = 
{
    {adxl345_test, "test adxl345",1, false},
    {cjson_test, "test cjosn", 10, false},
    {cjson_location_test, "test cjosn location", 10, false},
    {freertos_malloc_test, "test freertos malloc", 10, false},
    {timer_fun_test, "test init timer1/2", 1, false},
    {bat_level_test, "check battery", 1, false},
    {low_power_test, "test power consumption", 1, false},
    {rtc_test, "check rtc init", 1, false},
    {gu620_power_test, "test gprs power", 10, false},
    {gu620_test, "gps test", 1, true},
    {reliability_test, "reliability test", 1, false},
    {remove_test, "remove test", 1, true},
    {NULL, "empty", 0, false}
};

uint8_t msg_send_buf[MSG_BUF_LEN];			        /**< 发送缓冲区 */
coap_pdu msg_send = {msg_send_buf, 0, 256};         /**< 发送数据单元 */
uint8_t msg_recv_buf[MSG_BUF_LEN];                  /**< 接收缓冲区 */
coap_pdu msg_recv = {msg_recv_buf, 0, 256};         /**< 接收数据单元 */
escort_basis_info_t escort_basis_info;
const nrf_drv_timer_t timer_test = NRF_DRV_TIMER_INSTANCE(1);

gprs_net_config_t server =
{
    .type = "udp",
    .ip = "123.59.83.91",
    .port = 6683
};

SemaphoreHandle_t sem_escort_pcba;

typedef struct cnt_
{
    uint8_t total;
    uint8_t success;
    uint8_t failed;
} cnt_t;

typedef struct gprs_test_
{
    cnt_t cnt;
    uint8_t len;
    uint8_t buf[128];
} gprs_test_t;

static gprs_test_t gprs_test_send;
static gprs_test_t gprs_test_recv;

static void gu620_open(void)
{
    if(gprs.open() == TRUE)
    {    
        LEDS_ON(WORK_LED_BLUE_MASK);
        os_delay(2000);
        LEDS_OFF(WORK_LED_BLUE_MASK);
    }
}

uint8_t csq[20];
void gprs_test(int repeat)
{
    (void)repeat;   
    gu620_open();
    extern bool_t gprs_csq_check(void);
    while(gprs.connect(&server) == FALSE);

send:
    gprs_csq_check();
    LEDS_INVERT(WORK_LED_RED_MASK);
    sprintf((char *)gprs_test_send.buf, "gprs test [send success: %d, send failed: %d, read success: %d, read failed: %d, %s]\r\n", 
            gprs_test_send.cnt.success, gprs_test_send.cnt.failed, gprs_test_recv.cnt.success, gprs_test_recv.cnt.failed, csq);
    
    if(gprs.write(gprs_test_send.buf, strlen((char const *)gprs_test_send.buf)) == FALSE)
        gprs_test_send.cnt.failed++; 
    else
        gprs_test_send.cnt.success++;

    if(gprs.read(gprs_test_recv.buf, (uint8_t *)&gprs_test_recv.len) == TRUE)
    {
        if(memcmp(gprs_test_recv.buf, "gprs ok", gprs_test_recv.len) == 0)
            gprs_test_recv.cnt.success++;
        else
            gprs_test_recv.cnt.failed++;
    }
    else
        gprs_test_recv.cnt.failed++;

    if(gprs_test_send.cnt.total++ < GPRS_SEND_COUNT)
       goto send;
    
    gprs.close_connect();
    // return TRUE;
}


typedef struct gu620_debug_
{
    uint16_t seq;  
    uint16_t gprs_open_succ;
    uint16_t gprs_open_failed;
    uint16_t gps_open_succ;
    uint16_t gps_open_failed;
    uint16_t gprs_send_succ;
    uint16_t gprs_send_failed; 
} gu620_debug_t;

static gu620_debug_t gu620_debug;

void gu620_test(int repeat)
{
    (void)repeat;
    LEDS_ON(WORK_LED_BLUE_MASK);  
    LEDS_ON(WORK_LED_RED_MASK); 
    os_delay(5000); 
    LEDS_OFF(WORK_LED_BLUE_MASK);  
    LEDS_OFF(WORK_LED_RED_MASK);
    NRF_LOG_PRINTF("gu620 test start!\n"); 
    
    inner_rtc.init(inner_rtc_cfgpara);
    inner_rtc.start(inner_rtc_cfgpara);
    
    for(;;)
    {
        gu620_debug.seq++;

        gu620_debug.gps_open_succ = 0;
        gu620_debug.gps_open_failed = 0;
        gu620_debug.gprs_send_succ = 0;
        gu620_debug.gprs_send_failed = 0;     
           
        if(gprs.open() == TRUE)
        {    
            gu620_debug.gprs_open_succ++;

            for(uint8_t gps_test_index = 0; gps_test_index < 10; gps_test_index++)
            {
                LEDS_INVERT(WORK_LED_RED_MASK);
                if(gps.open() == TRUE)
                {
                    gu620_debug.gps_open_succ++;
                                  
                    if(gps.report() == TRUE)
                    {
                        gprs.connect(&server);

                        static uint32_t seq = 0; 

                        /** json **/
                        cJSON *root; 
                        char *out;
                        root = cJSON_CreateObject();
                        cJSON_AddNumberToObject(root, "lat", gps_info.lat);             // 纬度
                        cJSON_AddNumberToObject(root, "lon", gps_info.lon);             // 经度
                        cJSON_AddNumberToObject(root, "alt", gps_info.alt);             // 高度            
                        cJSON_AddNumberToObject(root, "speed", gps_info.speed);         // 速度
                        cJSON_AddNumberToObject(root, "dir", gps_info.direction);       // 方向
                        cJSON_AddStringToObject(root, "utc_timer", gps_info.utc);       // 时间  
                        cJSON_AddNumberToObject(root, "seq", seq++);                    // seq 
                        out = cJSON_PrintUnformatted(root);  

                        /** coap **/
                        uint8_t content_format = 50;
                        uint16_t message_id_counter = rand(); 
                        coap_init_pdu(&msg_send);             
                        coap_set_version(&msg_send, COAP_V1); 
                        coap_set_type(&msg_send, CT_CON);     
                        coap_set_code(&msg_send, CC_POST);
                        coap_set_mid(&msg_send, message_id_counter++);
                        coap_add_option(&msg_send, CON_URI_PATH, "gps", strlen("gps"));
                        coap_add_option(&msg_send, CON_URI_PATH, "test", strlen("test"));
                        coap_add_option(&msg_send, CON_CONTENT_FORMAT, &content_format, 1);
                        coap_set_payload(&msg_send, (uint8_t *)out, strlen((char const*)out)); 

                        if(gprs.write(msg_send.buf, msg_send.len) == TRUE)
                            gu620_debug.gprs_send_succ++;
                        else
                            gu620_debug.gprs_send_failed++;

                        cJSON_Delete(root);
                        free(out);  

                        gprs.close_connect();
                    }                 
                }
                else
                    gu620_debug.gps_open_failed++;  
            }
            gprs.close();
        }
        else
            gu620_debug.gprs_open_failed++;

        NRF_LOG_PRINTF("[seq : %d, gprs open succ: %d, gprs open failed: %d, gps open succ: %d, gps open failed: %d, gprs send succ: %d, gprs send failed: %d]\n", 
                        gu620_debug.seq, gu620_debug.gprs_open_succ, gu620_debug.gprs_open_failed, gu620_debug.gps_open_succ, gu620_debug.gps_open_failed, gu620_debug.gprs_send_succ, gu620_debug.gprs_send_failed);            
        os_delay(1000);
    }
}

// 验证GU620 打开和关闭
void gu620_power_test(int repeat)
{  
    LEDS_ON(WORK_LED_BLUE_MASK);
	
    adxl345.init();
	if(pdPASS != xTimerStart(m_acc_timer, 100))
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}
	
    LEDS_OFF(WORK_LED_BLUE_MASK);

    for(uint8_t i = 0; i < repeat; i++)
    {
        if(gprs.open() == TRUE)
        {
            NRF_LOG_PRINTF("gprs open success!\n");
            gprs.close();
            os_delay(5000);
        }
        else
            NRF_LOG_PRINTF("gprs open failed!\n");  
    }
}

void rtc_test(int repeat)
{
    (void)repeat;
    inner_rtc.init(inner_rtc_cfgpara);
    inner_rtc.start(inner_rtc_cfgpara);
    rtc_alarm.start(inner_rtc_cfgpara, inner_rtc_alarm_interval);
}

extern bool adxl345_read_register(uint8_t reg_addr, uint8_t *reg_value);
extern bool adxl345_write_register(uint8_t reg_addr, uint8_t reg_value);
void adxl345_test(int repeat)
{  
    LEDS_ON(WORK_LED_BLUE_MASK);
	remove_detect_init();
    adxl345.init();
//	if(pdPASS != xTimerStart(m_acc_timer, 100))
//	{
//		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//	}
    LEDS_OFF(WORK_LED_BLUE_MASK);
	
	os_delay(1000);
	
	while(1)
	{
		adxl345.value();
		static uint8_t retvalue1;
		retvalue1 = adxl345.read_int();
		static uint8_t break_num1 = 0;
		break_num1++;
		if((retvalue1 & 0x02) == 0x00)
		{
			break;
		}
	}
//	escort_dec_cmd = DEC_MOVE;
//    xSemaphoreGive(sem_escort_dec);

    // 进行100次I2C操作
//    static uint8_t reg_value = 0;
//    for(uint8_t seq = 0; seq < repeat; seq++)
//    {
//        adxl345_write_register(ADXL345_OFSX, 0x12);
//        adxl345_read_register(ADXL345_OFSX, &reg_value);
//
//        if(reg_value != 0x12)
//            NRF_LOG_PRINTF("[%d] adxl345 test failed!\n", seq);
//        else
//            NRF_LOG_PRINTF("[%d] adxl345 test success!\n", seq);
//        
//        os_delay(100);
//    }
}

void unuse_pin_init(void)
{
    for(uint8_t i = 0; i < 31; i++)
    {
        nrf_gpio_cfg_output(i);
        NRF_GPIO->OUTCLR = 1 << i;
    }
}

void low_power_test(int repeat)
{
    (void)repeat;
    unuse_pin_init();
    os_delay(10000); 

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    uint8_t err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);       
}

// 验证单层JSON结构
void cjson_test(int repeat)
{
    for(int i = 0; i < repeat; i++) {
        cJSON *root; 
        char *out;
        root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "msgcode", 8);
        cJSON_AddStringToObject(root, "version", "0.0.1");    
        cJSON_AddNumberToObject(root, "ts", 123456789); 
        cJSON_AddNumberToObject(root, "batt", 3.444);
        out = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);
        free(out);
        NRF_LOG_PRINTF("[%d] test cjson malloc\n", i);
        os_delay(10);
    }
}

// 验证多层JSON结构
void cjson_location_test(int repeat)
{
    for(int i = 0; i < repeat; i++) {
        /** json **/
        char *out;
        double gps_data[5];
        cJSON *root, *gps_array;     
        root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "msgcode", 5);
        cJSON_AddStringToObject(root, "version", "0.0.1");    
        cJSON_AddNumberToObject(root, "ts", 123456789); 
        
        gps_data[0] = 120.3735;
        gps_data[1] = 31.495235;
        gps_data[2] = 3.1;
        gps_data[3] = 0.5;
        gps_data[4] = 25;    
        gps_array = cJSON_CreateDoubleArray(gps_data, 5);
       
        cJSON_AddItemToObject(root, "gps", gps_array);
        out = cJSON_PrintUnformatted(root);  

        // cJSON_Delete(gps_array);
        cJSON_Delete(root);
        free(out);

        NRF_LOG_PRINTF("[%d] test cjson child malloc\n", i);
        os_delay(10);
    }
}

void freertos_malloc_test(int repeat)
{
    for(int i = 0; i < repeat; i++) {
        char* ptr = pvPortMalloc(128);
        NRF_LOG_PRINTF("[%d] test freertos malloc\n", i);
        os_delay(10);
        vPortFree(ptr);
    }
}

static TimerHandle_t app_heart_timer;
static double bat_level_min = 5.0;
static double bat_level_max = 0.0;
static double bat_level_now = 0.0;
static void app_heart_timeout_handler(TimerHandle_t xTimer)
{
//  	uint32_t time_start = xTaskGetTickCount();
//    bat_level_now = bat_level.get_volts();
//	uint32_t time_finish = xTaskGetTickCount();
//	NRF_LOG_PRINTF("start:%d, finish:%d\n", time_start, time_finish);
  	bat_level.updata();
    bat_level_now = bat_level.volts;

	if(bat_level_min > bat_level_now)
	{
		bat_level_min = bat_level_now;
	}
	if(bat_level_max < bat_level_now)
	{
		bat_level_max = bat_level_now;
	}
}
void bat_level_test(int repeat)
{
    (void)repeat;
	bat_level_min = 5.0;
	bat_level_max = 0.0;
	bat_level.volts = 0;
	app_heart_timer = xTimerCreate("HEART", 15 * 1000, pdTRUE, NULL, app_heart_timeout_handler);
    if(NULL == app_heart_timer)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
	if(pdPASS != xTimerStart(app_heart_timer, 100))
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}
}

void timer_test_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch(event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            LEDS_INVERT(WORK_LED_BLUE_MASK);
            break;
        
        default:
            //Do nothing.
            break;
    } 
}

void timer_fun_test(int repeat)
{
    (void)repeat;
    
    uint32_t time_ticks;
    uint32_t err_code = nrf_drv_timer_init(&timer_test, NULL, timer_test_event_handler);
    APP_ERROR_CHECK(err_code);
    
    time_ticks = nrf_drv_timer_ms_to_ticks(&timer_test, 1000);
    
    nrf_drv_timer_extended_compare(
         &timer_test, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    
    nrf_drv_timer_enable(&timer_test);  
}

static TimerHandle_t app_adv_timer;
uint8_t m_service_data[4] = {0x00, 0x00, 0x00, 0x00};
static void reliability_sampling(void)
{
  	for(int i = 0; i < 4; i++)
	  	m_service_data[i] = 0xFF;
 
	double bat_level_now = 0;
	bat_level.updata();
    bat_level_now = bat_level.volts;
	m_service_data[3] = (uint8_t)(bat_level_now * 10);
	
	if(gprs.open() == false)
	  	m_service_data[1] = 0x01;
	else if(gprs.close() == false)
		m_service_data[1] = 0x02;
	else
		m_service_data[1] = 0x00;

//	if(gps.open() == false)
//		m_service_data[2] = 0x01;
//	else if(gps.report() == false)
//		m_service_data[2] = 0x02;
//	else if(gps.close() == false)
//		m_service_data[2] = 0x03;
//	else
//		m_service_data[2] = 0x00;
	
//	if(gprs.close() == false)
//		if(gprs.close() == false)
//			gprs.close();
		
		
	adxl345_value_t acc_data;
	acc_data = adxl345.value();
	if(acc_data.x !=0 || acc_data.y !=0 || acc_data.z !=0)
	  	m_service_data[0] = 0x00;
	else
	  	m_service_data[0] = 0x01;
	
  	uint32_t time_start = xTaskGetTickCount();
	NRF_LOG_PRINTF("%d: %d, %d, %d, %d\n",time_start, m_service_data[0], m_service_data[1], m_service_data[2], m_service_data[3]);
	
	if(m_service_data[0] == 0x00 && m_service_data[1] == 0x00)
	{
		LEDS_ON(WORK_LED_BLUE_MASK);
		os_delay(1000);
		LEDS_OFF(WORK_LED_BLUE_MASK);
	}
	else
	{
		LEDS_ON(WORK_LED_RED_MASK);
		os_delay(1000);
		LEDS_OFF(WORK_LED_RED_MASK);
	}
	set_adv_info();
}
static void app_adv_timeout_handler(TimerHandle_t xTimer)
{
	reliability_sampling();
}
void reliability_test(int repeat)
{
	(void)repeat;
	LEDS_ON(WORK_LED_BLUE_MASK);
	adxl345.init();
	os_delay(1000);
	LEDS_OFF(WORK_LED_BLUE_MASK);

	reliability_sampling();
	app_adv_timer = xTimerCreate("ADV", 600 * 1000, pdTRUE, NULL, app_adv_timeout_handler);
    if(NULL == app_adv_timer)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
	if(pdPASS != xTimerStart(app_adv_timer, 100))
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}
}

void remove_test(int repeat)
{  
    LEDS_ON(WORK_LED_BLUE_MASK);
	remove_detect_init();
	os_delay(1000);
    LEDS_OFF(WORK_LED_BLUE_MASK);	
}
/**
 * @brief app status task.
 */
static void app_escort_pcba(void *pvParameters)
{
    for(;;)
    {
        // 循环运行所有被测函数
        NRF_LOG_PRINTF("Unit Test Begin");
        for (int i = 0; test_map[i].func != NULL; i++) {
            if (test_map[i].run) {
                NRF_LOG_PRINTF("%s\n", test_map[i].name);
                test_map[i].func(test_map[i].repeat);
            }
        }
        NRF_LOG_PRINTF("Unit Test Pass");
        
        // 长时间等待，暂时没有含义
        while(FALSE == xSemaphoreTake(sem_escort_pcba, portMAX_DELAY));
    }
}

void app_escort_pcba_init(void)
{  
    // for gu620 driver
    sem_gu620 = xSemaphoreCreateBinary();
    if(NULL == sem_gu620)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    
    sem_escort_pcba = xSemaphoreCreateBinary();
    if(NULL == sem_escort_pcba)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    if(pdPASS != xTaskCreate(app_escort_pcba, "app_escort_pcba", 256, NULL, APP_PCBA_PRIORITY, NULL))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}


/*
 *@brief Function for application main entry.
 */
int main(void)
{
    // 初始化所有硬件
    bsp_init();
    
    app_ble_init();
    app_escort_pcba_init();
    app_dec_init();

    NRF_LOG_INIT();
    NRF_LOG_PRINTF("PCBA Test\n");
    show_device_info();
    
    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    while (true)
    {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}

void vApplicationIdleHook( void )
{
    /* Clear Event Register */
    __SEV();
    /* Wait for event */
    __WFE();
    /* Wait for event */
    __WFE();
}

/**
 * @brief FreeRTOS 内存分配失败钩子函数
 */
void vApplicationMallocFailedHook( void )
{
    taskDISABLE_INTERRUPTS();
    NRF_LOG_PRINTF("freertos malloc failed!\n");
    for(;;);
}

/**
 * @brief FreeRTOS 栈溢出钩子函数
 */
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    (void) pxTask;

    NRF_LOG_PRINTF("application stack overflow!\n");
    NRF_LOG_PRINTF("application name: %s\n", pcTaskName);
    taskDISABLE_INTERRUPTS();
    for(;;);
}