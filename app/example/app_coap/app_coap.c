/**
 * @brief       : 
 *
 * @file        : app_coap.c
 * @author      : cuihongpeng
 * @version     : v0.0.1
 * @date        : 2016/7/11
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/7/11    v0.0.1      cuihongpeng    first version
 */

#include "app_coap.h"

static void coap_task(void *pvParameters);

#define MSG_BUF_LEN                         100     /**< 接收和发送缓冲区最大长度 */
#define CENTRAL_LINK_COUNT                   0      /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT                1      /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

uint8_t gprs_debug_err = 0;
uint8_t msg_send_buf[MSG_BUF_LEN];			        /**< 发送缓冲区 */
coap_pdu msg_send = {msg_send_buf, 0, 100};         /**< 发送数据单元 */
uint8_t msg_recv_buf[MSG_BUF_LEN];                  /**< 接收缓冲区 */
coap_pdu msg_recv = {msg_recv_buf, 0, 100};         /**< 接收数据单元 */

coap_head_t coap_head;						        /**< coap接收头部信息 */
coap_payload receive_coap_pyload;			        /**< coap接收负载信息 */
uint16_t receive_coap_token = 0;			        /**< coap接收token信息 */
coap_option coap_head_option;				        /**< coap option结构体 */
coap_data_type_t coap_data_type;			        /**< coap数据类型结构体 */

static gprs_net_config_t server_connect_config =
{
    // .type = "udp",
    // .ip = "123.59.83.91",
    // .port = 8563

    .type = "udp",
    .ip = "129.132.15.80",
    .port = 5683    
};

/**
 * @brief 填充coap数据.
 * @param[in] uri option路径
 * @param[in] payload 载荷
*/
void coap_set_data(uint8_t *url, uint8_t *payload)
{
    uint16_t message_id_counter = rand();         // 获取随机信息ID
    coap_init_pdu(&msg_send);                     // 初始化数据单元
    coap_set_version(&msg_send, COAP_V1);         // 设置版本
    coap_set_type(&msg_send, CT_CON);             // 设置类型
//    coap_set_token(&msg_send, 0xdddd, 2);         // 设置token
    coap_set_code(&msg_send, CC_GET);             // post方法
    coap_set_mid(&msg_send, message_id_counter++);// 设置信息ID
    // coap_add_option(&msg_send, CON_URI_PATH, "111", strlen((char const *)"111"));// 加入option
    coap_add_option(&msg_send, CON_URI_PATH, url, strlen((char const *)url));// 加入option
    // coap_add_option(&msg_send, CON_CONTENT_FORMATt, "", 0);// 加入option
//     coap_add_option(&msg_send, CON_BLOCK2, "`", 1);// 加入option
    // coap_set_payload(&msg_send, payload, strlen((char const *)payload));// 加入载荷
}

/**
 * @brief coap数据确认.
 * param[in] coap_pdu_r coap数据包指针
*/
bool_t coap_confirm_data(coap_pdu *coap_pdu_r)
{
    if(coap_validate_pkt(coap_pdu_r) != CE_INVALID_PACKET)// 数据包为有效的coap数据
    {
        #if 0
        coap_head.version = coap_get_version(coap_pdu_r);// 得到版本号
        coap_head.type = coap_get_type(coap_pdu_r);      // 得到类型
        coap_head.token_len = coap_get_tkl(coap_pdu_r);  // 得到token长度
        coap_head.message_id = coap_get_mid(coap_pdu_r); // 得到信息ID
        coap_head.code = coap_get_code(coap_pdu_r);      // 得到功能码
        if(coap_head.token_len > 0)
        {
            receive_coap_token = coap_get_token(coap_pdu_r);// 得到token
        }
        coap_head.coap_head_option = coap_get_option(coap_pdu_r, 0);// 得到option
        receive_coap_pyload = coap_get_payload(coap_pdu_r);// 得到负载
        memset(&coap_head, 0, sizeof(coap_head));
        #endif

        if(coap_get_code(coap_pdu_r) == CC_CONTENT)
        {
            return TRUE;
        }
    }   
    return FALSE;
}

void uart_event_handler(nrf_drv_uart_event_t * p_event, void* p_context)
{

}

uint32_t uart_init()
{
    static uint8_t rx_temp = 0;
    nrf_drv_uart_config_t config = NRF_DRV_UART_DEFAULT_CONFIG;
    config.baudrate = NRF_UART_BAUDRATE_115200;
    config.hwfc = NRF_UART_HWFC_DISABLED;
    config.interrupt_priority = APP_IRQ_PRIORITY_LOW;
    config.parity = NRF_UART_PARITY_EXCLUDED;
    config.pselcts = CTS_PIN_NUMBER;
    config.pselrts = RTS_PIN_NUMBER;
    config.pselrxd = RX_PIN_NUMBER;
    config.pseltxd = TX_PIN_NUMBER;

    uint32_t err_code = nrf_drv_uart_init(&config, uart_event_handler);
    
    nrf_drv_uart_rx_enable(); 
    
    return nrf_drv_uart_rx(&rx_temp, 1);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief COAP任务.
*/
static void coap_task(void *pvParameters)
{
    uart_init();
    
    extern void gprs_cb_init(void);
    gprs_cb_init();
    
    ble_stack_init();

    for(;;)
    {
        coap_set_data("test", NULL);

        if(gprs.open() == FALSE)
        {
            gprs_debug_err = 1;
            vTaskDelete(NULL); // 初始化失败 删除当前任务
        }
      
        if(gprs.connect(&server_connect_config) == FALSE)
        {
            gprs_debug_err = 2;
            vTaskDelete(NULL); // 服务器异常 删除当前任务
        }
send:        
        if(gprs.write(msg_send.buf, msg_send.len) == FALSE)
        {
            gprs_debug_err = 3;
            goto restart;// 重启模块  
        }
        
        if(gprs.read(msg_recv.buf, (uint8_t *)&msg_recv.len) == FALSE)
        {
            goto send;
        }
        
        // 确认是否推送成功
        if(coap_confirm_data(&msg_recv) == FALSE)
        {
            gprs_debug_err = 5; // 响应错误
        }

        gprs.close_connect();
        
        gprs_debug_err = 0;

        vTaskDelete(NULL); // 初始化失败 删除当前任务
        
restart:
      gprs.close();
      os_delay(5);
    }
}

int main( void )
{
    sem_gu620 = xSemaphoreCreateBinary();

    xTaskCreate(coap_task, "COAP", 512, NULL, tskIDLE_PRIORITY + 3, NULL);
    vTaskStartScheduler();

    while (true)
    {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}

void vApplicationMallocFailedHook( void )
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    taskDISABLE_INTERRUPTS();
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
    /* This function will be called by each tick interrupt if
    configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
    added here, but the tick hook is called from an interrupt context, so
    code must not attempt to block, and only the interrupt safe FreeRTOS API
    functions can be used (those that end in FromISR()). */
}
/*-----------------------------------------------------------*/

void vAssertCalled( unsigned long ulLine, const char * const pcFileName )
{
volatile unsigned long ulSetToNonZeroInDebuggerToContinue = 0;

    /* Parameters are not used. */
    ( void ) ulLine;
    ( void ) pcFileName;

    taskENTER_CRITICAL();
    {
        while( ulSetToNonZeroInDebuggerToContinue == 0 )
        {
            /* Use the debugger to set ulSetToNonZeroInDebuggerToContinue to a
            non zero value to step out of this function to the point that raised
            this assert(). */
            __asm volatile( "NOP" );
            __asm volatile( "NOP" );
        }
    }
    taskEXIT_CRITICAL();
}
