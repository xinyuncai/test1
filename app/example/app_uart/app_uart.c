/**
 * @brief       : 
 *
 * @file        : app_http.c
 * @author      : cuihongpeng
 * @version     : v0.0.1
 * @date        : 2016/7/11
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/7/11    v0.0.1      cuihongpeng    first version
 */

#include "app_uart.h"
 
static void uart_debug(void *pvParameters);

SemaphoreHandle_t sem_gu620;		/**< 发送成功信号量 */

void os_delay(uint8_t time)
{
    TickType_t next_send_time;
    next_send_time = xTaskGetTickCount();
    vTaskDelayUntil( &next_send_time, time * 1000/portTICK_RATE_MS);
}

void uart_event_handler(nrf_drv_uart_event_t * p_event, void* p_context)
{
    if (p_event->type == NRF_DRV_UART_EVT_RX_DONE)
    {

    }
    else if (p_event->type == NRF_DRV_UART_EVT_ERROR)
    {

    }
    else if (p_event->type == NRF_DRV_UART_EVT_TX_DONE)
    {

    }
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

#define CENTRAL_LINK_COUNT                   0                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT                1                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

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
 * @brief uart task.
*/
static void uart_debug(void *pvParameters)
{
    uart_init();
    
    extern void gprs_cb_init(void);
    gprs_cb_init();
    
    ble_stack_init();
    
    nrf_drv_uart_tx("Hello NRF52!\r\n", 14);
    
    for(;;)
    {  
        os_delay(1);
        nrf_drv_uart_tx("AT\r", 3);
    }
}

int main( void )
{
    // Start execution.
    if(pdPASS != xTaskCreate(uart_debug, "UART_DBUGE", 256, NULL, 1, NULL))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    // Start FreeRTOS scheduler.
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
