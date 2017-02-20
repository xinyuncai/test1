/** @} */

/**
 * @brief       : 
 *
 * @file        : app_rtc.c
 * @author      : xujing
 * @version     : v0.0.1
 * @date        : 2016/9/23
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/9/23    v0.0.1      xujing    first version
 */
#include "app_rtc.h"

#define MSG_BUF_LEN                         100     /**< 接收和发送缓冲区最大长度 */
#define CENTRAL_LINK_COUNT                   0      /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT                1      /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/


void os_delay(uint8_t time)
{
    TickType_t next_send_time;
    next_send_time = xTaskGetTickCount();
    vTaskDelayUntil( &next_send_time, time * 1000/portTICK_RATE_MS);
}

/**
 * @brief flash debug task.
*/
void hardware_init(void)
{
    //INNER_RTC
    inner_rtc.init(inner_rtc_cfgpara);
    inner_rtc.start(inner_rtc_cfgpara);
    rtc_alarm.start(inner_rtc_cfgpara, inner_rtc_alarm_interval);
    debug_init();
}

int main(void)
{     
//    ble_stack_init();
//    nrf_drv_clock_init(); 
    hardware_init();
    
    // Start execution.
      
    // Start FreeRTOS scheduler.
//    vTaskStartScheduler();
//
//    while (true)
//    {
//        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
//    }
    
    while (true)
    {
        __SEV();
        __WFE();
        __WFE();
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
