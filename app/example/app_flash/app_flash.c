/** @} */

/**
 * @brief       : 
 *
 * @file        : app_flash.c
 * @author      : cuihongpeng
 * @version     : v0.0.1
 * @date        : 2016/7/11
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/7/11    v0.0.1      cuihongpeng    first version
 */
#include "app_flash.h"

void os_delay(uint8_t time)
{
    TickType_t next_send_time;
    next_send_time = xTaskGetTickCount();
    vTaskDelayUntil( &next_send_time, time * 1000/portTICK_RATE_MS);
}

/**
 * @brief flash debug task.
*/
static void flash_debug(void *pvParameters)
{    
    for(;;)
    {  
        os_delay(3);
        
        static uint32_t code_page_size; 
        static uint32_t code_size;
        static uint32_t *addr;
        static uint32_t patrd;
         
        code_page_size = NRF_FICR->CODEPAGESIZE;
        code_size = NRF_FICR->CODESIZE - 1;

        // Start address: last page
        addr = (uint32_t*)(code_page_size * code_size);

        // Erase page:
        nrf_nvmc_page_erase((uint32_t)addr);

        for(uint32_t i = 0; i < NRF_FICR->CODEPAGESIZE; i += 4)
        {
            // Write flash
            nrf_nvmc_write_word((uint32_t)addr, 0x12345678);
            
            // Read from flash the last written data and send it back:
            patrd = *addr; 
            
            if(patrd != 0x12345678)
            {
                LEDS_ON(BSP_LED_1_MASK); 
                break;
            }
            ++addr; 
        }
        
        LEDS_ON(BSP_LED_0_MASK);    
        vTaskDelete(NULL);
    }
}

int main( void )
{     
    nrf_drv_clock_init(); 
    
    LEDS_CONFIGURE(BSP_LED_0_MASK | BSP_LED_1_MASK);
    LEDS_OFF(BSP_LED_0_MASK | BSP_LED_1_MASK);
    
    // Start execution.
    if(pdPASS != xTaskCreate(flash_debug, "FLASH_DBUGE", 128, NULL, 1, NULL))
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
