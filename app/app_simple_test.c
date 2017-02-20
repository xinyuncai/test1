
#include "app.h"


/*
 *@brief Function for application main entry.
 */
int main(void)
{
    NRF_LOG_INIT();
    NRF_LOG_PRINTF("Simple Test\n");
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