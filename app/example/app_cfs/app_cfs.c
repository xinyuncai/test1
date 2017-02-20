/** @} */

/**
 * @brief       : 
 *
 * @file        : app_cfs.c
 * @author      : cuihongpeng
 * @version     : v0.0.1
 * @date        : 2016/7/11
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/7/11    v0.0.1      cuihongpeng    first version
 */
#include "app_cfs.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Formatting is needed if the storage device is in an unknown state;
   e.g., when using Coffee on the storage device for the first time. */
#define NEED_FORMATTING 1
#ifndef NEED_FORMATTING
#define NEED_FORMATTING 0
#endif

#define FILENAME "test"

int coffee_example(void)
{
    #define MSG1  "#1.hello world."
    #define MSG2  "#2.contiki is amazing!"
    #define EMPTY "empty string"
    static uint8_t msg1_len, msg2_len;
    msg1_len = strlen(MSG1);
    msg2_len = strlen(MSG2);

    /*        */
    /* step 1 */
    /*        */
    char message[32];
    char buf[100];
    strcpy(message, MSG1);
    strcpy(buf,message);
    PRINTF("step 1: %s\n", buf );

    /* End Step 1. We will add more code below this comment later */    
    /*        */
    /* step 2 */
    /*        */
    /* writing to cfs */
    char *filename = "msg_file";
    int fd_write, fd_read;
    // int n;
    fd_write = cfs_open(filename, CFS_WRITE);
    if(fd_write != -1) 
    {
        // n = cfs_write(fd_write, message, sizeof(message));
        cfs_write(fd_write, message, msg1_len);
        cfs_close(fd_write);
        PRINTF("step 2: successfully written to cfs. wrote %i bytes\n", n);
    } 
    else 
    {
        PRINTF("ERROR: could not write to memory in step 2.\n");
        return 0;
    }

    /*        */
    /* step 3 */
    /*        */
    /* reading from cfs */
    strcpy(buf, EMPTY);
    fd_read = cfs_open(filename, CFS_READ);
    if(fd_read!=-1) 
    {
        cfs_read(fd_read, buf, msg1_len);
        PRINTF("step 3: %s\n", buf);
        cfs_close(fd_read);
    } 
    else 
    {
        PRINTF("ERROR: could not read from memory in step 3.\n");
        return 0;    
    }

    /*        */
    /* step 4 */
    /*        */
    /* adding more data to cfs */
    strcpy(buf, EMPTY);
    strcpy(message, MSG2);
    fd_write = cfs_open(filename, CFS_WRITE | CFS_APPEND);  
    if(fd_write != -1) 
    {
        // n = cfs_write(fd_write, message, sizeof(message));
        cfs_write(fd_write, message, msg2_len);
        cfs_close(fd_write);
        PRINTF("step 4: successfully appended data to cfs. wrote %i bytes  \n",n);
    } 
    else 
    {
        PRINTF("ERROR: could not write to memory in step 4.\n");
        return 0;    
    }

    /*        */
    /* step 5 */
    /*        */
    /* seeking specific data from cfs */
    strcpy(buf, EMPTY);
    fd_read = cfs_open(filename, CFS_READ);
    if(fd_read != -1) 
    {
        cfs_read(fd_read, buf, msg1_len);
        PRINTF("step 5: #1 - %s\n", buf);
        cfs_seek(fd_read, msg1_len, CFS_SEEK_SET);
        cfs_read(fd_read, buf, msg2_len);
        PRINTF("step 5: #2 - %s\n", buf);
        cfs_close(fd_read);
    } 
    else 
    {
        PRINTF("ERROR: could not read from memory in step 5.\n");
        return 0;    
    }

    /*        */
    /* step 6 */
    /*        */
    /* remove the file from cfs */
    cfs_remove(filename);
    fd_read = cfs_open(filename, CFS_READ);
    if(fd_read == -1) 
    {
        PRINTF("Successfully removed file\n");
    } 
    else 
    {
        PRINTF("ERROR: could read from memory in step 6.\n");
        return 0;    
    }

    return 1;
}  

/*---------------------------------------------------------------------------*/
static int dir_test(void)
{
  struct cfs_dir dir;
  struct cfs_dirent dirent;

  /* Coffee provides a root directory only. */
  if(cfs_opendir(&dir, "/") != 0) 
  {
    PRINTF("failed to open the root directory\n");
    return 0;
  }

  /* List all files and their file sizes. */
  PRINTF("Available files\n");
  while(cfs_readdir(&dir, &dirent) == 0) 
  {
    PRINTF("%s (%lu bytes)\n", dirent.name, (unsigned long)dirent.size);
  }

  cfs_closedir(&dir);

  return 1;
}

/**
 * @brief flash debug task.
*/
static void cfs_debug(void *pvParameters)
{    
    for(;;)
    {         
        #if NEED_FORMATTING
        cfs_coffee_format();
        #endif
        
        if(coffee_example() == 0)
        {
            PRINTF("coffee example test failed\n");
            LEDS_ON(BSP_LED_1_MASK);
        }

        if(dir_test() == 0) {
           PRINTF("dir test failed\n");
           LEDS_ON(BSP_LED_2_MASK);
        }

        LEDS_ON(LEDS_MASK);
        vTaskDelay(5000);
        LEDS_OFF(LEDS_MASK);
        
        vTaskDelete(NULL);
    }
}

int main( void )
{     
    nrf_drv_clock_init(); 

    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
    
    // Start execution.
    if(pdPASS != xTaskCreate(cfs_debug, "cfs_debug", 128, NULL, 1, NULL))
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
