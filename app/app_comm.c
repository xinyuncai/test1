/**@*/

/**
 * @brief       : 
 *
 * @file        : app_comm.c
 * @author      : xujing
 * @version     : v0.0.1
 * @date        : 2016/10/12
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2016/10/12    v0.0.1      xujing    first version
 */
#include "app.h"
#include "queue.h"
#include "rtc.h"

static QueueSetHandle_t queset_comm_coap;                         /* communication queue */
static xQueueHandle que_com_coap;                          /* common queue */
static xQueueHandle que_repeat_1min;                       /* 1min repeat queue */
static xQueueHandle que_repeat_1hour;                      /* 1hour repeat queue */
static xQueueHandle que_repeat_data;                       /* repeat queue */
static SemaphoreHandle_t sem_rtc_repeat1hour;

static TimerHandle_t mgprs_1mintimer;

uint8_t msg_send_buf[MSG_BUF_LEN];			        /**< 发送缓冲区 */
coap_pdu msg_send = {msg_send_buf, 0, 256};         /**< 发送数据单元 */
uint8_t msg_recv_buf[MSG_BUF_LEN];                  /**< 接收缓冲区 */
coap_pdu msg_recv = {msg_recv_buf, 0, 256};         /**< 接收数据单元 */
escort_basis_info_t escort_basis_info;
static repeat_que_data_t repeat_msg_send = {0, 0, 0};
repeat_buf_t repeat_buf[REPEAT_BUF_LEN];                /**< 存放重发数据帧 */

static uint8_t mgprs_needclose = 0;                         /* 0: 不需要； 1：需要 */

#define DEVICE_STATE(x)			(fsm.current() == x)
uint32_t inner_rtc_repeat1hour_interval = RTC_REPEAT1HOUR_INTERVAL;
// static TimerHandle_t app_stationary_timer;

gprs_net_config_t server =
{
    .type = "udp",
    .ip = "123.59.86.188",
    .port = 5683
};

/**
 * @brief repeat_buf数组中提取可用buf
*/
repeat_buf_t* reapeat_getbuf(void)
{
    uint8_t mcnt = 0;

    while( mcnt < REPEAT_BUF_LEN)
    {
        if( repeat_buf[mcnt].status == EMPTY)
        {
            repeat_buf[mcnt].status = BUSY;
            return (&repeat_buf[mcnt]);
        }

        mcnt++;
    }

    return NULL;
}

/**
 * @brief only suitable for non_isr interface.
*/
uint8_t coap_request(comm_coap_para_t comm_coap_para)
{
    uint8_t res;

    res = xQueueSend(que_com_coap, (const void * const)&comm_coap_para, COMMCOAP_TIMEOUT);

    return res;
}

/**
 * @brief coap send.
*/
uint8_t coap_send_frame(coap_code code)
{
    uint8_t re_send_cnt = 0;

send:
    if(gprs.connect(&server) == FALSE)
        return FALSE;

    if(gprs.write(msg_send.buf, msg_send.len) == FALSE)
    {
        gprs.close_connect();
        return FALSE;  
    }

    if(gprs.read(msg_recv.buf, (uint8_t *)&msg_recv.len) == FALSE)
    {
        if(re_send_cnt++ < RESEND_CNT_MAX)
        {
            gprs.close_connect();
            goto send;
        }
        else
        {
            gprs.close_connect();
            return FALSE;
        }
    }

    if(coap_get_code(&msg_recv) != code)
    {
        if(re_send_cnt++ < RESEND_CNT_MAX)
        {
            gprs.close_connect();
            goto send;
        }
        else
        {
            gprs.close_connect();
            return FALSE;
        }
    }

    gprs.close_connect();//todo
    return TRUE;
}

/**
 * @brief 设置gprs关闭状态
*/
static void gprs_closereq(status_e status)
{
    mgprs_needclose = status; 
}

/**
 * @brief 判断是否需要关闭gprs
*/
static uint8_t gprs_needclose(void)
{

    return mgprs_needclose;
}

/**
 * @brief 重发1min超时处理函数
*/
static void gprs_1mintimer_timeout_handler(TimerHandle_t xTimer)
{
    uint8_t res;
    repeat_que_data_t mque1min;

    NRF_LOG_PRINTF("gprs 1mintimer timeout! \r\n");

    if (pdPASS != xTimerStop(mgprs_1mintimer, 10))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    /* 读取队列直至队列为空 */
    while (pdPASS == xQueueReceive( que_repeat_1min, &mque1min, 0))
    {
        res = xQueueSend(que_repeat_data, (const void * const)&mque1min, COMMCOAP_TIMEOUT);

        if (res == pdFALSE)
        {
        
        }
    }
}

/**
 * @brief from ISR. 激活重发1hour任务
*/
void active_repeat_1hour_task(void)
{
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    xSemaphoreGiveFromISR(sem_rtc_repeat1hour, &xHigherPriorityTaskWoken);
}

/**
 * @brief gprs重发1hour任务
*/
static void app_repeat_1hour(void* para)
{
    uint8_t res;
    uint32_t interval;
    repeat_que_data_t mque1hour;

    for(;;)
    {
        xQueueReceive( que_repeat_1hour, &mque1hour, portMAX_DELAY);
        rtc_alarm.start_moment( inner_rtc_cfgpara, mque1hour.time);

        xSemaphoreTake(sem_rtc_repeat1hour, portMAX_DELAY);        
        NRF_LOG_PRINTF("rtc active app_repeat_1hour. \r\n");
        res = xQueueSend(que_repeat_data, (const void * const)&mque1hour, COMMCOAP_TIMEOUT);
        if (res == pdFALSE)
        {
        
        }
    }
}

/**
 * @brief gprs发送失败后，处理gprs重发。
*/
static void gprs_repeatprocess(bool success, repeat_que_data_t repeat_data)
{
    if ( success == TRUE)
    {
        NRF_LOG_PRINTF("gprs repeatprocess successful. \r\n");
        repeat_data.pdata->status = EMPTY;
        repeat_data.time = 0;
        repeat_data.count = 0;
    }
    else
    {
        repeat_data.count++;
        repeat_data.time = rtc_alarm.wakeup_moment(inner_rtc_cfgpara, inner_rtc_repeat1hour_interval);

        NRF_LOG_PRINTF("repeat_data.count is : %d. \r\n", repeat_data.count);

        if (repeat_data.count < 4)
        {
            uint8_t res;

            if (repeat_data.count == 1)
            {
                repeat_data.pdata = reapeat_getbuf();

                if (repeat_data.pdata != NULL)
                {                    
                    repeat_data.pdata->len = msg_send.len;
                    memcpy(repeat_data.pdata->data, msg_send.buf, msg_send.len);

                    NRF_LOG_PRINTF("repeat_getbuf success!\r\n");
                }
                else
                {
                    NRF_LOG_PRINTF("repeat_getbuf failed. \r\n");
                }
            }

            if (xTimerIsTimerActive(mgprs_1mintimer) == pdFALSE)
            {
                if(pdPASS != xTimerStart(mgprs_1mintimer, 100))
                {
                    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
                }
            }

            res = xQueueSend(que_repeat_1min, (const void * const)&repeat_data, COMMCOAP_TIMEOUT);
            if (res == pdFALSE)
            {
            
            }

        }
        else
        {
            uint8_t res;

            NRF_LOG_PRINTF("gprs enter 1hour task. \r\n");

            repeat_data.count = 0;
            repeat_data.pdata->status = EMPTY;
            res = xQueueSend(que_repeat_1hour, (const void * const)&repeat_data, COMMCOAP_TIMEOUT);
            if (res == pdFALSE)
            {
            
            }
        }
#if GU620_DEBUG
        NRF_LOG_PRINTF("gprs store data: %s. \r\n", repeat_data.pdata->data);
#endif
    }
}

/**
 * @brief app comm coap task.
*/
static void app_comm_coap(void *para)
{
    bool res = FALSE;
    uint8_t que_status = 0;
    comm_coap_para_t mquecom;
    // repeat_que_data_t mquerepeat;
    static uint8_t last_state = UNKNOWN;
    static QueueSetMemberHandle_t xActivedMember;
    
    for(;;)
    {
        xActivedMember = xQueueSelectFromSet( queset_comm_coap, portMAX_DELAY );

        repeat_msg_send.pdata = 0;
        repeat_msg_send.time = 0;
        repeat_msg_send.count = 0;

        if (TRUE == gprs.open())
        {
            if ( xActivedMember == que_com_coap )
            {
                xQueueReceive( xActivedMember, &mquecom, 0);

                NRF_LOG_PRINTF("receive common gprs queue. command: %d. \r\n", mquecom.command);

                if ( mquecom.command <= ONSTATION )
                {
                    gprs_closereq(BUSY);
                    switch (mquecom.command)
                    {           
                        case BIND:  
                            escort_bind_operate(&msg_send);
                            gprs_closereq(DONE);
                            res = coap_send_frame(CC_CHANGED);
                            break;
                        
                        case UNBIND:
                            escort_unbind_operate(&msg_send);
                            break;

                        case MOVEMENT: 
                            gps.open();
                            escort_movement_operate(&msg_send);
                            res = coap_send_frame(CC_CHANGED);
                            break;
                            
                        case STATIONARY:  
                            escort_stationary_operate(&msg_send);
                            res = coap_send_frame(CC_CHANGED);
                            break;

                        case LOCATION:
                            gps.report();
                            escort_location_operate(&msg_send);
                            if (last_state == STATIONARY)
                            {
                                gps.close();
                                gprs_closereq(DONE); 
                            }
                            res = coap_send_frame(CC_CHANGED);
                            break;
                        
                        case REMOVE:  
                            escort_remove_operate(&msg_send);
                            gprs_closereq(DONE);
                            res = coap_send_frame(CC_CHANGED);
                            break;

                        case LOW_POWER: 
                            escort_lowpower_operate(&msg_send);
                            if (!DEVICE_STATE(MOVE))
                            {
                                gprs_closereq(DONE);
                            }
                            res = coap_send_frame(CC_CHANGED);
                            break;
                            
                        case HEARTBEAT:
                            escort_heartbeat_operate(&msg_send);
                            if (!DEVICE_STATE(MOVE))
                            {
                                gprs_closereq(DONE);
                            }
                            res = coap_send_frame(CC_CHANGED);
                            break;
                            
                        case ONWAY:
                            escort_onway_operate(&msg_send);
                            if (!DEVICE_STATE(MOVE))
                            {
                                gprs_closereq(DONE);
                            }
                            res = coap_send_frame(CC_CHANGED);
                            break;

                        case ONSTATION:
                            escort_onstation_operate(&msg_send);
                            if (!DEVICE_STATE(MOVE))
                            {
                                gprs_closereq(DONE);
                            }
                            res = coap_send_frame(CC_CHANGED);
                            break;
                            
                        default:
                            break;
                    }
                    last_state = mquecom.command;
                    mquecom.func(res);     // 回调函数
                }
                else
                {

                }
            }
            else if ( xActivedMember == que_repeat_data)
            {
                xQueueReceive( xActivedMember, &repeat_msg_send, 0);

                NRF_LOG_PRINTF("receive repeat gprs queue. \r\n");

                msg_send.buf = (uint8_t*)repeat_msg_send.pdata;
                msg_send.len = repeat_msg_send.pdata->len;
                res = coap_send_frame(CC_CHANGED);
            }    

            gprs_repeatprocess(res, repeat_msg_send);

            if (((gprs_needclose() == DONE) || (gprs_needclose() == EMPTY)) &&                                            \
                ((uxQueueMessagesWaiting(que_com_coap) == 0) && (uxQueueMessagesWaiting(que_com_coap) == 0)))
            {
                gprs.close();
                gprs_closereq(EMPTY);
            }
        }
        else
        {
            res = FALSE;

            if ( xActivedMember == que_com_coap )
            {
                xQueueReceive( xActivedMember, &mquecom, 0);

                NRF_LOG_PRINTF("receive common gprs queue. command: %d. \r\n", mquecom.command);
                
                if ( mquecom.command <= ONSTATION )
                {
                    gprs_closereq(BUSY);
                    switch (mquecom.command)
                    {           
                        case BIND:  
                            escort_bind_operate(&msg_send);
                            gprs_closereq(DONE);
                            break;
                        
                        case UNBIND:
                            escort_unbind_operate(&msg_send);
                            break;

                        case MOVEMENT: 
                            // gps.open();
                            escort_movement_operate(&msg_send);
                            break;
                            
                        case STATIONARY:  
                            escort_stationary_operate(&msg_send);
                            break;

                        case LOCATION:
                            // gps.report();
                            escort_location_operate(&msg_send);
                            if (last_state == STATIONARY)
                            {
                                // gps.close();
                                gprs_closereq(DONE); 
                            }
                            break;
                        
                        case REMOVE:  
                            escort_remove_operate(&msg_send);
                            gprs_closereq(DONE);
                            break;

                        case LOW_POWER: 
                            escort_lowpower_operate(&msg_send);
                            if (!DEVICE_STATE(MOVE))
                            {
                                gprs_closereq(DONE);
                            }
                            break;
                            
                        case HEARTBEAT:
                            escort_heartbeat_operate(&msg_send);
                            if (!DEVICE_STATE(MOVE))
                            {
                                gprs_closereq(DONE);
                            }
                            break;
                            
                        case ONWAY:
                            escort_onway_operate(&msg_send);
                            if (!DEVICE_STATE(MOVE))
                            {
                                gprs_closereq(DONE);
                            }
                            break;

                        case ONSTATION:
                            escort_onstation_operate(&msg_send);
                            if (!DEVICE_STATE(MOVE))
                            {
                                gprs_closereq(DONE);
                            }
                            break;
                            
                        default:
                            break;
                    }
                    last_state = mquecom.command;
                    mquecom.func(res);     // 回调函数
                }
                else
                {

                }
            }
            else if ( xActivedMember == que_repeat_data)
            {
                xQueueReceive( xActivedMember, &repeat_msg_send, 0);

                NRF_LOG_PRINTF("receive repeat gprs queue. \r\n");

                msg_send.buf = (uint8_t*)repeat_msg_send.pdata;
                msg_send.len = repeat_msg_send.pdata->len;
            }

            gprs_repeatprocess(res, repeat_msg_send);
        }
    }
}

/**
 * @brief app comm coap init.
*/
void app_comm_init(void)
{ 

    inner_rtc.init(inner_rtc_cfgpara);
    inner_rtc.start(inner_rtc_cfgpara);

    int coappara_size = sizeof(comm_coap_para_t);

    /* Create timers. */
    mgprs_1mintimer = xTimerCreate("mgprs_1mintimer", 60*1000, pdTRUE, NULL, gprs_1mintimer_timeout_handler);
    if(NULL == mgprs_1mintimer)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
      
    /* for gu620 driver */
    sem_gu620 = xSemaphoreCreateBinary();
    if(NULL == sem_gu620)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }  

    /* for rtc 1hour semaphore */
    sem_rtc_repeat1hour = xSemaphoreCreateBinary();
    if(NULL == sem_rtc_repeat1hour)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }  

    que_com_coap = xQueueCreate(QUECOAP_LENGTH, coappara_size);
    if(NULL == que_com_coap)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    } 
    /* init que_repeat_1min */
    que_repeat_1min = xQueueCreate(QUECOAP_LENGTH, sizeof(repeat_que_data_t));
    if(NULL == que_repeat_1min)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    } 
    /* init que_repeat_1hour */
    que_repeat_1hour = xQueueCreate(QUECOAP_LENGTH, sizeof(repeat_que_data_t));
    if(NULL == que_repeat_1hour)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    } 
    /* init que_repeat_data */
    que_repeat_data = xQueueCreate(QUECOAP_LENGTH*2, sizeof(repeat_que_data_t));
    if(NULL == que_repeat_data)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    } 

    queset_comm_coap = xQueueCreateSet(QUESETCOAP_LENGTH);
    if(NULL == queset_comm_coap)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    } 

    xQueueAddToSet(que_com_coap, queset_comm_coap);
    xQueueAddToSet(que_repeat_data, queset_comm_coap);

    /* init comm_coap task */
    if(pdPASS != xTaskCreate(app_comm_coap, "app_comm_coap", 512, NULL, APP_COMM_PRIORITY, NULL))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }    

    /* init 1hour task */
    if(pdPASS != xTaskCreate(app_repeat_1hour, "app_repeat_1hour", 128, NULL, APP_REPEAT1HOUR_PRIORITY, NULL))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }  

}

/* end of file */