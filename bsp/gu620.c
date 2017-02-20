/**
 * @brief       : 
 *
 * @file        : gu620.c
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
#include "gu620.h"   

static bool_t gprs_open(void);
static bool_t gprs_init(void);
static bool_t gprs_connect(gprs_net_config_t *gprs_net_config);
static bool_t gprs_write(uint8_t *buff, uint8_t len);
static bool_t gprs_read(uint8_t *buff, uint8_t *len);
static bool_t gprs_close_connect(void);
static bool_t gprs_close(void);

static bool_t gps_open(void);
static void gps_success_cb(void);
static bool_t gps_close(void);
static bool_t gps_report(void);

gps_info_t gps_info;
SemaphoreHandle_t sem_gu620;
static uint8_t net_rev_data_len = 0;

/**
 * @breaf GPRS串口信息结构体初始化
 */
static gprs_uart_info_t gprs_uart_info =
{
    .uart_rev_index = 0,
    .rev_data.len = 0,
    .rev_data.buff = {0}
};

/**
 * @breaf GPRS模块接口结构体初始化
 */
device_gprs_t gprs = 
{
    .open = gprs_open,
    .connect = gprs_connect,
    .write = gprs_write,
    .read = gprs_read,
    .close_connect = gprs_close_connect,
    .close = gprs_close,
    .interrupt_cb = NULL,
};

/**
 * @breaf GPRS模块接口结构体初始化
 */
device_gps_t gps = 
{
    .open = gps_open,
    .report = gps_report,
    .close = gps_close,
    .interrupt_cb = NULL,
};

#if GU620_DEBUG
void gu620_raw_print(void)
{
    char temp[256];
    
    temp[gprs_uart_info.rev_data.len] = 0;
    memcpy(temp, gprs_uart_info.rev_data.buff, gprs_uart_info.rev_data.len);
    NRF_LOG_PRINTF("\n"); 
    NRF_LOG_PRINTF("gprs raw data: %d", gprs_uart_info.rev_data.len);
    NRF_LOG_PRINTF(temp);
    NRF_LOG_PRINTF("\n");   
}
#endif

/**
 * @breaf 比较str1的前n个字符串中是否包含str2
 * @param[in] str1 源字符串指针.
 * @param[in] str2 待查找字符串指针.
 * @param[in] length 查找长度.
 *
 * @return TRUE 成功，FALSE 失败.
 */
static bool_t gprs_strncmp(uint8_t *str1, uint8_t *str2, uint16_t length)
{
    uint8_t str2_len = strlen((char const*)str2);
    if(length >= DATA_MAX_LEN)
    {
        return FALSE;
    }
    for(uint8_t str1_start_index = 0; str1_start_index < length; str1_start_index++)
    {
        if(str1[str1_start_index] == str2[0])// 找到头
        {
            for(uint8_t str2_index = 0; str2_index < str2_len; str1_start_index++, str2_index++)// 依次比较
            {
                if((str1[str1_start_index] != str2[str2_index]) || (str1_start_index >= length))
                {
                    break;// 不匹配 跳出
                }
                else
                {
                    if(str2_index == (str2_len - 1))
                    {
                        return TRUE;// 匹配 返回成功
                    }
                }
            }
        }
    }
    return FALSE;
}

/**
 * @breaf GPRS LDO IO
 */
void gu620_io_config(void)
{
    nrf_gpio_cfg_output(GU620_PWON);
    nrf_gpio_cfg_output(GU620_RST);

#if BOARD_TYPE == DK  
    nrf_gpio_pin_set(GU620_PWON);
    nrf_gpio_pin_set(GU620_RST);  
#elif BOARD_TYPE == ESCART 
    nrf_gpio_pin_clear(GU620_PWON);
    nrf_gpio_pin_clear(GU620_RST);    
#endif     
}

/**
 * @breaf GPRS LDO IO ON
 */
static void gprs_device_on(void)
{
#if BOARD_TYPE == DK   
    nrf_gpio_pin_clear(GU620_PWON);
#if DEBUG_TIME
    uint32_t mRTime = nrf_rtc_counter_get(NRF_RTC2);
#endif
    os_delay(1500);
#if DEBUG_TIME
    uint32_t mDiff = nrf_rtc_counter_get(NRF_RTC2) - mRTime;
    NRF_LOG_PRINTF("mdiff is %d . unit:1/8s. \n", mDiff);
#endif
    nrf_gpio_pin_set(GU620_PWON);  
#elif BOARD_TYPE == ESCART     
    nrf_gpio_pin_set(GU620_PWON);
#if DEBUG_TIME
    uint32_t mRTime = nrf_rtc_counter_get(NRF_RTC2);
#endif
    os_delay(1500);
#if DEBUG_TIME
    uint32_t mDiff = nrf_rtc_counter_get(NRF_RTC2) - mRTime;
    NRF_LOG_PRINTF("mdiff is %d . unit:1/8s. \n", mDiff);
#endif
    nrf_gpio_pin_clear(GU620_PWON);    
#endif  
}

/**
 * @breaf GPRS LDO IO OFF
 */
static void gprs_device_off(void)
{
#if BOARD_TYPE == DK     
    nrf_gpio_pin_clear(GU620_PWON);
#if DEBUG_TIME
    uint32_t mRTime = nrf_rtc_counter_get(NRF_RTC2);
#endif
    os_delay(2500);
#if DEBUG_TIME
    uint32_t mDiff = nrf_rtc_counter_get(NRF_RTC2) - mRTime;
    NRF_LOG_PRINTF("mdiff is %d . unit:1/8s. \n", mDiff);
#endif
    nrf_gpio_pin_set(GU620_PWON);
#elif BOARD_TYPE == ESCART    
    nrf_gpio_pin_set(GU620_PWON);
#if DEBUG_TIME
    uint32_t mRTime = nrf_rtc_counter_get(NRF_RTC2);
#endif
    os_delay(2500);
#if DEBUG_TIME
    uint32_t mDiff = nrf_rtc_counter_get(NRF_RTC2) - mRTime;
    NRF_LOG_PRINTF("mdiff is %d . unit:1/8s. \n", mDiff);
#endif
    nrf_gpio_pin_clear(GU620_PWON); 
#endif 
    
    os_delay(5000); // 
}

#if 0
/**
 * @breaf GPRS RESET
 */
static void gprs_device_reset(void)
{
    nrf_gpio_pin_clear(GU620_RST);
    os_delay(1);
    nrf_gpio_pin_set(GU620_RST); 
}
#endif

/**
 * @breaf gprs发送AT命令
 * @param[in] cmd 待发送命令指针.
 * @param[in] length 发送长度.
 */
static void gprs_send_at_cmd(uint8_t *cmd, uint8_t length)
{
#if GU620_DEBUG    
    NRF_LOG_PRINTF("write at cmd:");
    NRF_LOG_PRINTF((char *)cmd);
#endif    
    nrf_drv_uart_tx(cmd, length);    
}

/**
 * @breaf 串口接收中断回调函数
 * @param[in] serial_id 串口ID.
 * @param[in] ch 接收字符.
 */
void uart_receive_cb(uint8_t ch)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE; 
    gprs_uart_info.rev_data.len++;
    gprs_uart_info.rev_data.buff[gprs_uart_info.uart_rev_index++] = ch;
    if(gprs_uart_info.uart_rev_index >= DATA_MAX_LEN)
    {
        gprs_uart_info.uart_rev_index = 0;
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/**
 * @breaf gprs ready cb
 */
static void gprs_ready_cb(void)
{
    gprs_uart_info.uart_rev_index = 0;
    uint8_t rev_temp = gprs_uart_info.rev_data.len;
    gprs_uart_info.rev_data.len = 0;
    if(gprs_strncmp((uint8_t *)gprs_uart_info.rev_data.buff, "@@Ver", rev_temp) == TRUE)
    {
        // xSemaphoreGive(sem_gu620);
        static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(sem_gu620, &xHigherPriorityTaskWoken);
    }
}

/**
 * @breaf gprs初始化回调函数
 */
static void gprs_success_cb(void)
{
    gprs_uart_info.uart_rev_index = 0;
    uint8_t rev_temp = gprs_uart_info.rev_data.len;
    gprs_uart_info.rev_data.len = 0;
    if(gprs_strncmp((uint8_t *)gprs_uart_info.rev_data.buff, GPRS_OK, rev_temp) == TRUE)
    {
        // xSemaphoreGive(sem_gu620);
        static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(sem_gu620, &xHigherPriorityTaskWoken);
    }
}

/**
 * @breaf gprs连接网络回调函数
 */
static void gprs_connect_cb(void)
{
    gprs_uart_info.uart_rev_index = 0;
    uint8_t rev_temp = gprs_uart_info.rev_data.len;
    gprs_uart_info.rev_data.len = 0;
    if((gprs_strncmp((uint8_t *)gprs_uart_info.rev_data.buff, "OK", rev_temp) == TRUE)
       || (gprs_strncmp((uint8_t *)gprs_uart_info.rev_data.buff, "CME ERROR: 2", rev_temp) == TRUE))
    {
          // xSemaphoreGive(sem_gu620);
        static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(sem_gu620, &xHigherPriorityTaskWoken);
    }
}

/**
 * @breaf gprs发送网络数据命令回调函数
 */
static void gprs_send_data_cmd_cb(void)
{
    gprs_uart_info.uart_rev_index = 0;
    uint8_t rev_temp = gprs_uart_info.rev_data.len;
    gprs_uart_info.rev_data.len = 0;
    if(gprs_strncmp((uint8_t *)gprs_uart_info.rev_data.buff, ">", rev_temp) == TRUE)
    {
        // xSemaphoreGive(sem_gu620);
        static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(sem_gu620, &xHigherPriorityTaskWoken);
    }
}

/**
 * @breaf gprs发送网络数据命令回调函数
 */
static void gprs_send_data_cb(void)
{ 
    gprs_uart_info.uart_rev_index = 0;
    uint8_t rev_temp = gprs_uart_info.rev_data.len;
    gprs_uart_info.rev_data.len = 0;
    if(gprs_strncmp((uint8_t *)gprs_uart_info.rev_data.buff, "SEND OK", rev_temp) == TRUE)
    {
        // xSemaphoreGive(sem_gu620);
        static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(sem_gu620, &xHigherPriorityTaskWoken);
    }
}

/**
 * @breaf gprs网络数据接收回调函数
 */
static void gprs_rev_data_cb(void)
{    
    //+IPD: 接收长度有待优化
    gprs_uart_info.uart_rev_index = 0;
    uint8_t rev_temp = gprs_uart_info.rev_data.len;
    gprs_uart_info.rev_data.len = 0;
    
    if(gprs_strncmp((uint8_t *)gprs_uart_info.rev_data.buff, "+IPD,", rev_temp) == TRUE)
    {
        net_rev_data_len = rev_temp;
        // xSemaphoreGive(sem_gu620);
        static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(sem_gu620, &xHigherPriorityTaskWoken);
    }
}

/**
 * @breaf gprs初始化回调函数
 */
static void gprs_csq_cb(void)
{
    gprs_uart_info.uart_rev_index = 0;
    uint8_t rev_temp = gprs_uart_info.rev_data.len;
    gprs_uart_info.rev_data.len = 0;
    if(gprs_strncmp((uint8_t *)gprs_uart_info.rev_data.buff, "+CSQ:", rev_temp) == TRUE)
    {
        extern uint8_t csq[20];
        memcpy(csq, gprs_uart_info.rev_data.buff + 3, 11);
        // xSemaphoreGive(sem_gu620);
        static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(sem_gu620, &xHigherPriorityTaskWoken);
    }
}

/**
 * @breaf gprs 信号质量查询
 * @return TRUE 成功，FALSE 失败.
 */
bool_t gprs_csq_check(void)
{
    gprs.interrupt_cb = gprs_csq_cb;
    gprs_send_at_cmd("AT+CSQ\r", strlen("AT+CSQ\r"));
    if(xSemaphoreTake(sem_gu620, GU620_WAIT_RES_TIME) == pdTRUE)
    {
        return TRUE;
    }
    return FALSE;
}

/**
 * @breaf gprs串口检测
 * @return TRUE 成功，FALSE 失败.
 */
static bool_t gprs_at_check(void)
{
    gprs.interrupt_cb = gprs_success_cb;
    gprs_send_at_cmd(GPRS_AT, strlen(GPRS_AT));
    if(xSemaphoreTake(sem_gu620, GU620_WAIT_RES_TIME) == pdTRUE)
    {
        return TRUE;
    }
    return FALSE;
}

/**
 * @breaf gprs串口检测
 * @return TRUE 成功，FALSE 失败.
 */
static bool_t gprs_uart_check(void)
{
    uint8_t gprs_uart_check_cnt = 0;
    while(gprs_at_check() == FALSE)
    {        
        if(gprs_uart_check_cnt++ >= GPRS_AT_CHECK_MAX_CNT)
        { 
            return FALSE;
        }
    }
    return TRUE;
}

/**
 * @breaf gprs关闭回显
 * @return TRUE 成功，FALSE 失败.
 */
static bool_t gprs_ready(void)
{
   gprs.interrupt_cb = gprs_ready_cb;  
   if(xSemaphoreTake(sem_gu620, GU620_WAIT_READY_TIME) == pdTRUE)
   {
       return TRUE;
   }
   return FALSE;
}

/**
 * @breaf gprs关闭回显
 * @return TRUE 成功，FALSE 失败.
 */
static bool_t gprs_uart_ate0(void)
{
    gprs.interrupt_cb = gprs_success_cb;
    gprs_send_at_cmd(GPRS_ATE0, strlen(GPRS_ATE0));
    if(xSemaphoreTake(sem_gu620, GU620_WAIT_RES_TIME ) == pdTRUE)
    {
        return TRUE;
    }
    return FALSE;
}

/**
 * @breaf gprs关闭回显
 * @return TRUE 成功，FALSE 失败.
 */
static bool_t gprs_gsensor_close(void)
{
    gprs.interrupt_cb = gprs_success_cb;
    gprs_send_at_cmd("AT+GSUSE=0\r", strlen("AT+GSUSE=0\r"));
    if(xSemaphoreTake(sem_gu620, GU620_WAIT_RES_TIME ) == pdTRUE)
    {
        return TRUE;
    }
    return FALSE;
}

/**
 * @breaf gprs SIM卡检测
 * @return TRUE 成功，FALSE 失败.
 */
static bool_t gprs_pppopen(void)
{
    gprs.interrupt_cb = gprs_success_cb;  
    gprs_send_at_cmd(GPRS_ZPPPOPEN, strlen(GPRS_ZPPPOPEN));
    if(xSemaphoreTake(sem_gu620, GU620_WAIT_RES_TIME) == pdTRUE)
    {
        return TRUE;
    }
    return FALSE;
}

/**
 * @breaf gprs附着网络
 * @return TRUE 成功，FALSE 失败.
 */
static bool_t gprs_attach_net(void)
{
    gprs.interrupt_cb = gprs_success_cb;
    gprs_send_at_cmd(GPRS_CGATT, strlen(GPRS_CGATT));
    if(xSemaphoreTake(sem_gu620, ATTACH_NET_TIME) == pdTRUE)
    {
        return TRUE;
    }
    return FALSE;
}

/**
 * @breaf gprs发送网络数据命令
 * @param[in] len 待发送数据长度
 * @return TRUE 成功，FALSE 失败.
 */
static bool_t gprs_send_data_cmd(uint8_t len)
{
    uint8_t send_data_cmd[64];
    memset(send_data_cmd,0,sizeof(send_data_cmd)/sizeof(send_data_cmd[0]));
    sprintf((char *)send_data_cmd, "AT+CIPSEND=%d\x00D", len);
    gprs.interrupt_cb = gprs_send_data_cmd_cb;
    gprs_send_at_cmd(send_data_cmd, strlen((char const*)send_data_cmd));
    if(xSemaphoreTake(sem_gu620, GU620_WAIT_RES_TIME) == pdTRUE)
    {
        return TRUE;
    }
    return FALSE;
}

/**
 * @breaf gprs发送网络数据
 * @param[in] pyload 待发送数据指针
 * @param[in] len 待发送数据长度
 * @return TRUE 成功，FALSE 失败.
 */
static bool_t gprs_send_data(uint8_t *pyload, uint8_t len)
{
    gprs.interrupt_cb = gprs_send_data_cb;
    gprs_send_at_cmd(pyload, len);
    if(xSemaphoreTake(sem_gu620, GU620_WAIT_RES_TIME) == pdTRUE)
    {
#if GU620_DEBUG
		NRF_LOG_PRINTF("gu620 wait for data:\n");
#endif
        gprs.interrupt_cb = gprs_rev_data_cb;
        return TRUE;
    }
    return FALSE;
}

/**
 * @breaf gprs模块读取网络数据
 * @param[in] buff 读取缓冲区指针
 * @param[in] len 读取缓冲区长度地址
 * @return TRUE 成功，FALSE 失败.
 */
static bool_t gprs_read(uint8_t *buff, uint8_t *len)
{
    if(xSemaphoreTake(sem_gu620, 5000) == pdTRUE)
    {
        uint8_t l_index, len[4];
        uint8_t *p_len = NULL;
        p_len = (uint8_t *)strstr((char const *)gprs_uart_info.rev_data.buff, ":");
        for(l_index = 0; l_index < p_len - gprs_uart_info.rev_data.buff - 5; l_index++)
        {
            if((l_index >= 4) || (5 + l_index >= 256))
            {
                return FALSE;
            }
            len[l_index] = gprs_uart_info.rev_data.buff[5 + l_index];
        }
        net_rev_data_len = atoi((char const *)len);
        memcpy(len, &net_rev_data_len, 1);
        memcpy(buff, gprs_uart_info.rev_data.buff + 8, net_rev_data_len);
        return TRUE;
    }
    return FALSE;  
}

static bool_t gprs_power_on(void)
{   
    uint8_t gprs_ate0_cnt = 0;
    uint8_t gprs_gsensor_close_cnt = 0;

    if(gprs_uart_check() == FALSE)
    {
        gprs_device_on();
        // GPRS模块准备就绪
        if(gprs_ready() == FALSE)
        {
            NRF_LOG_PRINTF("gprs not ready!\r\n");
            return FALSE;
        }
        os_delay(5000);

        // 关闭回显
        while(gprs_uart_ate0() == FALSE)
        {        
            if(gprs_ate0_cnt++ >= GPRS_ATE0_MAX_CNT)
            { 
                NRF_LOG_PRINTF("gprs ate0 failed!\r\n");
                return FALSE;
            }
        }

        // 关闭gsensor
        while(gprs_gsensor_close() == FALSE)
        {         
            if(gprs_gsensor_close_cnt++ >= GPRS_GSENSOR_MAX_CNT)
            { 
                NRF_LOG_PRINTF("gsensor close failed!\r\n");
                return FALSE;
            }        
        }
    }    

    return TRUE;
}

/**
 * @breaf 打开gprs模块
 * @return TRUE 成功，FALSE 失败.
 */
static bool_t gprs_open(void)
{ 
    uint8_t power_on_cnt = 0;
    bsp_uart_init();

    NRF_LOG_PRINTF("gprs open start!\n");
    
    while(gprs_power_on() == FALSE)
    {
        if(power_on_cnt++ >= POWER_ON_MAX_CNT)
        {
            // 电源异常
            NRF_LOG_PRINTF("gprs power on failed!\r\n");
            return FALSE;
        }
    }
    return gprs_init();
}

/**
 * @breaf 初始化gprs模块
 * @return TRUE 成功，FALSE 失败.
 */
static bool_t gprs_init(void)
{
    uint8_t attach_net_cnt = 0;
    uint8_t ppp_open_cnt = 0;

    // 附着网络
    while(gprs_attach_net() == FALSE)
    {
        if(attach_net_cnt++ >= ATTACH_NET_MAX_CNT)
        {
            // 附着网络失败
            NRF_LOG_PRINTF("gprs attach net failed!\r\n");
            gprs.close();
            return FALSE;
        }
    }
    
    // 打开数据链接
    while(gprs_pppopen() == FALSE)
    {
        if(ppp_open_cnt++ >= PPP_OPEN_MAX_CNT)
        {
            NRF_LOG_PRINTF("gprs ppp failed!\r\n");
            gprs.close();
            return FALSE;   
        }
    }

    NRF_LOG_PRINTF("gprs open success!\r\n");
    return TRUE;
}

/**
 * @breaf gprs模块连接网络
 * @param[in] gprs_net_config gprs网络参数结构体指针
 * @return TRUE 成功，FALSE 失败.
 */
static bool_t gprs_connect(gprs_net_config_t *gprs_net_config)
{
    uint8_t net_config[50];
    memset(net_config, 0, sizeof(net_config)/sizeof(net_config[0]));
    sprintf((char*)net_config, "AT+CIPSTART=\"%s\",\"%s\",%d\x00D",gprs_net_config->type, gprs_net_config->ip, gprs_net_config->port);
    gprs.interrupt_cb = gprs_connect_cb;    
    for(int i = 0; i <= GPRS_CONNECT_CNT; i++)
    {
        gprs_send_at_cmd(net_config, strlen((char const*)net_config));
        if(xSemaphoreTake(sem_gu620, GU620_WAIT_RES_TIME) == pdTRUE)
        {
            return TRUE;
        }  
    }
    return FALSE;
}

/**
 * @breaf gprs发送网络数据
 * @param[in] pyload 待发送数据指针
 * @param[in] len 待发送数据长度
 * @return TRUE 成功，FALSE 失败.
 */
static bool_t gprs_write(uint8_t *buff, uint8_t len)
{
    for(int i = 0; i <= GPRS_SEND_CNT; i++)
    {
        if((gprs_send_data_cmd(len) == FALSE) || (gprs_send_data(buff, len) == FALSE))
        {
            return FALSE;
        }  
        break;
    }
    return TRUE;
}

/**
 * @breaf gprs模块读取网络数据
 * @param[in] buff 读取缓冲区指针
 * @param[in] len 读取缓冲区长度地址
 * @return TRUE 成功，FALSE 失败.
 */
static bool_t gprs_close_connect(void)
{
    gprs.interrupt_cb = gprs_success_cb;
    gprs_send_at_cmd(GPRS_ZIPCLOSE, strlen(GPRS_ZIPCLOSE));
    if(xSemaphoreTake(sem_gu620, GU620_WAIT_RES_TIME) == pdTRUE)
    {
        return TRUE;
    }
    return FALSE;
}

/**
 * @breaf 关闭gprs模块
 * @return TRUE 成功，FALSE 失败.
 */
static bool_t gprs_close(void)
{  
    uint8_t gprs_close_cnt = 0;
    while(gprs_close_cnt <= GPRS_CLOSE_MAX_CNT)
    {
        if(gprs_uart_check() == TRUE)
        {
            gprs_close_cnt++;
            gprs_device_off();
        }
        else
        {
            nrf_drv_uart_uninit();
            NRF_LOG_PRINTF("gprs close success [%d]!\n", gprs_close_cnt);
            return TRUE;
        }
    }
    NRF_LOG_PRINTF("gprs close failed!\n");
    return FALSE;
}

/**
 * @breaf gps open callback
 */
static void gps_success_cb(void)
{
    gprs_uart_info.uart_rev_index = 0;
    uint8_t rev_temp = gprs_uart_info.rev_data.len;
    gprs_uart_info.rev_data.len = 0;
    if(gprs_strncmp((uint8_t *)gprs_uart_info.rev_data.buff, GPS_OK, rev_temp) == TRUE)
    {
          // xSemaphoreGive(sem_gu620);
        static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(sem_gu620, &xHigherPriorityTaskWoken);
    }
} 

double gps_format(double ndegree)
{
    int degree = (int)(ndegree / 100);
    double min = (ndegree / 100 - degree) / 60 * 100;
    return degree + min;
}


// +GPSLOC: 0,3129.6789,12022.4019,9.2,20160919060850.000,3,12,0.01,0.00,34
void gprs_loc_parse(uint8_t *buff, uint8_t len, gps_info_t *gps_info)
{
    char *ptr;

    strtok((char *)buff, ",");// mode

    ptr = strtok(NULL, ",");// lat
    gps_info->lat = gps_format(atof(ptr));
    
    ptr = strtok(NULL, ",");// lon 
    gps_info->lon = gps_format(atof(ptr));
    
    ptr = strtok(NULL, ",");// alt   
    gps_info->alt = atof(ptr);   

    ptr = strtok(NULL, ",");// utc
    memcpy(gps_info->utc, ptr, 14);

    ptr = strtok(NULL, ",");// fix
    ptr = strtok(NULL, ",");// num

    ptr = strtok(NULL, ",");// speed
    gps_info->speed = atof(ptr);
    
    ptr = strtok(NULL, ",");// direction
    gps_info->direction = atof(ptr);  

    ptr = strtok(NULL, ",");// spend
    gps_info->spend = atof(ptr); 
}

/**
 * @breaf gps data receive callback
 */
static void gps_type_loc_cb(void)
{
    gprs_uart_info.uart_rev_index = 0;
    uint8_t rev_temp = gprs_uart_info.rev_data.len;
    gprs_uart_info.rev_data.len = 0;
    if((gprs_strncmp((uint8_t *)gprs_uart_info.rev_data.buff, "+GPSLOC:", rev_temp) == TRUE))
    {
        // 解析经纬度
        gprs_loc_parse(gprs_uart_info.rev_data.buff, rev_temp, &gps_info);
        // xSemaphoreGive(sem_gu620);
        static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(sem_gu620, &xHigherPriorityTaskWoken);
    }
}
/**
 * @breaf report gps
 */
static bool_t gps_type_loc(void)
{
    gprs.interrupt_cb = gps_type_loc_cb;
    gprs_send_at_cmd(GPS_TYPE_LOC, strlen(GPS_TYPE_LOC));
    if(xSemaphoreTake(sem_gu620, GU620_WAIT_RES_TIME) == pdTRUE)
    {
        return TRUE;  
    }
    return FALSE; 
}

/**
 * @breaf report gps
 */
bool_t gps_report(void)
{
    if(gps_type_loc() == TRUE)
    {        
        return TRUE;
    }
    return FALSE;   
}

/**
 * @breaf 获取最新的GPS定位结果
 */
bool_t gps_get_info(gps_info_t *info)
{
    memcpy((void*)info, (void *)&gps_info, sizeof(gps_info_t));
    return TRUE;
}

static bool_t gps_init(void)
{
    gprs.interrupt_cb = gps_success_cb;
    gprs_send_at_cmd(GPS_OPEN, strlen(GPS_OPEN));
    if(xSemaphoreTake(sem_gu620, GU620_WAIT_RES_TIME) == pdTRUE)
    {
        return TRUE;
    }
    return FALSE;    
}

/**
 * @breaf open gps
 */
static bool_t gps_open(void)
{
    uint8_t gps_open_cnt = 0;
    
    while(gps_init() == FALSE)
    {
        if(gps_open_cnt++ >= GPS_OPEN_MAX_CNT)
        {
            // gps open failed!
            NRF_LOG_PRINTF("gps open failed!\r\n");
            return FALSE;
        }
    }
    NRF_LOG_PRINTF("gps open success!\r\n");
    return TRUE;
}

static bool_t gps_off(void)
{
    gprs.interrupt_cb = gps_success_cb;
    gprs_send_at_cmd(GPS_CLOSE, strlen(GPS_CLOSE));
    if(xSemaphoreTake(sem_gu620, GU620_WAIT_RES_TIME) == pdTRUE)
    {
        return TRUE;
    }
    return FALSE;     
}

/**
 * @breaf close gps
 */
static bool_t gps_close(void)
{
    uint8_t gps_close_cnt = 0;
    while(gps_off() == FALSE)
    {
        if(gps_close_cnt++ >= GPS_CLOSE_MAX_CNT)
        {
            // gps close failed!
            NRF_LOG_PRINTF("gps close failed!\r\n");
            return FALSE;
        }
    }
    NRF_LOG_PRINTF("gps close success!\r\n");
    return TRUE;
}
