/**
* @brief       : 
*
* @file        : device_gprs.h
* @author      : cuihongpeng
* @version     : v0.0.1
* @date        : 2016/7/11
*
* Change Logs  :
*
* Date        Version      Author      Notes
* 2016/7/11    v0.0.1     cuihongpeng    first version
*/

#ifndef __DEVICE_GPRS_H
#define __DEVICE_GPRS_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "nrf_drv_uart.h"
#include "nrf_gpio.h"
#include "boards.h"

#define DK                      (0u)
#define ESCART                  (1u)
#define BOARD_TYPE              ESCART

#define GU620_DEBUG             (0u)

/**
 * @breaf AT CMD
 */
#define  GPRS_AT		        ("AT\r")		            /**< AT测试指令 */
#define  GPRS_ATE0		        ("ATE0\r")	                /**< 关闭回显指令 */
#define  GPRS_ZPPPOPEN		    ("AT+CIICR\r")	            /**< 激活网络  */
#define  GPRS_CGATT             ("AT+CGATT=1\r")            /**< 附着网络指令 */ 
#define  GPRS_CGATT_GET		    ("AT+CGATT?\r")		        /**< 附着网络指令 */
#define  GPRS_RXGET             ("AT+CIPRXGET?\r")          /**< 手动接收数据 */ 
#define  GPRS_ZIPCLOSE          ("AT+CIPCLOSE=1\r")         /**< 关闭链接  */
#define  GPRS_OK		        ("OK\r\n")			        /**< GPRS响应正确 */   

#define  GPS_OPEN               ("AT+GPSPWR=1\r\n")         /**< 打开GPS电源*/ 
#define  GPS_RST                ("AT+GPSRST=2,0\r\n")       /**< 热启动，关闭agps*/  
#define  GPS_SLEEP              ("AT+GPSPWR=2\r\n")         /**< 休眠GPS */
#define  GPS_TYPE_RMC           ("AT+GPSLOC=32\r\n")        /**< 上报GPS RMC*/
#define  GPS_TYPE_LOC           ("AT+GPSLOC=0\r\n")         /**< 上报GPS LOC*/
#define  GPS_REPORT             ("AT+GPSUART=1,0\r\n")      /**< 输出NMEA到第一个串口*/
 
#define  GPS_CLOSE              ("AT+GPSPWR=0\r\n")         /**< 关闭GPS */
#define  GPS_WAKEUP             ("AT+GPSPWR=3\r\n")         /**< 唤醒GPS */
#define  GPS_STOP_REPORT        ("AT+GPSUART=0,0\r\n")      /**< 关闭上报GPS */
#define  GPS_OK                 ("OK\r\n")                  /**< GPS响应正确 */

#define RX_PIN_NUMBER           (14u)
#define TX_PIN_NUMBER           (18u)
#define GU620_PWON              (16u)    
#define GU620_RST               (12u)

#define GPRS_AT_CHECK_MAX_CNT   (2u)
#define GPRS_ATE0_MAX_CNT       (2u)
#define GPRS_GSENSOR_MAX_CNT    (2u)
#define POWER_ON_MAX_CNT        (2u) 
#define ATTACH_NET_MAX_CNT      (60u)  
#define PPP_OPEN_MAX_CNT        (2u)
#define GPRS_CLOSE_MAX_CNT      (3u)
#define GPS_OPEN_MAX_CNT        (2u)
#define GPS_CLOSE_MAX_CNT       (2u) 

#define ATTACH_NET_TIME         (500u)
#define GU620_WAIT_RES_TIME     (2000u)
#define GU620_WAIT_READY_TIME   (10000u)

#define GPRS_NON                (0u)
#define GPRS_CON                (1u)
#define GPRS_CONNECT_CNT        (3u)
#define GPRS_SEND_CNT           (3u)
#define DATA_MAX_LEN		    (256u) /**< 最大接收长度 */

#define bool_t                  bool
#define TRUE                    true
#define FALSE                   false

/**
 *@breaf 循环队列元素定义
 */
typedef struct
{
    uint8_t len;		        /**< 数据长度 */
    uint8_t buff[DATA_MAX_LEN];	/**< 数据缓存 */
}rev_data_t;

/**
* @breaf GPRS网络参数结构体
*/
typedef struct gprs_net_config_
{
    uint8_t *type;
    uint8_t ip[24];
    uint32_t port;
} gprs_net_config_t;

/**
* @breaf GPRS串口信息结构体
*/
typedef struct gprs_uart_info_
{
    uint32_t uart_rev_index;
    rev_data_t rev_data;
} gprs_uart_info_t;

#if 0
typedef struct utc_time_
{
    int     year;       /**< Years since 1900 */
    int     mon;        /**< Months since January - [0,11] */
    int     day;        /**< Day of the month - [1,31] */
    int     hour;       /**< Hours since midnight - [0,23] */
    int     min;        /**< Minutes after the hour - [0,59] */
    int     sec;        /**< Seconds after the minute - [0,59] */
    int     hsec;       /**< Hundredth part of second - [0,99] */
} utc_time_t;
#endif

typedef struct gps_info_t_
{
    // utc_time_t utc;       /**< UTC of position */
    char utc[15];
    double  lat;        /**< Latitude in NDEG - +/-[degree][min].[sec/60] */
    double  lon;        /**< Longitude in NDEG - +/-[degree][min].[sec/60] */
    double  alt;        /**< Antenna altitude above/below mean sea level (geoid) in meters */
    double  speed;      /**< Speed over the ground in kilometers/hour */
    double  direction;  /**< Track angle in degrees True */
    double  spend;
} gps_info_t;

/**
* @breaf GPRS模块接口
*/
typedef struct device_gprs_
{
    bool_t (*open)(void);
    bool_t (*connect)(gprs_net_config_t *gprs_net_config);
    bool_t (*write)(uint8_t *buff, uint8_t len);
    bool_t (*read)(uint8_t *buff, uint8_t *len);
    bool_t (*close_connect)(void);
    bool_t (*close)(void);
    void (*interrupt_cb)(void);
} device_gprs_t;

/**
* @breaf GPS模块接口
*/
typedef struct device_gps_
{
    bool_t (*open)(void);
    bool_t (*report)(void);
    bool_t (*close)(void);
    void (*interrupt_cb)(void);
} device_gps_t;

extern device_gprs_t gprs;
extern device_gps_t gps;
extern gps_info_t gps_info;
extern SemaphoreHandle_t sem_gu620;

void gu620_io_config(void);

#if GU620_DEBUG 
void gu620_raw_print(void);
#endif

#endif
