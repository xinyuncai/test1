/**
 * @brief       : 
 *
 * @file        : app_ble.h
 * @author      : wangyaoting
 * @version     : v0.0.1
 * @date        : 2016/10/12
 *
 * Change Logs  :
 *
 * Date        Version      Author          Notes
 * 2016/10/12  v0.0.1       wangyaoting     first version
 */
#ifndef __APP_BLE_H
#define __APP_BLE_H

#include "boards.h"

#define MAX_REC_COUNT                        4                       /**< Maximum records count. */
#define IS_SRVC_CHANGED_CHARACT_PRESENT      1                                          /**< Include the Service Changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */
#define CENTRAL_LINK_COUNT                   0                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT                1                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                          "BLE-ESCART"                                 /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                    "NordicSemiconductor"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                     300                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS           30                                         /**< The advertising time-out in units of seconds. */
#define APP_TIMER_PRESCALER                  0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE              4                                          /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL          2000                                       /**< Battery level measurement interval (ms). */
#define MIN_BATTERY_LEVEL                    81                                         /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                    100                                        /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT              1                                          /**< Increment between each simulated battery level measurement. */

#define HEART_RATE_MEAS_INTERVAL             1000                                       /**< Heart rate measurement interval (ms). */
#define MIN_HEART_RATE                       140                                        /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                       300                                        /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_INCREMENT                 10                                         /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

#define RR_INTERVAL_INTERVAL                 300                                        /**< RR interval interval (ms). */
#define MIN_RR_INTERVAL                      100                                        /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_RR_INTERVAL                      500                                        /**< Maximum RR interval as returned by the simulated measurement function. */
#define RR_INTERVAL_INCREMENT                1                                          /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */
#define SENSOR_CONTACT_DETECTED_INTERVAL     5000                                       /**< Sensor Contact Detected toggle interval (ms). */

#define MIN_CONN_INTERVAL                    MSEC_TO_UNITS(400, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                    MSEC_TO_UNITS(650, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                        0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                     MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY       5000                                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY        30000                                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT         3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                       1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                       0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                       0                                          /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                   0                                          /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES            BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                        0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE               7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE               16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                            0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define OSTIMER_WAIT_FOR_QUEUE               2                                          /**< Number of ticks to wait for the timer queue to be ready */
#define BLE_NUS_MAX_DATA_LEN                128
#define UART_TX_BUF_SIZE                    128                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                    128                                         /**< UART RX buffer size. */

#define devName_payload                     "BLE-AB02"
#define devID_payload                       (get_device_id())//"4111201606171192"   
#define RESEND_CNT_MAX 		                (3u)

/**@brief 命令更新参数类型. */
typedef enum
{ 
    NULL_OPERATE,           // 0: null
    STORE_OPERATE,          // 1: put into the warehouse;
    REGISTER_OPERATE,       // 2: register to the web.
    DEBUG_FORCE_LOG,        // 3: debug force log
} cmd_update_type_t;

/**@brief 车押服务信息存储结构体. */
typedef struct
{
    uint8_t vehichle_identify_number[18];
    uint8_t user_info[16];
    uint8_t store_info[16];
    uint8_t escort_state;          //0: null;  1: in transition;   2: in warehouse;    3: in factory;  4: idle.
    uint8_t cmd_update;            //0: null;  1: put into the warehouse;  2: register to the web.
    uint8_t coap_ack_type;
    int ts;
} escort_basis_info_t;
uint8_t coap_send_frame(coap_code code);

extern escort_basis_info_t escort_basis_info;
extern coap_pdu msg_recv;
extern coap_pdu msg_send;

void app_ble_init(void);
void bsp_ble_init(void);
void app_escort_ble_init(void);
static void nfc_init(bool erase_bonds);
uint8_t get_adv_status(void);
void advertising_start(void);
void set_adv_info(void);
#endif