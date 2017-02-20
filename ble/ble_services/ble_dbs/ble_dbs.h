/* This file was generated by plugin 'Nordic Semiconductor nRF5x v.1.2.2' (BDS version 1.0.2095.0) */

#ifndef BLE_DEBUG_H__
#define BLE_DEBUG_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "app_util_bds.h"



/**@brief DEBUG event type. */
typedef enum
{ 
    BLE_DEBUG_FORCE_LOG_MESSAGE_SUBMIT_EVT_NOTIFICATION_ENABLED,  /**< force LOG message submit value notification enabled event. */
    BLE_DEBUG_FORCE_LOG_MESSAGE_SUBMIT_EVT_NOTIFICATION_DISABLED, /**< force LOG message submit value notification disabled event. */
    BLE_DEBUG_FORCE_LOG_MESSAGE_SUBMIT_EVT_WRITE, /**< force LOG message submit write event. */
    BLE_DEBUG_ADJUST_TIME_EVT_NOTIFICATION_ENABLED,  /**< adjust time value notification enabled event. */
    BLE_DEBUG_ADJUST_TIME_EVT_NOTIFICATION_DISABLED, /**< adjust time value notification disabled event. */
    BLE_DEBUG_ADJUST_TIME_EVT_WRITE, /**< adjust time write event. */
    BLE_DEBUG_SETUP_IP_ADDR_EVT_NOTIFICATION_ENABLED,  /**< setup ip addr value notification enabled event. */
    BLE_DEBUG_SETUP_IP_ADDR_EVT_NOTIFICATION_DISABLED, /**< setup ip addr value notification disabled event. */
    BLE_DEBUG_SETUP_IP_ADDR_EVT_WRITE, /**< setup ip addr write event. */
    BLE_DEBUG_SETUP_IP_PORT_EVT_NOTIFICATION_ENABLED,  /**< setup ip port value notification enabled event. */
    BLE_DEBUG_SETUP_IP_PORT_EVT_NOTIFICATION_DISABLED, /**< setup ip port value notification disabled event. */
    BLE_DEBUG_SETUP_IP_PORT_EVT_WRITE, /**< setup ip port write event. */
} ble_debug_evt_type_t;

// Forward declaration of the ble_debug_t type.
typedef struct ble_debug_s ble_debug_t;








/**@brief force LOG message submit structure. */
typedef struct
{
    uint8_t flogs;
} ble_debug_force_log_message_submit_t;
/**@brief adjust time structure. */
typedef struct
{
    uint32_t adjust_time_value;
} ble_debug_adjust_time_t;
/**@brief setup ip addr structure. */
typedef struct
{
    uint8_array_t ipaddr;
} ble_debug_setup_ip_addr_t;
/**@brief setup ip port structure. */
typedef struct
{
    uint16_t ipport;
} ble_debug_setup_ip_port_t;

/**@brief DEBUG Service event. */
typedef struct
{
    ble_debug_evt_type_t evt_type;    /**< Type of event. */
    union {
        uint16_t cccd_value; /**< Holds decoded data in Notify and Indicate event handler. */
        ble_debug_force_log_message_submit_t force_log_message_submit; /**< Holds decoded data in Write event handler. */
        ble_debug_adjust_time_t adjust_time; /**< Holds decoded data in Write event handler. */
        ble_debug_setup_ip_addr_t setup_ip_addr; /**< Holds decoded data in Write event handler. */
        ble_debug_setup_ip_port_t setup_ip_port; /**< Holds decoded data in Write event handler. */
    } params;
} ble_debug_evt_t;

/**@brief DEBUG Service event handler type. */
typedef void (*ble_debug_evt_handler_t) (ble_debug_t * p_debug, ble_debug_evt_t * p_evt);

/**@brief DEBUG Service init structure. This contains all options and data needed for initialization of the service */
typedef struct
{
    ble_debug_evt_handler_t     evt_handler; /**< Event handler to be called for handling events in the DEBUG Service. */
    ble_debug_force_log_message_submit_t ble_debug_force_log_message_submit_initial_value; /**< If not NULL, initial value of the force LOG message submit characteristic. */ 
    ble_debug_adjust_time_t ble_debug_adjust_time_initial_value; /**< If not NULL, initial value of the adjust time characteristic. */ 
    ble_debug_setup_ip_addr_t ble_debug_setup_ip_addr_initial_value; /**< If not NULL, initial value of the setup ip addr characteristic. */ 
    ble_debug_setup_ip_port_t ble_debug_setup_ip_port_initial_value; /**< If not NULL, initial value of the setup ip port characteristic. */ 
} ble_debug_init_t;

/**@brief DEBUG Service structure. This contains various status information for the service.*/
struct ble_debug_s
{
    ble_debug_evt_handler_t evt_handler; /**< Event handler to be called for handling events in the DEBUG Service. */
    uint16_t service_handle; /**< Handle of DEBUG Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t force_log_message_submit_handles; /**< Handles related to the force LOG message submit characteristic. */
    ble_gatts_char_handles_t adjust_time_handles; /**< Handles related to the adjust time characteristic. */
    ble_gatts_char_handles_t setup_ip_addr_handles; /**< Handles related to the setup ip addr characteristic. */
    ble_gatts_char_handles_t setup_ip_port_handles; /**< Handles related to the setup ip port characteristic. */
    uint16_t conn_handle; /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
};

/**@brief Function for initializing the DEBUG.
 *
 * @param[out]  p_debug       DEBUG Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_debug_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_debug_init(ble_debug_t * p_debug, const ble_debug_init_t * p_debug_init);

/**@brief Function for handling the Application's BLE Stack events.*/
void ble_debug_on_ble_evt(ble_debug_t * p_debug, ble_evt_t * p_ble_evt);

/**@brief Function for setting the force LOG message submit.
 *
 * @details Sets a new value of the force LOG message submit characteristic. The new value will be sent
 *          to the client the next time the client reads the force LOG message submit characteristic.
 *          This function is only generated if the characteristic's Read property is not 'Excluded'.
 *
 * @param[in]   p_debug                 DEBUG Service structure.
 * @param[in]   p_force_log_message_submit  New force LOG message submit.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_debug_force_log_message_submit_set(ble_debug_t * p_debug, ble_debug_force_log_message_submit_t * p_force_log_message_submit);

/**@brief Function for setting the setup ip addr.
 *
 * @details Sets a new value of the setup ip addr characteristic. The new value will be sent
 *          to the client the next time the client reads the setup ip addr characteristic.
 *          This function is only generated if the characteristic's Read property is not 'Excluded'.
 *
 * @param[in]   p_debug                 DEBUG Service structure.
 * @param[in]   p_setup_ip_addr  New setup ip addr.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_debug_setup_ip_addr_set(ble_debug_t * p_debug, ble_debug_setup_ip_addr_t * p_setup_ip_addr);

/**@brief Function for setting the setup ip port.
 *
 * @details Sets a new value of the setup ip port characteristic. The new value will be sent
 *          to the client the next time the client reads the setup ip port characteristic.
 *          This function is only generated if the characteristic's Read property is not 'Excluded'.
 *
 * @param[in]   p_debug                 DEBUG Service structure.
 * @param[in]   p_setup_ip_port  New setup ip port.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_debug_setup_ip_port_set(ble_debug_t * p_debug, ble_debug_setup_ip_port_t * p_setup_ip_port);

#endif //_BLE_DEBUG_H__
