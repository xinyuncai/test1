/* This file was generated by plugin 'Nordic Semiconductor nRF5x v.1.2.2' (BDS version 1.0.2095.0) */

#include "ble_dbs.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_util_bds.h"
#include "ble_defined_uuids.h"

#define OPCODE_LENGTH 1 /**< Length of opcode inside DEBUG packet. */
#define HANDLE_LENGTH 2 /**< Length of handle inside DEBUG packet. */

/* TODO Consider changing the max values if encoded data for characteristic/descriptor is fixed length */ 
#define MAX_FORCE_LOG_MESSAGE_SUBMIT_LEN (BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum size of a transmitted force LOG message submit. */ 
#define MAX_ADJUST_TIME_LEN (BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum size of a transmitted adjust time. */ 
#define MAX_SETUP_IP_ADDR_LEN (BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum size of a transmitted setup ip addr. */ 
#define MAX_SETUP_IP_PORT_LEN (BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum size of a transmitted setup ip port. */ 

/**@brief Function for encoding force LOG message submit.
 *
 * @param[in]   p_force_log_message_submit              force LOG message submit characteristic structure to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t force_log_message_submit_encode(ble_debug_force_log_message_submit_t * p_force_log_message_submit, uint8_t * encoded_buffer)
{
    uint8_t len = 0; 
    encoded_buffer[len++] = p_force_log_message_submit->flogs;
    return len;
}

/**@brief Function for decoding force LOG message submit.
 *
 * @param[in]   data_len              Length of the field to be decoded.
 * @param[in]   p_data                Buffer where the encoded data is stored.
 * @param[out]  p_write_val           Decoded data.
 *
 * @return      Length of the decoded field.
 */
static uint8_t force_log_message_submit_decode(uint8_t data_len, uint8_t * p_data, ble_debug_force_log_message_submit_t * p_write_val)
{
    uint8_t pos = 0;
    p_write_val->flogs = p_data[pos++]; 

    return pos;
} 
/**@brief Function for encoding adjust time.
 *
 * @param[in]   p_adjust_time              adjust time characteristic structure to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t adjust_time_encode(ble_debug_adjust_time_t * p_adjust_time, uint8_t * encoded_buffer)
{
    uint8_t len = 0; 
    len += bds_uint32_encode(&p_adjust_time->adjust_time_value, &encoded_buffer[len]); 
    return len;
}

/**@brief Function for decoding adjust time.
 *
 * @param[in]   data_len              Length of the field to be decoded.
 * @param[in]   p_data                Buffer where the encoded data is stored.
 * @param[out]  p_write_val           Decoded data.
 *
 * @return      Length of the decoded field.
 */
static uint8_t adjust_time_decode(uint8_t data_len, uint8_t * p_data, ble_debug_adjust_time_t * p_write_val)
{
    uint8_t pos = 0;
    pos += bds_uint32_decode((data_len-pos), &p_data[pos], &p_write_val->adjust_time_value); 

    return pos;
} 
/**@brief Function for encoding setup ip addr.
 *
 * @param[in]   p_setup_ip_addr              setup ip addr characteristic structure to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t setup_ip_addr_encode(ble_debug_setup_ip_addr_t * p_setup_ip_addr, uint8_t * encoded_buffer)
{
    uint8_t len = 0; 
    len += bds_uint8_array_encode(&p_setup_ip_addr->ipaddr, &encoded_buffer[len]); 
    return len;
}

/**@brief Function for decoding setup ip addr.
 *
 * @param[in]   data_len              Length of the field to be decoded.
 * @param[in]   p_data                Buffer where the encoded data is stored.
 * @param[out]  p_write_val           Decoded data.
 *
 * @return      Length of the decoded field.
 */
static uint8_t setup_ip_addr_decode(uint8_t data_len, uint8_t * p_data, ble_debug_setup_ip_addr_t * p_write_val)
{
    uint8_t pos = 0;
    pos += bds_uint8_array_decode((data_len-pos), &p_data[pos], &p_write_val->ipaddr); 

    return pos;
} 
/**@brief Function for encoding setup ip port.
 *
 * @param[in]   p_setup_ip_port              setup ip port characteristic structure to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t setup_ip_port_encode(ble_debug_setup_ip_port_t * p_setup_ip_port, uint8_t * encoded_buffer)
{
    uint8_t len = 0; 
    len += bds_uint16_encode(&p_setup_ip_port->ipport, &encoded_buffer[len]); 
    return len;
}

/**@brief Function for decoding setup ip port.
 *
 * @param[in]   data_len              Length of the field to be decoded.
 * @param[in]   p_data                Buffer where the encoded data is stored.
 * @param[out]  p_write_val           Decoded data.
 *
 * @return      Length of the decoded field.
 */
static uint8_t setup_ip_port_decode(uint8_t data_len, uint8_t * p_data, ble_debug_setup_ip_port_t * p_write_val)
{
    uint8_t pos = 0;
    pos += bds_uint16_decode((data_len-pos), &p_data[pos], &p_write_val->ipport); 

    return pos;
} 

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_debug       DEBUG Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_debug_t * p_debug, ble_evt_t * p_ble_evt)
{
    p_debug->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_debug       DEBUG Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_debug_t * p_debug, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_debug->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_debug       DEBUG Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_debug_t * p_debug, ble_gatts_evt_write_t * p_ble_evt)
{
    
    if(p_ble_evt->handle == p_debug->force_log_message_submit_handles.value_handle)
    {
        if(p_debug->evt_handler != NULL)
        {
            ble_debug_evt_t evt;
            evt.evt_type = BLE_DEBUG_FORCE_LOG_MESSAGE_SUBMIT_EVT_WRITE;
            force_log_message_submit_decode(p_ble_evt->len, p_ble_evt->data, &evt.params.force_log_message_submit);
            p_debug->evt_handler(p_debug, &evt);
        }
    }
    if(p_ble_evt->handle == p_debug->adjust_time_handles.value_handle)
    {
        if(p_debug->evt_handler != NULL)
        {
            ble_debug_evt_t evt;
            evt.evt_type = BLE_DEBUG_ADJUST_TIME_EVT_WRITE;
            adjust_time_decode(p_ble_evt->len, p_ble_evt->data, &evt.params.adjust_time);
            p_debug->evt_handler(p_debug, &evt);
        }
    }
    if(p_ble_evt->handle == p_debug->setup_ip_addr_handles.value_handle)
    {
        if(p_debug->evt_handler != NULL)
        {
            ble_debug_evt_t evt;
            evt.evt_type = BLE_DEBUG_SETUP_IP_ADDR_EVT_WRITE;
            setup_ip_addr_decode(p_ble_evt->len, p_ble_evt->data, &evt.params.setup_ip_addr);
            p_debug->evt_handler(p_debug, &evt);
        }
    }
    if(p_ble_evt->handle == p_debug->setup_ip_port_handles.value_handle)
    {
        if(p_debug->evt_handler != NULL)
        {
            ble_debug_evt_t evt;
            evt.evt_type = BLE_DEBUG_SETUP_IP_PORT_EVT_WRITE;
            setup_ip_port_decode(p_ble_evt->len, p_ble_evt->data, &evt.params.setup_ip_port);
            p_debug->evt_handler(p_debug, &evt);
        }
    }
}

/**@brief Authorize WRITE request event handler.
 *
 * @details Handles WRITE events from the BLE stack.
 *
 * @param[in]   p_sc_ctrlpt  SC Ctrlpt structure.
 * @param[in]   p_gatts_evt  GATTS Event received from the BLE stack.
 *
 */
static void on_rw_authorize_request(ble_debug_t * p_debug, ble_gatts_evt_t * p_gatts_evt)
{
    ble_gatts_evt_rw_authorize_request_t * p_auth_req = &p_gatts_evt->params.authorize_request;
    if (p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
    {
        if (   (p_gatts_evt->params.authorize_request.request.write.op
                != BLE_GATTS_OP_PREP_WRITE_REQ)
            && (p_gatts_evt->params.authorize_request.request.write.op
                != BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
            && (p_gatts_evt->params.authorize_request.request.write.op
                != BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL)
           )
        {
        
            if (p_auth_req->request.write.handle == p_debug->adjust_time_handles.value_handle)
            {
                on_write(p_debug, &p_auth_req->request.write);
            }
        }
    }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_debug       DEBUG Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
void ble_debug_on_ble_evt(ble_debug_t * p_debug, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_debug, p_ble_evt);
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_debug, p_ble_evt);
            break;
        case BLE_GATTS_EVT_WRITE:
            on_write(p_debug, &p_ble_evt->evt.gatts_evt.params.write);
            break;
         case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            on_rw_authorize_request(p_debug, &p_ble_evt->evt.gatts_evt);
            break;
        default:
            //No implementation needed.
            break;
    }
}

/**@brief Function for initializing the DEBUG. */
uint32_t ble_debug_init(ble_debug_t * p_debug, const ble_debug_init_t * p_debug_init)
{
    uint32_t err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_debug->evt_handler = p_debug_init->evt_handler;
    p_debug->conn_handle = BLE_CONN_HANDLE_INVALID;
    
    // Add a custom base UUID.
    ble_uuid128_t bds_base_uuid = BLE_DBS;
    uint8_t       uuid_type;
    err_code = sd_ble_uuid_vs_add(&bds_base_uuid, &uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    ble_uuid.type = uuid_type;
    ble_uuid.uuid = BLE_DBS_BASE;
        
    // Add service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_debug->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    } 

    // Add force LOG message submit characteristic
    ble_debug_force_log_message_submit_t force_log_message_submit_initial_value = p_debug_init->ble_debug_force_log_message_submit_initial_value; 

    uint8_t force_log_message_submit_encoded_value[MAX_FORCE_LOG_MESSAGE_SUBMIT_LEN];
    ble_add_char_params_t add_force_log_message_submit_params;
    memset(&add_force_log_message_submit_params, 0, sizeof(add_force_log_message_submit_params));
    
    add_force_log_message_submit_params.uuid                = BLE_DBS_FLOG;
    add_force_log_message_submit_params.uuid_type           = ble_uuid.type; 
    add_force_log_message_submit_params.max_len             = MAX_FORCE_LOG_MESSAGE_SUBMIT_LEN;
    add_force_log_message_submit_params.init_len            = force_log_message_submit_encode(&force_log_message_submit_initial_value, force_log_message_submit_encoded_value);
    add_force_log_message_submit_params.p_init_value        = force_log_message_submit_encoded_value; 
    add_force_log_message_submit_params.char_props.read     = 1; 
    add_force_log_message_submit_params.read_access         = SEC_OPEN; 
    add_force_log_message_submit_params.char_props.write_wo_resp    = 1; 
    add_force_log_message_submit_params.write_access        = SEC_OPEN; 
    // 1 for variable length and 0 for fixed length.
    add_force_log_message_submit_params.is_var_len          = 1; 

    err_code = characteristic_add(p_debug->service_handle, &add_force_log_message_submit_params, &(p_debug->force_log_message_submit_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    } 

    // Add adjust time characteristic
    ble_debug_adjust_time_t adjust_time_initial_value = p_debug_init->ble_debug_adjust_time_initial_value; 

    uint8_t adjust_time_encoded_value[MAX_ADJUST_TIME_LEN];
    ble_add_char_params_t add_adjust_time_params;
    memset(&add_adjust_time_params, 0, sizeof(add_adjust_time_params));
    
    add_adjust_time_params.uuid                = BLE_DBS_ADJUSTTIME;
    add_adjust_time_params.uuid_type           = ble_uuid.type; 
    add_adjust_time_params.max_len             = MAX_ADJUST_TIME_LEN;
    add_adjust_time_params.init_len            = adjust_time_encode(&adjust_time_initial_value, adjust_time_encoded_value);
    add_adjust_time_params.p_init_value        = adjust_time_encoded_value; 
    add_adjust_time_params.char_props.write    = 1; 
    add_adjust_time_params.write_access        = SEC_OPEN; 
    // 1 for variable length and 0 for fixed length.
    add_adjust_time_params.is_var_len          = 1; 

    err_code = characteristic_add(p_debug->service_handle, &add_adjust_time_params, &(p_debug->adjust_time_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    } 

    // Add setup ip addr characteristic
    ble_debug_setup_ip_addr_t setup_ip_addr_initial_value = p_debug_init->ble_debug_setup_ip_addr_initial_value; 

    uint8_t setup_ip_addr_encoded_value[MAX_SETUP_IP_ADDR_LEN];
    ble_add_char_params_t add_setup_ip_addr_params;
    memset(&add_setup_ip_addr_params, 0, sizeof(add_setup_ip_addr_params));
    
    add_setup_ip_addr_params.uuid                = BLE_DBS_IPADDR;
    add_setup_ip_addr_params.uuid_type           = ble_uuid.type; 
    add_setup_ip_addr_params.max_len             = MAX_SETUP_IP_ADDR_LEN;
    add_setup_ip_addr_params.init_len            = setup_ip_addr_encode(&setup_ip_addr_initial_value, setup_ip_addr_encoded_value);
    add_setup_ip_addr_params.p_init_value        = setup_ip_addr_encoded_value; 
    add_setup_ip_addr_params.char_props.read     = 1; 
    add_setup_ip_addr_params.read_access         = SEC_OPEN; 
    add_setup_ip_addr_params.char_props.write_wo_resp    = 1; 
    add_setup_ip_addr_params.write_access        = SEC_OPEN; 
    // 1 for variable length and 0 for fixed length.
    add_setup_ip_addr_params.is_var_len          = 1; 

    err_code = characteristic_add(p_debug->service_handle, &add_setup_ip_addr_params, &(p_debug->setup_ip_addr_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    } 

    // Add setup ip port characteristic
    ble_debug_setup_ip_port_t setup_ip_port_initial_value = p_debug_init->ble_debug_setup_ip_port_initial_value; 

    uint8_t setup_ip_port_encoded_value[MAX_SETUP_IP_PORT_LEN];
    ble_add_char_params_t add_setup_ip_port_params;
    memset(&add_setup_ip_port_params, 0, sizeof(add_setup_ip_port_params));
    
    add_setup_ip_port_params.uuid                = BLE_DBS_IPPORT;
    add_setup_ip_port_params.uuid_type           = ble_uuid.type; 
    add_setup_ip_port_params.max_len             = MAX_SETUP_IP_PORT_LEN;
    add_setup_ip_port_params.init_len            = setup_ip_port_encode(&setup_ip_port_initial_value, setup_ip_port_encoded_value);
    add_setup_ip_port_params.p_init_value        = setup_ip_port_encoded_value; 
    add_setup_ip_port_params.char_props.read     = 1; 
    add_setup_ip_port_params.read_access         = SEC_OPEN; 
    add_setup_ip_port_params.char_props.write_wo_resp    = 1; 
    add_setup_ip_port_params.write_access        = SEC_OPEN; 
    // 1 for variable length and 0 for fixed length.
    add_setup_ip_port_params.is_var_len          = 1; 

    err_code = characteristic_add(p_debug->service_handle, &add_setup_ip_port_params, &(p_debug->setup_ip_port_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    } 

    return NRF_SUCCESS;
}

/**@brief Function for setting the force LOG message submit. */
uint32_t ble_debug_force_log_message_submit_set(ble_debug_t * p_debug, ble_debug_force_log_message_submit_t * p_force_log_message_submit)
{
    ble_gatts_value_t gatts_value;
    uint8_t encoded_value[MAX_FORCE_LOG_MESSAGE_SUBMIT_LEN];

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = force_log_message_submit_encode(p_force_log_message_submit, encoded_value);
    gatts_value.offset  = 0;
    gatts_value.p_value = encoded_value;

    return sd_ble_gatts_value_set(p_debug->conn_handle, p_debug->force_log_message_submit_handles.value_handle, &gatts_value);
}

/**@brief Function for setting the setup ip addr. */
uint32_t ble_debug_setup_ip_addr_set(ble_debug_t * p_debug, ble_debug_setup_ip_addr_t * p_setup_ip_addr)
{
    ble_gatts_value_t gatts_value;
    uint8_t encoded_value[MAX_SETUP_IP_ADDR_LEN];

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = setup_ip_addr_encode(p_setup_ip_addr, encoded_value);
    gatts_value.offset  = 0;
    gatts_value.p_value = encoded_value;

    return sd_ble_gatts_value_set(p_debug->conn_handle, p_debug->setup_ip_addr_handles.value_handle, &gatts_value);
}

/**@brief Function for setting the setup ip port. */
uint32_t ble_debug_setup_ip_port_set(ble_debug_t * p_debug, ble_debug_setup_ip_port_t * p_setup_ip_port)
{
    ble_gatts_value_t gatts_value;
    uint8_t encoded_value[MAX_SETUP_IP_PORT_LEN];

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = setup_ip_port_encode(p_setup_ip_port, encoded_value);
    gatts_value.offset  = 0;
    gatts_value.p_value = encoded_value;

    return sd_ble_gatts_value_set(p_debug->conn_handle, p_debug->setup_ip_port_handles.value_handle, &gatts_value);
}

