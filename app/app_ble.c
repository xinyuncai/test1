
#include "app.h"

#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"

/**< 自定义蓝牙服务 */
#include "ble_bas.h"
#include "ble_dbs.h"
#include "ble_ebs.h"
#include "ble_conn_params.h"

#include "hal_nfc_t2t.h"
#include "nfc_t2t_lib.h"
#include "nfc_ndef_msg.h"
#include "nfc_text_rec.h"

static uint16_t                              m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
static ble_bas_t                             m_bas;                                     /**< Structure used to identify the battery service. */
static ble_escort_basis_t                    m_escort_basis;                            /**< Structure used to identify the escort basis service. */
static ble_debug_t                           m_debug;                                   /**< Structure used to identify the debug service. */
//static sensorsim_cfg_t                       m_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
//static sensorsim_state_t                     m_battery_sim_state;                       /**< Battery Level sensor simulator state. */
static dm_application_instance_t             m_app_handle;                              /**< Application identifier allocated by device manager. */
volatile static uint8_t                      oder_id_status=0xff;
static ble_uuid_t                            m_adv_uuids[] =                            /**< Universally unique service identifiers. */
{
    {BLE_UUID_BATTERY_SERVICE,               BLE_UUID_TYPE_BLE},
};

static SemaphoreHandle_t                     m_ble_event_ready;                         /**< Semaphore raised if there is a new event to be processed in the BLE thread. */
static TaskHandle_t                          m_ble_stack_thread;                        /**< Definition of BLE stack thread. */

uint8_t ndef_msg_buf[256];
static uint8_t MAC_payload[17];                                                         // NFC MAC Adderss.
static uint8_t vehichleID_payload[17] = {0,};
uint8_t cmd_update_type;
static uint8_t now_adv_status = 0;

/**
* @brief 命令更新响应函数.
 */
static void on_cmd_update(void)
{
#ifdef PCBA_TEST
    xSemaphoreGive(sem_escort_pcba);
#else
    switch(cmd_update_type)
    {
        case BIND:
            if(true == get_remove_bar_state() && true == adxl345.is_init())
                fsm.tran(BINDED);
            else
                NRF_LOG_PRINTF("remove bar up or adxl345 uninit!");
            break;

        case STATIONARY:
            fsm.tran(STOP);
            break;

        case MOVEMENT:
            fsm.tran(MOVE);
            break;

        case REMOVE:
            fsm.tran(REMOVED);
            break;

        case ONWAY:
            fsm.tran(WAY);
            break;
			
        case ONSTATION:
            fsm.tran(STATION);
            break;
	}
#endif
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**
* @brief 修改并发送广播信息.
 */
static void advertising_init(void);
void set_adv_info(void)
{
  	advertising_init();
	ble_advertising_start(BLE_ADV_MODE_FAST);
}

/**
* @brief 查询ble是否在广播.
 */
uint8_t get_adv_status(void)
{
    return now_adv_status;
}

/**
* @brief BLE开启广播.
 */
void advertising_start(void)
{
  	if(now_adv_status == 0)
	{
		NRF_LOG_PRINTF("start ble adv!\r\n");
		ble_advertising_start(BLE_ADV_MODE_FAST);
	}
    else
    {
        NRF_LOG_PRINTF("the ble is advertising!\r\n");
    }
}

/**
 * @brief Function for nfc callback.
 */
static void nfc_callback(void * context, NfcEvent event, const char *data, size_t dataLength)
{
    (void) context;
    
    switch (event)
    {
        case NFC_EVENT_FIELD_ON:
            if(true == get_remove_bar_state())
                advertising_start();
            else
                NRF_LOG_PRINTF("remove bar state is up!");
            break;

        case NFC_EVENT_FIELD_OFF:
            break;

        default:
            break;
    }
    return;
}
/**
 * @brief Function for creating a MAC adderss record.
 */
static void MAC_payload_add(nfc_ndef_msg_desc_t * p_ndef_msg_desc)
{
    uint32_t             err_code;
    uint8_t              mac_address[7];        //BLE MAC address,mac_address[0] MAC Address Type;
    static const uint8_t en_code[] = {'e', 'n'};

    // Get BLE address
    err_code = sd_ble_gap_address_get((ble_gap_addr_t*)mac_address);
    sprintf((char*)MAC_payload,"%02X:%02X:%02X:%02X:%02X:%02X", mac_address[6], 
            mac_address[5],mac_address[4],mac_address[3],mac_address[2],mac_address[1]);
    
    NFC_NDEF_TEXT_RECORD_DESC_DEF(en_text_rec,
                                  UTF_8,
                                  en_code,
                                  sizeof(en_code),
                                  MAC_payload,
                                  sizeof(MAC_payload));

    err_code = nfc_ndef_msg_record_add(p_ndef_msg_desc,
                                       &NFC_NDEF_TEXT_RECORD_DESC(en_text_rec));
    APP_ERROR_CHECK(err_code);
 
}
/**
 * @brief Function for creating a device ID record.
 */
static void devID_payload_add(nfc_ndef_msg_desc_t * p_ndef_msg_desc)
{
    uint32_t             err_code;
    static const uint8_t en_code[] = {'e', 'n'};

    NFC_NDEF_TEXT_RECORD_DESC_DEF(en_text_rec,
                                  UTF_8,
                                  en_code,
                                  sizeof(en_code),
                                  (uint8_t*)devID_payload,
                                  strlen((char const *)devID_payload));

    err_code = nfc_ndef_msg_record_add(p_ndef_msg_desc,
                                       &NFC_NDEF_TEXT_RECORD_DESC(en_text_rec));
    APP_ERROR_CHECK(err_code);
 
}
/**
 * @brief Function for creating a device name record.
 */
static void devName_payload_add(nfc_ndef_msg_desc_t * p_ndef_msg_desc)
{
    uint32_t             err_code;
    static const uint8_t en_code[] = {'e', 'n'};

    NFC_NDEF_TEXT_RECORD_DESC_DEF(en_text_rec,
                                  UTF_8,
                                  en_code,
                                  sizeof(en_code),
                                  devName_payload,
                                  strlen((char const *)devName_payload));

    err_code = nfc_ndef_msg_record_add(p_ndef_msg_desc,
                                       &NFC_NDEF_TEXT_RECORD_DESC(en_text_rec));
    APP_ERROR_CHECK(err_code);
 
}
/**
 * @brief Function for creating a device name record.
 */
static void vehichleID_payload_add(nfc_ndef_msg_desc_t * p_ndef_msg_desc)
{
    uint32_t             err_code;
    static const uint8_t en_code[] = {'e', 'n'};

    NFC_NDEF_TEXT_RECORD_DESC_DEF(en_text_rec,
                                  UTF_8,
                                  en_code,
                                  sizeof(en_code),
                                  vehichleID_payload,
                                  sizeof(vehichleID_payload));

    err_code = nfc_ndef_msg_record_add(p_ndef_msg_desc,
                                       &NFC_NDEF_TEXT_RECORD_DESC(en_text_rec));
    APP_ERROR_CHECK(err_code);
 
}
/**
 * @brief Function for encoding the NFC message.
 */
static void NFC_msg_encode(uint8_t * p_buffer, uint32_t * p_len)
{
    NFC_NDEF_MSG_DEF(nfc_msg, MAX_REC_COUNT);

    nfc_ndef_msg_clear(&NFC_NDEF_MSG(nfc_msg));
    MAC_payload_add(&NFC_NDEF_MSG(nfc_msg));
    devID_payload_add(&NFC_NDEF_MSG(nfc_msg));
    devName_payload_add(&NFC_NDEF_MSG(nfc_msg));
    vehichleID_payload_add(&NFC_NDEF_MSG(nfc_msg));

    /** @snippet [NFC text usage_2] */
    uint32_t err_code = nfc_ndef_msg_encode(&NFC_NDEF_MSG(nfc_msg),
                                            p_buffer,
                                            p_len);
    APP_ERROR_CHECK(err_code);
    /** @snippet [NFC text usage_2] */
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the escort basis events.
 *
 * @details This function will be called for all escort basis events which are passed to
 *          the application.
 *
 * @param[in]   p_escort_basis   escort_basis structure.
 * @param[in]   p_evt   Event received from the escort basis.
 */
static void on_escort_basis_evt(ble_escort_basis_t * p_escort_basis, ble_escort_basis_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    { 
        case BLE_ESCORT_BASIS_VEHICHLE_IDENTIFY_NUMBER_EVT_WRITE:
            memset(escort_basis_info.vehichle_identify_number, 0, sizeof(escort_basis_info.vehichle_identify_number) / sizeof(escort_basis_info.vehichle_identify_number[0]));
            memcpy(escort_basis_info.vehichle_identify_number, p_evt->params.vehichle_identify_number.vin.p_data, p_evt->params.vehichle_identify_number.vin.size);
            memcpy(vehichleID_payload, escort_basis_info.vehichle_identify_number, p_evt->params.vehichle_identify_number.vin.size);
            nfcStopEmulation();
            nfcDone();
            nfc_init(true);
            break;
            
        case BLE_ESCORT_BASIS_USER_INFO_EVT_WRITE:
            memset(escort_basis_info.user_info, 0, sizeof(escort_basis_info.user_info) / sizeof(escort_basis_info.user_info[0]));
            memcpy(escort_basis_info.user_info, p_evt->params.user_info.userid.p_data,  p_evt->params.user_info.userid.size);
            break;
            
        case BLE_ESCORT_BASIS_WAREHOUSE_INFO_EVT_WRITE:
            memset(escort_basis_info.store_info, 0, sizeof(escort_basis_info.store_info) / sizeof(escort_basis_info.store_info[0]));
            memcpy(escort_basis_info.store_info, p_evt->params.warehouse_info.warehouseinfo.p_data,  p_evt->params.warehouse_info.warehouseinfo.size);
            break;
            
        case BLE_ESCORT_BASIS_ESCORT_STATE_EVT_WRITE:
            escort_basis_info.escort_state = p_evt->params.escort_state.escortstate;
            break;
            
        case BLE_ESCORT_BASIS_CMD_UPDATE_EVT_WRITE:
            cmd_update_type = (cmd_e)p_evt->params.cmd_update.cmd;
            on_cmd_update();
            break;
            
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling the debug events.
 *
 * @details This function will be called for all debug events which are passed to
 *          the application.
 *
 * @param[in]   p_debug   debug structure.
 * @param[in]   p_evt   Event received from the debug.
 */
static void on_debug_evt(ble_debug_t * p_debug, ble_debug_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    { 
        case BLE_DEBUG_FORCE_LOG_MESSAGE_SUBMIT_EVT_WRITE:
            cmd_update_type = (cmd_e)p_evt->params.force_log_message_submit.flogs;
            on_cmd_update();
            break; 
            
        case BLE_DEBUG_ADJUST_TIME_EVT_WRITE:
            inner_rtc.setcounter(inner_rtc_cfgpara, p_evt->params.adjust_time.adjust_time_value);
            break;
            
        case BLE_DEBUG_SETUP_IP_ADDR_EVT_WRITE:
            memset(server.ip, 0, sizeof(server.ip) / sizeof(server.ip[0]));
            memcpy(server.ip, p_evt->params.setup_ip_addr.ipaddr.p_data, p_evt->params.setup_ip_addr.ipaddr.size);
            break;
            
        case BLE_DEBUG_SETUP_IP_PORT_EVT_WRITE:
            server.port = p_evt->params.setup_ip_port.ipport;
            break; 
            
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_bas_init_t bas_init;
    ble_escort_basis_init_t escort_basis_init;
    ble_debug_init_t debug_init;

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // escort basis Service.
    memset(&escort_basis_init, 0, sizeof(escort_basis_init));
    
    escort_basis_init.evt_handler = on_escort_basis_evt;
    
    err_code = ble_escort_basis_init(&m_escort_basis, &escort_basis_init);
    APP_ERROR_CHECK(err_code);
    
    // debug Service.
    memset(&debug_init, 0, sizeof(debug_init));
    
    debug_init.evt_handler = on_debug_evt;
    
    err_code = ble_debug_init(&m_debug, &debug_init);
    APP_ERROR_CHECK(err_code); 
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
	now_adv_status = ble_adv_evt;
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            break;

        case BLE_ADV_EVT_IDLE:
            break;

        default:
            break;
    }
}

void connect_indicate(void)
{    
    LEDS_ON(WORK_LED_BLUE_MASK);
    os_delay(500);
    LEDS_OFF(WORK_LED_BLUE_MASK);
}

void disconnect_indicate(void)
{  
    now_adv_status = 0;
    LEDS_ON(WORK_LED_RED_MASK);
    os_delay(500);
    LEDS_OFF(WORK_LED_RED_MASK);
}

/**@brief Function for receiving the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t  err_code;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            connect_indicate();
            NRF_LOG_PRINTF("BLE Connect!\r\n");
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            disconnect_indicate();
            NRF_LOG_PRINTF("BLE Disconnect!\r\n");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_debug_on_ble_evt(&m_debug, p_ble_evt);
    ble_escort_basis_on_ble_evt(&m_escort_basis, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**
 * @brief Event handler for new BLE events
 *
 * This function is called from the SoftDevice handler.
 * It is called from interrupt level.
 *
 * @return The returned value is checked in the softdevice_handler module,
 *         using the APP_ERROR_CHECK macro.
 */
static uint32_t ble_new_event_handler(void)
{
    BaseType_t yield_req = pdFALSE;
    // The returned value may be safely ignored, if error is returned it only means that
    // the semaphore is already given (raised).
    UNUSED_VARIABLE(xSemaphoreGiveFromISR(m_ble_event_ready, &yield_req));
    portYIELD_FROM_ISR(yield_req);
    return NRF_SUCCESS;
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, ble_new_event_handler);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in]   p_evt   Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);
    return NRF_SUCCESS;
}



/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.lesc         = SEC_PARAM_LESC;
    register_param.sec_param.keypress     = SEC_PARAM_KEYPRESS;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
//	advdata.short_name_len			= 4;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

#ifdef PCBA_TEST
	ble_advdata_service_data_t advdata_service_data;
	advdata_service_data.service_uuid = BLE_UUID_BATTERY_SERVICE;
	advdata_service_data.data.size = sizeof(m_service_data);
	advdata_service_data.data.p_data = m_service_data;	
	advdata.p_service_data_array = &advdata_service_data;
	advdata.service_data_count = 1;
#endif

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing nfc.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void nfc_init(bool erase_bonds)
{
    NfcRetval     ret_val;
    uint32_t  len = sizeof(ndef_msg_buf);

    /* Start NFC */
    ret_val = nfcSetup(nfc_callback, NULL);
    if (ret_val != NFC_RETVAL_OK)
    {
        APP_ERROR_CHECK((uint32_t) ret_val);
    }
    
    /* Encode Device NFC message */
    NFC_msg_encode(ndef_msg_buf, &len);
    
    /* Set created message as the NFC payload */
    ret_val = nfcSetPayload( (char*)ndef_msg_buf, len);
    if (ret_val != NFC_RETVAL_OK)
    {
        APP_ERROR_CHECK((uint32_t) ret_val);
    }

    ret_val = nfcStartEmulation();
    if (ret_val != NFC_RETVAL_OK)
    {
        APP_ERROR_CHECK((uint32_t) ret_val);
    }

    return;
}

/**@brief Thread for handling the Application's BLE Stack events.
 *
 * @details This thread is responsible for handling BLE Stack events sent from on_ble_evt().
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void ble_stack_thread(void * arg)
{
    for(;;)
    {
        /* Wait for event from SoftDevice */
        while(pdFALSE == xSemaphoreTake(m_ble_event_ready, portMAX_DELAY))
        {
            // Just wait again in the case when INCLUDE_vTaskSuspend is not enabled
        }

        // This function gets events from the SoftDevice and processes them by calling the function
        // registered by softdevice_ble_evt_handler_set during stack initialization.
        // In this code ble_evt_dispatch would be called for every event found.
        intern_softdevice_events_execute();
    }
}

void bsp_ble_init(void)
{
    bool erase_bonds = FALSE;

    // Initialize.
    ble_stack_init();

    device_manager_init(erase_bonds);
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();
    nfc_init(erase_bonds);
}

void app_ble_init(void)
{  
    bsp_ble_init();
    
    m_ble_event_ready = xSemaphoreCreateBinary();
    if(NULL == m_ble_event_ready)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    // Start execution.
    if(pdPASS != xTaskCreate(ble_stack_thread, "BLE", 256, NULL, APP_BLE_PRIORITY, &m_ble_stack_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}