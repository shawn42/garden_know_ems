/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_app_beacon_main main.c
 * @{
 * @ingroup ble_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for a beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ble_conn_params.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "pstorage.h"
#include "app_gpiote.h"
#include "app_timer.h"
#include "app_button.h"
#include "pca20006.h"
#include "ble_bcs.h"
#include "ble_dfu.h"
#include "dfu_app_handler_mod.h"
#include "led_softblink.h"
#include "beacon.h"

/* Button definitions */
#define BOOTLOADER_BUTTON_PIN           BUTTON_0                                    /**< Button used to enter DFU mode. */
#define CONFIG_MODE_BUTTON_PIN          BUTTON_1                                    /**< Button used to enter config mode. */
#define APP_GPIOTE_MAX_USERS            2                                           /**< Max users of app_gpiote */
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Button detection delay in ticks */

/* LED definitions */
#define LED_R_MSK                       (1UL << LED_RGB_RED)                            /**< Red LED bitmask */
#define LED_G_MSK                       (1UL << LED_RGB_GREEN)                          /**< Green LED bitmask */
#define LED_B_MSK                       (1UL << LED_RGB_BLUE)                           /**< Blue LED bitmask */
#define APP_CONFIG_MODE_LED_MSK         (LED_R_MSK | LED_G_MSK)                     /**< Blinking yellow when device is in config mode */
#define APP_BEACON_MODE_LED_MSK         (LED_R_MSK | LED_B_MSK)                     /**< Blinking purple when device is advertising as beacon */

#define ASSERT_LED_PIN_NO               LED_RGB_RED                                     /**< Red LED to indicate assert */

/* Beacon configuration mode parameters */ 
#define BEACON_CONFIG_NAME              "BeaconConfig"                              /**< Name of device. Will be included in the advertising data. */

#define APP_CONFIG_ADV_INTERVAL         MSEC_TO_UNITS(100, UNIT_0_625_MS)           /**< The advertising interval in configuration mode (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_CONFIG_ADV_TIMEOUT          30                                          /**< The advertising timeout (in units of seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  0                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

/* App timer parameters */
#define APP_TIMER_PRESCALER             0                                           /**< RTC prescaler value used by app_timer */
#define APP_TIMER_MAX_TIMERS            3                                           /**< One for each module + one for ble_conn_params + a few extra */
#define APP_TIMER_OP_QUEUE_SIZE         3                                           /**< Maximum number of timeout handlers pending execution */

/* App scheduler parameters */
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

/* DFU definitions */
#define DFU_REV_MAJOR                   0x00                                        /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                   0x01                                        /** DFU Minor revision number to be exposed. */
#define DFU_REVISION                    ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)      /** DFU Revision number to be exposed. Combined of major and minor versions. */


#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/



/* Static variables */
static beacon_mode_t        m_beacon_mode;                                          /**< Current beacon mode */
static beacon_flash_db_t    *p_beacon;                                              /**< Pointer to beacon params */
static pstorage_handle_t    m_pstorage_block_id;                                    /**< Pstorage handle for beacon params */
                                                                                    
static ble_gap_sec_params_t m_sec_params;                                           /**< Security requirements for this application. */
static uint16_t             m_conn_handle = BLE_CONN_HANDLE_INVALID;                /**< Handle of the current connection. */
static ble_bcs_t            m_bcs;                                                  /**< Beacon Configuration Service structure.*/
static ble_dfu_t            m_dfus;                                                 /**< Structure used to identify the DFU service. */
                                                                                    
static ble_gap_adv_params_t m_adv_params;                                           /**< Parameters to be passed to the stack when starting advertising. */
                                                                                    
static bool                 m_beacon_reset = false;                                 /**< Flag to reset system after flash access has finished. */
static bool                 m_beacon_start = false;                                 /**< Flag to setup and start beacon after flash access has finished. */

/* Prototypes */
static void beacon_reset(void);
static void beacon_start(beacon_mode_t mode);

/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    uint32_t err_code = error_code;
    uint32_t lin_num  = line_num;
    const uint8_t * p_fname = p_file_name;
    
    nrf_gpio_pin_clear(ASSERT_LED_PIN_NO);

    UNUSED_VARIABLE(err_code);
    UNUSED_VARIABLE(lin_num);
    UNUSED_VARIABLE(p_fname);
    // On assert, the system can only recover on reset.
    NVIC_SystemReset();
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

/**@brief Function for handeling storage access complete events.
 *
 * @details This function is called through the sheduler from sys_evt_dispatch when storage access is complete
 *
 * @param[in]   data   Event data, not used.
 * @param[in]   size   Event data size, not used.
 */
static void storage_access_complete_handler(void *data, uint16_t size)
{
    if (m_beacon_reset)
    {
        beacon_reset();
    }
    else if (m_beacon_start)
    {
        beacon_start(m_beacon_mode);
    }
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
    uint32_t err_code;
    uint32_t count;
    
    pstorage_sys_event_handler(sys_evt);
    // Check if storage access is in progress.
    err_code = pstorage_access_status_get(&count);
    if ((err_code == NRF_SUCCESS) && (count == 0))
    {
        err_code = app_sched_event_put(0, 0, storage_access_complete_handler);
        APP_ERROR_CHECK(err_code);
    }    
}



/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application and starts softblink timer.
 */
static void leds_init(void)
{
    uint32_t err_code;
    led_sb_init_params_t led_sb_init_params;
    
    nrf_gpio_cfg_output(LED_RGB_RED);
    nrf_gpio_cfg_output(LED_RGB_GREEN);
    nrf_gpio_cfg_output(LED_RGB_BLUE);
    
    nrf_gpio_pin_set(LED_RGB_RED);
    nrf_gpio_pin_set(LED_RGB_GREEN);
    nrf_gpio_pin_set(LED_RGB_BLUE);
    
    led_sb_init_params.active_high     = false;
    led_sb_init_params.duty_cycle_max  = 20;
    led_sb_init_params.duty_cycle_min  = 0;
    led_sb_init_params.duty_cycle_step = 1;
    led_sb_init_params.leds_pin_bm     = (LED_R_MSK | LED_G_MSK | LED_B_MSK);
    led_sb_init_params.off_time_ms     = 4000;
    led_sb_init_params.on_time_ms      = 0;
    
    err_code = led_softblink_init(&led_sb_init_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(beacon_mode_t mode)
{
    if (mode == beacon_mode_normal)
    {
        uint32_t        err_code;
        ble_advdata_t   advdata;
        uint8_t         flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
        ble_advdata_manuf_data_t manuf_specific_data;
        
        manuf_specific_data.company_identifier = p_beacon->data.company_id;
        manuf_specific_data.data.p_data        = p_beacon->data.beacon_data;
        manuf_specific_data.data.size          = APP_BEACON_MANUF_DATA_LEN;
        
        // Build and set advertising data.
        memset(&advdata, 0, sizeof(advdata));

        advdata.name_type               = BLE_ADVDATA_NO_NAME;
        advdata.flags.size              = sizeof(flags);
        advdata.flags.p_data            = &flags;
        advdata.p_manuf_specific_data   = &manuf_specific_data;

        err_code = ble_advdata_set(&advdata, NULL);
        APP_ERROR_CHECK(err_code);

        // Initialize advertising parameters (used when starting advertising).
        memset(&m_adv_params, 0, sizeof(m_adv_params));

        m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
        m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
        m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
        m_adv_params.interval    = MSEC_TO_UNITS(p_beacon->data.adv_interval, UNIT_0_625_MS);
        m_adv_params.timeout     = APP_BEACON_ADV_TIMEOUT;
    }
    else if (mode == beacon_mode_config)
    {
        uint32_t      err_code;
        ble_advdata_t advdata;
        ble_advdata_t scanrsp;
        uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
        
        ble_uuid_t adv_uuids[] = {{BCS_UUID_SERVICE, m_bcs.uuid_type}};

        // Build and set advertising data
        memset(&advdata, 0, sizeof(advdata));
        advdata.name_type               = BLE_ADVDATA_FULL_NAME;
        advdata.include_appearance      = true;
        advdata.flags.size              = sizeof(flags);
        advdata.flags.p_data            = &flags;
        
        memset(&scanrsp, 0, sizeof(scanrsp));
        scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
        scanrsp.uuids_complete.p_uuids  = adv_uuids;
        
        err_code = ble_advdata_set(&advdata, &scanrsp);
        APP_ERROR_CHECK(err_code);
        
        // Initialize advertising parameters (used when starting advertising).
        memset(&m_adv_params, 0, sizeof(m_adv_params));
        
        m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
        m_adv_params.p_peer_addr = NULL;
        m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
        m_adv_params.interval    = APP_CONFIG_ADV_INTERVAL;
        m_adv_params.timeout     = APP_CONFIG_ADV_TIMEOUT;           
    }
    else
    {
        APP_ERROR_CHECK_BOOL(false);
    }
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handeling button presses.
 */
static void button_handler(uint8_t pin_no, uint8_t action)
{
    if (action == APP_BUTTON_PUSH)
    {
        if(pin_no == CONFIG_MODE_BUTTON_PIN)
        {
            beacon_reset();
        }
        else if (pin_no == BOOTLOADER_BUTTON_PIN)
        {
            beacon_reset();
        }
        else
        {
            APP_ERROR_CHECK_BOOL(false);
        }
    }
}

/**@brief Function for initializing and enabling the app_button module.
 */
static void buttons_init(void)
{
    uint32_t err_code;
    
    // @note: Array must be static because a pointer to it will be saved in the Button handler 
    // module.
    static app_button_cfg_t buttons[] =
    {
        {CONFIG_MODE_BUTTON_PIN, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler},
        {BOOTLOADER_BUTTON_PIN, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler}
    };

    APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY, true);
    
    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);    
}


/**@brief Function for handling the writes to the configuration characteristics of the beacon configuration service. 
 * @detail A pointer to this function is passed to the service in its init structure. 
 */
static void beacon_write_handler(ble_bcs_t * p_lbs, beacon_data_type_t type, uint8_t *data)
{
    uint32_t err_code;
    
    static beacon_flash_db_t tmp;
    
    memcpy(&tmp, p_beacon, sizeof(beacon_flash_db_t));
    
    tmp.data.magic_byte = MAGIC_FLASH_BYTE;
    
    switch(type)
    {
        case beacon_maj_min_data:
            tmp.data.beacon_data[BEACON_MANUF_DAT_MAJOR_H_IDX] = data[0];
            tmp.data.beacon_data[BEACON_MANUF_DAT_MAJOR_L_IDX] = data[1];
            tmp.data.beacon_data[BEACON_MANUF_DAT_MINOR_H_IDX] = data[2];
            tmp.data.beacon_data[BEACON_MANUF_DAT_MINOR_L_IDX] = data[3];
            break;
        
        case beacon_measured_rssi_data:
            tmp.data.beacon_data[BEACON_MANUF_DAT_RSSI_IDX] = data[0];
            break;
        
        case beacon_uuid_data:
            memcpy(&tmp.data.beacon_data[BEACON_MANUF_DAT_UUID_IDX], data, BCS_DATA_ID_LEN);      
            break;
        
        case beacon_company_id_data:
            tmp.data.company_id = (data[1] << 8) + data[0];
            break;
        
        case beacon_adv_interval_data:
            tmp.data.adv_interval = (data[1] << 8) + data[0];
            if (tmp.data.adv_interval < APP_BEACON_ADV_INTERVAL_MIN_MS)
            {
                tmp.data.adv_interval = APP_BEACON_ADV_INTERVAL_MIN_MS;
            }
            else if (tmp.data.adv_interval > APP_BEACON_ADV_INTERVAL_MAX_MS)
            {
                tmp.data.adv_interval = APP_BEACON_ADV_INTERVAL_MAX_MS;
            }
            break;
        
        case beacon_led_data:
            tmp.data.led_state = data[0];
            break;
        
        default:
            break;
    }
    
    err_code = pstorage_clear(&m_pstorage_block_id, sizeof(beacon_flash_db_t));
    APP_ERROR_CHECK(err_code);    
    
    err_code = pstorage_store(&m_pstorage_block_id, (uint8_t *)&tmp, sizeof(beacon_flash_db_t), 0);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function shall be used to setup all the necessary GAP (Generic Access Profile) 
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode, (uint8_t const * const)BEACON_CONFIG_NAME, (uint16_t)strlen(BEACON_CONFIG_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for stopping advertising.
 */
static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);

    err_code = led_softblink_stop(APP_CONFIG_MODE_LED_MSK);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for preparing before reset.
 */
static void dfu_reset_prepare(void)
{
    uint32_t err_code;
    
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        err_code = led_softblink_stop(APP_CONFIG_MODE_LED_MSK);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // If not connected, then the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }

    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t err_code;
    ble_bcs_init_t init;
    ble_dfu_init_t   dfus_init;

    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler    = dfu_app_on_dfu_evt;
    dfus_init.error_handler  = NULL; //service_error_handler - Not used as only the switch from app to DFU mode is required and not full dfu service.
    dfus_init.evt_handler    = dfu_app_on_dfu_evt;
    dfus_init.revision       = DFU_REVISION;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);
    
    dfu_app_reset_prepare_set(dfu_reset_prepare);   

    
    init.beacon_write_handler = beacon_write_handler;
    init.p_beacon = p_beacon;
    
    err_code = ble_bcs_init(&m_bcs, &init);
    APP_ERROR_CHECK(err_code);    
}

/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;  
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
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
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
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

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code = NRF_SUCCESS;
    static ble_gap_evt_auth_status_t m_auth_status;
    ble_gap_enc_info_t *             p_enc_info;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            beacon_reset();
            break;
            
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_SUCCESS, 
                                                   &m_sec_params);
            break;
            
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;
            
        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            p_enc_info = &m_auth_status.periph_keys.enc_info;
            if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div)
            {
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL);
            }
            else
            {
                // No keys found for this device
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            { 
                beacon_reset();
            }
            break;

        default:
            break;
    }

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_bcs_on_ble_evt(&m_bcs, p_ble_evt);
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    on_ble_evt(p_ble_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_50_PPM, true);
    
    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);   
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);    
}

/** @brief Function for handling pstorage events
 */
static void pstorage_ntf_cb(pstorage_handle_t *  p_handle,
                            uint8_t              op_code,
                            uint32_t             result,
                            uint8_t *            p_data,
                            uint32_t             data_len)
{
    APP_ERROR_CHECK(result);
}

/** @brief Function for initializing pstorage
 */
static void flash_access_init(void)
{
    uint32_t err_code;
    
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);    
}

/** @brief Function to get a pointer to beacon parameters in flash. Should only be called once during initialization.
*/
static beacon_flash_db_t * beacon_params_get(void)
{
    uint32_t err_code;
    pstorage_module_param_t pstorage_param;
    
    pstorage_param.cb = pstorage_ntf_cb;
    pstorage_param.block_size = sizeof(beacon_flash_db_t);
    pstorage_param.block_count = 1;
    
    err_code = pstorage_register(&pstorage_param, &m_pstorage_block_id);
    APP_ERROR_CHECK(err_code);
    
    return (beacon_flash_db_t *)m_pstorage_block_id.block_id;
}

/** @brief Function for writing default beacon parameters to flash.
*/
static void beacon_params_default_set(void)
{
    uint32_t err_code;
    static beacon_flash_db_t tmp;
    
    uint8_t beacon_data[APP_BEACON_MANUF_DATA_LEN] =  /**< Information advertised by the beacon. */
    {
        APP_DEVICE_TYPE,           // Manufacturer specific information. Specifies the device type in this 
                                   // implementation. 
        APP_ADV_DATA_LENGTH,       // Manufacturer specific information. Specifies the length of the 
                                   // manufacturer specific data in this implementation.
        APP_DEFAULT_BEACON_UUID,   // 128 bit UUID value. 
        APP_DEFAULT_MAJOR_VALUE,   // Major arbitrary value that can be used to distinguish between beacons. 
        APP_DEFAULT_MINOR_VALUE,   // Minor arbitrary value that can be used to distinguish between beacons. 
        APP_DEFAULT_MEASURED_RSSI  // Manufacturer specific information. The beacon's measured TX power in 
                                   // this implementation. 
    };    
    
    tmp.data.magic_byte   = MAGIC_FLASH_BYTE;
    tmp.data.adv_interval = APP_BEACON_DEFAULT_ADV_INTERVAL_MS;
    tmp.data.company_id   = APP_DEFAULT_COMPANY_IDENTIFIER;
    tmp.data.led_state    = 0x01;
    
    beacon_data[BEACON_MANUF_DAT_MINOR_L_IDX] = (uint8_t)(NRF_FICR->DEVICEADDR[0] & 0xFFUL);
    beacon_data[BEACON_MANUF_DAT_MINOR_H_IDX] = (uint8_t)((NRF_FICR->DEVICEADDR[0] >>  8) & 0xFFUL);
    beacon_data[BEACON_MANUF_DAT_MAJOR_L_IDX] = (uint8_t)((NRF_FICR->DEVICEADDR[0] >> 16) & 0xFFUL);
    beacon_data[BEACON_MANUF_DAT_MAJOR_H_IDX] = (uint8_t)((NRF_FICR->DEVICEADDR[0] >> 24) & 0xFFUL);
    
    memcpy(tmp.data.beacon_data, beacon_data, APP_BEACON_MANUF_DATA_LEN);
    
    err_code = pstorage_clear(&m_pstorage_block_id, sizeof(beacon_flash_db_t));
    APP_ERROR_CHECK(err_code);
    
    err_code = pstorage_store(&m_pstorage_block_id, (uint8_t *)&tmp, sizeof(beacon_flash_db_t), 0);
    APP_ERROR_CHECK(err_code);
}

/** @brief Function for setup of beacon operation.
*/
static void beacon_setup(beacon_mode_t mode)
{
    if(mode == beacon_mode_config)
    {
        gap_params_init();
        services_init();
        advertising_init(mode);
        conn_params_init();
        sec_params_init();
        led_softblink_off_time_set(2000);
        led_softblink_start(APP_CONFIG_MODE_LED_MSK);
    }
    else
    {
        advertising_init(mode);
        
        if (p_beacon->data.led_state)
        {
            led_softblink_start(APP_BEACON_MODE_LED_MSK);
        }
    }    
}

/** @brief Function for reseting the beacon.
*/
static void beacon_reset(void)
{
    uint32_t err_code;
    uint32_t count;
    
    // Check if storage access is in progress.
    err_code = pstorage_access_status_get(&count);
    APP_ERROR_CHECK(err_code);
    if (count == 0)
    {
        m_beacon_reset = false;
        NVIC_SystemReset();
    }
    else
    {
        m_beacon_reset = true;
    }
}

/** @brief Function for starting beacon operation.
*/
static void beacon_start(beacon_mode_t mode)
{
    uint32_t err_code;
    uint32_t count;
    
    // Check if storage access is in progress.
    err_code = pstorage_access_status_get(&count);
    APP_ERROR_CHECK(err_code);
    if (count == 0)
    {
        m_beacon_start = false;
        // Setup beacon mode
        beacon_setup(m_beacon_mode);
        
        // Start advertising
        advertising_start();
    }
    else
    {
        m_beacon_start = true;
    }        
}

/** @brief Function for reading the beacon mode button.
*/
static beacon_mode_t beacon_mode_button_read(void)
{
    uint32_t err_code;
    bool config_mode;
    
    err_code = app_button_is_pushed(0, &config_mode);
    APP_ERROR_CHECK(err_code);
    
    return config_mode ? beacon_mode_config : beacon_mode_normal;
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
    buttons_init();    
    leds_init();
    ble_stack_init();
    flash_access_init();
    
    // Read beacon mode
    m_beacon_mode = beacon_mode_button_read();
    // Read beacon params from flash
    p_beacon = beacon_params_get();
    if (p_beacon->data.magic_byte != MAGIC_FLASH_BYTE)
    {
        // No valid params found, write default params.
        beacon_params_default_set();
    }
    
    beacon_start(m_beacon_mode);

    // Enter main loop.
    for (;;)
    {
        app_sched_execute();
        power_manage();
    }
}

/**
 * @}
 */
