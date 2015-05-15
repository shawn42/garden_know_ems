/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the license.txt file.
 */

/** @file
 * @brief    ADC example with UART and BLE connectivity
 
 * This example should be operated in the same way as the UART example for the evaluation board
 * in the SDK. Follow the same guide for this example, given on:
 * https://devzone.nordicsemi.com/documentation/nrf51/6.0.0/s110/html/a00066.html#project_uart_nus_eval_test.
 * The ADC functionality has been added on top of the functionality that is described in the link above. The
 * The UART connectivity part of this example uses the app_uart library (event driven).
 
 * An application timer is
 * set up to trigger the ADC sampling which is performed once per second. ADC is sampled
 * with 8 bit resolution, samples from analog input pin 2 (P0.01), uses the internal 1.2VBG fixed
 * reference voltage as reference, one third prescaling which enables input voltage range
 * of 0-3.6V. When the sample is taken, pin 10 (led 2) is toggled and when the sample is ready, pin
 * 11 (led 3) is toggled. The output is then sent both over the UART and over BLE (see the ADC_IRQHandler
 * in this file)
 */
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "boards.h"
#include "ble_error_log.h"
#include "ble_debug_assert_handler.h"
#include "app_util_platform.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/


#define LED_0 BSP_LED_0
#define LED_1 BSP_LED_1
#define LED_2 BSP_LED_2
#define LED_3 BSP_LED_0

#define WAKEUP_BUTTON_PIN               BUTTON_0                                    /**< Button used to wake up the application. */

#define ADVERTISING_LED_PIN_NO          LED_0                                       /**< LED to indicate advertising state. */
#define CONNECTED_LED_PIN_NO            LED_1                                       /**< LED to indicate connected state. */

#define DEVICE_NAME                     "GardenKnowEm"                               /**< Name of device. Will be included in the advertising data. */

//#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define NON_CONNECTABLE_ADV_INTERVAL  MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define APP_CFG_NON_CONN_ADV_TIMEOUT  0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */

//#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                         /**< TURN OFF ADV TIMEOUT. */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            2                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define ADC_SAMPLING_INTERVAL           APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)  /**< Sampling rate for the ADC */

#define MIN_CONN_INTERVAL               7.5                                          /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               60                                          /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< slave latency. */
#define CONN_SUP_TIMEOUT                400                                         /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_GPIOTE_MAX_USERS            1

//static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static app_timer_id_t                   m_adc_sampling_timer_id;                                        /**< ADC timer */

// static bool ble_buffer_available = true;
// static bool tx_complete = false;
static uint8_t adc_result = 0;


/**@brief     Error handler function, which is called when an error has occurred.
 *
 * @warning   This handler is an example only and does not fit a final product. You need to analyze
 *            how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    //ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}


/**@brief       Assert macro callback function.
 *
 * @details     This function will be called in case of an assert in the SoftDevice.
 *
 * @warning     This handler is an example only and does not fit a final product. You need to
 *              analyze how your product is supposed to react in case of Assert.
 * @warning     On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief   Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    nrf_gpio_cfg_output(ADVERTISING_LED_PIN_NO);
    nrf_gpio_cfg_output(CONNECTED_LED_PIN_NO);
        nrf_gpio_cfg_output(LED_2);                   //Added for ADC operation
      nrf_gpio_cfg_output(LED_3);                   //Added for ADC operation
}

/**@brief   Function for Timer initialization.
 *
 * @details Initializes the timer module.
 */

// ADC timer handler to start ADC sampling
static void adc_sampling_timeout_handler(void * p_context)
{
    uint32_t p_is_running = 0;
        
    sd_clock_hfclk_request();
    while(! p_is_running) {                             //wait for the hfclk to be available
        sd_clock_hfclk_is_running((&p_is_running));
    }               
    nrf_gpio_pin_toggle(LED_2);     //Toggle LED2 to indicate start of sampling
    NRF_ADC->TASKS_START = 1;                           //Start ADC sampling
}

static void timers_init(void)
{
      uint32_t err_code;
    
    // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
    
        err_code = app_timer_create(&m_adc_sampling_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                adc_sampling_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief   Function for the GAP initialization.
 *
 * @details This function will setup all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    // ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
}


/**@brief   Function for the Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    sd_ble_gap_tx_power_set(4);          // accepted values are -40, -30, -20, -16, -12, -8, -4, 0, and 4 dBm
    uint32_t      err_code;
    ble_advdata_t advdata;
//    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    uint8_t         flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;


    ble_advdata_manuf_data_t manuf_specific_data;
    memset(&manuf_specific_data, 0, sizeof(manuf_specific_data));

    manuf_specific_data.company_identifier = 7777;
    manuf_specific_data.data.size = sizeof(adc_result);
    manuf_specific_data.data.p_data = &adc_result;
    advdata.p_manuf_specific_data   = &manuf_specific_data;
    
    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing security parameters.
 */
// static void sec_params_init(void)
// {
//     m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
//     m_sec_params.bond         = SEC_PARAM_BOND;
//     m_sec_params.mitm         = SEC_PARAM_MITM;
//     m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
//     m_sec_params.oob          = SEC_PARAM_OOB;  
//     m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
//     m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
// }


/**@brief       Function for handling an event from the Connection Parameters Module.
 *
 * @details     This function will be called for all events in the Connection Parameters Module
 *              which are passed to the application.
 *
 * @note        All this function does is to disconnect. This could have been done by simply setting
 *              the disconnect_on_fail config parameter, but instead we use the event handler
 *              mechanism to demonstrate its use.
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


/**@brief       Function for handling errors from the Connection Parameters module.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;

        //ADC timer start
        err_code = app_timer_start(m_adc_sampling_timer_id, ADC_SAMPLING_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;
    
    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));
    
    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;// BLE_GAP_ADV_TYPE_ADV_DIRECT_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    // adv_params.interval    = APP_ADV_INTERVAL;
    // adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;


    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}


/**@brief       Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    // uint32_t                         err_code;
    // static ble_gap_evt_auth_status_t m_auth_status;
    // ble_gap_enc_info_t *             p_enc_info;
    
    // switch (p_ble_evt->header.evt_id)
    // {
    //     case BLE_GAP_EVT_CONNECTED:
    //         nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
    //         nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
    //         m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    //         break;
            
    //     case BLE_GAP_EVT_DISCONNECTED:
    //         nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
    //         m_conn_handle = BLE_CONN_HANDLE_INVALID;

    //         advertising_start();

    //         break;
            
    //     case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
    //         err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
    //                                                BLE_GAP_SEC_STATUS_SUCCESS, 
    //                                                &m_sec_params);
    //         APP_ERROR_CHECK(err_code);
    //         break;
            
    //     case BLE_GATTS_EVT_SYS_ATTR_MISSING:
    //         err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
    //         APP_ERROR_CHECK(err_code);
    //         break;

    //     case BLE_GAP_EVT_AUTH_STATUS:
    //         m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
    //         break;
            
    //     case BLE_GAP_EVT_SEC_INFO_REQUEST:
    //         p_enc_info = &m_auth_status.periph_keys.enc_info;
    //         if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div)
    //         {
    //             err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL);
    //             APP_ERROR_CHECK(err_code);
    //         }
    //         else
    //         {
    //             // No keys found for this device
    //             err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
    //             APP_ERROR_CHECK(err_code);
    //         }
    //         break;

    //     case BLE_GAP_EVT_TIMEOUT:
    //         if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
    //         { 
    //             nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);

    //             // Configure buttons with sense level low as wakeup source.
    //             nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN,
    //                                      BUTTON_PULL,
    //                                      NRF_GPIO_PIN_SENSE_LOW);
                
    //             // Go to system-off mode (this function will not return; wakeup will cause a reset)
    //             err_code = sd_power_system_off();    
    //             APP_ERROR_CHECK(err_code);
    //         }
    //         break;
    //     case BLE_EVT_TX_COMPLETE:
    //         if(!ble_buffer_available) tx_complete = true;
    //         break;

    //     default:
    //         // No implementation needed.
    //         break;
    // }
}


/**@brief       Function for dispatching a S110 SoftDevice event to all modules with a S110
 *              SoftDevice event handler.
 *
 * @details     This function is called from the S110 SoftDevice event interrupt handler after a
 *              S110 SoftDevice event has been received.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief   Function for the S110 SoftDevice initialization.
 *
 * @details This function initializes the S110 SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief  Function for configuring the buttons.
 */
static void buttons_init(void)
{
    nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN,
                             BUTTON_PULL, 
                             NRF_GPIO_PIN_SENSE_LOW);    
}


/**@brief  Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

//ADC initialization
static void adc_init(void)
{   
    /* Enable interrupt on ADC sample ready event*/     
    NRF_ADC->INTENSET = ADC_INTENSET_END_Msk;   
    sd_nvic_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);  
    sd_nvic_EnableIRQ(ADC_IRQn);
    
    NRF_ADC->CONFIG = (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos) /* Bits 17..16 : ADC external reference pin selection. */
                                    | (ADC_CONFIG_PSEL_AnalogInput2 << ADC_CONFIG_PSEL_Pos)                 /*!< Use analog input 2 as analog input. */
                                    | (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos)                          /*!< Use internal 1.2V bandgap voltage as reference for conversion. */
                                    | (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) /*!< Analog input specified by PSEL with no prescaling used as input for the conversion. */
                                    | (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos);                                  /*!< 8bit ADC resolution. */ 
    
    /* Enable ADC*/
    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
}
/* Interrupt handler for ADC data ready event */
void ADC_IRQHandler(void)
{
    //uint8_t adc_result;
    
    /* Clear dataready event */
    NRF_ADC->EVENTS_END = 0;  

    /* Write ADC result both to the UART and over BLE */
    adc_result = NRF_ADC->RESULT;
    nrf_gpio_pin_toggle(LED_3);        //indicate on LED that the ADC interrupt handler is executing
    advertising_init();
    
    //Use the STOP task to save current. Workaround for PAN_028 rev1.5 anomaly 1.
    NRF_ADC->TASKS_STOP = 1;
    
    //Release the external crystal
    sd_clock_hfclk_release();
}   

/**@brief  Application main function.
 */
int main(void)
{
    // Initialize
    leds_init();
    timers_init();
    buttons_init();
    ble_stack_init();
    gap_params_init();
    advertising_init();
    adc_init();         //Initialize ADC
    
    application_timers_start();
    advertising_start();
    
    // Enter main loop
    for (;;)
    { 
        power_manage();
    }
}


