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
#include "twi_master.h"

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

#define NON_CONNECTABLE_ADV_INTERVAL  MSEC_TO_UNITS(1000, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define APP_CFG_NON_CONN_ADV_TIMEOUT  0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */

//#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                         /**< TURN OFF ADV TIMEOUT. */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            2                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define ADC_SAMPLING_INTERVAL           APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)  /**< Sampling rate for the ADC */

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
// static uint8_t adc_result = 0;
static uint16_t adc_result = 0;

static bool should_sample = false;
static bool sampling = false;
static bool read_humidity = false;
static bool first_read = true;


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
    nrf_gpio_cfg_output(LED_1);
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
    // should_sample = true;
    // nrf_gpio_pin_toggle(LED_1);
    read_humidity = true; 
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
    manuf_specific_data.data.p_data = (uint8_t*)&adc_result;
    advdata.p_manuf_specific_data   = &manuf_specific_data;
    
    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
  }



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

    // nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
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
    // adc_init();         //Initialize ADC
    
    application_timers_start();
    advertising_start();

    // init TWI
    // nrf_gpio_cfg_output(0);
    // nrf_gpio_cfg_output(24);
    if(twi_master_init()) {
      nrf_gpio_pin_toggle(LED_1);
    } else {
      nrf_gpio_pin_toggle(LED_0);
    }
    // #define TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER (24U)  # go copy in twi_master_config.h and make sure its config dir isn't in the include path!
    // #define TWI_MASTER_CONFIG_DATA_PIN_NUMBER (25U)

    
    // Enter main loop
    for (;;)
    { 
      if(should_sample) {
        should_sample = false;
        sampling = true;
        uint32_t p_is_running = 0;
      
        sd_clock_hfclk_request();
        while(! p_is_running) {                             //wait for the hfclk to be available
          sd_clock_hfclk_is_running((&p_is_running));
        }               
        // nrf_gpio_pin_toggle(LED_2);     //Toggle LED2 to indicate start of sampling
        NRF_ADC->TASKS_START = 1;                           //Start ADC sampling

        // do stuff on main thread...
      } else if(sampling) {
        sampling = false;
      } else if(read_humidity) {
        if(first_read) {
          first_read = false;
          uint8_t write_address = 0x80;
          uint8_t soft_reset_command = 0xFE;
          twi_master_transfer(write_address, &soft_reset_command, 1, true);
          // nrf_gpio_pin_toggle(LED_1);
        } else {

          // GPIO 0 -> clock   P0.00
          // GPIO 24 -> data   P0.24

          // Trigger Temperature Measurement 0xE3 Hold master
          // Trigger Humidity Measurement 0xE5 Hold master
          // Trigger Temperature Measurement 0xF3 No Hold master
          // Trigger Humidity Measurement 0xF5 No Hold master
          // Write user register 0xE6 
          // Read user register  0xE7 
          // Soft Reset 0xFE

          // device address is 0x40 ..  0x80 write  0x81 read
          read_humidity = false;
          nrf_gpio_pin_toggle(LED_2);

          uint8_t read_address = 0x81;
          uint8_t write_address = 0x80;

          uint8_t read_humidity_command = 0xE5;
          // uint8_t read_temperature_command = 0xE3;
          //     [MSB, LSB => [6,1(0:temp, 1:hum),1], CHKSUM]
          uint8_t read_data[3] = {0,0,0};

          // twi_master_transfer(uint8_t   address, uint8_t * data, uint8_t   data_length, bool      issue_stop_condition)
          twi_master_transfer(write_address, &read_humidity_command, 1, false);
          if(twi_master_transfer(read_address, &read_data[0], sizeof(read_data), true)) {
            nrf_gpio_pin_set(LED_1);
          } else {
            nrf_gpio_pin_set(LED_0);
          }

          // TODO store only the raw for transmissoin
          uint16_t humidity_raw = ((read_data[1] & 0xFC) >> 2) | (read_data[0] << 6);
          adc_result = humidity_raw;
          advertising_init();
          // int32_t relative_humidity = -6 + (125 * humidity_raw) / (1 << 16);
        }
      } else {
        power_manage();
      }
    }
  }


