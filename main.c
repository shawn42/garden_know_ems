#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "app_util_platform.h"
#include "softdevice_handler.h"
#include "ble_advdata.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "twi_master.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"

// defines
#define DEVICE_NAME "GardenKnowEm"
#define INCLUDE_SERVICE_CHANGED_CHARACTERISTIC 0

/** 
  The advertising interval for non-connectable advertisement (100 ms).
  This value can vary between 100ms to 10.24s). */
#define NON_CONNECTABLE_ADV_INTERVAL  MSEC_TO_UNITS(100, UNIT_0_625_MS)

/**
  Time for which the device must be advertising in non-connectable mode (in seconds). 
  0 disables timeout. */
#define APP_CFG_NON_CONN_ADV_TIMEOUT_SEC  0
#define ADVERTISING_DURATION APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) 
#define SLEEP_DURATION       APP_TIMER_TICKS(60000, APP_TIMER_PRESCALER) 

/**
  4 bytes for now...  */
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(uint32_t)
#define SCHED_QUEUE_SIZE                10

#define APP_TIMER_PRESCALER             0 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            2 /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4 /**< Size of timer operation queues. */

#define ADVERTISING_LED_PIN_NO LED_RGB_BLUE
#define POWER_LED_PIN_NO LED_RGB_GREEN
#define ERROR_LED_PIN_NO LED_RGB_RED

#define SOIL_SIDE_A_PIN_NO 1  // P0.01
#define SOIL_SIDE_B_PIN_NO 11 // P0.11


typedef enum {
  STARTING,
  READ_HUMIDITY,
  READ_TEMP,
  READ_LIGHT,
  READ_SOIL,
  RESET_SOIL_SENSOR,
  START_ADVERTISING,
  ADVERTISING,
  STOP_ADVERTISING,
  GO_TO_SLEEP,
  SLEEPING
} states_type;

typedef struct know_em_packet_type
{
  uint16_t humidity;
  uint16_t temp;
  uint16_t soil;
  uint16_t light;
  uint16_t packet_number;
} know_em_packet_type;

// globals
static uint8_t READ_HUMIDITY_ADDRESS = 0x81;
static uint8_t WRITE_HUMIDITY_ADDRESS = 0x80;
static uint8_t READ_HUMIDITY_COMMAND = 0xE5;

static uint8_t READ_TEMP_ADDRESS = 0x81;
static uint8_t WRITE_TEMP_ADDRESS = 0x80;
static uint8_t READ_TEMP_COMMAND = 0xE3;

static states_type current_state = STARTING;
static know_em_packet_type know_em_packet = {0,0,0,0,0};

static app_timer_id_t advertising_timer_id;
static app_timer_id_t sleeping_timer_id;


// methods
static void done_advertising_handler(void*);
static void done_sleeping_handler(void*);
static void led_on(uint32_t);
static void led_off(uint32_t);

static void transition_to(states_type new_state)
{
  current_state = new_state;
}

static void power_manage(void)
{
  uint32_t err_code = sd_app_evt_wait();
  APP_ERROR_CHECK(err_code);
}

static void ble_stack_init(void)
{
  uint32_t err_code;

  SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

  ble_enable_params_t ble_enable_params;
  memset(&ble_enable_params, 0, sizeof(ble_enable_params));
  ble_enable_params.gatts_enable_params.service_changed = INCLUDE_SERVICE_CHANGED_CHARACTERISTIC;
  err_code = sd_ble_enable(&ble_enable_params);
  APP_ERROR_CHECK(err_code);
}

static void gap_params_init(void)
{
  uint32_t err_code;

  ble_gap_conn_sec_mode_t sec_mode;
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
  
  err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) DEVICE_NAME, strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);
}

static void update_advertising_data(void)
{
  uint32_t err_code;
  ble_advdata_t advdata;
  uint8_t flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

  memset(&advdata, 0, sizeof(advdata));
  advdata.name_type               = BLE_ADVDATA_FULL_NAME;
  advdata.flags.size              = sizeof(flags);
  advdata.flags.p_data            = &flags;

  ble_advdata_manuf_data_t manuf_specific_data;
  memset(&manuf_specific_data, 0, sizeof(manuf_specific_data));

  know_em_packet.packet_number++;
  manuf_specific_data.company_identifier = 0xEFBE;
  manuf_specific_data.data.size = sizeof(know_em_packet);
  manuf_specific_data.data.p_data = (uint8_t*)&know_em_packet;
  advdata.p_manuf_specific_data   = &manuf_specific_data;

  err_code = ble_advdata_set(&advdata, NULL);
  APP_ERROR_CHECK(err_code);
}

static void advertising_init(void)
{
  uint32_t err_code;

  err_code = sd_ble_gap_tx_power_set(4);          // accepted values are -40, -30, -20, -16, -12, -8, -4, 0, and 4 dBm
  APP_ERROR_CHECK(err_code);

  update_advertising_data();
}

static void timers_init(void)
{
  uint32_t err_code;
  
  APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
  
  err_code = app_timer_create(&advertising_timer_id, APP_TIMER_MODE_SINGLE_SHOT, done_advertising_handler);
  APP_ERROR_CHECK(err_code);

  err_code = app_timer_create(&sleeping_timer_id, APP_TIMER_MODE_SINGLE_SHOT, done_sleeping_handler);
  APP_ERROR_CHECK(err_code);
}


static void start_advertising(void)
{
  uint32_t err_code;

  ble_gap_adv_params_t adv_params;
  memset(&adv_params, 0, sizeof(adv_params));
  adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
  adv_params.p_peer_addr = NULL;
  adv_params.fp          = BLE_GAP_ADV_FP_ANY;
  adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
  adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT_SEC;

  // led_on(ADVERTISING_LED_PIN_NO);

  err_code = sd_ble_gap_adv_start(&adv_params);
  APP_ERROR_CHECK(err_code);

  update_advertising_data();

  // start timer for advertising duration
  err_code = app_timer_start(advertising_timer_id, ADVERTISING_DURATION, NULL);
  APP_ERROR_CHECK(err_code);
}

static void done_advertising_handler(void* context)
{
  transition_to(STOP_ADVERTISING);
}

static void done_sleeping_handler(void* context)
{
  led_on(POWER_LED_PIN_NO);
  transition_to(READ_HUMIDITY);
}

static void stop_advertising(void)
{
  uint32_t err_code = sd_ble_gap_adv_stop();
  APP_ERROR_CHECK(err_code);

  led_off(ADVERTISING_LED_PIN_NO);
}

static void scheduler_init(void)
{
  APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

static void soil_sensor_init(void)
{
  nrf_gpio_cfg_output(SOIL_SIDE_A_PIN_NO);
  nrf_gpio_cfg_output(SOIL_SIDE_B_PIN_NO);
}

static void led_on(uint32_t pin)
{
  nrf_gpio_pin_clear(pin);
}
static void led_off(uint32_t pin)
{
  nrf_gpio_pin_set(pin);
}

static void leds_init(void)
{
  nrf_gpio_cfg_output(ADVERTISING_LED_PIN_NO);
  nrf_gpio_cfg_output(POWER_LED_PIN_NO);
  nrf_gpio_cfg_output(ERROR_LED_PIN_NO);

  // led_on(POWER_LED_PIN_NO);
}

static void read_humidity()
{
  twi_master_init();
  //     [MSB, LSB => [6,1(0:temp, 1:hum),1], CHKSUM]
  uint8_t humidity_response[3] = {0,0,0};
  twi_master_transfer(WRITE_HUMIDITY_ADDRESS, &READ_HUMIDITY_COMMAND, 1, false);
  twi_master_transfer(READ_HUMIDITY_ADDRESS, &humidity_response[0], sizeof(humidity_response), true);

  // uint16_t humidity_raw = ((humidity_response[1] & 0xFC) >> 2) | (humidity_response[0] << 6);
  uint16_t humidity_raw = ((humidity_response[1] & 0xFC)) | (humidity_response[0] << 8);
  know_em_packet.humidity = humidity_raw;
}

static void read_temp()
{
  twi_master_init();
  uint8_t temp_response[3] = {0,0,0};
  twi_master_transfer(WRITE_TEMP_ADDRESS, &READ_TEMP_COMMAND, 1, false);
  twi_master_transfer(READ_TEMP_ADDRESS, &temp_response[0], sizeof(temp_response), true);

  // uint16_t temp_raw = ((temp_response[1] & 0xFC) >> 2) | (temp_response[0] << 6);
  uint16_t temp_raw = ((temp_response[1] & 0xFC)) | (temp_response[0] << 8);
  know_em_packet.temp = temp_raw;
}

static void request_clock(void)
{
  uint32_t p_is_running = 0;

  sd_clock_hfclk_request();
  while(! p_is_running) {                             //wait for the hfclk to be available
    sd_clock_hfclk_is_running((&p_is_running));
  }               
}

static void release_clock(void)
{
  sd_clock_hfclk_release();
}

static void read_light()
{
  request_clock();

  // interrupt ADC
  NRF_ADC->INTENSET = (ADC_INTENSET_END_Disabled << ADC_INTENSET_END_Pos);
          
  // config ADC
  NRF_ADC->CONFIG = (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos) /* Bits 17..16 : ADC external reference pin selection. */
                    | (ADC_CONFIG_PSEL_AnalogInput6 << ADC_CONFIG_PSEL_Pos)         
                    | (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos)             
                    | (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) 
                    | (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos);
  
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
  NRF_ADC->TASKS_START = 1;
  while (!NRF_ADC->EVENTS_END)
  {}
  NRF_ADC->EVENTS_END = 0;

  uint16_t res = NRF_ADC->RESULT;  
  know_em_packet.light = res;
    
  NRF_ADC->TASKS_STOP = 1;

  release_clock();
}

static void reset_soil_sensor()
{
  request_clock();
  // take a reading the other way to get the juices flowing backwards
  // interrupt ADC
  NRF_ADC->INTENSET = (ADC_INTENSET_END_Disabled << ADC_INTENSET_END_Pos);


  nrf_gpio_pin_clear(SOIL_SIDE_A_PIN_NO);
  nrf_gpio_pin_set(SOIL_SIDE_B_PIN_NO);
          
  // config ADC
  NRF_ADC->CONFIG = (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos) /* Bits 17..16 : ADC external reference pin selection. */
                    | (ADC_CONFIG_PSEL_AnalogInput4 << ADC_CONFIG_PSEL_Pos)         
                    | (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos)             
                    | (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) 
                    | (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos);
  
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
  NRF_ADC->TASKS_START = 1;
  while (!NRF_ADC->EVENTS_END)
  {}
  NRF_ADC->EVENTS_END = 0;

  nrf_gpio_pin_clear(SOIL_SIDE_B_PIN_NO);

  // throw away the junk response
  NRF_ADC->RESULT;  
    
  NRF_ADC->TASKS_STOP = 1;
  release_clock();
}

static void read_soil()
{
  request_clock();
  // interrupt ADC
  NRF_ADC->INTENSET = (ADC_INTENSET_END_Disabled << ADC_INTENSET_END_Pos);

  nrf_gpio_pin_clear(SOIL_SIDE_B_PIN_NO);
  nrf_gpio_pin_set(SOIL_SIDE_A_PIN_NO);
          
  // config ADC
  NRF_ADC->CONFIG = (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos) /* Bits 17..16 : ADC external reference pin selection. */
                    | (ADC_CONFIG_PSEL_AnalogInput4 << ADC_CONFIG_PSEL_Pos)         
                    | (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos)             
                    | (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) 
                    | (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos);
  
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
  NRF_ADC->TASKS_START = 1;
  while (!NRF_ADC->EVENTS_END)
  {}
  NRF_ADC->EVENTS_END = 0;

  know_em_packet.soil = NRF_ADC->RESULT;  
    
  NRF_ADC->TASKS_STOP = 1;
  release_clock();
  nrf_gpio_pin_clear(SOIL_SIDE_A_PIN_NO);
}

static void go_to_sleep(void)
{
  led_off(POWER_LED_PIN_NO);
  uint16_t err_code = app_timer_start(sleeping_timer_id, SLEEP_DURATION, NULL);

  NRF_TWI1->ENABLE = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;

  APP_ERROR_CHECK(err_code);
}

static void startup(void)
{
  NRF_POWER->TASKS_LOWPWR = 1;

  leds_init();
  soil_sensor_init();
  scheduler_init();
  ble_stack_init();
  gap_params_init();
  advertising_init();
  timers_init();
}

int main(void)
{
  for (;;)
  { 
    switch(current_state) {
      case STARTING:
        startup();
        transition_to(READ_HUMIDITY);
        break;

      case READ_HUMIDITY:
        read_humidity();
        transition_to(READ_TEMP);
        break;
      case READ_TEMP:
        read_temp();
        transition_to(READ_LIGHT);
        break;
      case READ_LIGHT:
        read_light();
        transition_to(READ_SOIL);
        break;
      case READ_SOIL:
        read_soil();
        transition_to(RESET_SOIL_SENSOR);
        break;
      case RESET_SOIL_SENSOR:
        reset_soil_sensor();
        transition_to(START_ADVERTISING);
        break;
      case START_ADVERTISING:
        start_advertising();
        transition_to(ADVERTISING);
        break;
      case STOP_ADVERTISING:
        stop_advertising();
        transition_to(GO_TO_SLEEP);
        break;

      case GO_TO_SLEEP:
        go_to_sleep();
        transition_to(SLEEPING);
        break;

      case SLEEPING:
      case ADVERTISING:

      default:
        // app_sched_execute();
        power_manage();
    }
  }
}
