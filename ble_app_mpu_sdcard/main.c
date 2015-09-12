#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble_srv_common.h"
#include "ble_nrf6310_pins.h"
#include "ble_hci.h"
#include "boards.h"
#include "app_timer.h"
#include "ble_error_log.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "ble_debug_assert_handler.h"
#include "nrf_delay.h"
#include "nrf_adc.h"
#include "ble_mps.h"
#include "ble_init.h"
#include "simple_uart.h"
#include "spi_master.h"
#include "ff.h"
#include "diskio.h"
#include "integer.h"
#include "fattime.h"
#include "sdcard_manage.h"

#define APP_TIMER_MAX_TIMERS            3                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */
#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of users of the GPIOTE handler. */
#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define COMMAND_SET_CREDIT							0x21
#define COMMAND_SET_PEER_NUM						0x22
#define COMMAND_SET_SEQ_NUM							0x23

#define CLOCK_RESET											0x00
#define CLOCK_START											0x01
#define CLOCK_STOP											0x02

static app_timer_id_t										m_clock_timer_id;														/**Clock timer*/
static ble_init_t                       m_ble_init;
static mpu_config_t											m_mpu_config;
sdcard_fileio_t		 											m_sdcard_fileio;
static uint8_t													adc_data;
													
static bool															LED_BLINK_ON;

void blink(void)
{
		uint16_t len = MPS_FORMAT_TIME_LENGTH;
		uint8_t time_buf[len];
		sd_ble_gatts_value_get(m_ble_init.m_ble_mps.time_handle.value_handle, 0, &len, time_buf);
		if(LED_BLINK_ON)
			nrf_gpio_pin_set(ASSERT_LED_PIN_NO);
		else 
			nrf_gpio_pin_clear(ASSERT_LED_PIN_NO);

		LED_BLINK_ON = !LED_BLINK_ON;
}

/**@brief Error handler function, which is called when an error has occurred. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    nrf_gpio_pin_set(ASSERT_LED_PIN_NO);
    ble_debug_assert_handler(error_code, line_num, p_file_name);
    //NVIC_SystemReset();
}


/**@brief Assert macro callback function.*/
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief LEDs initialization.*/
static void leds_init(void)
{
    GPIO_LED_CONFIG(ADVERTISING_LED_PIN_NO);
    GPIO_LED_CONFIG(CONNECTED_LED_PIN_NO);
    GPIO_LED_CONFIG(ASSERT_LED_PIN_NO);
}
/**@brief Initialize button handler module.
 */
static void buttons_init(void)
{
    GPIO_WAKEUP_BUTTON_CONFIG(WAKEUP_BUTTON_PIN);
}


static void clock_tick_handler(void * p_context)
{
		uint16_t len = MPS_FORMAT_TIME_LENGTH;
		uint8_t format_time[len];
		memset(format_time,0,len);
		sd_ble_gatts_value_get(m_ble_init.m_ble_mps.time_handle.value_handle, 0, &len, format_time);
		
		format_time[0]++;
		if(format_time[0] == 10)
		{
			format_time[0]  = 0;
			format_time[1]++;
		}
		if(format_time[1] == 60)
		{
			format_time[1] = 0;
			format_time[2]++;
		}
		if(format_time[2] == 60)
		{
			format_time[2] = 0;
			format_time[3]++;
		}
		if(format_time[3] == 24)
		{
			format_time[3] = 0;
			format_time[4]++;
		}
		
		sd_ble_gatts_value_set(m_ble_init.m_ble_mps.time_handle.value_handle, 0, &len, (const uint8_t *) format_time);
}

/**@brief Timer initialization.*/
static void timers_init(void)
{
		uint32_t err_code;
	
    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
		err_code = app_timer_create(&m_clock_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                clock_tick_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Start timers.*/
static void timers_start(void)
{
		uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_clock_timer_id, CLOCK_TICK_100MS, NULL);
    APP_ERROR_CHECK(err_code);
}


void adc_init(void){
		nrf_gpio_range_cfg_output(8, 15);
    nrf_adc_init(ADC_RES_8bit, ADC_INPUT_AIN2_P01, ADC_INT_DISABLED);
}

/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.*/
static void battery_level_update(uint8_t adc_data)
{
    uint32_t err_code;
    
    err_code = ble_bas_battery_level_update(&m_ble_init.m_ble_bas, adc_data);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

/**@Brief Event handler when receiving commands (Command characteristic is written)*/
static void command_handler(ble_mps_t * p_mps, uint8_t * data, uint16_t length)
{		
		if(length%2 != 0)
		{	//invalid command format
				return;
		}
		if( m_ble_init.m_ble_mps.mode == MPS_MODE_READY )
		{ //MPU configuration is only allowed in ready mode
				for(int i =0 ; i < length/2 ; i++)
				{
						mpu_config_command(data[2*i], data[2*i+1]);
				}
		}
		else if( m_ble_init.m_ble_mps.mode == MPS_MODE_CALIBRATION)
		{
				
		}
}	

/**@Brief Event handler when clock is set (Time characteristic is written)*/
static void time_handler(ble_mps_t * p_mps, uint8_t * data, uint16_t length)
{	
		//Check time offset format
		if(data[0]>=10 || data[1]>=60 || data[2] >= 60 || data[3]>=24 )
			return;
		
		//Reset clock timer and time stamp
		app_timer_stop(m_clock_timer_id);
		timers_start();
}

/**@brief Power manager.*/
static void power_manage(void)
{
    uint32_t err_code = sd_app_event_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.*/
int main(void)
{		
	
		nrf_delay_ms(500);
    //Initialize MCU configuration
    leds_init();
    timers_init();
    buttons_init();
		adc_init();
		spi_master_init( SPI1, SPI_MODE3, false);

		//Initialize twi configuration & mpu9150
		mpu9150_init(&m_mpu_config, DEVICE_NAME);

		//Initialize BLE stack
		m_ble_init.m_clock_timer_id = m_clock_timer_id;
		m_ble_init.command_handler = command_handler;
		m_ble_init.timer_handler = time_handler;

		ble_init(&m_ble_init);

    if(!sdcard_fs_init(&m_sdcard_fileio, &m_mpu_config))
			    nrf_gpio_pin_set(ASSERT_LED_PIN_NO);

    //Start execution
    timers_start();
    advertising_start();
    
		
		adc_data = (uint8_t) nrf_adc_read();
		battery_level_update(adc_data);

    // Enter main loop
    for (;;)
    {				
				switch(m_ble_init.m_ble_mps.mode)
				{
					case MPS_MODE_DISCONNECT:
						
						power_manage();
						break;
						
					case MPS_MODE_READY:
						nrf_gpio_pin_clear(ASSERT_LED_PIN_NO);
						power_manage();
						break;
					
					case MPS_MODE_MEASURING:
						if(record())
							blink();
						break;
					
					case MPS_MODE_CALIBRATION:
						
						break;
					
					default:
            break;
				}
			
    }
}

/** 
 * @}
 */
