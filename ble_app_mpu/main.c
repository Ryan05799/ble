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
#include "mpu_config.h"
#include "simple_uart.h"

#define APP_TIMER_MAX_TIMERS            3                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */
#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of users of the GPIOTE handler. */
#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define CLOCK_TICK_10MS								  APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)  	
#define BLE_TRANSACTION_TIMEOUT					APP_TIMER_TICKS(4000, APP_TIMER_PRESCALER)	

#define COMMAND_SET_CREDIT							0x21
#define COMMAND_SET_PEER_NUM						0x22
#define COMMAND_SET_SEQ_NUM							0x23

#define CLOCK_RESET											0x00
#define CLOCK_START											0x01
#define CLOCK_STOP											0x02

static app_timer_id_t										m_clock_timer_id;														/**Clock timer*/
static app_timer_id_t										m_ble_trans_timer_id;												/**Timer for trigger BLE transaction timeout event*/
static ble_init_t                       m_ble_init;
static mpu_config_t											m_mpu_config;
static uint8_t													adc_data;

static uint32_t													time_stamp;

static bool															report_start;
static uint8_t													credit;
static uint8_t													peer_num;
static uint8_t													seq_num;
static bool															trans_timer_started;													

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
		uint16_t len = sizeof(time_stamp);
	
		time_stamp++;
		
		sd_ble_gatts_value_set(m_ble_init.m_ble_mps.time_handle.value_handle, 0, &len, (const uint8_t *) &time_stamp);
}

static void ble_transaction_timeout_handler(void * p_context)
{
		trans_timer_started = false;
		m_ble_init.m_ble_mps.mode = MPS_MODE_READY;//To make sure no HVN is being sending
		sd_ble_gap_disconnect(m_ble_init.m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
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
	
		err_code = app_timer_create(&m_ble_trans_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                ble_transaction_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Start timers.*/
static void timers_start(void)
{
		uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_clock_timer_id, CLOCK_TICK_10MS, NULL);
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

/**@Brief Function for getting MPU measurements according to MPU configuration*/
static uint16_t get_mpu_measurement(uint8_t *buf)
{
		uint16_t len = 0;
	
		if(m_mpu_config.acc_enable)
		{
				i2c_read(MPU9150_REG_ACCEL_XOUT, buf+len, 6);
				len+=6;
				
		}
		if(m_mpu_config.gyro_enable)
		{
				i2c_read(MPU9150_REG_GYRO_XOUT, buf+len, 6);
				len+=6;
				
		}
		if(m_mpu_config.mag_enable)
		{
				i2c_read(AK8975C_XOUT, buf+len, 6);
				len+=6;
				
		}
		if(m_mpu_config.temp_enable)
		{
				i2c_read(MPU9150_REG_TEMP_OUT, buf+len, 2);
				len+=2;
			
		}		
		
		return len;
}

/**@Brief Function for reporting MPU measurements by hvn packet*/
static void report_measurement_hvn(uint8_t *buf)
{
		uint32_t delay;
		uint16_t len;
	
		if(report_start)
		{//first delay
				report_start = false;
				delay = m_mpu_config.sampling_rate * seq_num;//Shift to avoid collision
				nrf_delay_ms(delay);
		}
		else if(credit > 0)
		{
				delay = m_mpu_config.sampling_rate * peer_num;
				nrf_delay_ms(delay);
				len = get_mpu_measurement(buf);
				ble_mps_send_hvn( &m_ble_init.m_ble_mps, buf, len);	
				credit--;
		}
		
}

/**@Brief Event handler when receiving commands (Command characteristic is written)*/
static void command_handler(ble_mps_t * p_mps, uint8_t * data, uint16_t length)
{
		uint32_t err_code;
		
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
		else if( m_ble_init.m_ble_mps.mode == MPS_MODE_REPORT)
		{	//
				if(data[0] == COMMAND_SET_CREDIT && data[2] == COMMAND_SET_PEER_NUM && data[4] == COMMAND_SET_SEQ_NUM )
				{
						report_start = true;
						credit = data[1];
						peer_num = data[3];
						seq_num = data[5];
				}
				else if(data[0] == COMMAND_SET_CREDIT)
				{
						credit = data[1];
				}
				//Reset transaction timer on each command write
				if(trans_timer_started)
				{
						err_code = app_timer_stop(m_ble_trans_timer_id);
						APP_ERROR_CHECK(err_code);
				}
				err_code = app_timer_start(m_ble_trans_timer_id, BLE_TRANSACTION_TIMEOUT, NULL);
				APP_ERROR_CHECK(err_code);
				trans_timer_started = true;
				
		}
		else if( m_ble_init.m_ble_mps.mode == MPS_MODE_CALIBRATION)
		{
				
		}
}	

/**@Brief Event handler when clock is set (Time characteristic is written)*/
static void time_handler(ble_mps_t * p_mps, uint8_t * data, uint16_t length)
{	
		switch(data[0])
		{
			case CLOCK_RESET:
					time_stamp = 0;
			break;
			
			case CLOCK_START:
				timers_start();
			break;
			
			case CLOCK_STOP:
				app_timer_stop(m_clock_timer_id);
				time_stamp = 0;
			break;
		}
}


/**@brief Power manager.*/
static void power_manage(void)
{
    uint32_t err_code = sd_app_event_wait();
    APP_ERROR_CHECK(err_code);
}


/**@Brief UART for debug*/
static void uart_init(void)
{
    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, false);
    
    NRF_UART0->INTENSET = UART_INTENSET_RXDRDY_Enabled << UART_INTENSET_RXDRDY_Pos;
    
    NVIC_SetPriority(UART0_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(UART0_IRQn);
}
static void send_msg_uart(char * msg)
{
		uint16_t len = sizeof(msg);
		msg[len] = '\n';
	
		simple_uart_putstring((uint8_t *) msg);
}


/**@brief Application main function.*/
int main(void)
{
		uart_init();
		
		uint8_t meas_buf[20];
		uint16_t meas_len;
		
		time_stamp = 0;
		
		
		trans_timer_started = false;
		credit = 0;
		peer_num = 1;
		seq_num = 1;
	
    //Initialize MCU configuration
    leds_init();
    timers_init();
    buttons_init();
		adc_init();
	
		//Initialize twi configuration & mpu9150
		mpu9150_init(&m_mpu_config);
		
		//Initialize BLE stack
		m_ble_init.command_handler = command_handler;
		m_ble_init.timer_handler = time_handler;
		ble_init(&m_ble_init);
    
    
    //Start execution
    timers_start();
    advertising_start();
    
		
		adc_data = (uint8_t) nrf_adc_read();
		battery_level_update(adc_data);
		

    // Enter main loop
    for (;;)
    {
        //power_manage();

				switch(m_ble_init.m_ble_mps.mode)
				{
					case MPS_MODE_DISCONNECT:
						
						power_manage();
						break;
						
					case MPS_MODE_READY:
						
						power_manage();
						break;
					
					case MPS_MODE_MEASURING:
						memset(meas_buf, 0, sizeof(meas_buf));
						meas_len = get_mpu_measurement(meas_buf);
						sd_ble_gatts_value_set(m_ble_init.m_ble_mps.measurement_handle.value_handle, 0, &meas_len, meas_buf);
						break;
					
					case MPS_MODE_REPORT:
						report_measurement_hvn(meas_buf);
						
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
