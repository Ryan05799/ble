/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup from button, advertise, get a connection
 * restart advertising on disconnect and if no new connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified with 'YOUR_JOB' indicates where
 * and how you can customize.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_nrf6310_pins.h"
#include "app_scheduler.h"
#include "ble_stack_handler.h"
#include "app_timer.h"
#include "ble_error_log.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "ble_debug_assert_handler.h"
#include "ble_nus.h"
#include "simple_uart.h"
#include "boards.h"
#include "twi_master.h"
#include "nrf_adc.h"
#include "spi_master.h"
#include "ff.h"

#define WAKEUP_BUTTON_PIN               NRF6310_BUTTON_0                            /**< Button used to wake up the application. */

#define DEVICE_NAME                     "Triaxial"                           /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                       /**< The advertising timeout (in units of seconds). */

// YOUR_JOB: Modify these according to requirements.
#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            2                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               16                                           /**< Minimum acceptable connection interval (7.5 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               40                                           /**< Maximum acceptable connection interval (7.5 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                400                                         /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of users of the GPIOTE handler. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define	ADXL345_W_ADDRESS7          		0xA6																				/**I2C write address for ADXL345 accelerometer		*/		
#define	ADXL345_R_ADDRESS7          		0xA7																				/**I2C read address for ADXL345 accelerometer	*/
#define TIMEOUT_LIMIT										20

#define MPU9150_W_ADDRESS								0xD0
#define MPU9150_R_ADDRESS								0xD1	
#define AK8975C_W_ADDRESS								0x18
#define AK8975C_R_ADDRESS								0x19		

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_nus_t                        m_nus;

static uint8_t													cmd_buffer[2]; 
static uint8_t													i2c_buffer[6];
static uint8_t													data_txt_buffer[14];
static int															acc_data[3];
static uint8_t													adv_timeout_count;
static uint32_t													adc_data;


/**@brief Error handler function, which is called when an error has occurred. 
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
    nrf_gpio_pin_set(ASSERT_LED_PIN_NO);

    // This call can be used for debug purposes during development of an application.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}


/**@brief Assert macro callback function.
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


/**@brief Service error handler.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the 
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
/*
// YOUR_JOB: Uncomment this function and make it handle error situations sent back to your 
//           application by the services it uses.
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
} */


/**@brief LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    GPIO_LED_CONFIG(ADVERTISING_LED_PIN_NO);
    GPIO_LED_CONFIG(CONNECTED_LED_PIN_NO);
    GPIO_LED_CONFIG(ASSERT_LED_PIN_NO);
	
		GPIO_LED_CONFIG(UART_TEST_LED_PIN_NO);
}


/**@brief Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
    
    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
    err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
    APP_ERROR_CHECK(err_code); */
}


/**@brief GAP initialization.
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
    
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;//scan response
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
	
    adv_timeout_count = 0;//Show how many times that advertising fails
	
    ble_uuid_t adv_uuids[] = {{BLE_UUID_NUS_SERVICE, m_nus.uuid_type}};

    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;
    
    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}

void receive_data_handler(ble_nus_t * p_nus, uint8_t * data, uint16_t length)
{
    //data[length] = '\0';	
		cmd_buffer[0] = data[0];
		cmd_buffer[1] = data[1];
}


/**@brief Initialize services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t err_code;
    static ble_nus_init_t nus_init;
    
    memset(&nus_init, 0, sizeof nus_init);
    nus_init.data_handler = receive_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Initialize security parameters.
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


/**@brief Connection Parameters Module handler.
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


/**@brief Connection Parameters module error handler.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Initialize the Connection Parameters module.
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


/**@brief Start timers.
*/
static void timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
    uint32_t err_code;
    
    err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code); */
}


/**@brief Start advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;
    
    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));
    
    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}


/**@brief Application's BLE Stack event handler.
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
            nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
            nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						//adv_timeout_count = 0;
            /* YOUR_JOB: Uncomment this part if you are using the app_button module to handle button
                         events (assuming that the button events are only needed in connected
                         state). If this is uncommented out here,
                            1. Make sure that app_button_disable() is called when handling
                               BLE_GAP_EVT_DISCONNECTED below.
                            2. Make sure the app_button module is initialized.
            err_code = app_button_enable();
            */
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            /* YOUR_JOB: Uncomment this part if you are using the app_button module to handle button
                         events. This should be done to save power when not connected
                         to a peer.
            err_code = app_button_disable();
            */
            if (err_code == NRF_SUCCESS)
            {
                advertising_start();
            }
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
                nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
								GPIO_WAKEUP_BUTTON_CONFIG(WAKEUP_BUTTON_PIN);
                // Go to system-off mode if exceeds timeout limit, otherwise restart advertising
								if(adv_timeout_count < TIMEOUT_LIMIT)
								{
									advertising_start();
									adv_timeout_count++;
								
								}else
								{

									adv_timeout_count = 0;
									err_code = sd_power_system_off();  
								}
  
            }
            break;

        default:
            break;
    }

    APP_ERROR_CHECK(err_code);
}


/**@brief Dispatches a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief BLE stack initialization.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    // YOUR_JOB: If the MTU size is changed by the application, the MTU_SIZE parameter to
    //           BLE_STACK_HANDLER_INIT() must be changed accordingly.
    BLE_STACK_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM,
                           BLE_L2CAP_MTU_DEF,
                           ble_evt_dispatch,
                           false);
}

/**@brief Initialize button handler module.
 */
static void buttons_init(void)
{
    GPIO_WAKEUP_BUTTON_CONFIG(WAKEUP_BUTTON_PIN);
}


/**@brief Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_event_wait();
    APP_ERROR_CHECK(err_code);
}

/*
static void uart_init(void)
{
    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, false);
    
    NRF_UART0->INTENSET = UART_INTENSET_RXDRDY_Enabled << UART_INTENSET_RXDRDY_Pos;
    
    NVIC_SetPriority(UART0_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(UART0_IRQn);
}
*/
void UART0_IRQHandler(void)
{
    static uint8_t data_array[NUS_MAX_DATA_LENGTH];
    static uint8_t index = 0;
    uint32_t err_code;
    
	
	
    data_array[index] = simple_uart_get();
    if (data_array[index] == '\n' || index >= NUS_MAX_DATA_LENGTH)
    {
        err_code = ble_nus_send_string(&m_nus, data_array, index);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
        
        index = 0;
    }
    else
    {
        index++;
    }
}
void ADC_init(void){
		nrf_gpio_range_cfg_output(8, 15);
    nrf_adc_init(ADC_RES_8bit, ADC_INPUT_AIN2_P01, ADC_INT_DISABLED);
}

void MPU9150_init(void)
{
		i2c_buffer[0]=0x19;
		i2c_buffer[1]=0x18;
		twi_master_transfer(MPU9150_W_ADDRESS, i2c_buffer, 2 ,TWI_ISSUE_STOP);
		i2c_buffer[0]=0x1A;
		i2c_buffer[1]=0x01;
		twi_master_transfer(MPU9150_W_ADDRESS, i2c_buffer, 2 ,TWI_ISSUE_STOP);
		i2c_buffer[0]=0x1B;
		i2c_buffer[1]=0x02;
		twi_master_transfer(MPU9150_W_ADDRESS, i2c_buffer, 2 ,TWI_ISSUE_STOP);
		i2c_buffer[0]=0x1C;
		i2c_buffer[1]=0x08;
		twi_master_transfer(MPU9150_W_ADDRESS, i2c_buffer, 2 ,TWI_ISSUE_STOP);
		i2c_buffer[0]=0x23;
		i2c_buffer[1]=0x00;
		twi_master_transfer(MPU9150_W_ADDRESS, i2c_buffer, 2 ,TWI_ISSUE_STOP);
		i2c_buffer[0]=0x37;
		i2c_buffer[1]=0x02;
		twi_master_transfer(MPU9150_W_ADDRESS, i2c_buffer, 2 ,TWI_ISSUE_STOP);
		i2c_buffer[0]=0x38;
		i2c_buffer[1]=0x00;
		twi_master_transfer(MPU9150_W_ADDRESS, i2c_buffer, 2 ,TWI_ISSUE_STOP);
		i2c_buffer[0]=0x6A;
		i2c_buffer[1]=0x00;
		twi_master_transfer(MPU9150_W_ADDRESS, i2c_buffer, 2 ,TWI_ISSUE_STOP);
		i2c_buffer[0]=0x6B;
		i2c_buffer[1]=0x00;
		twi_master_transfer(MPU9150_W_ADDRESS, i2c_buffer, 2 ,TWI_ISSUE_STOP);
}
void MPU9150_get(uint8_t reg){
				i2c_buffer[0]=reg;
				twi_master_transfer(MPU9150_W_ADDRESS, i2c_buffer, 1 ,TWI_ISSUE_STOP);
				twi_master_transfer(MPU9150_R_ADDRESS, i2c_buffer, 6 ,TWI_ISSUE_STOP);
				
									
				data_txt_buffer[0]=HEX2ASCII_Hi(i2c_buffer[0]);
				data_txt_buffer[1]=HEX2ASCII_Low(i2c_buffer[0]);
				data_txt_buffer[2]=HEX2ASCII_Hi(i2c_buffer[1]);
				data_txt_buffer[3]=HEX2ASCII_Low(i2c_buffer[1]);
				data_txt_buffer[5]=HEX2ASCII_Hi(i2c_buffer[2]);
				data_txt_buffer[6]=HEX2ASCII_Low(i2c_buffer[2]);
				data_txt_buffer[7]=HEX2ASCII_Hi(i2c_buffer[3]);
				data_txt_buffer[8]=HEX2ASCII_Low(i2c_buffer[3]);
				data_txt_buffer[10]=HEX2ASCII_Hi(i2c_buffer[4]);
				data_txt_buffer[11]=HEX2ASCII_Low(i2c_buffer[4]);
				data_txt_buffer[12]=HEX2ASCII_Hi(i2c_buffer[5]);
				data_txt_buffer[13]=HEX2ASCII_Low(i2c_buffer[5]);
				data_txt_buffer[4] = ';';
				data_txt_buffer[9] = ';';
				ble_nus_send_string(&m_nus, data_txt_buffer, 14);			
}

/**@brief Application main function.
 */
int main(void)
{
    // Initialize
    leds_init();
    timers_init();
    buttons_init();
    //uart_init();
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    sec_params_init();
	  spi_master_init( SPI1, SPI_MODE3, false);
		twi_master_init();
		MPU9150_init();	
    
    // Start execution
    timers_start();
    advertising_start();

		ADC_init();
		
		
		nrf_delay_ms(500);
	
		//FF
		//FAT file system
		FATFS *fs_getfree;
		FATFS fs;            // Work area (file system object) for logical drive
		FIL fsrc, fdst;      // file objects
		//BYTE buffer[512];   // file copy buffer
		FRESULT res;         // FatFs function common result code
		UINT br, bw;         // File R/W count
		DWORD j32;+
		
		f_mount(0, &fs);
		res = f_getfree("0:", &j32, &fs_getfree);
		
		
		f_mkdir("CoraTest");

		if(res!=FR_OK)
		{
			 nrf_gpio_pin_set(ASSERT_LED_PIN_NO);
		}
		
		res = f_open(&fdst, "info.txt", FA_OPEN_ALWAYS | FA_READ);
	 
		//res = f_write(&fdst, "aaa/r/n", 5, &bw);
				
		if(res!=FR_OK)
		{
			nrf_gpio_pin_set(ASSERT_LED_PIN_NO);
		}
		
		f_close(&fdst);
				
		// Enter main loop
    for (;;)
    {
			power_manage();
			//cmd_buffer[0] = 0x01;
			switch(cmd_buffer[0])	
			{
				case CMD_SEND_ACC_DATA :
					MPU9150_get(0x3B);

					int count_down = 100*(int)cmd_buffer[1]; 
					while( count_down >0)
						count_down--;
					ble_nus_send_string(&m_nus, data_txt_buffer, 14);
					
					cmd_buffer[0] = 0x00;
				break;
					
				case CMD_SEND_GYRO_DATA :
					MPU9150_get(0x43);
					ble_nus_send_string(&m_nus, data_txt_buffer, 14);
					cmd_buffer[0] = 0x00;
				break;
				
				case CMD_SEND_TEMP_DATA:
					MPU9150_get(0x41);
					ble_nus_send_string(&m_nus, data_txt_buffer, 4);
					cmd_buffer[0] = 0x00;
				break;
				
				case CMD_SEND_BAT_DATA :
					adc_data = nrf_adc_read();
					data_txt_buffer[0]=HEX2ASCII_Hi((adc_data & 0xFF000000)>>24);
					data_txt_buffer[1]=HEX2ASCII_Low((adc_data & 0xFF000000)>>24);
					data_txt_buffer[2]=HEX2ASCII_Hi((adc_data & 0x00FF0000)>>16);
					data_txt_buffer[3]=HEX2ASCII_Low((adc_data & 0x00FF0000)>>16);
					data_txt_buffer[4]=HEX2ASCII_Hi((adc_data & 0x0000FF00)>>8);
					data_txt_buffer[5]=HEX2ASCII_Low((adc_data & 0x0000FF00)>>8);
					data_txt_buffer[6]=HEX2ASCII_Hi((adc_data & 0x000000FF));
					data_txt_buffer[7]=HEX2ASCII_Low((adc_data & 0x000000FF));
					ble_nus_send_string(&m_nus, data_txt_buffer, 8);
				  cmd_buffer[0] = 0x00;
				break;
				
				

			}

    }
}

/** 
 * @}
 */
