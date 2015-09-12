#include <stdint.h>
#include <string.h>
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_nrf6310_pins.h"
#include "ble_mps.h"
#include "ble_init.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "app_timer.h"

static ble_init_t * p_ble_init;
static int adv_attempts;

/**@brief GAP initialization.
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
    ble_advdata_t scanrsp;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    
    ble_uuid_t adv_uuids[] = {{BLE_UUID_MPS_SERVICE, p_ble_init->m_ble_mps.uuid_type}};

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


/**@brief Initialize services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t err_code;
		ble_bas_init_t  bas_init;
		
		p_ble_init->m_ble_mps.m_clock_timer_id = p_ble_init->m_clock_timer_id;
    p_ble_init->m_ble_mps.command_handler = p_ble_init->command_handler;
		p_ble_init->m_ble_mps.timer_handler = p_ble_init->timer_handler;

    err_code = ble_mps_init(&p_ble_init->m_ble_mps);
    APP_ERROR_CHECK(err_code);

		// Initialize Battery Service
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
    
    err_code = ble_bas_init(&p_ble_init->m_ble_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Initialize security parameters.
 */
static void sec_params_init(void)
{
    p_ble_init->m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    p_ble_init->m_sec_params.bond         = SEC_PARAM_BOND;
    p_ble_init->m_sec_params.mitm         = SEC_PARAM_MITM;
    p_ble_init->m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    p_ble_init->m_sec_params.oob          = SEC_PARAM_OOB;  
    p_ble_init->m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    p_ble_init->m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief Connection Parameters Module handler.*/
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
				if(p_ble_init->m_ble_mps.mode != MPS_MODE_REPORT)
				{
						err_code = sd_ble_gap_disconnect(p_ble_init->m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
						APP_ERROR_CHECK(err_code);
				}

    }
}


/**@brief Connection Parameters module error handler.*/
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


/**@brief Start advertising.*/
void advertising_start(void)
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


/**@brief Application's BLE Stack event handler.*/
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
            p_ble_init->m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						adv_attempts = 0;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
            p_ble_init->m_conn_handle = BLE_CONN_HANDLE_INVALID;

            if (err_code == NRF_SUCCESS)
            {
                advertising_start();
            }
            break;
            
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(p_ble_init->m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_SUCCESS, 
                                                   &p_ble_init->m_sec_params);
            break;
            
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(p_ble_init->m_conn_handle, NULL, 0);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;
            
        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            p_enc_info = &m_auth_status.periph_keys.enc_info;
            if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div)
            {
                err_code = sd_ble_gap_sec_info_reply(p_ble_init->m_conn_handle, p_enc_info, NULL);
            }
            else
            {
                // No keys found for this device
                err_code = sd_ble_gap_sec_info_reply(p_ble_init->m_conn_handle, NULL, NULL);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            { 
                nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
								if( adv_attempts < ADVERTISE_MAX_RETRY)
								{
										advertising_start();
										adv_attempts++;
								}
								else
								{ 	// Go to system-off mode (this function will not return; wakeup will cause a reset)
										GPIO_WAKEUP_BUTTON_CONFIG(WAKEUP_BUTTON_PIN);
										err_code = sd_power_system_off();
										adv_attempts = 0;
								}
 
            }
            break;

        default:
            break;
    }

    APP_ERROR_CHECK(err_code);
}


/**@brief Dispatches a BLE stack event to all modules with a BLE stack event handler. */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_mps_on_ble_evt(&p_ble_init->m_ble_mps, p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief BLE stack initialization.*/
static void ble_stack_init(void)
{
    BLE_STACK_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM,
                           BLE_L2CAP_MTU_DEF,
                           ble_evt_dispatch,
                           false);
}


void ble_init(ble_init_t * p_init)
{
		p_ble_init = p_init;
		p_ble_init->m_conn_handle = BLE_CONN_HANDLE_INVALID;   
		
		adv_attempts = 0;

		ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    sec_params_init();
		
		uint16_t len = sizeof(MPS_MESSAGE_DEFAULT);
		sd_ble_gatts_value_set(p_init->m_ble_mps.message_handle.value_handle, 0, &len, (const uint8_t *) MPS_MESSAGE_DEFAULT);
}
