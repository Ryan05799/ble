#include <stdint.h>
#include <stdbool.h>
#include "ble_mps.h"
#include "ble_bas.h"
#include "ble_conn_params.h"
#include "app_timer.h"
#include "ble_stack_handler.h"

#define DEVICE_NAME                     "MPU_Sensor"                           /**< Name of device. Will be included in the advertising data. */
#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */

#define MIN_CONN_INTERVAL               16                                          /**< Minimum acceptable connection interval (7.5 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               40                                          /**< Maximum acceptable connection interval (7.5 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                400                                         /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define ADVERTISE_MAX_RETRY							5																						/***/	

#define WAKEUP_BUTTON_PIN               NRF6310_BUTTON_0                            /**< Button used to wake up the application. */


typedef struct ble_init_s ble_init_t;

typedef struct ble_init_s
{
	ble_gap_sec_params_t            m_sec_params;                               /**< Security requirements for this application. */
	uint16_t                        m_conn_handle;    													/**< Handle of the current connection. */
	ble_mps_t												m_ble_mps;																	
	ble_bas_t												m_ble_bas;																	
	ble_mps_command_handler_t       command_handler;
	ble_mps_timer_handler_t       	timer_handler;
}	ble_init_t;


//Function for initailze BLE
void ble_init(ble_init_t * p_ble_init);

//Function for start advertising
void advertising_start(void);
