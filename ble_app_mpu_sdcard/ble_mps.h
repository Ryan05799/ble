#ifndef BLE_MPS_H__
#define BLE_MPS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "app_timer.h"

#define BLE_UUID_MPS_SERVICE 0x0001
#define BLE_UUID_MPS_RECORDS_CHARACTERISTIC 0x0101
#define BLE_UUID_MPS_MODE_CHARACTERISTIC 0x0201
#define BLE_UUID_MPS_COMMAND_CHARACTERISTIC 0x0202
#define BLE_UUID_MPS_TIME_CHARACTERISTIC 0x0301
#define BLE_UUID_MPS_MESSAGE_CHARACTERISTIC 0x0302

#define MPS_MODE_DISCONNECT 0x00
#define MPS_MODE_READY 0x01
#define MPS_MODE_MEASURING 0x02
#define MPS_MODE_REPORT 0x03
#define MPS_MODE_CALIBRATION 0x04 

#define MPS_MESSAGE_DEFAULT "DEFAULT MESSAGE"
#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define MPS_RECONNECT_TIMEOUT APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)
#define CLOCK_TICK_100MS			APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)  	

#define MPS_FORMAT_TIME_LENGTH 6;											// Size of the time characteristic value. Format: (m)/(d)/(h)/(m)/(s)/(ms)

typedef struct ble_mps_init_s ble_mps_init_t;

typedef struct ble_mps_s ble_mps_t;

typedef void (*ble_mps_write_handler_t) (ble_mps_t * p_mps, uint8_t * data, uint16_t length);

/**MPU Service structure*/
typedef struct ble_mps_s
{    
	uint8_t                 	uuid_type;               	/**< UUID type for MPU Service Base UUID. */
  uint16_t                	service_handle;          	/**< Handle of MPU Service (as provided by the BLE stack). */
	uint16_t                	conn_handle;             	/**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
	ble_gatts_char_handles_t	records_handle;						/**< Handles related to the records number characteristic. */
	ble_gatts_char_handles_t	message_handle;						/**< Handles related to the file message characteristic. */
	ble_gatts_char_handles_t	mode_handle;							/**< Handles related to the Mode characteristic. */
	ble_gatts_char_handles_t	command_handle;						/**< Handles related to the Command characteristic. */
	ble_gatts_char_handles_t	time_handle;							/**< Handles related to the Time characteristic. */

	bool											list_hvn_enabled;
	uint8_t										mode;
	app_timer_id_t						m_clock_timer_id;					/**< Timer for reconnect timeout*/
	ble_mps_write_handler_t		command_handler;					/**< Event handler to be called for handling received command. */
	ble_mps_write_handler_t		timer_handler;						/**< Event handler to be called for handling Time characterisic related operations. */
} ble_mps_t;

//Function for initializing MPU service
uint32_t ble_mps_init(ble_mps_t * p_mps);

//Function for sending out a hvn packet
uint32_t ble_mps_send_hvn(ble_mps_t * p_mps, uint8_t * string, uint16_t length);

//Event handler for MPU Service
void ble_mps_on_ble_evt(ble_mps_t * p_mps, ble_evt_t * p_ble_evt);

//Functions for reading formatted time from the Time characteristic
void get_time(uint8_t *read_buf);
void get_time_str(char *read_buf);

//Functions for getting message charateristic 
void get_mps_message(char * buffer);

void set_mps_records(char *buffer);	


#endif
