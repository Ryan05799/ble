#ifndef BLE_MPS_H__
#define BLE_MPS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define BLE_UUID_MPS_SERVICE 0x0001
#define BLE_UUID_MPS_MEASUREMENT_CHARACTERISTIC 0x0101
#define BLE_UUID_MPS_EVENT_CHARACTERISTIC 0x0102
#define BLE_UUID_MPS_MODE_CHARACTERISTIC 0x0201
#define BLE_UUID_MPS_COMMAND_CHARACTERISTIC 0x0202
#define BLE_UUID_MPS_TIME_CHARACTERISTIC 0x0301
#define BLE_UUID_MPS_EXTRA_CHARACTERISTIC 0x0302

#define MPS_MODE_DISCONNECT 0x00
#define MPS_MODE_READY 0x01
#define MPS_MODE_MEASURING 0x02
#define MPS_MODE_REPORT 0x03
#define MPS_MODE_CALIBRATION 0x04 



typedef struct ble_mps_init_s ble_mps_init_t;

typedef struct ble_mps_s ble_mps_t;

typedef void (*ble_mps_command_handler_t) (ble_mps_t * p_mps, uint8_t * data, uint16_t length);

typedef void (*ble_mps_timer_handler_t) (ble_mps_t * p_mps, uint8_t * data, uint16_t length );

/**MPU Service structure*/
typedef struct ble_mps_s
{    
	uint8_t                 	uuid_type;               	/**< UUID type for MPU Service Base UUID. */
  uint16_t                	service_handle;          	/**< Handle of MPU Service (as provided by the BLE stack). */
	uint16_t                	conn_handle;             	/**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
	ble_gatts_char_handles_t	measurement_handle;				/**< Handles related to the Measurement characteristic. */
	ble_gatts_char_handles_t	mode_handle;							/**< Handles related to the Mode characteristic. */
	ble_gatts_char_handles_t	command_handle;						/**< Handles related to the Command characteristic. */
	ble_gatts_char_handles_t	time_handle;							/**< Handles related to the Time characteristic. */
	ble_gatts_char_handles_t	extra_handle;							/**< Handles related to the Extra characteristic. */
	bool											meas_hvn_enabled;
	uint8_t										mode;
	ble_mps_command_handler_t	command_handler;					/**< Event handler to be called for handling received command. */
	ble_mps_timer_handler_t		timer_handler;						/**< Event handler to be called for handling Time characterisic related operations. */
	
} ble_mps_t;

//Function for initializing MPU service
uint32_t ble_mps_init(ble_mps_t * p_mps);

//Function for sending out a hvn packet
uint32_t ble_mps_send_hvn(ble_mps_t * p_mps, uint8_t * string, uint16_t length);

//Event handler for MPU Service
void ble_mps_on_ble_evt(ble_mps_t * p_mps, ble_evt_t * p_ble_evt);

#endif
