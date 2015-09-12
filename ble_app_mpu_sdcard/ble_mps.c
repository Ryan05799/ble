#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include "ble_mps.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_timer.h"
#include "sdcard_manage.h"

#include "nrf_gpio.h"
#include "ble_nrf6310_pins.h"

ble_mps_t * p_ble_mps;

void set_mps_records(char *buffer)	
{
		uint16_t len = 4;
		sd_ble_gatts_value_set(p_ble_mps->records_handle.value_handle, 0, &len, (uint8_t *) buffer);

}
void get_mps_message(char * buffer)
{
		uint16_t len = 20;
		sd_ble_gatts_value_get(p_ble_mps->message_handle.value_handle, 0, &len, (uint8_t *) buffer);
		
}

void get_time(uint8_t *read_buf)
{		
		uint16_t len = MPS_FORMAT_TIME_LENGTH;
		uint8_t buf[len];
		sd_ble_gatts_value_get(p_ble_mps->time_handle.value_handle, 0, &len, buf);
		memcpy(read_buf, buf, len);
	  app_timer_start(p_ble_mps->m_clock_timer_id, CLOCK_TICK_100MS, NULL);

}

void get_time_str(char * buffer)
{
		uint8_t time_buf[6];
		memset(time_buf, 0, sizeof(time_buf));
		//Check buffer size
		get_time(time_buf);
		//Convert each byte to 2-bit decimal number
		
		for(int i = 0; i < 6; i++){
				buffer[2*i] = (char)(((int)'0')+time_buf[5-i]/10);
				buffer[2*i+1] = (char)(((int)'0')+time_buf[5-i]%10);			
		}
}


/**@brief Connect event handler.*/
static void on_connect(ble_mps_t * p_mps, ble_evt_t * p_ble_evt)
{
		p_mps->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		p_mps->mode = MPS_MODE_READY;
		uint16_t len = sizeof(uint8_t);
		sd_ble_gatts_value_set(p_mps->mode_handle.value_handle, 0, &len, &p_mps->mode);
		
}

/**@brief disconnect event handler.*/
static void on_disconnect(ble_mps_t * p_mps, ble_evt_t * p_ble_evt)
{
		p_mps->conn_handle = BLE_CONN_HANDLE_INVALID;
		if(p_mps->mode == MPS_MODE_MEASURING)
		{	//Disconnect when measuring, terminate measuring and close file
			stop_record();
		}
		p_mps->mode = MPS_MODE_DISCONNECT;

}

/**@brief Write event handler.*/
static void on_write(ble_mps_t * p_mps, ble_evt_t * p_ble_evt)
{
		ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
		
		//Mode set
		if( p_evt_write->handle == p_mps->mode_handle.value_handle )
		{
				uint8_t *mode = p_evt_write->data;
				
				if( mode[0] == MPS_MODE_READY)
				{
						if(p_mps->mode == MPS_MODE_MEASURING)
						{//Stop measuring
							stop_record();
						}
						p_mps->mode = MPS_MODE_READY;
				}
				else if( (p_mps->mode == MPS_MODE_READY) &&
					( mode[0] == MPS_MODE_MEASURING || mode[0] == MPS_MODE_REPORT || mode[0] == MPS_MODE_CALIBRATION))
				{
						p_mps->mode = mode[0];
						if( mode[0] == MPS_MODE_MEASURING)
						{
							//Start recording measurements					
							char time[12];
							char filename[] = "xxxxxxxx.txt";
							memset(time, 0, sizeof(time));
							get_time_str(time);
							strncpy(filename,time,8);
							start_record(filename);
						}
				}
				else
				{		//Invalid mode
						uint16_t len = sizeof(uint8_t);
						sd_ble_gatts_value_set(p_mps->mode_handle.value_handle, 0, &len, &p_mps->mode);
				}
				
		}
		
		//Receive commands
		if( p_evt_write->handle == p_mps->command_handle.value_handle )
		{
				p_mps->command_handler(p_mps, p_evt_write->data , p_evt_write->len);
		}
		//Clock set
		if( p_evt_write->handle == p_mps->time_handle.value_handle )
		{
				p_mps->timer_handler(p_mps, p_evt_write->data , p_evt_write->len);
		}
		
}

void ble_mps_on_ble_evt(ble_mps_t * p_mps, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_mps, p_ble_evt);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_mps, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write(p_mps, p_ble_evt);
            break;
            
        default:
            break;
    }
}


/**@Brief Funtions for adding charactersitics to BLE stack
*/
static uint32_t records_char_add(ble_mps_t * p_mps)
{
	  ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    
    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    
    memset(&char_md, 0, sizeof(char_md));
    
	  char_md.char_props.read   = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_mps->uuid_type;
    ble_uuid.uuid = BLE_UUID_MPS_RECORDS_CHARACTERISTIC;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint32_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint32_t);
    
    return sd_ble_gatts_characteristic_add(p_mps->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_mps->records_handle);
}

static uint32_t message_char_add(ble_mps_t * p_mps)
{
	  ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    
    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    
    memset(&char_md, 0, sizeof(char_md));
    
	  char_md.char_props.read   = 1;
		char_md.char_props.write   = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_mps->uuid_type;
    ble_uuid.uuid = BLE_UUID_MPS_MESSAGE_CHARACTERISTIC;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = 20;
    
    return sd_ble_gatts_characteristic_add(p_mps->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_mps->message_handle);
}

static uint32_t mode_char_add(ble_mps_t * p_mps)
{
		ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.write  = 1;
	  char_md.char_props.read   = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_mps->uuid_type;
    ble_uuid.uuid = BLE_UUID_MPS_MODE_CHARACTERISTIC;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = 1;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = 1;
    
    return sd_ble_gatts_characteristic_add(p_mps->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_mps->mode_handle);
}

static uint32_t cmd_char_add(ble_mps_t * p_mps)
{
		ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_mps->uuid_type;
    ble_uuid.uuid = BLE_UUID_MPS_COMMAND_CHARACTERISTIC;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = 1;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = 20;
    
    return sd_ble_gatts_characteristic_add(p_mps->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_mps->command_handle);
}



static uint32_t time_char_add(ble_mps_t * p_mps)
{
		ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.write  = 1;
	  char_md.char_props.read   = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_mps->uuid_type;
    ble_uuid.uuid = BLE_UUID_MPS_TIME_CHARACTERISTIC;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = MPS_FORMAT_TIME_LENGTH;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = MPS_FORMAT_TIME_LENGTH;
    
    return sd_ble_gatts_characteristic_add(p_mps->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_mps->time_handle);
}

/**@Brief Function for initialize MPU Service*/
uint32_t ble_mps_init(ble_mps_t * p_mps)
{
		uint32_t   err_code;
    ble_uuid_t ble_uuid;
    ble_uuid128_t nus_base_uuid = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E};
    
		p_ble_mps = p_mps;
		
    // Initialize service structure
    p_mps->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_mps->list_hvn_enabled = true;
    p_mps->mode = MPS_MODE_DISCONNECT;

    // Add custom base UUID
    err_code = sd_ble_uuid_vs_add(&nus_base_uuid, &p_mps->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add service
    ble_uuid.type = p_mps->uuid_type;
    ble_uuid.uuid = BLE_UUID_MPS_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_mps->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }    
		
		err_code = records_char_add(p_mps);
		if (err_code != NRF_SUCCESS)
		{
			return err_code;
		}
		
		err_code = mode_char_add(p_mps);
		if (err_code != NRF_SUCCESS)
		{
			return err_code;
		}
		
		err_code = cmd_char_add(p_mps);
		if (err_code != NRF_SUCCESS)
		{
			return err_code;
		}
		
		err_code = time_char_add(p_mps);
		if (err_code != NRF_SUCCESS)
		{
			return err_code;
		}

		err_code = message_char_add(p_mps);
		if (err_code != NRF_SUCCESS)
		{
			return err_code;
		}
		
		return NRF_SUCCESS;
}
