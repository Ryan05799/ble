#include "ble_mps.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"

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
		p_mps->mode = MPS_MODE_DISCONNECT;

}

/**@brief Write event handler.*/
static void on_write(ble_mps_t * p_mps, ble_evt_t * p_ble_evt)
{
		ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

		//notification enabled
		if( p_evt_write->handle == p_mps->measurement_handle.cccd_handle )
		{
				uint16_t value = *(uint16_t *) p_evt_write->data;
        if (value == 0x01)
        {
            p_mps->meas_hvn_enabled = true;
        }
        else
        {
            p_mps->meas_hvn_enabled = false;
        }
		}
		
		//Mode set
		if( p_evt_write->handle == p_mps->mode_handle.value_handle )
		{
				uint8_t *mode = p_evt_write->data;
			
				if( mode[0] == MPS_MODE_READY)
				{
						p_mps->mode = MPS_MODE_READY;
				}
				else if( (p_mps->mode == MPS_MODE_READY) &&
					( mode[0] == MPS_MODE_MEASURING || mode[0] == MPS_MODE_REPORT || mode[0] == MPS_MODE_CALIBRATION))
				{
						p_mps->mode = mode[0];
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
static uint32_t meas_char_add(ble_mps_t * p_mps)
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
    
    char_md.char_props.notify = 1;
	  char_md.char_props.read   = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_mps->uuid_type;
    ble_uuid.uuid = BLE_UUID_MPS_MEASUREMENT_CHARACTERISTIC;
    
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
                                               &p_mps->measurement_handle);
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
    attr_char_value.init_len     = 1;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = 6;
    
    return sd_ble_gatts_characteristic_add(p_mps->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_mps->time_handle);
}

static uint32_t extra_char_add(ble_mps_t * p_mps)
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
    ble_uuid.uuid = BLE_UUID_MPS_EXTRA_CHARACTERISTIC;
    
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
    attr_char_value.init_len     = 20;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = 100;
    
    return sd_ble_gatts_characteristic_add(p_mps->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_mps->extra_handle);
}

/**@Brief Function for initialize MPU Service*/
uint32_t ble_mps_init(ble_mps_t * p_mps)
{
		uint32_t   err_code;
    ble_uuid_t ble_uuid;
    ble_uuid128_t nus_base_uuid = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E};
    
    // Initialize service structure
    p_mps->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_mps->meas_hvn_enabled  = false;
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
    
		//Add characteristics
		err_code = meas_char_add(p_mps);
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
    
		err_code = extra_char_add(p_mps);
		if (err_code != NRF_SUCCESS)
		{
			return err_code;
		}
		
		return NRF_SUCCESS;
}


uint32_t ble_mps_send_hvn(ble_mps_t * p_mps, uint8_t * data, uint16_t length)
{
		ble_gatts_hvx_params_t hvx_params;
    
    if (!p_mps->meas_hvn_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_mps->measurement_handle.value_handle;
    hvx_params.p_data = data;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    
    return sd_ble_gatts_hvx(p_mps->conn_handle, &hvx_params);
}
