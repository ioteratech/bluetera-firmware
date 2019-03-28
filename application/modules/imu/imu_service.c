/*
	MIT License

	Copyright (c) 2019 Tensor Technologies LTD

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

#include "imu_service.h"
#include "bluetera_config.h"

#include <string.h>

#include <sdk_common.h>
#include <ble_srv_common.h>
#include <app_scheduler.h>

// Configure logging
#define NRF_LOG_MODULE_NAME bltr_imu
#if BLTR_IMU_CONFIG_LOG_ENABLED
    #define NRF_LOG_LEVEL       BLTR_IMU_CONFIG_LOG_LEVEL
	#define NRF_LOG_INFO_COLOR  BLTR_IMU_CONFIG_INFO_COLOR
	#define NRF_LOG_DEBUG_COLOR BLTR_IMU_CONFIG_DEBUG_COLOR
#else
    #define NRF_LOG_LEVEL       0
#endif
#include <nrf_log.h>
#include <nrf_log_ctrl.h>
NRF_LOG_MODULE_REGISTER();

// b1478917-0da6-40a4-bd05-386dc19120ca
#define IMU_SERVICE_BASE { 0xCA, 0x20, 0x91, 0xC1, 0x6D, 0x38, 0x05, 0xBD, 0xA4, 0x40, 0xA6, 0x0D, 0x17, 0x89, 0x47, 0xB1 }

#define IMU_SERVICE_UUID               	 0x8917
#define IMU_SERVICE_QUAT_DATA_UUID		 0x8918
#define IMU_SERVICE_ACC_DATA_UUID        0x8919
#define IMU_SERVICE_RAW_DATA_UUID        0x891A
#define IMU_SERVICE_LOG_TOGGLE_UUID      0x891B
#define IMU_SERVICE_FSR_UUID     		 0x891C

static uint32_t add_quaternion_char(bltr_imu_service_t* service, const bltr_imu_service_init_t* service_init);
static uint32_t add_acceleration_char(bltr_imu_service_t* service, const bltr_imu_service_init_t* service_init);
static uint32_t add_raw_char(bltr_imu_service_t* service, const bltr_imu_service_init_t* service_init);
static uint32_t add_fsr_char(bltr_imu_service_t* service, const bltr_imu_service_init_t* service_init);

uint32_t bltr_imu_service_init(bltr_imu_service_t* service, const bltr_imu_service_init_t* service_init)
{
    if (service == NULL || service_init == NULL)
        return NRF_ERROR_NULL;

    service->conn_handle = BLE_CONN_HANDLE_INVALID;
	service->sensor_quat_enabled = false;
	service->sensor_acc_enabled = false;

    // add custom service UUID
    ble_uuid128_t base_uuid = { IMU_SERVICE_BASE };
    uint32_t err_code =  sd_ble_uuid_vs_add(&base_uuid, &service->uuid_type);
    VERIFY_SUCCESS(err_code);
    
	ble_uuid_t ble_uuid;
    ble_uuid.type = service->uuid_type;
    ble_uuid.uuid = IMU_SERVICE_UUID;

    // add the custom service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &service->service_handle);
    
	// add all the characteristics
    err_code = add_quaternion_char(service, service_init);
	err_code = add_acceleration_char(service, service_init);
	//err_code = add_raw_char(service, service_init);
	err_code = add_fsr_char(service, service_init);

    return err_code;
}

void bltr_imu_service_on_ble_evt(ble_evt_t const* p_ble_evt, void* p_context)
{
    bltr_imu_service_t* imu_service = (bltr_imu_service_t *)p_context;
    
    if (imu_service == NULL || p_ble_evt == NULL)
        return;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
			imu_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
			imu_service->conn_handle = BLE_CONN_HANDLE_INVALID;
			imu_service->sensor_quat_enabled = false;
			imu_service->sensor_acc_enabled = false;
            break;
		case BLE_GATTS_EVT_WRITE:
			{
				uint8_t cbdata[8];

				if(p_ble_evt->evt.gatts_evt.params.write.handle == imu_service->quat_char_handles.cccd_handle)
				{
					imu_service->sensor_quat_enabled = p_ble_evt->evt.gatts_evt.params.write.data[0];
				}
				else if(p_ble_evt->evt.gatts_evt.params.write.handle == imu_service->acc_char_handles.cccd_handle)
				{
					imu_service->sensor_acc_enabled = p_ble_evt->evt.gatts_evt.params.write.data[0];
				}
				else if(p_ble_evt->evt.gatts_evt.params.write.handle == imu_service->raw_char_handles.cccd_handle)
				{
					cbdata[0] = IMU_SENSOR_RAW;
					cbdata[1] = p_ble_evt->evt.gatts_evt.params.write.data[0]; // enable or disable notifications
				}
				else if(p_ble_evt->evt.gatts_evt.params.write.handle == imu_service->fsr_char_handles.value_handle)
				{
					cbdata[0] = IMU_SENSOR_SET_FSR;
					memcpy(cbdata + 1, p_ble_evt->evt.gatts_evt.params.write.data, 2); // acc fsr
					memcpy(cbdata + 3, p_ble_evt->evt.gatts_evt.params.write.data + 2, 2); // gyro fsr
				}
				else
				{
					cbdata[0] = IMU_SENSOR_INVALID;
				}

				if(p_ble_evt->evt.gatts_evt.params.write.handle == imu_service->quat_char_handles.cccd_handle || p_ble_evt->evt.gatts_evt.params.write.handle == imu_service->acc_char_handles.cccd_handle)
				{
					if(imu_service->sensor_quat_enabled || imu_service->sensor_acc_enabled)
					{
						cbdata[0] = IMU_SENSOR_SET_ENABLED;
						cbdata[1] = 1;
					}
					else if(!imu_service->sensor_quat_enabled && !imu_service->sensor_acc_enabled)
					{
						cbdata[0] = IMU_SENSOR_SET_ENABLED;
						cbdata[1] = 0;
					}
				}
				
				if(cbdata[0] != IMU_SENSOR_INVALID)
					app_sched_event_put(&cbdata, sizeof(cbdata), imu_service->on_sensor_enable);
			}
			break;
        default:
            break;
    }
}

uint32_t bltr_imu_service_update_quat(bltr_imu_service_t* service, uint8_t* data)
{
	if (service == NULL)
        return NRF_ERROR_NULL;

    uint32_t err_code = NRF_SUCCESS;

    ble_gatts_value_t gatts_value;
    gatts_value.len = CHAR_QUAT_LENGTH;
    gatts_value.offset = 0;
    gatts_value.p_value = data;

    // send value if connected and notifying.
    if (service->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        ble_gatts_hvx_params_t hvx_params;
        hvx_params.handle = service->quat_char_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(service->conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t bltr_imu_service_update_acc(bltr_imu_service_t* service, uint8_t* data)
{
	if (service == NULL)
        return NRF_ERROR_NULL;

    uint32_t err_code = NRF_SUCCESS;

    ble_gatts_value_t gatts_value;
    gatts_value.len = CHAR_ACC_LENGTH;
    gatts_value.offset = 0;
    gatts_value.p_value = data;

    // send value if connected and notifying.
    if (service->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        ble_gatts_hvx_params_t hvx_params;
        hvx_params.handle = service->acc_char_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(service->conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t bltr_imu_service_update_raw(bltr_imu_service_t* service, uint8_t* data)
{
    if (service == NULL)
        return NRF_ERROR_NULL;

    uint32_t err_code = NRF_SUCCESS;

    ble_gatts_value_t gatts_value;
    gatts_value.len = CHAR_RAW_ACC_GYRO_LENGTH;
    gatts_value.offset = 0;
    gatts_value.p_value = data;

    // send value if connected and notifying.
    if (service->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        ble_gatts_hvx_params_t hvx_params;
        hvx_params.handle = service->raw_char_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(service->conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t bltr_imu_service_update_fsr(bltr_imu_service_t* service, uint16_t acc_fsr, uint16_t gyro_fsr)
{
    if (service == NULL)
        return NRF_ERROR_NULL;
    
    ble_gatts_value_t gatts_value;
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = CHAR_FSR_LENGTH;
    gatts_value.offset  = 0;

	uint8_t data[CHAR_FSR_LENGTH];

	memcpy(data, &acc_fsr, 2);
	memcpy(data + 2, &gyro_fsr, 2);

    gatts_value.p_value = data;

    // update database.
    uint32_t err_code = sd_ble_gatts_value_set(service->conn_handle, service->fsr_char_handles.value_handle, &gatts_value);
	
    if (err_code != NRF_SUCCESS)
        return err_code;

	return err_code;
}

static uint32_t add_quaternion_char(bltr_imu_service_t* service, const bltr_imu_service_init_t* service_init)
{
	ble_gatts_attr_md_t cccd_md = { 0 };
    // read operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.write_perm = service_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

	ble_gatts_char_md_t char_md = { 0 };

    char_md.char_props.read   = 0;
    char_md.char_props.write  = 0;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
	
	ble_uuid_t ble_uuid;
    ble_uuid.type = service->uuid_type;
    ble_uuid.uuid = IMU_SERVICE_QUAT_DATA_UUID;

	ble_gatts_attr_md_t attr_md = { 0 };

    attr_md.read_perm  = service_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = service_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

	ble_gatts_attr_t attr_char_value = { 0 };

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = CHAR_QUAT_LENGTH;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = CHAR_QUAT_LENGTH;

    uint32_t err_code = sd_ble_gatts_characteristic_add(service->service_handle, &char_md, &attr_char_value, &service->quat_char_handles);

    if (err_code != NRF_SUCCESS)
        return err_code;

    return NRF_SUCCESS;
}

static uint32_t add_acceleration_char(bltr_imu_service_t* service, const bltr_imu_service_init_t* service_init)
{
	ble_gatts_attr_md_t cccd_md = { 0 };
    // read operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.write_perm = service_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

	ble_gatts_char_md_t char_md = { 0 };

    char_md.char_props.read   = 0;
    char_md.char_props.write  = 0;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
	
	ble_uuid_t ble_uuid;
    ble_uuid.type = service->uuid_type;
    ble_uuid.uuid = IMU_SERVICE_ACC_DATA_UUID;

	ble_gatts_attr_md_t attr_md = { 0 };

    attr_md.read_perm  = service_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = service_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

	ble_gatts_attr_t attr_char_value = { 0 };

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = CHAR_ACC_LENGTH;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = CHAR_ACC_LENGTH;

    uint32_t err_code = sd_ble_gatts_characteristic_add(service->service_handle, &char_md, &attr_char_value, &service->acc_char_handles);

    if (err_code != NRF_SUCCESS)
        return err_code;

    return NRF_SUCCESS;
}

static uint32_t add_raw_char(bltr_imu_service_t* service, const bltr_imu_service_init_t* service_init)
{
    ble_gatts_attr_md_t cccd_md = { 0 };
    // read operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.write_perm = service_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

	ble_gatts_char_md_t char_md = { 0 };

    char_md.char_props.read   = 0;
    char_md.char_props.write  = 0;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
	
	ble_uuid_t ble_uuid;
    ble_uuid.type = service->uuid_type;
    ble_uuid.uuid = IMU_SERVICE_RAW_DATA_UUID;

	ble_gatts_attr_md_t attr_md = { 0 };

    attr_md.read_perm  = service_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = service_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

	ble_gatts_attr_t attr_char_value = { 0 };

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = CHAR_RAW_ACC_GYRO_LENGTH;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = CHAR_RAW_ACC_GYRO_LENGTH;

    uint32_t err_code = sd_ble_gatts_characteristic_add(service->service_handle, &char_md, &attr_char_value, &service->raw_char_handles);

    if (err_code != NRF_SUCCESS)
        return err_code;

    return NRF_SUCCESS;
}

static uint32_t add_fsr_char(bltr_imu_service_t* service, const bltr_imu_service_init_t* service_init)
{
    ble_gatts_attr_md_t cccd_md = { 0 };
    // read operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.write_perm = service_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

	ble_gatts_char_md_t char_md = { 0 };

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 0; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
	
	ble_uuid_t ble_uuid;
    ble_uuid.type = service->uuid_type;
    ble_uuid.uuid = IMU_SERVICE_FSR_UUID;

	ble_gatts_attr_md_t attr_md = { 0 };

    attr_md.read_perm  = service_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = service_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

	ble_gatts_attr_t attr_char_value = { 0 };

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = CHAR_FSR_LENGTH;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = CHAR_FSR_LENGTH;

    uint32_t err_code = sd_ble_gatts_characteristic_add(service->service_handle, &char_md, &attr_char_value, &service->fsr_char_handles);

    if (err_code != NRF_SUCCESS)
        return err_code;

    return NRF_SUCCESS;
}