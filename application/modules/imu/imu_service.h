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

#ifndef IMU_SERVICE_H_
#define IMU_SERVICE_H_

#include <stdint.h>
#include <stdbool.h>
#include <ble.h>
#include <ble_srv_common.h>

// 4 bytes timestamp
// 16 bytes quat
#define CHAR_QUAT_LENGTH 20

// 4 bytes timestamp
// 12 bytes vector
#define CHAR_ACC_LENGTH	 16

// 4 bytes timestamp
// 6 bytes acc
// 6 bytes gyro
#define CHAR_RAW_ACC_GYRO_LENGTH 16

// 2 bytes acc fsr
// 2 bytes gyro fsr
#define CHAR_FSR_LENGTH 4

#define IMU_SENSOR_INVALID 			0x00
#define IMU_SENSOR_SET_ENABLED		0x01
#define IMU_SENSOR_RAW				0x02
#define IMU_SENSOR_SET_FSR			0x03

#define BLE_IMU_DEF(_name) \
static bltr_imu_service_t _name; \
NRF_SDH_BLE_OBSERVER(_name ## _obs,  \
                     BLE_HRS_BLE_OBSERVER_PRIO, \
                     bltr_imu_service_on_ble_evt, &_name)

typedef struct
{
    uint8_t                       initial_custom_value;           /**< Initial custom value */
    ble_srv_cccd_security_mode_t  custom_value_char_attr_md;     /**< Initial security level for Custom characteristics attribute */
} bltr_imu_service_init_t;

typedef struct
{
    uint16_t                      service_handle;
    ble_gatts_char_handles_t      quat_char_handles;
	ble_gatts_char_handles_t      acc_char_handles;
	ble_gatts_char_handles_t      raw_char_handles;
	ble_gatts_char_handles_t      log_toggle_char_handles;
	ble_gatts_char_handles_t      fsr_char_handles;
    uint16_t                      conn_handle;
    uint8_t                       uuid_type;

	bool sensor_quat_enabled;
	bool sensor_acc_enabled;
	
	void (*on_sensor_enable)(void* p_event_data, uint16_t event_size);
} bltr_imu_service_t;

uint32_t bltr_imu_service_init(bltr_imu_service_t* service, const bltr_imu_service_init_t* service_init);
void bltr_imu_service_on_ble_evt(ble_evt_t const * p_ble_evt, void* p_context);
uint32_t bltr_imu_service_update_quat(bltr_imu_service_t* service, uint8_t* data);
uint32_t bltr_imu_service_update_acc(bltr_imu_service_t* service, uint8_t* data);
uint32_t bltr_imu_service_update_raw(bltr_imu_service_t* service, uint8_t* data);
uint32_t bltr_imu_service_update_fsr(bltr_imu_service_t* service, uint16_t acc_fsr, uint16_t gyro_fsr);

#endif