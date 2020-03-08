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

#ifndef BLTR_IMU_H_
#define BLTR_IMU_H_

#include <stdint.h>

typedef enum 
{
	BLTR_IMU_SENSOR_TYPE_ACCELEROMETER                = 1,
	BLTR_IMU_SENSOR_TYPE_MAGNETOMETER                 = 2,
	BLTR_IMU_SENSOR_TYPE_GYROSCOPE                    = 4,
	BLTR_IMU_SENSOR_TYPE_ROTATION_VECTOR              = 11,
	BLTR_IMU_SENSOR_TYPE_RAW_ACCELEROMETER            = 32,
	BLTR_IMU_SENSOR_TYPE_RAW_GYROSCOPE                = 33,
	BLTR_IMU_SENSOR_TYPE_RAW_MAGNETOMETER             = 34,
	BLTR_IMU_SENSOR_TYPE_RAW						  = 100
} bltr_imu_sensor_t;

typedef struct
{
	bltr_imu_sensor_t sensor;
	uint64_t timestamp;		// in microseconds
	union {
		float acceleration[3];
		float gyroscope[3];
		float quaternion[4];
		struct {
			int16_t acceleration[3];
			int16_t gyroscope[3];
		} raw;
	};
} bltr_imu_sensor_data_t;

// TODO TEMPORARY! REMOVE!
typedef enum
{
	BLTR_IMU_DATA_TYPE_QUATERNION = 0x01,
	BLTR_IMU_DATA_TYPE_ACCELEROMETER = 0x02,
	BLTR_IMU_DATA_TYPE_GYROSCOPE = 0x04
} bltr_imu_data_type_t;

typedef struct {
	uint32_t data_types;
	uint32_t odr;
	uint32_t acc_fsr;
	uint32_t gyro_fsr;
} bltr_imu_config_t;

typedef void(*bltr_imu_data_handler_t)(const bltr_imu_sensor_data_t* data);

typedef struct
{
	bltr_imu_data_handler_t imu_data_handler;
} bltr_imu_init_t;

#endif	// BLTR_IMU_H_