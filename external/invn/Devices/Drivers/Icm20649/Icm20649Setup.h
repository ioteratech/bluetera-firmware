/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

/** @defgroup DriverIcm20649Setup Icm20649 driver setup
 *  @brief Low-level function to setup an Icm20649 device
 *  @ingroup  DriverIcm20649
 *  @{
 */

#ifndef _INV_ICM20649_SETUP_H_
#define _INV_ICM20649_SETUP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "Invn/InvExport.h"
#include "Invn/InvBool.h"

/* forward declaration */
struct inv_icm20649;

/** @brief Sensor identifier for control function
 */
enum inv_icm20649_sensor {
	INV_ICM20649_SENSOR_RAW_ACCELEROMETER,
	INV_ICM20649_SENSOR_RAW_GYROSCOPE,
	INV_ICM20649_SENSOR_GYROSCOPE,
	INV_ICM20649_SENSOR_GYROSCOPE_UNCALIBRATED,
	INV_ICM20649_SENSOR_GAME_ROTATION_VECTOR,
	INV_ICM20649_SENSOR_ACCELEROMETER,
	INV_ICM20649_SENSOR_WOM,
	INV_ICM20649_SENSOR_MAX,
};

int INV_EXPORT inv_icm20649_get_whoami(struct inv_icm20649 * s, uint8_t * whoami);
void INV_EXPORT inv_icm20649_init_matrix(struct inv_icm20649 * s);
int INV_EXPORT inv_icm20649_set_matrix(struct inv_icm20649 * s, const float matrix[9], enum inv_icm20649_sensor sensor);
int INV_EXPORT inv_icm20649_initialize(struct inv_icm20649 * s, const uint8_t *dmp3_image, uint32_t dmp3_image_size);
int INV_EXPORT inv_icm20649_init_scale(struct inv_icm20649 * s);
int INV_EXPORT inv_icm20649_set_wom_threshold(struct inv_icm20649 * s, uint8_t threshold);
int INV_EXPORT inv_icm20649_set_fsr(struct inv_icm20649 * s, enum inv_icm20649_sensor sensor, const void * fsr);
int INV_EXPORT inv_icm20649_get_fsr(struct inv_icm20649 * s, enum inv_icm20649_sensor sensor, const void * fsr);
int INV_EXPORT inv_icm20649_soft_reset(struct inv_icm20649 * s);
int INV_EXPORT inv_icm20649_enable_sensor(struct inv_icm20649 * s, enum inv_icm20649_sensor sensor, inv_bool_t state);
int INV_EXPORT inv_icm20649_set_sensor_period(struct inv_icm20649 * s, enum inv_icm20649_sensor sensor, uint32_t period);
int INV_EXPORT inv_icm20649_enable_batch_timeout(struct inv_icm20649 * s, unsigned short batchTimeoutMs);
int INV_EXPORT inv_icm20649_poll_sensor(struct inv_icm20649 * s, void * context,
		void (*handler)(void * context, enum inv_icm20649_sensor sensor, uint64_t timestamp, const void * data, const void *arg));
int INV_EXPORT inv_icm20649_load(struct inv_icm20649 * s, const uint8_t * image, unsigned short size);
int INV_EXPORT inv_icm20649_init_structure(struct inv_icm20649 * s);
enum inv_icm20649_sensor INV_EXPORT inv_icm20649_sensor_android_2_sensor_type(int sensor);

/** @brief Have the chip to enter low-power or low-noise mode
* @param[in] lowpower_or_highperformance		0=low-power, 1=low-noise
*/
int INV_EXPORT inv_icm20649_set_lowpower_or_highperformance(struct inv_icm20649 * s, uint8_t lowpower_or_highperformance);
int INV_EXPORT inv_icm20649_get_lowpower_or_highperformance(struct inv_icm20649 * s, uint8_t * lowpower_or_highperformance);

#ifdef __cplusplus
}
#endif

#endif /* _INV_ICM20649_SETUP_H_ */

/** @} */
