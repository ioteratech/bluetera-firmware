/*
* ________________________________________________________________________________________________________
* Copyright © 2014-2015 InvenSense Inc. Portions Copyright © 2014-2015 Movea. All rights reserved.
* This software, related documentation and any modifications thereto (collectively “Software”) is subject
* to InvenSense and its licensors' intellectual property rights under U.S. and international copyright and
* other intellectual property rights laws.
* InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
* and any use, reproduction, disclosure or distribution of the Software without an express license
* agreement from InvenSense is strictly prohibited.
* ________________________________________________________________________________________________________
*/
/** @defgroup icm20649_base_control base_control
	@ingroup  SmartSensor_driver
	@{
*/
#ifndef INV_ICM20649_BASE_CONTROL_H__HWDFWQ__
#define INV_ICM20649_BASE_CONTROL_H__HWDFWQ__

#include "Icm20649DataBaseDriver.h"
#include "Icm20649Defs.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* forward declaration */
struct inv_icm20649;

#define INV_ODR_MIN_DELAY   200     // Limited by 8-bit HW Gyro rate divider register "GYRO_SMPLRT_DIV"
// Run at minimum 20Hz when using gyro/grv to avoid wrong behavior when calculating gyro_sf.
// It reaches his max value when gyro_level is equal to 3 (4000dps) and div is equal to 63 (17 Hz)
// And align all sensors to this min frequency to simplify system, so acc is also at min 20Hz
#define INV_ODR_MIN_DELAY_OUT   50 //ms, 20Hz

// Determines which base sensor needs to be on based upon inv_androidSensorsOn_mask[0]
#define INV_NEEDS_ACCEL_MASK	((1L<<1)|        (1L<<3)|        (1L<<5)|      (1L<<7))
#define INV_NEEDS_GYRO_MASK		(        (1L<<2)|(1L<<3)|(1L<<4)|       (1L<<6))

#define GYRO_AVAILABLE		0x1
#define ACCEL_AVAILABLE		0x2

// data output control reg 1
#define ACCEL_SET		0x8000
#define GYRO_SET		0x4000
#define CPASS_SET		0x2000
#define ALS_SET			0x1000
#define QUAT6_SET		0x0800
#define QUAT9_SET		0x0400
#define PQUAT6_SET		0x0200
#define GEOMAG_SET		0x0100
#define PRESSURE_SET	0x0080
#define GYRO_CALIBR_SET	0x0040
#define CPASS_CALIBR_SET 0x0020
#define PED_STEPDET_SET	0x0010
#define HEADER2_SET		0x0008
#define PED_STEPIND_SET 0x0007

// data output control reg 2
#define ACCEL_ACCURACY_SET		0x4000
#define GYRO_ACCURACY_SET		0x2000
#define CPASS_ACCURACY_SET		0x1000
#define COMPASS_CAL_INPUT_SET	0x1000
#define FLIP_PICKUP_SET			0x0400
#define ACT_RECOG_SET			0x0080
#define BATCH_MODE_EN			0x0100
// motion event control reg
#define INV_BAC_WEARABLE_EN		0x8000
#define INV_PEDOMETER_EN		0x4000
#define INV_PEDOMETER_INT_EN	0x2000
#define INV_SMD_EN				0x0800
#define INV_BTS_EN				0x0020
#define FLIP_PICKUP_EN			0x0010
#define GEOMAG_EN   			0x0008
#define INV_ACCEL_CAL_EN		0x0200
#define INV_GYRO_CAL_EN			0x0100
#define INV_COMPASS_CAL_EN		0x0080
#define INV_NINE_AXIS_EN        0x0040
#define INV_BRING_AND_LOOK_T0_SEE_EN  0x0004  // Aded by ONn for 20649

// data packet size reg 1
#define HEADER_SZ		2
#define ACCEL_DATA_SZ	6
#define GYRO_DATA_SZ	6
#define CPASS_DATA_SZ	6
#define ALS_DATA_SZ		8
#define QUAT6_DATA_SZ	12
#define QUAT9_DATA_SZ	14
#define PQUAT6_DATA_SZ	6
#define GEOMAG_DATA_SZ	14
#define PRESSURE_DATA_SZ		6
#define GYRO_BIAS_DATA_SZ	6
#define CPASS_CALIBR_DATA_SZ	12
#define PED_STEPDET_TIMESTAMP_SZ	4
#define FOOTER_SZ		2

// data packet size reg 2
#define HEADER2_SZ			2
#define ACCEL_ACCURACY_SZ	2
#define GYRO_ACCURACY_SZ	2
#define CPASS_ACCURACY_SZ	2
#define FSYNC_SZ			2
#define FLIP_PICKUP_SZ      2
#define ACT_RECOG_SZ        6
#define ODR_CNT_GYRO_SZ	2

/** @brief Initialize structure values
* @param[in] base state structre
*/
int INV_EXPORT inv_icm20649_base_control_init(struct inv_icm20649 * s);

/** @brief Sets the odr for a sensor
* @param[in] androidSensor  Sensor Identity
* @param[in] delayInMs  the delay between two values in ms
* @return 0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20649_set_odr(struct inv_icm20649 * s, unsigned char androidSensor, unsigned short delayInMs);

/** @brief Enables / disables a sensor 
* @param[in] androidSensor  Sensor Identity
* @param[in] enable			0=off, 1=on
* @return 					0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20649_ctrl_enable_sensor(struct inv_icm20649 * s, unsigned char androidSensor, unsigned char enable);

/** @brief Enables / disables batch for the sensors
* @param[in] enable			0=off, 1=on
* @return 0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20649_ctrl_enable_batch(struct inv_icm20649 * s, unsigned char enable);

/** @brief Set batch mode status
* @param[in] enable			0=off, 1=on
*/
void INV_EXPORT inv_icm20649_ctrl_set_batch_mode_status(struct inv_icm20649 * s, unsigned char enable);

/** @brief Get batch mode status
* @return 					0=batch mode disable, 1=batch mode enable
*/
unsigned char INV_EXPORT inv_icm20649_ctrl_get_batch_mode_status(struct inv_icm20649 * s);

/** @brief Sets the timeout for the batch in second 
* @param[in] batch_time_in_seconds  time in second
* @return 0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20649_ctrl_set_batch_timeout(struct inv_icm20649 * s, unsigned short batch_time_in_seconds);

/** @brief Sets the timeout for the batch in millisecond 
* @param[in] batch_time_in_ms  time in millisecond
* @return 0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20649_ctrl_set_batch_timeout_ms(struct inv_icm20649 * s, unsigned short batch_time_in_ms);

/** @brief Returns the mask for the different sensors enabled
* @return the mask
*/
unsigned long INV_EXPORT *inv_icm20649_ctrl_get_androidSensorsOn_mask(struct inv_icm20649 * s);

/** @brief Check if a sensor is enabled
* @return 1 if sensor is enabled
*/
unsigned long INV_EXPORT inv_icm20649_ctrl_androidSensor_enabled(struct inv_icm20649 * s, unsigned char androidSensor);

/** @brief Enumeration for the Type of ODR : Millisecondes / Microsecondes / Ticks */
enum INV_ODR_TYPE {
    ODR_IN_Ms,
    ODR_IN_Us,
    ODR_IN_Ticks
};

/** @brief Gets the odr for a sensor
* @param[in] SensorId  	Sensor Identity
* @param[out] odr  	pointer to the ODR for this sensor
* @param[in] odr_units  unit expected for odr, one of INV_ODR_TYPE
* @return 0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20649_ctrl_get_odr(struct inv_icm20649 * s, unsigned char SensorId, uint32_t *odr, enum INV_ODR_TYPE odr_units);

/** @brief Sets accel quaternion gain according to accel engine rate.
* @param[in] hw_smplrt_divider  hardware sample rate divider such that accel engine rate = 1125Hz/hw_smplrt_divider
* @return 0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20649_ctrl_set_accel_quaternion_gain(struct inv_icm20649 * s, unsigned short hw_smplrt_divider);

#ifdef __cplusplus
}
#endif
#endif // INV_ICM20649_BASE_CONTROL_H__HWDFWQ__

/** @} */
