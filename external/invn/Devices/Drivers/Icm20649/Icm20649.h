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

/** @defgroup DriverIcm20649 Icm20649 driver
 *  @brief    Low-level driver for ICM20649 devices
 *  @ingroup  Drivers
 *  @{
 */

#ifndef _INV_ICM20649_H_
#define _INV_ICM20649_H_

#include "Invn/InvExport.h"
#include "Invn/InvBool.h"
#include "Invn/InvError.h"


#include "Icm20649Setup.h"
#include "Icm20649Serif.h"
#include "Icm20649Transport.h"
#include "Icm20649DataConverter.h"
#include "Icm20649SelfTest.h"


#include <stdint.h>
#include <assert.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief ICM20649 driver states definition
 */
typedef struct{
	uint64_t odr_applied_us;
	uint64_t odr_us;
}sensor_type_20649_t;

typedef enum{
	COUNT_1_STABLE_20649,
	COUNT_1_TRANSITION_20649,
	COUNT_DIFF_STABLE_20649,
	COUNT_DIFF_TRANSITION_20649,
}sample_count_state_20649_t;

typedef enum{
	CHIP_LOW_NOISE,
	CHIP_LOW_POWER,
}chip_lp_ln_mode_t;

typedef struct inv_icm20649 {
	struct inv_icm20649_serif serif;
	/** @brief struct for the base_driver : this contains the Mems information */
	struct base_driver_t
	{
		unsigned char wake_state;
		chip_lp_ln_mode_t chip_lp_ln_mode;
		unsigned char pwr_mgmt_1;
		unsigned char pwr_mgmt_2;
		unsigned char user_ctrl;
		unsigned char gyro_div;
		// unsigned short secondary_div;
		short accel_div;
		unsigned char gyro_averaging;
		unsigned char accel_averaging;
		uint8_t gyro_fullscale; 
		uint8_t accel_fullscale;
		uint8_t lp_en_support:1;
		uint8_t firmware_loaded:1;
		uint8_t serial_interface;
		uint8_t timebase_correction_pll;
	}base_state;
	/* self test */
	uint8_t selftest_done;
	uint8_t gyro_st_data[3];
	uint8_t accel_st_data[3];
	/* mpu fifo control */
	struct fifo_info_t
	{
		int fifoError;
		unsigned char fifo_overflow;
	} fifo_info;
	/* interface mapping */
	/* data converter */
	long s_quat_chip_to_body[4];
	/* base driver */
	uint8_t sAllowLpEn;
	/* base sensor ctrl*/
	unsigned short inv_dmp_odr_dividers[37];//INV_SENSOR_NUM_MAX /!\ if the size change 
	unsigned short inv_dmp_odr_delays[37];//INV_SENSOR_NUM_MAX /!\ if the size change
	unsigned short inv_sensor_control;
	unsigned short inv_sensor_control2;
	unsigned long inv_androidSensorsOn_mask ;// Each bit corresponds to a sensor being on
	unsigned char sGmrvIsOn; // indicates if GMRV was requested to be ON by end-user. Once this variable is set, it is either GRV or GMRV which is enabled internally
	unsigned short lLastHwSmplrtDividerAcc;
	unsigned short lLastHwSmplrtDividerGyr;
	unsigned char sBatchMode;
	uint8_t header2_count;
	char mems_put_to_sleep;
	uint8_t go_back_lp_when_odr_low; // set to 1 when we forced a switch from LP to LN mode to be able to reach 1kHz ODR, so we will need to go back to LP mode ASAP
	unsigned short odr_acc_ms; // ODR in ms requested for ANDROID_SENSOR_ACCELEROMETER
	unsigned short odr_acc_wom_ms; // ODR in ms requested for ANDROID_SENSOR_WOM when using ACC
	unsigned short odr_racc_ms; // ODR in ms requested for ANDROID_SENSOR_RAW_ACCELEROMETER
	unsigned short odr_gyr_ms; // ODR in ms requested for ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED
	unsigned short odr_rgyr_ms; // ODR in ms requested for ANDROID_SENSOR_RAW_GYROSCOPE
	/* Icm20649Fifo usage */
	signed char mounting_matrix[9];
	uint8_t skip_sample[INV_ICM20649_SENSOR_MAX+1];
	uint64_t timestamp[INV_ICM20649_SENSOR_MAX+1];
	uint8_t sFirstBatch[INV_ICM20649_SENSOR_MAX+1];
	sensor_type_20649_t sensorlist[INV_ICM20649_SENSOR_MAX+1];
	sample_count_state_20649_t count_state;
	unsigned short saved_count;
	/* Icm20649Transport*/
	unsigned char reg;
	unsigned char lastBank;
	unsigned char lLastBankSelected;
	/* Icm20649Setup */
	short set_accuracy;
	int new_accuracy;
	unsigned char wake_on_motion_enabled;
	unsigned char wake_on_motion_thr;
	unsigned char wake_on_motion_drop_first_event;
} inv_icm20649_t;

/** @brief ICM20649 driver states singleton declaration
 *  Because of Low-level driver limitation only one insance of the driver is allowed
 */
extern struct inv_icm20649 * icm20649_instance;

/** @brief Hook for low-level system sleep() function to be implemented by upper layer
 *  @param[in] us number of microsecond the calling thread should sleep
 */
extern void inv_icm20649_sleep_us(int us);

/** @brief Hook for low-level system time() function to be implemented by upper layer
 *  @return monotonic timestamp in us
 */
extern uint64_t inv_icm20649_get_time_us(void);
extern uint64_t inv_icm20649_get_dataready_interrupt_time_us(void);

/** @brief Reset and initialize driver states
 *  @param[in] s             handle to driver states structure
 */
static inline void inv_icm20649_reset_states(struct inv_icm20649 * s,
		const struct inv_icm20649_serif * serif)
{
	assert(icm20649_instance == 0);

	memset(s, 0, sizeof(*s));
	s->serif = *serif;
	icm20649_instance = s;
}

#ifdef __cplusplus
}
#endif

#endif /* _INV_ICM20649_H_ */

/** @} */
