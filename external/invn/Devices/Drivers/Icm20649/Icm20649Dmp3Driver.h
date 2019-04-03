/*
* ________________________________________________________________________________________________________
* Copyright © 2014 InvenSense Inc.  All rights reserved.
*
* This software and/or documentation  (collectively “Software”) is subject to InvenSense intellectual property rights 
* under U.S. and international copyright and other intellectual property rights laws.
*
* The Software contained herein is PROPRIETARY and CONFIDENTIAL to InvenSense and is provided 
* solely under the terms and conditions of a form of InvenSense software license agreement between 
* InvenSense and you and any use, modification, reproduction or disclosure of the Software without 
* such agreement or the express written consent of InvenSense is strictly prohibited.
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

#ifndef _DMP_3_DEFAULT_20649_XFSD_H__
#define _DMP_3_DEFAULT_20649_XFSD_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* forward declaration */
struct inv_icm20649;

/* enum for sensor
   The sequence is important.
   It represents the order of apperance from DMP */
enum INV_SENSORS {
	INV_SENSOR_ACCEL = 0,
	INV_SENSOR_GYRO,        
	INV_SENSOR_LPQ,             // 20610:  we'll find out if it breaks 20628 being inserted here....       
	INV_SENSOR_COMPASS,
	INV_SENSOR_ALS,
	INV_SENSOR_SIXQ,
	INV_SENSOR_NINEQ,
	INV_SENSOR_GEOMAG,
INV_SENSOR_PEDQ,
INV_SENSOR_PRESSURE,
	INV_SENSOR_CALIB_GYRO,
	INV_SENSOR_CALIB_COMPASS,
	INV_SENSOR_STEP_COUNTER,
	INV_SENSOR_ACTIVITY_CLASSIFIER,
	INV_SENSOR_FLIP_PICKUP,
    INV_SENSOR_BRING_TO_SEE,

INV_SENSOR_SIXQ_accel,
INV_SENSOR_NINEQ_accel,
INV_SENSOR_GEOMAG_cpass,
INV_SENSOR_NINEQ_cpass,

	INV_SENSOR_WAKEUP_ACCEL,
	INV_SENSOR_WAKEUP_GYRO,        
//INV_SENSOR_WAKEUP_LPQ,
	INV_SENSOR_WAKEUP_COMPASS,
	INV_SENSOR_WAKEUP_ALS,
	INV_SENSOR_WAKEUP_SIXQ,
	INV_SENSOR_WAKEUP_NINEQ,
	INV_SENSOR_WAKEUP_GEOMAG,
INV_SENSOR_WAKEUP_PEDQ,
INV_SENSOR_WAKEUP_PRESSURE,
	INV_SENSOR_WAKEUP_CALIB_GYRO,
	INV_SENSOR_WAKEUP_CALIB_COMPASS,
	INV_SENSOR_WAKEUP_STEP_COUNTER,
	INV_SENSOR_WAKEUP_TILT_DETECTOR,
//INV_SENSOR_WAKEUP_ACTIVITY_CLASSIFIER,

INV_SENSOR_WAKEUP_SIXQ_accel,
INV_SENSOR_WAKEUP_NINEQ_accel,
INV_SENSOR_WAKEUP_GEOMAG_cpass,
INV_SENSOR_WAKEUP_NINEQ_cpass,

	INV_SENSOR_NUM_MAX,
	INV_SENSOR_INVALID,
};


enum accel_cal_params {
    ACCEL_CAL_ALPHA_VAR = 0,
    ACCEL_CAL_A_VAR,
    ACCEL_CAL_DIV,
    NUM_ACCEL_CAL_PARAMS
};

enum compass_cal_params {
	CPASS_CAL_TIME_BUFFER = 0,
	CPASS_CAL_RADIUS_3D_THRESH_ANOMALY,
    NUM_CPASS_CAL_PARAMS
};

int inv_load_firmware_20649(struct inv_icm20649 * s, const unsigned char *dmp3_image, unsigned int dmp3_image_size);
void inv_get_dmp_start_address_20649(struct inv_icm20649 * s, unsigned short *dmp_cnfg);
int dmp_reset_control_registers_20649(struct inv_icm20649 * s);
int dmp_set_data_output_control1_20649(struct inv_icm20649 * s, int output_mask);
int dmp_set_data_output_control2_20649(struct inv_icm20649 * s, int output_mask);
int dmp_set_data_interrupt_control_20649(struct inv_icm20649 * s, uint32_t interrupt_ctl);
int dmp_set_FIFO_watermark_20649(struct inv_icm20649 * s, unsigned short fifo_wm);
int dmp_set_data_rdy_status_20649(struct inv_icm20649 * s, unsigned short data_rdy);
int dmp_set_motion_event_control_20649(struct inv_icm20649 * s, unsigned short motion_mask);
int dmp_set_sensor_rate_20649(struct inv_icm20649 * s, int sensor, short divider);
int dmp_set_batchmode_params_20649(struct inv_icm20649 * s, unsigned int thld, short mask);
int dmp_set_bias_20649(struct inv_icm20649 * s, int *bias);
int dmp_get_bias_20649(struct inv_icm20649 * s, int *bias);
int dmp_set_gyro_sf_20649(struct inv_icm20649 * s, long gyro_sf);
int dmp_set_accel_feedback_gain_20649(struct inv_icm20649 * s, int accel_gain);
int dmp_set_wom_enable_20649(struct inv_icm20649 * s, unsigned char enable);
int dmp_set_wom_motion_threshold_20649(struct inv_icm20649 * s, int threshold);
int dmp_set_wom_time_threshold_20649(struct inv_icm20649 * s, unsigned short threshold);
int dmp_set_accel_fsr_20649(struct inv_icm20649 * s, short accel_fsr);
int dmp_set_accel_scale2_20649(struct inv_icm20649 * s, short accel_fsr);

#ifdef __cplusplus
}
#endif

// _DMP_3_DEFAULT_20649_XFSD_H__
#endif
