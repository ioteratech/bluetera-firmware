/*
* ________________________________________________________________________________________________________
* Copyright � 2014 InvenSense Inc.  All rights reserved.
*
* This software and/or documentation  (collectively �Software�) is subject to InvenSense intellectual property rights 
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

#include "Icm20649.h"
#include "Icm20649Dmp3Driver.h"
#include "Icm20649LoadFirmware.h"
#include "Icm20649Defs.h"

#define CFG_FIFO_SIZE                   (4184)

// data output control
#define DATA_OUT_CTL1			(4 * 16)
#define DATA_OUT_CTL2			(4 * 16 + 2)
#define DATA_INTR_CTL			(4 * 16 + 12)
#define FIFO_WATERMARK			(31 * 16 + 14)

// motion event control
#define MOTION_EVENT_CTL		(4 * 16 + 14)

// indicates to DMP which sensors are available
/*	1: gyro samples available
	2: accel samples available
	8: secondary samples available	*/
#define DATA_RDY_STATUS			(8 * 16 + 10)

// batch mode
#define BM_BATCH_CNTR			(27 * 16)
#define BM_BATCH_THLD			(19 * 16 + 12)
#define BM_BATCH_MASK			(21 * 16 + 14)

// sensor output data rate
#define ODR_ACCEL				(11 * 16 + 14)
#define ODR_GYRO				(11 * 16 + 10)
#define ODR_CPASS				(11 * 16 +  6)
#define ODR_ALS					(11 * 16 +  2)
#define ODR_QUAT6				(10 * 16 + 12)
#define ODR_QUAT9				(10 * 16 +  8)
#define ODR_PQUAT6				(10 * 16 +  4)
#define ODR_GEOMAG				(10 * 16 +  0)
#define ODR_PRESSURE			(11 * 16 + 12)
#define ODR_GYRO_CALIBR			(11 * 16 +  8)
#define ODR_CPASS_CALIBR		(11 * 16 +  4)

// sensor output data rate counter
#define ODR_CNTR_ACCEL			(9 * 16 + 14)
#define ODR_CNTR_GYRO			(9 * 16 + 10)
#define ODR_CNTR_CPASS			(9 * 16 +  6)
#define ODR_CNTR_ALS			(9 * 16 +  2)
#define ODR_CNTR_QUAT6			(8 * 16 + 12)
#define ODR_CNTR_QUAT9			(8 * 16 +  8)
#define ODR_CNTR_PQUAT6			(8 * 16 +  4)
#define ODR_CNTR_GEOMAG			(8 * 16 +  0)
#define ODR_CNTR_PRESSURE		(9 * 16 + 12)
#define ODR_CNTR_GYRO_CALIBR	(9 * 16 +  8)
#define ODR_CNTR_CPASS_CALIBR	(9 * 16 +  4)

// mounting matrix
#define CPASS_MTX_00            (23 * 16)
#define CPASS_MTX_01            (23 * 16 + 4)
#define CPASS_MTX_02            (23 * 16 + 8)
#define CPASS_MTX_10            (23 * 16 + 12)
#define CPASS_MTX_11            (24 * 16)
#define CPASS_MTX_12            (24 * 16 + 4)
#define CPASS_MTX_20            (24 * 16 + 8)
#define CPASS_MTX_21            (24 * 16 + 12)
#define CPASS_MTX_22            (25 * 16)

#define GYRO_SF					(19 * 16)
#define ACCEL_FB_GAIN			(34 * 16)
#define ACCEL_ONLY_GAIN			(16 * 16 + 12)

// bias calibration
#define GYRO_BIAS_X				(139 * 16 +  4)
#define GYRO_BIAS_Y				(139 * 16 +  8)
#define GYRO_BIAS_Z				(139 * 16 + 12)
#define GYRO_ACCURACY			(138 * 16 +  2)
#define GYRO_BIAS_SET			(138 * 16 +  6)
#define GYRO_LAST_TEMPR			(134 * 16)
#define GYRO_SLOPE_X			( 78 * 16 +  4)
#define GYRO_SLOPE_Y			( 78 * 16 +  8)
#define GYRO_SLOPE_Z			( 78 * 16 + 12)

#define ACCEL_BIAS_X            (110 * 16 +  4)
#define ACCEL_BIAS_Y            (110 * 16 +  8)
#define ACCEL_BIAS_Z            (110 * 16 + 12)
#define ACCEL_ACCURACY			(97 * 16)
#define ACCEL_CAL_RESET			(77 * 16)
#define ACCEL_VARIANCE_THRESH	(93 * 16)
#define ACCEL_CAL_RATE			(94 * 16 + 4)
#define ACCEL_PRE_SENSOR_DATA	(97 * 16 + 4)
#define ACCEL_COVARIANCE		(101 * 16 + 8)
#define ACCEL_ALPHA_VAR			(91 * 16)
#define ACCEL_A_VAR				(92 * 16)
#define ACCEL_CAL_INIT			(94 * 16 + 2)

#define CPASS_BIAS_X            (126 * 16 +  4)
#define CPASS_BIAS_Y            (126 * 16 +  8)
#define CPASS_BIAS_Z            (126 * 16 + 12)
#define CPASS_ACCURACY			(37 * 16)
#define CPASS_BIAS_SET			(34 * 16 + 14)
#define MAR_MODE				(37 * 16 + 2)
#define CPASS_COVARIANCE		(115 * 16)
#define CPASS_COVARIANCE_CUR	(118 * 16 +  8)
#define CPASS_REF_MAG_3D		(122 * 16)
#define CPASS_CAL_INIT			(114 * 16)
#define CPASS_EST_FIRST_BIAS	(113 * 16)
#define MAG_DISTURB_STATE		(113 * 16 + 2)
#define CPASS_VAR_COUNT			(112 * 16 + 6)
#define CPASS_COUNT_7			( 87 * 16 + 2)
#define CPASS_MAX_INNO			(124 * 16)
#define CPASS_BIAS_OFFSET		(113 * 16 + 4)
#define CPASS_CUR_BIAS_OFFSET	(114 * 16 + 4)
#define CPASS_PRE_SENSOR_DATA	( 87 * 16 + 4)

// Compass Cal params to be adjusted according to sampling rate
#define CPASS_TIME_BUFFER		(112 * 16 + 14)
#define CPASS_RADIUS_3D_THRESH_ANOMALY	(112 * 16 + 8)

#define CPASS_STATUS_CHK		(25 * 16 + 12)

// 9-axis
#define MAGN_THR_9X				(80 * 16)
#define MAGN_LPF_THR_9X			(80 * 16 +  8)
#define QFB_THR_9X				(80 * 16 + 12)

// DMP running counter
#define DMPRATE_CNTR			(18 * 16 + 4)

// pedometer
#define PEDSTD_BP_B				(49 * 16 + 12)
#define PEDSTD_BP_A4			(52 * 16)
#define PEDSTD_BP_A3			(52 * 16 +  4)
#define PEDSTD_BP_A2			(52 * 16 +  8)
#define PEDSTD_BP_A1			(52 * 16 + 12)
#define PEDSTD_SB				(50 * 16 +  8)
#define PEDSTD_SB_TIME			(50 * 16 + 12)
#define PEDSTD_PEAKTHRSH		(57 * 16 +  8)
#define PEDSTD_TIML				(50 * 16 + 10)
#define PEDSTD_TIMH				(50 * 16 + 14)
#define PEDSTD_PEAK				(57 * 16 +  4)
#define PEDSTD_STEPCTR			(54 * 16)
#define PEDSTD_STEPCTR2			(58 * 16 +  8)
#define PEDSTD_TIMECTR			(60 * 16 +  4)
#define PEDSTD_DECI				(58 * 16)
#define PEDSTD_SB2				(60 * 16 + 14)
#define STPDET_TIMESTAMP		(18 * 16 +  8)

// SMD
#define SMD_VAR_TH              (141 * 16 + 12)
#define SMD_VAR_TH_DRIVE        (143 * 16 + 12)
#define SMD_DRIVE_TIMER_TH      (143 * 16 +  8)
#define SMD_TILT_ANGLE_TH       (179 * 16 + 12)
#define BAC_SMD_ST_TH           (179 * 16 +  8)
#define BAC_ST_ALPHA4           (180 * 16 + 12)
#define BAC_ST_ALPHA4A          (176 * 16 + 12)

// Wake on Motion
#define WOM_ENABLE              (64 * 16 + 14)
#define WOM_STATUS              (64 * 16 + 6)
#define WOM_THRESHOLD           (64 * 16)
#define WOM_CNTR_TH             (64 * 16 + 12)

// Activity Recognition
#define BAC_RATE                (48  * 16 + 10)
#define BAC_STATE               (179 * 16 +  0)
#define BAC_STATE_PREV          (179 * 16 +  4)
#define BAC_ACT_ON              (182 * 16 +  0)
#define BAC_ACT_OFF             (183 * 16 +  0)
#define BAC_STILL_S_F           (177 * 16 +  0)
#define BAC_RUN_S_F             (177 * 16 +  4)
#define BAC_DRIVE_S_F           (178 * 16 +  0)
#define BAC_WALK_S_F            (178 * 16 +  4)
#define BAC_SMD_S_F             (178 * 16 +  8)
#define BAC_BIKE_S_F            (178 * 16 + 12)
#define BAC_E1_SHORT            (146 * 16 +  0)
#define BAC_E2_SHORT            (146 * 16 +  4)
#define BAC_E3_SHORT            (146 * 16 +  8)
#define BAC_VAR_RUN             (148 * 16 + 12)
#define BAC_TILT_INIT           (181 * 16 +  0)
#define BAC_MAG_ON              (225 * 16 +  0)
#define BAC_PS_ON               (74  * 16 +  0)
#define BAC_BIKE_PREFERENCE     (173 * 16 +  8)
#define BAC_MAG_I2C_ADDR        (229 * 16 +  8)
#define BAC_PS_I2C_ADDR         (75  * 16 +  4)
#define BAC_DRIVE_CONFIDENCE    (144 * 16 +  0)
#define BAC_WALK_CONFIDENCE     (144 * 16 +  4)
#define BAC_SMD_CONFIDENCE      (144 * 16 +  8)
#define BAC_BIKE_CONFIDENCE     (144 * 16 + 12)
#define BAC_STILL_CONFIDENCE    (145 * 16 +  0)
#define BAC_RUN_CONFIDENCE      (145 * 16 +  4)

// Flip/Pick-up
#define FP_VAR_ALPHA            (245 * 16 +  8)
#define FP_STILL_TH             (246 * 16 +  4)
#define FP_MID_STILL_TH         (244 * 16 +  8)
#define FP_NOT_STILL_TH         (246 * 16 +  8)
#define FP_VIB_REJ_TH           (241 * 16 +  8)
#define FP_MAX_PICKUP_T_TH      (244 * 16 + 12)
#define FP_PICKUP_TIMEOUT_TH    (248 * 16 +  8)
#define FP_STILL_CONST_TH       (246 * 16 + 12)
#define FP_MOTION_CONST_TH      (240 * 16 +  8)
#define FP_VIB_COUNT_TH         (242 * 16 +  8)
#define FP_STEADY_TILT_TH       (247 * 16 +  8)
#define FP_STEADY_TILT_UP_TH    (242 * 16 + 12)
#define FP_Z_FLAT_TH_MINUS      (243 * 16 +  8)
#define FP_Z_FLAT_TH_PLUS       (243 * 16 + 12)
#define FP_DEV_IN_POCKET_TH     (76  * 16 + 12)
#define FP_PICKUP_CNTR          (247 * 16 +  4)
#define FP_RATE                 (240 * 16 + 12)

// Accel FSR
#define ACC_SCALE               (30 * 16 + 0)
#define ACC_SCALE2              (79 * 16 + 4)

// S-Health keys
#define S_HEALTH_WALK_RUN_1		(213 * 16 +  12)
#define S_HEALTH_WALK_RUN_2		(213 * 16 +   8)
#define S_HEALTH_WALK_RUN_3		(213 * 16 +   4)
#define S_HEALTH_WALK_RUN_4		(213 * 16 +   0)
#define S_HEALTH_WALK_RUN_5		(212 * 16 +  12)
#define S_HEALTH_WALK_RUN_6		(212 * 16 +   8)
#define S_HEALTH_WALK_RUN_7		(212 * 16 +   4)
#define S_HEALTH_WALK_RUN_8		(212 * 16 +   0)
#define S_HEALTH_WALK_RUN_9		(211 * 16 +  12)
#define S_HEALTH_WALK_RUN_10	(211 * 16 +   8)
#define S_HEALTH_WALK_RUN_11	(211 * 16 +   4)
#define S_HEALTH_WALK_RUN_12	(211 * 16 +   0)
#define S_HEALTH_WALK_RUN_13	(210 * 16 +  12)
#define S_HEALTH_WALK_RUN_14	(210 * 16 +   8)
#define S_HEALTH_WALK_RUN_15	(210 * 16 +   4)
#define S_HEALTH_WALK_RUN_16	(210 * 16 +   0)
#define S_HEALTH_WALK_RUN_17	(209 * 16 +  12)
#define S_HEALTH_WALK_RUN_18	(209 * 16 +   8)
#define S_HEALTH_WALK_RUN_19	(209 * 16 +   4)
#define S_HEALTH_WALK_RUN_20	(209 * 16 +   0)
#define S_HEALTH_CADENCE1		(213 * 16 +  14)
#define S_HEALTH_CADENCE2		(213 * 16 +  10)
#define S_HEALTH_CADENCE3		(213 * 16 +   6)
#define S_HEALTH_CADENCE4		(213 * 16 +   2)
#define S_HEALTH_CADENCE5		(212 * 16 +  14)
#define S_HEALTH_CADENCE6		(212 * 16 +  10)
#define S_HEALTH_CADENCE7		(212 * 16 +   6)
#define S_HEALTH_CADENCE8		(212 * 16 +   2)
#define S_HEALTH_CADENCE9		(211 * 16 +  14)
#define S_HEALTH_CADENCE10		(211 * 16 +  10)
#define S_HEALTH_CADENCE11		(211 * 16 +   6)
#define S_HEALTH_CADENCE12		(211 * 16 +   2)
#define S_HEALTH_CADENCE13		(210 * 16 +  14)
#define S_HEALTH_CADENCE14		(210 * 16 +  10)
#define S_HEALTH_CADENCE15		(210 * 16 +   6)
#define S_HEALTH_CADENCE16		(210 * 16 +   2)
#define S_HEALTH_CADENCE17		(209 * 16 +  14)
#define S_HEALTH_CADENCE18		(209 * 16 +  10)
#define S_HEALTH_CADENCE19		(209 * 16 +   6)
#define S_HEALTH_CADENCE20		(209 * 16 +   2)
#define S_HEALTH_INT_PERIOD		(214 * 16 +   6)
#define S_HEALTH_INT_PERIOD2	(214 * 16 +  10)
#define S_HEALTH_BACKUP1		(214 * 16 +   0)
#define S_HEALTH_BACKUP2		(214 * 16 +   2)
#define S_HEALTH_RATE           (208 * 16 +  14)

// EIS authentication
#define EIS_AUTH_INPUT			(160 * 16 +   4)
#define EIS_AUTH_OUTPUT			(160 * 16 +   0)

#define DMP_START_ADDRESS   ((unsigned short)0x1000)
#define DMP_MEM_BANK_SIZE   256
#define DMP_LOAD_START      0x90

#define DMP_CODE_SIZE 5355

/** Loads the dmp firmware for the 20649 part.
* @param[in] dmp_image_sram Load DMP3 image from SRAM.
*/
int inv_load_firmware_20649(struct inv_icm20649 * s, const unsigned char *dmp3_image, unsigned int dmp3_image_size)
{
	return inv_icm20649_firmware_load(s, dmp3_image, dmp3_image_size, DMP_LOAD_START);
}

/** Loads the dmp firmware for the 20649 part.
* @param[out] dmp_cnfg The config item
*/
void inv_get_dmp_start_address_20649(struct inv_icm20649 * s, unsigned short *dmp_cnfg)
{

	(void)s;

	*dmp_cnfg = DMP_START_ADDRESS;
}

/**
* Sets data output control register 1.
* @param[in] output_mask	Turns sensors on/off according to following bit definition,
*							bit set indicates on, bit clear indicates off.
*							DMP will also turn hw sensors on/off based on bits set in output_mask.
*
*	ACCEL_SET			0x8000 - calibrated accel if accel calibrated, raw accel otherwise
*	GYRO_SET			0x4000 - raw gyro
*	CPASS_SET			0x2000 - raw magnetic
*	ALS_SET				0x1000 - ALS/proximity
*	QUAT6_SET			0x0800 - game rotation vector
*	QUAT9_SET			0x0400 - rotation vector with heading accuracy
*	PQUAT6_SET			0x0200 - truncated game rotation vector for batching
*	GEOMAG_SET			0x0100 - geomag rotation vector with heading accuracy
*	PRESSURE_SET		0x0080 - pressure
*	GYRO_CALIBR_SET		0x0040 - calibrated gyro
*	CPASS_CALIBR_SET	0x0020 - calibrated magnetic
*	PED_STEPDET_SET		0x0010 - timestamp when each step is detected
*	HEADER2_SET			0x0008 - enable/disable data output in data output control register 2
*	PED_STEPIND_SET		0x0007 - number of steps detected will be attached to the 3 least significant bits of header
*/
int dmp_set_data_output_control1_20649(struct inv_icm20649 * s, int output_mask)
{
  
    int result;
    unsigned char data_output_control_reg1[2];
    
    data_output_control_reg1[0] = (unsigned char)(output_mask >> 8);
    data_output_control_reg1[1] = (unsigned char)(output_mask & 0xff);

    result = inv_icm20649_write_mems(s, DATA_OUT_CTL1, 2, data_output_control_reg1);

    return result;
}

/**
* Sets data output control register 2.
* @param[in] output_mask	Turns features on/off according to following bit definition,
*							bit set indicates on, bit clear indicates off.
*
*	ACCEL_ACCURACY_SET	0x4000 - accel accuracy when changes, HEADER2_SET also needs to be set in data output control regsiter 1
*	GYRO_ACCURACY_SET	0x2000 - gyro accuracy when changes, HEADER2_SET also needs to be set in data output control regsiter 1
*	CPASS_ACCURACY_SET	0x1000 - compass accuracy when changes, HEADER2_SET also needs to be set in data output control regsiter 1
*	BATCH_MODE_EN		0x0100 - enable batching
*/
int dmp_set_data_output_control2_20649(struct inv_icm20649 * s, int output_mask)
{
    int result;
	static unsigned char data_output_control_reg2[2]={0};
    
    data_output_control_reg2[0] = (unsigned char)(output_mask >> 8);
    data_output_control_reg2[1] = (unsigned char)(output_mask & 0xff);

    result = inv_icm20649_write_mems(s, DATA_OUT_CTL2, 2, data_output_control_reg2);

    return result;
}

/**
* Clears all output control registers:
*	data output control register 1, data output control register 2, data interrupt control register, motion event control regsiter, data ready status register
*/
int dmp_reset_control_registers_20649(struct inv_icm20649 * s)
{
    int result;
    unsigned char data[4]={0};
    
    //reset data output control registers
    result = inv_icm20649_write_mems(s, DATA_OUT_CTL1, 2, &data[0]);
    result += inv_icm20649_write_mems(s, DATA_OUT_CTL2, 2, &data[0]);

	//reset data interrupt control register
    result += inv_icm20649_write_mems(s, DATA_INTR_CTL, 2, &data[0]);

    //reset motion event control register
    result += inv_icm20649_write_mems(s, MOTION_EVENT_CTL, 2, &data[0]);

    //reset data ready status register
    result += inv_icm20649_write_mems(s, DATA_RDY_STATUS, 2, &data[0]);
	//result += inv_icm20649_write_mems(s, DATA_RDY_STATUS, 2, inv_icm20649_convert_int16_to_big8(3, data)); //fixme

	return 0;
}

/**
* Sets data interrupt control register.
* @param[in] interrupt_ctl	Determines which sensors can generate interrupt according to following bit definition,
*							bit set indicates interrupt, bit clear indicates no interrupt.
*
*	ACCEL_SET			0x8000 - calibrated accel if accel calibrated, raw accel otherwise
*	GYRO_SET			0x4000 - raw gyro
*	CPASS_SET			0x2000 - raw magnetic
*	ALS_SET				0x1000 - ALS/proximity
*	QUAT6_SET			0x0800 - game rotation vector
*	QUAT9_SET			0x0400 - rotation vector with heading accuracy
*	PQUAT6_SET			0x0200 - truncated game rotation vector for batching
*	GEOMAG_SET			0x0100 - geomag rotation vector with heading accuracy
*	PRESSURE_SET		0x0080 - pressure
*	GYRO_CALIBR_SET		0x0040 - calibrated gyro
*	CPASS_CALIBR_SET	0x0020 - calibrated magnetic
*	PED_STEPDET_SET		0x0010 - timestamp when each step is detected
*	HEADER2_SET			0x0008 - data output defined in data output control register 2
*	PED_STEPIND_SET		0x0007 - number of steps detected will be attached to the 3 least significant bits of header
*/
int dmp_set_data_interrupt_control_20649(struct inv_icm20649 * s, uint32_t interrupt_ctl)
{	
    unsigned char big8[2] = {0};

	return inv_icm20649_write_mems(s, DATA_INTR_CTL, 2, inv_icm20649_convert_int16_to_big8(interrupt_ctl, big8));
}

/**
* Sets FIFO watermark. DMP will send FIFO interrupt if FIFO count > FIFO watermark
* @param[in] fifo_wm	FIFO watermark set to 80% of actual FIFO size by default
*/
int dmp_set_FIFO_watermark_20649(struct inv_icm20649 * s, unsigned short fifo_wm)
{
    int result;
	unsigned char big8[2]={0};

	result = inv_icm20649_write_mems(s, FIFO_WATERMARK, 2, inv_icm20649_convert_int16_to_big8(fifo_wm,big8));

	if (result)
        return result;

	return 0;
}

/**
* Sets data rdy status register.
* @param[in] data_rdy	Indicates which sensor data is available.
*
*	gyro samples available		0x1
*	accel samples available		0x2
*	secondary samples available	0x8
*/
int dmp_set_data_rdy_status_20649(struct inv_icm20649 * s, unsigned short data_rdy)
{
    unsigned char big8[2]={0};

	return inv_icm20649_write_mems(s, DATA_RDY_STATUS, 2, inv_icm20649_convert_int16_to_big8(data_rdy, big8));
}

/**
* Sets motion event control register.
* @param[in] output_mask	Turns features on/off according to following bit definition,
*							bit set indicates on, bit clear indicates off.
*
*	PEDOMETER_EN		0x4000 - pedometer engine
*	PEDOMETER_INT_EN	0x2000 - pedometer step detector interrupt
*	SMD_EN				0x0800 - significant motion detection interrupt
*	ACCEL_CAL_EN		0x0200 - accel calibration
*	GYRO_CAL_EN			0x0100 - gyro calibration
*	COMPASS_CAL_EN		0x0080 - compass calibration
*	NINE_AXIS_EN        0x0040 - 9-axis algorithm execution
*	GEOMAG_EN			0x0008 - Geomag algorithm execution
*	BAC_ACCEL_ONLY_EN   0x0004 - run BAC as accel only
*/
int dmp_set_motion_event_control_20649(struct inv_icm20649 * s, unsigned short output_mask)
{
    int result;
    unsigned char motion_event_control_reg[2];
    
    motion_event_control_reg[0] = (unsigned char)(output_mask >> 8);
    motion_event_control_reg[1] = (unsigned char)(output_mask & 0xff);

    result = inv_icm20649_write_mems(s, MOTION_EVENT_CTL, 2, motion_event_control_reg);

    return result;
}

/**
* Sets sensor ODR.
* @param[in] sensor		sensor number based on INV_SENSORS
*	enum INV_SENSORS {
*		INV_SENSOR_ACCEL = 0,
*		INV_SENSOR_GYRO,        
*	    INV_SENSOR_LPQ,
*		INV_SENSOR_COMPASS,
*		INV_SENSOR_ALS,
*		INV_SENSOR_SIXQ,
*		INV_SENSOR_NINEQ,
*		INV_SENSOR_GEOMAG,
*		INV_SENSOR_PEDQ,
*		INV_SENSOR_PRESSURE,
*		INV_SENSOR_CALIB_GYRO,
*		INV_SENSOR_CALIB_COMPASS,
*		INV_SENSOR_NUM_MAX,
*		INV_SENSOR_INVALID,
*	};					
* @param[in] divider	desired ODR = base engine rate/(divider + 1)
*/
int dmp_set_sensor_rate_20649(struct inv_icm20649 * s, int invSensor, short divider)
{
	int result;
    unsigned char big8[2]={0};
	int odr_addr = 0;

    switch (invSensor) {
		case INV_SENSOR_ACCEL:
			odr_addr = ODR_ACCEL;
			break;
		case INV_SENSOR_GYRO:
			odr_addr = ODR_GYRO;
			break;
		case INV_SENSOR_COMPASS:
			odr_addr = ODR_CPASS;
			break;
		case INV_SENSOR_ALS:
			odr_addr = ODR_ALS;
			break;
		case INV_SENSOR_SIXQ:
			odr_addr = ODR_QUAT6;
			break;
		case INV_SENSOR_NINEQ:
			odr_addr = ODR_QUAT9;
			break;
		case INV_SENSOR_GEOMAG:
			odr_addr = ODR_GEOMAG;
			break;
		case INV_SENSOR_PEDQ:
			odr_addr = ODR_PQUAT6;
			break;
		case INV_SENSOR_PRESSURE:
			odr_addr = ODR_PRESSURE;
			break;
		case INV_SENSOR_CALIB_GYRO:
			odr_addr = ODR_GYRO_CALIBR;
			break;
		case INV_SENSOR_CALIB_COMPASS:
			odr_addr = ODR_CPASS_CALIBR;
			break;
		case INV_SENSOR_STEP_COUNTER:
			//odr_addr = PED_RATE + 2; //PED_RATE is a 4-byte address but only writing 2 bytes here
			break;
    }	

    result = inv_icm20649_write_mems(s, odr_addr, 2, inv_icm20649_convert_int16_to_big8(divider, big8));

	if (result)
        return result;

	return 0;
}

/**
* Resets batch counter and sets batch mode parameters.
* @param[in] thld	sets batch timeout in DMP ticks, e.g. batch 1 sec, thld= (1 sec * engine base rate in Hz)
* @param[in] mask	ties batch counter to engine specified with same bit definiton as HW register DATA_RDY_STATUS,
*					i.e. batch counter increments only if the engine specified is available in multi-rate setting
*	BIT 0 set: 1 - tie to gyro
*	BIT 1 set: 2 - tie to accel
*	BIT 2 set: 4 - tie to pressure in Diamond
*	BIT 3 set: 8 - tie to secondary
*/
int dmp_set_batchmode_params_20649(struct inv_icm20649 * s, unsigned int thld, short mask)
{
    int result;
	unsigned char big8[4]={0};
	unsigned char data[2]={0};

	result = inv_icm20649_write_mems(s, BM_BATCH_CNTR, 4, big8);
	result += inv_icm20649_write_mems(s, BM_BATCH_THLD, 4, inv_icm20649_convert_int32_to_big8(thld,big8));
	result += inv_icm20649_write_mems(s, BM_BATCH_MASK, 2, inv_icm20649_convert_int16_to_big8(mask,data));

	if (result)
        return result;

	return 0;
}

/**
* Sets bias in DMP.
* @param[in] bias
*	array is set as follows:
*	[0] accel_x
*	[1] accel_y
*	[2] accel_z
*	[3] gyro_x
*	[4] gyro_y
*	[5] gyro_z
*	[6] compass_x
*	[7] compass_y
*	[8] compass_z
*/
int dmp_set_bias_20649(struct inv_icm20649 * s, int *bias)
{  
    int result;
	unsigned char big8[4]={0};

	result = inv_icm20649_write_mems(s, ACCEL_BIAS_X, 4, inv_icm20649_convert_int32_to_big8(bias[0], big8));
	result += inv_icm20649_write_mems(s, ACCEL_BIAS_Y, 4, inv_icm20649_convert_int32_to_big8(bias[1], big8));
    result += inv_icm20649_write_mems(s, ACCEL_BIAS_Z, 4, inv_icm20649_convert_int32_to_big8(bias[2], big8));
    
    if (result)
	{
		return result;
	}

	result = inv_icm20649_write_mems(s, GYRO_BIAS_X, 4, inv_icm20649_convert_int32_to_big8(bias[3], big8));
	result += inv_icm20649_write_mems(s, GYRO_BIAS_Y, 4, inv_icm20649_convert_int32_to_big8(bias[4], big8));
    result += inv_icm20649_write_mems(s, GYRO_BIAS_Z, 4, inv_icm20649_convert_int32_to_big8(bias[5], big8));
    
    if (result)
	{
		return result;
	}

	result = inv_icm20649_write_mems(s, CPASS_BIAS_X, 4, inv_icm20649_convert_int32_to_big8(bias[6], big8));
	result += inv_icm20649_write_mems(s, CPASS_BIAS_Y, 4, inv_icm20649_convert_int32_to_big8(bias[7], big8));
    result += inv_icm20649_write_mems(s, CPASS_BIAS_Z, 4, inv_icm20649_convert_int32_to_big8(bias[8], big8));

	return 0; 
}

/**
* Gets bias from DMP.
* @param[in] bias
* @param[out] bias
*	array is set as follows:
*	[0] accel_x
*	[1] accel_y
*	[2] accel_z
*	[3] gyro_x
*	[4] gyro_y
*	[5] gyro_z
*	[6] compass_x
*	[7] compass_y
*	[8] compass_z
*/
int dmp_get_bias_20649(struct inv_icm20649 * s, int *bias)
{
    int result;
	unsigned char big8[4]={0};
	
	result = inv_icm20649_read_mems(s, ACCEL_BIAS_X, 4, big8);
	bias[0] = inv_icm20649_convert_big8_to_int32(big8);
	result += inv_icm20649_read_mems(s, ACCEL_BIAS_Y, 4, big8);
	bias[1] = inv_icm20649_convert_big8_to_int32(big8);
	result += inv_icm20649_read_mems(s, ACCEL_BIAS_Z, 4, big8);
	bias[2] = inv_icm20649_convert_big8_to_int32(big8);

	result = inv_icm20649_read_mems(s, GYRO_BIAS_X, 4, big8);
	bias[3] = inv_icm20649_convert_big8_to_int32(big8);
	result += inv_icm20649_read_mems(s, GYRO_BIAS_Y, 4, big8);
	bias[4] = inv_icm20649_convert_big8_to_int32(big8);
	result += inv_icm20649_read_mems(s, GYRO_BIAS_Z, 4, big8);
	bias[5] = inv_icm20649_convert_big8_to_int32(big8);

	result = inv_icm20649_read_mems(s, CPASS_BIAS_X, 4, big8);
	bias[6] = inv_icm20649_convert_big8_to_int32(big8);
	result += inv_icm20649_read_mems(s, CPASS_BIAS_Y, 4, big8);
	bias[7] = inv_icm20649_convert_big8_to_int32(big8);
	result += inv_icm20649_read_mems(s, CPASS_BIAS_Z, 4, big8);
	bias[8] = inv_icm20649_convert_big8_to_int32(big8);

	return 0; 
}

/**
* Sets the gyro_sf used by quaternions on the DMP.
* @param[in] gyro_sf	see inv_icm20649_set_gyro_sf() for value to set based on gyro rate and gyro fullscale range
*/
int dmp_set_gyro_sf_20649(struct inv_icm20649 * s, long gyro_sf)
{
    int result;
    unsigned char big8[4];

    result = inv_icm20649_write_mems(s, GYRO_SF, 4, inv_icm20649_convert_int32_to_big8(gyro_sf, big8));

    return result;
}

/**
* Sets the accel gain used by accel quaternion on the DMP.
* @param[in] accel_gain		value changes with accel engine rate
*/
int dmp_set_accel_feedback_gain_20649(struct inv_icm20649 * s, int accel_gain)
{
	int result;
    unsigned char big8[4]={0};

    result = inv_icm20649_write_mems(s, ACCEL_ONLY_GAIN, 4, inv_icm20649_convert_int32_to_big8(accel_gain, big8));

	if (result)
        return result;

	return 0;
}

/**
* Turns software wake on motion feature on/off.
* @param[in] enable		0=off, 1=on
*/
int dmp_set_wom_enable_20649(struct inv_icm20649 * s, unsigned char enable)
{
	int result;
    unsigned char big8[2]={0};

	if (enable) {
		big8[1]= 0x1;
    }

    result = inv_icm20649_write_mems(s, WOM_ENABLE, 2, big8);

	if (result)
        return result;

	return 0;
}

/**
* Sets motion threshold to determine motion/no motion for wake on motion feature.
* @param[in] threshold
*/
int dmp_set_wom_motion_threshold_20649(struct inv_icm20649 * s, int threshold)
{
	int result;
    unsigned char big8[4]={0};

    result = inv_icm20649_write_mems(s, WOM_THRESHOLD, 4, inv_icm20649_convert_int32_to_big8(threshold, big8));

	if (result)
        return result;

	return 0;
}

/**
* Sets minimum time threshold of no motion before DMP goes to sleep.
* @param[in] threshold
*/
int dmp_set_wom_time_threshold_20649(struct inv_icm20649 * s, unsigned short threshold)
{
	int result;
    unsigned char big8[2]={0};

    result = inv_icm20649_write_mems(s, WOM_CNTR_TH, 2, inv_icm20649_convert_int16_to_big8(threshold, big8));

	if (result)
        return result;

	return 0;
}

/**
* Sets scale in DMP to convert accel data to 1g=2^25 regardless of fsr.
* @param[in] fsr for accel parts
             2: 2g. 4: 4g. 8: 8g. 16: 16g. 32: 32g.

             For 2g parts, 2g = 2^15 -> 1g = 2^14,.
             DMP takes raw accel data and left shifts by 16 bits, so 1g=2^14 (<<16) becomes 1g=2^30, to make 1g=2^25, >>5 bits.
             In Q-30 math, >> 5 equals multiply by 2^25 = 33554432.

             For 8g parts, 8g = 2^15 -> 1g = 2^12.
             DMP takes raw accel data and left shifts by 16 bits, so 1g=2^12 (<<16) becomes 1g=2^28, to make 1g=2^25, >>3bits.
             In Q-30 math, >> 3 equals multiply by 2^27 = 134217728.
*/
int dmp_set_accel_fsr_20649(struct inv_icm20649 * s, short accel_fsr)
{
    unsigned char reg[4];
    int result;
    long scale;

    switch (accel_fsr) {
    case 2:
        scale =  33554432L;  // 2^25
        break;
    case 4:
        scale =  67108864L;  // 2^26
        break;
    case 8:
        scale = 134217728L;  // 2^27
        break;
    case 16:
        scale = 268435456L;  // 2^28
        break;
    case 32:
        scale = 536870912L;  // 2^29
        break;
    default:
        return -1;
    }

    result = inv_icm20649_write_mems(s, ACC_SCALE, 4, inv_icm20649_convert_int32_to_big8(scale,reg));

    if (result) {
        return result;
    } else {
        return 0;
    }
}

/**
* According to input fsr, a scale factor will be set at memory location ACC_SCALE2
* to convert calibrated accel data to 16-bit format same as what comes out of MPU register.
* It is a reverse scaling of the scale factor written to ACC_SCALE.
* @param[in] fsr for accel parts
			 2: 2g. 4: 4g. 8: 8g. 16: 16g. 32: 32g.
*/
int dmp_set_accel_scale2_20649(struct inv_icm20649 * s, short accel_fsr)
{
    unsigned char reg[4];
    int result;
    long scale;

    switch (accel_fsr) {
    case 2:
        scale = 524288L;  // 2^19
        break;
    case 4:
        scale = 262144L;  // 2^18
        break;
    case 8:
        scale = 131072L;  // 2^17
        break;
    case 16:
        scale = 65536L;  // 2^16
        break;
    case 32:
        scale = 32768L;  // 2^15
        break;
    default:
        return -1;
    }

    result = inv_icm20649_write_mems(s, ACC_SCALE2, 4, inv_icm20649_convert_int32_to_big8(scale,reg));

    if (result) {
        return result;
    } else {
        return 0;
    }
}