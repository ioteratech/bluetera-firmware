/** ________________________________________________________________________________________________________
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
#include "Invn/Devices/SensorTypes.h"

#include "Icm20649Setup.h"
#include "Icm20649.h"
#include "Icm20649Defs.h"
#include "Icm20649DataBaseDriver.h"
#include "Icm20649DataBaseControl.h"
#include "Icm20649MPUFifoControl.h"
#include "Icm20649LoadFirmware.h"
#include "Icm20649Dmp3Driver.h"

#include "Invn/EmbUtils/DataConverter.h"

#include <assert.h>

static uint8_t sensor_type_2_android_sensor(enum inv_icm20649_sensor sensor)
{
	switch(sensor) {
	case INV_ICM20649_SENSOR_RAW_ACCELEROMETER:             return ANDROID_SENSOR_RAW_ACCELEROMETER;
	case INV_ICM20649_SENSOR_RAW_GYROSCOPE:                 return ANDROID_SENSOR_RAW_GYROSCOPE;
	case INV_ICM20649_SENSOR_GYROSCOPE:                     return ANDROID_SENSOR_GYROSCOPE;
	case INV_ICM20649_SENSOR_GYROSCOPE_UNCALIBRATED:        return ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED;
	case INV_ICM20649_SENSOR_GAME_ROTATION_VECTOR:          return ANDROID_SENSOR_GAME_ROTATION_VECTOR;
	case INV_ICM20649_SENSOR_ACCELEROMETER:                 return ANDROID_SENSOR_ACCELEROMETER;
	case INV_ICM20649_SENSOR_WOM:                           return ANDROID_SENSOR_WOM;
	default:                                                return ANDROID_SENSOR_NUM_MAX;
	}
}

enum inv_icm20649_sensor inv_icm20649_sensor_android_2_sensor_type(int sensor)
{
	switch(sensor) {
	case ANDROID_SENSOR_RAW_ACCELEROMETER:                return INV_ICM20649_SENSOR_RAW_ACCELEROMETER;
	case ANDROID_SENSOR_RAW_GYROSCOPE:                    return INV_ICM20649_SENSOR_RAW_GYROSCOPE;
	case ANDROID_SENSOR_GYROSCOPE:                        return INV_ICM20649_SENSOR_GYROSCOPE;
	case ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED:           return INV_ICM20649_SENSOR_GYROSCOPE_UNCALIBRATED;
	case ANDROID_SENSOR_GAME_ROTATION_VECTOR:             return INV_ICM20649_SENSOR_GAME_ROTATION_VECTOR;
	case ANDROID_SENSOR_ACCELEROMETER:                    return INV_ICM20649_SENSOR_ACCELEROMETER;
	case ANDROID_SENSOR_WOM:                              return INV_ICM20649_SENSOR_WOM;
	default:                                              return INV_ICM20649_SENSOR_MAX;
	}
}

static int skip_sensor(struct inv_icm20649 * s, unsigned char androidSensor)
{
	enum inv_icm20649_sensor icm20649_sensor_id = inv_icm20649_sensor_android_2_sensor_type(androidSensor);
	uint8_t skip_sample = s->skip_sample[icm20649_sensor_id];
	
	if (s->skip_sample[icm20649_sensor_id])
		s->skip_sample[icm20649_sensor_id]--;

	return skip_sample;
}

static int inv_icm20649_enable_wom(struct inv_icm20649 * s, inv_bool_t state)
{
	unsigned char data;
	int result = 0;

	if(state) {
		//Enable WOM IT
		result |= inv_icm20649_read_mems_reg(s, REG_INT_ENABLE, 1, &data); 
		data |= (1 << 3); 
		result |= inv_icm20649_write_mems_reg(s, REG_INT_ENABLE, 1, &data);
		//In the case we are in Mode 1 we have to drop first wom event
		if(s->wake_on_motion_drop_first_event == 0xFF) {
			if(BIT_ACCEL_INTEL_MODE_INT == 1){ 
				s->wake_on_motion_drop_first_event = 0x01;
			}
			else{
				s->wake_on_motion_drop_first_event = 0x00;
			}
		}
		//Configure WOM and thr
		data = BIT_ACCEL_INTEL_MODE_INT | BIT_ACCEL_INTEL_ENABLE_WOM;
		result |= inv_icm20649_write_mems_reg(s, REG_ACCEL_INTEL_CTRL, 1, &data);
		data = s->wake_on_motion_thr;
		result |= inv_icm20649_write_mems_reg(s, REG_ACCEL_WOM_THR, 1, &data);
		s->wake_on_motion_enabled = 1;
	}
	else {
		//Disable WOM IT
		result |= inv_icm20649_read_mems_reg(s, REG_INT_ENABLE, 1, &data); 
		data &= ~(1 << 3); 
		result |= inv_icm20649_write_mems_reg(s, REG_INT_ENABLE, 1, &data);
		//Disable WOM
		data =~((unsigned char)(BIT_ACCEL_INTEL_ENABLE_WOM | BIT_ACCEL_INTEL_MODE_INT));
		result |= inv_icm20649_write_mems_reg(s, REG_ACCEL_INTEL_CTRL, 1, &data);
		s->wake_on_motion_enabled = 0;
	}

	return result;
}

/* Identification related functions */
int inv_icm20649_get_whoami(struct inv_icm20649 * s, uint8_t * whoami)
{
	return inv_icm20649_read_reg_one(s, REG_WHO_AM_I, whoami);
}

void inv_icm20649_init_matrix(struct inv_icm20649 * s)
{
	// initialize chip to body
	s->s_quat_chip_to_body[0] = (1L<<30);
	s->s_quat_chip_to_body[1] = 0;
	s->s_quat_chip_to_body[2] = 0;
	s->s_quat_chip_to_body[3] = 0;
	//initialize mounting matrix
	memset(s->mounting_matrix, 0, sizeof(s->mounting_matrix));
	s->mounting_matrix[0] = 1;
	s->mounting_matrix[4] = 1;
	s->mounting_matrix[8] = 1;

	inv_icm20649_set_chip_to_body_axis_quaternion(s, s->mounting_matrix, 0.0);
}

int inv_icm20649_init_structure(struct inv_icm20649 * s)
{
	inv_icm20649_base_control_init(s);
	inv_icm20649_transport_init(s);
	//Init state
	s->count_state = COUNT_1_STABLE_20649;
	s->set_accuracy = 0;
	s->new_accuracy = 0;
	s->wake_on_motion_thr = 0xFF;
	s->wake_on_motion_drop_first_event = 0xFF;
	return 0;
}

int inv_icm20649_initialize(struct inv_icm20649 * s, const uint8_t *dmp3_image, uint32_t dmp3_image_size)
{
	if(s->serif.is_spi) {
		/* Hardware initialization */
		// No image to be loaded from flash, no pointer to pass.
		if (inv_icm20649_initialize_lower_driver(s, SERIAL_INTERFACE_SPI, dmp3_image, dmp3_image_size)) {
			return -1;
		}
	}
	else {
		/* Hardware initialization */
		// No image to be loaded from flash, no pointer to pass.
		if (inv_icm20649_initialize_lower_driver(s, SERIAL_INTERFACE_I2C, dmp3_image, dmp3_image_size)) {
			return -1;
		}
	}
	return 0;
}

int inv_icm20649_init_scale(struct inv_icm20649 * s)
{
	int result = 0;

	/* Force accelero fullscale to 30g and gyro to 4000dps */
	result |= inv_icm20649_set_accel_fullscale(s, MPU_FS_30G);
	result |= inv_icm20649_set_gyro_fullscale(s, MPU_FS_4000dps);

	return result;
}

int inv_icm20649_set_wom_threshold(struct inv_icm20649 * s, uint8_t threshold)
{
	s->wake_on_motion_thr = threshold;
	return 0;
}

int inv_icm20649_set_fsr(struct inv_icm20649 * s, enum inv_icm20649_sensor sensor, const void * fsr)
{
	int result = 0;
	int * castedvalue = (int*) fsr;
	if((sensor == INV_ICM20649_SENSOR_RAW_ACCELEROMETER) ||
	   (sensor == INV_ICM20649_SENSOR_ACCELEROMETER)){
		enum mpu_accel_fs afsr;
		if(*castedvalue == 4)
			afsr = MPU_FS_4G;
		else if(*castedvalue == 8)
			afsr = MPU_FS_8G;
		else if(*castedvalue == 16)
			afsr = MPU_FS_16G;
		else if(*castedvalue == 30)
			afsr = MPU_FS_30G;
		else 
			return -1;
		result |= inv_icm20649_set_accel_fullscale(s, afsr);
	}
	else if((sensor == INV_ICM20649_SENSOR_GYROSCOPE) ||
			(sensor == INV_ICM20649_SENSOR_RAW_GYROSCOPE) ||
			(sensor == INV_ICM20649_SENSOR_GYROSCOPE_UNCALIBRATED)) {
		enum mpu_gyro_fs gfsr;
		if(*castedvalue == 500)
			gfsr = MPU_FS_500dps;
		else if(*castedvalue == 1000)
		 	gfsr = MPU_FS_1000dps;
		else if(*castedvalue == 2000)
		 	gfsr = MPU_FS_2000dps;
		else if(*castedvalue == 4000)
			gfsr = MPU_FS_4000dps;
		else 
			return -1;
		result |= inv_icm20649_set_gyro_fullscale(s, gfsr);
	}
	return result;
}

int inv_icm20649_get_fsr(struct inv_icm20649 * s, enum inv_icm20649_sensor sensor, const void * fsr)
{
	
	if((sensor == INV_ICM20649_SENSOR_RAW_ACCELEROMETER) ||
	   (sensor == INV_ICM20649_SENSOR_ACCELEROMETER)){
		unsigned char * castedvalue = (unsigned char*) fsr;
		int afsr = inv_icm20649_get_accel_fullscale(s);
		if(afsr == MPU_FS_4G)
			* castedvalue = 4;
		else if(afsr == MPU_FS_8G)
			* castedvalue = 8;
		else if(afsr == MPU_FS_16G)
			* castedvalue = 16;
		else if(afsr == MPU_FS_30G)
			* castedvalue = 30;
		else 
			return -1;
		
		return 1;
	}
	else if((sensor == INV_ICM20649_SENSOR_GYROSCOPE) ||
			(sensor == INV_ICM20649_SENSOR_RAW_GYROSCOPE) ||
			(sensor == INV_ICM20649_SENSOR_GYROSCOPE_UNCALIBRATED)) {
		unsigned short * castedvalue = (unsigned short*) fsr;
		int gfsr = inv_icm20649_get_gyro_fullscale(s);
		if(gfsr == MPU_FS_500dps)
			* castedvalue = 500;
		else if(gfsr == MPU_FS_1000dps)
		 	* castedvalue = 1000;
		else if(gfsr == MPU_FS_2000dps)
		 	* castedvalue = 2000;
		else if(gfsr == MPU_FS_4000dps)
			* castedvalue = 4000;
		else 
			return -1;
		
		return 2;
	}
	
	return 0;
}

static void DmpDriver_convertion(signed char transformedtochar[9],
                                 const int32_t MatrixInQ30[9])
{
	// To convert Q30 to signed char value
	uint8_t iter;
	for (iter = 0; iter < 9; ++iter)
		transformedtochar[iter] = MatrixInQ30[iter] >> 30;
}

int inv_icm20649_set_matrix(struct inv_icm20649 * s, const float matrix[9], enum inv_icm20649_sensor sensor)
{
	int32_t mounting_mq30[9];
	int result = 0;
	int i;

	if ((sensor == INV_ICM20649_SENSOR_RAW_ACCELEROMETER) || 
		(sensor == INV_ICM20649_SENSOR_ACCELEROMETER) || 
		(sensor == INV_ICM20649_SENSOR_RAW_GYROSCOPE) || 
		(sensor == INV_ICM20649_SENSOR_GYROSCOPE) ||
		(sensor == INV_ICM20649_SENSOR_GYROSCOPE_UNCALIBRATED)) {
		for(i = 0; i < 9; ++i)
			mounting_mq30[i] = (int32_t)(matrix[i] * (1 << 30));
		// Convert mounting matrix in char
		DmpDriver_convertion(s->mounting_matrix, mounting_mq30);
		/*Apply new matrix */
		inv_icm20649_set_chip_to_body_axis_quaternion(s, s->mounting_matrix, 0.0);
	}

	return result;
}

int inv_icm20649_soft_reset(struct inv_icm20649 * s)
{
	//soft reset like
	int rc = inv_icm20649_write_single_mems_reg(s, REG_PWR_MGMT_1, BIT_H_RESET);
	// max start-up time is 100 msec
	inv_icm20649_sleep_us(100000);
	return rc;
}

int inv_icm20649_enable_sensor(struct inv_icm20649 * s, enum inv_icm20649_sensor sensor, inv_bool_t state)
{
	uint8_t androidSensor = sensor_type_2_android_sensor(sensor);
	int result = 0;
	
	//WOM case 
	if(sensor == INV_ICM20649_SENSOR_WOM) {
		inv_icm20649_enable_wom(s, state);
	}
	if(0!=inv_icm20649_ctrl_enable_sensor(s, androidSensor, state))
		return -1;

	return result;
}

int inv_icm20649_set_sensor_period(struct inv_icm20649 * s, enum inv_icm20649_sensor sensor, uint32_t period)
{
	uint8_t androidSensor = sensor_type_2_android_sensor(sensor);

	if(0!=inv_icm20649_set_odr(s, androidSensor, period))
		return -1;

	return 0; 
}

int inv_icm20649_enable_batch_timeout(struct inv_icm20649 * s, unsigned short batchTimeoutMs)
{
	int rc;
	/* Configure batch timeout */
	if (inv_icm20649_ctrl_set_batch_timeout_ms(s, batchTimeoutMs) == 0) {
		/* If configuration was succesful then we enable it */
		if((rc = inv_icm20649_ctrl_enable_batch(s, 1)) != 0)
			return rc;
	} else {         
		/* Else we disable it */
		if((rc = inv_icm20649_ctrl_enable_batch(s, 0)) != 0)
			return rc;                     
	}
	return 0;
}

int inv_icm20649_load(struct inv_icm20649 * s, const uint8_t * image, unsigned short size)
{
	return inv_icm20649_firmware_load(s, image, size, DMP_LOAD_START);
}


int inv_icm20649_set_lowpower_or_highperformance(struct inv_icm20649 * s, uint8_t lowpower_or_highperformance)
{
	s->go_back_lp_when_odr_low = 0;
	if(lowpower_or_highperformance)
		return inv_icm20649_enter_low_noise_mode(s);
	else
		return inv_icm20649_enter_duty_cycle_mode(s);
}


int inv_icm20649_get_lowpower_or_highperformance(struct inv_icm20649 * s, uint8_t * lowpower_or_highperformance)
{
	*lowpower_or_highperformance = CHIP_LOW_NOISE;
	return 1;
}

/** @brief Preprocess all timestamps so that they either contain very last time at which MEMS IRQ was fired 
 * or last time sent for the sensor + ODR */
static uint8_t inv_icm20649_updateTs(struct inv_icm20649 * s, int * data_left_in_fifo, 
	unsigned short * total_sample_cnt, unsigned short * sample_cnt_array)
{
	/** @brief Very last time in us at which IRQ was fired since flushing FIFO process was started */
	uint64_t lastIrqTimeUs = inv_icm20649_get_dataready_interrupt_time_us();
	uint8_t i;

	(void) data_left_in_fifo, (void)total_sample_cnt;

	// we parse all senosr according to android type
	for (i = 0; i < GENERAL_SENSORS_MAX; i++) {
			if (sample_cnt_array[i]) {
				/** Number of samples present in MEMS FIFO last time we mirrored it */
				unsigned short fifo_sample_cnt = sample_cnt_array[i];

				/** In case of first batch we have less than the expected number of samples in the batch */
				/** To avoid a bad timestamping we recompute the startup time based on the theorical ODR and the number of samples */
				if (s->sFirstBatch[inv_icm20649_sensor_android_2_sensor_type(i)]) {
					s->timestamp[inv_icm20649_sensor_android_2_sensor_type(i)] += lastIrqTimeUs-s->timestamp[inv_icm20649_sensor_android_2_sensor_type(i)] 
																	- fifo_sample_cnt*s->sensorlist[inv_icm20649_sensor_android_2_sensor_type(i)].odr_us;
					s->sFirstBatch[inv_icm20649_sensor_android_2_sensor_type(i)] = 0;
				}
				
				/** In case we flush explicitely the FIFO, update timestamp for all streamed sensors
				depending on theoretical ODR */
				/** In all other cases, update timestamp for all streamed sensors depending on number of samples available in FIFO
				first time to be printed is t1+(t2-t1)/N
				- t1 is last time we sent data
				- t2 is when IRQ was fired so that we pop the FIFO
				- N is number of samples */
				s->sensorlist[inv_icm20649_sensor_android_2_sensor_type(i)].odr_applied_us = (lastIrqTimeUs-s->timestamp[inv_icm20649_sensor_android_2_sensor_type(i)])/fifo_sample_cnt;

				switch (s->count_state){
				case COUNT_1_STABLE_20649:
					s->sensorlist[inv_icm20649_sensor_android_2_sensor_type(i)].odr_applied_us = (lastIrqTimeUs-s->timestamp[inv_icm20649_sensor_android_2_sensor_type(i)])/fifo_sample_cnt;
					if(fifo_sample_cnt != 1)
						s->count_state = COUNT_DIFF_TRANSITION_20649;
					else
						break;
				case COUNT_DIFF_TRANSITION_20649:
					s->saved_count = fifo_sample_cnt - 1;
					s->count_state = COUNT_DIFF_STABLE_20649;
					break;
				case COUNT_DIFF_STABLE_20649:
					if(s->saved_count  == 0) {
						if(fifo_sample_cnt == 1)
							s->count_state = COUNT_1_TRANSITION_20649;
						else
							s->count_state = COUNT_DIFF_TRANSITION_20649;
					}
					else {
						s->saved_count ++;
					}
					break;
				case COUNT_1_TRANSITION_20649:
					lastIrqTimeUs = inv_icm20649_get_time_us();
					s->sensorlist[inv_icm20649_sensor_android_2_sensor_type(i)].odr_applied_us = (lastIrqTimeUs-s->timestamp[inv_icm20649_sensor_android_2_sensor_type(i)])/fifo_sample_cnt;
					s->count_state = COUNT_1_STABLE_20649;
					break;
				default :
					break;
				}
				// Get the new applied timestamp
				s->timestamp[inv_icm20649_sensor_android_2_sensor_type(i)] += s->sensorlist[inv_icm20649_sensor_android_2_sensor_type(i)].odr_applied_us;
			}
	}
	
	return 0;
}

static uint8_t inv_icm20649_get_total_count(struct inv_icm20649 * s, int * data_left_in_fifo, 
	unsigned short * total_sample_cnt, unsigned short * sample_cnt_array)
{

	/** Mirror the DMP FIFO into a SW FIFO,
	and then extract total number of samples, whatever the sensor, and number of sample per sensor */
	if (inv_icm20649_fifo_swmirror(s, data_left_in_fifo, total_sample_cnt, sample_cnt_array)) {
		return -1;
	}

	return 0;
}

int inv_icm20649_poll_sensor(struct inv_icm20649 * s, void * context,
		void (*handler)(void * context, enum inv_icm20649_sensor sensor, uint64_t timestamp, const void * data, const void *arg))
{
	short int_read_back=0;
	unsigned short header=0, header2 = 0; 
	int data_left_in_fifo=0;
	short short_data[3] = {0};
	signed long  long_data[3] = {0};
	signed long  long_quat[3] = {0};
	float gyro_raw_float[3];
	float gyro_bias_float[3];
	int gyro_accuracy = 0;
	int dummy_accuracy = 0;
	float grv_float[4];
	float gyro_float[3];
	unsigned short sample_cnt_array[GENERAL_SENSORS_MAX] = {0};

	inv_icm20649_identify_interrupt(s, &int_read_back);
	
	if (int_read_back & (BIT_MSG_DMP_INT | BIT_MSG_DMP_INT_0)){
		do {
			unsigned short total_sample_cnt = 0;

			memset(sample_cnt_array, 0, sizeof(sample_cnt_array));
			/* Mirror FIFO contents and stop processing FIFO if an error was detected*/
			if(inv_icm20649_get_total_count(s, &data_left_in_fifo, &total_sample_cnt, sample_cnt_array))
				break;

			while(total_sample_cnt--) {
				if(inv_icm20649_updateTs(s, &data_left_in_fifo, &total_sample_cnt, sample_cnt_array))
					break;
				/* Read FIFO contents and parse it, and stop processing FIFO if an error was detected*/
				if (inv_icm20649_fifo_pop(s, &header, &header2, &data_left_in_fifo))
					break;
				
				/* Gyro sample available from DMP FIFO */
				if (header & GYRO_SET) {
					float lScaleDeg = (1 << inv_icm20649_get_gyro_fullscale(s)) * 500.f ;// From raw to dps to degree per seconds
					signed long  lRawGyroQ15[3] = {0};
					signed long  lBiasGyroQ20[3] = {0};
					
					/* Read raw gyro out of DMP FIFO and convert it from Q15 raw data format to degree per seconds */
					inv_icm20649_dmp_get_raw_gyro(short_data);  
					lRawGyroQ15[0] = (long) short_data[0];
					lRawGyroQ15[1] = (long) short_data[1];
					lRawGyroQ15[2] = (long) short_data[2];
					inv_icm20649_convert_dmp3_to_body(s, lRawGyroQ15, lScaleDeg/(1L<<15), gyro_raw_float);
					
					if(inv_icm20649_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_GYROSCOPE) && !skip_sensor(s, ANDROID_SENSOR_RAW_GYROSCOPE)) {
						long out[3];
						invn_convert_quat_rotate_fxp(s->s_quat_chip_to_body, lRawGyroQ15, out);
						handler(context, INV_ICM20649_SENSOR_RAW_GYROSCOPE, s->timestamp[INV_ICM20649_SENSOR_RAW_GYROSCOPE], out, &dummy_accuracy);
					}
					/* Read bias gyro out of DMP FIFO and convert it from Q20 raw data format to degree per seconds */
					inv_icm20649_dmp_get_gyro_bias(short_data);
					lBiasGyroQ20[0] = (long) short_data[0];
					lBiasGyroQ20[1] = (long) short_data[1];
					lBiasGyroQ20[2] = (long) short_data[2];
					inv_icm20649_convert_dmp3_to_body(s, lBiasGyroQ20, lScaleDeg/(1L<<20), gyro_bias_float);
					/* Extract accuracy and calibrated gyro data based on raw/bias data if calibrated gyro sensor is enabled */
					gyro_accuracy = inv_icm20649_get_gyro_accuracy();
					/* If accuracy has changed previously we update the new accuracy the same time as bias*/
					if(s->set_accuracy){
						s->set_accuracy = 0;
						s->new_accuracy = gyro_accuracy;
					}
					/* gyro accuracy has changed, we will notify it the next time*/
					if(gyro_accuracy != s->new_accuracy){
						s->set_accuracy = 1;
					}
					if(inv_icm20649_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_GYROSCOPE) && !skip_sensor(s, ANDROID_SENSOR_GYROSCOPE)) {
						
						// shift to Q20 to do all calibrated gyrometer operations in Q20
						lRawGyroQ15[0] <<= 5;
						lRawGyroQ15[1] <<= 5;
						lRawGyroQ15[2] <<= 5;
						/* Compute calibrated gyro data based on raw and bias gyro data and convert it from Q20 raw data format to degree per seconds */
						inv_icm20649_dmp_get_calibrated_gyro(long_data, lRawGyroQ15, lBiasGyroQ20);
						inv_icm20649_convert_dmp3_to_body(s, long_data, lScaleDeg/(1L<<20), gyro_float);
						handler(context, INV_ICM20649_SENSOR_GYROSCOPE, s->timestamp[INV_ICM20649_SENSOR_GYROSCOPE], gyro_float, &s->new_accuracy);
					}
					if(inv_icm20649_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED) && !skip_sensor(s, ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED)) {
						float raw_bias_gyr[6];
						raw_bias_gyr[0] = gyro_raw_float[0];
						raw_bias_gyr[1] = gyro_raw_float[1];
						raw_bias_gyr[2] = gyro_raw_float[2];
						raw_bias_gyr[3] = gyro_bias_float[0];
						raw_bias_gyr[4] = gyro_bias_float[1];
						raw_bias_gyr[5] = gyro_bias_float[2];
						/* send raw float and bias for uncal gyr*/
						handler(context, INV_ICM20649_SENSOR_GYROSCOPE_UNCALIBRATED, s->timestamp[INV_ICM20649_SENSOR_GYROSCOPE_UNCALIBRATED], raw_bias_gyr, &s->new_accuracy);
					}
				}
				/* Calibrated accel sample available from DMP FIFO */
				if (header & ACCEL_SET) {
					/* Read calibrated accel out of DMP FIFO from Q28/Q27/Q26/Q25 depending on full scale applied */
					inv_icm20649_dmp_get_accel(long_data);

					if(inv_icm20649_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_ACCELEROMETER) && !skip_sensor(s, ANDROID_SENSOR_RAW_ACCELEROMETER)) {
						long out[3];
						invn_convert_quat_rotate_fxp(s->s_quat_chip_to_body, long_data, out);
						/* convert to raw data format to Q12/Q11/Q10/Q9 depending on full scale applied,
						so that it fits on 16bits so that it can go through any protocol, even the one which have raw data on 16b */
						out[0] = out[0] >> 15;
						out[1] = out[1] >> 15;
						out[2] = out[2] >> 15;
						handler(context, INV_ICM20649_SENSOR_RAW_ACCELEROMETER, s->timestamp[INV_ICM20649_SENSOR_RAW_ACCELEROMETER], out, &dummy_accuracy);
					}
					if(inv_icm20649_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_ACCELEROMETER) && !skip_sensor(s, ANDROID_SENSOR_ACCELEROMETER)) {
						float accel_float[3];
						/* convert from raw unit to g's :
						1g is represented in raw data by 1 in Q25 to Q28 depending on FS,
						then convert this value to float,
						then apply it to amount of raw data read previously (and apply mounting matrix) */
						uint8_t acc_fs = inv_icm20649_get_accel_fullscale(s);
						uint32_t one_g_in_raw = 1L<<(28-acc_fs);
						float scale = 1.f / one_g_in_raw;
						inv_icm20649_convert_dmp3_to_body(s, long_data, scale, accel_float);
						handler(context, INV_ICM20649_SENSOR_ACCELEROMETER, s->timestamp[INV_ICM20649_SENSOR_ACCELEROMETER], accel_float, &dummy_accuracy);
					}
				}
				/* 6axis AG orientation quaternion sample available from DMP FIFO */
				if (header & QUAT6_SET) {
					float ref_quat[4];
					/* Read 6 axis quaternion out of DMP FIFO in Q30 */
					inv_icm20649_dmp_get_6quaternion(long_quat);
					if(inv_icm20649_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_GAME_ROTATION_VECTOR) && !skip_sensor(s, ANDROID_SENSOR_GAME_ROTATION_VECTOR)) {
						/* and convert it from Q30 DMP format to Android format only if GRV sensor is enabled */
						inv_icm20649_convert_rotation_vector(s, long_quat, grv_float);
						ref_quat[0] = grv_float[3];
						ref_quat[1] = grv_float[0];
						ref_quat[2] = grv_float[1];
						ref_quat[3] = grv_float[2];
						handler(context, INV_ICM20649_SENSOR_GAME_ROTATION_VECTOR, s->timestamp[INV_ICM20649_SENSOR_GAME_ROTATION_VECTOR], ref_quat, 0);
					}
				}
			}
		} while(data_left_in_fifo);
	}
	else if((s->wake_on_motion_enabled)&& (int_read_back & BIT_WOM_INT) ){
		//We drop first event if requested
		if(s->wake_on_motion_drop_first_event == 0x01){
			s->wake_on_motion_drop_first_event = 0x00;
			return 0;
		}
		handler(context, INV_ICM20649_SENSOR_WOM, inv_icm20649_get_dataready_interrupt_time_us(), 0, 0);
	}
	return 0;
}