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

#include "Icm20649.h"
#include "Icm20649DataBaseControl.h"

#include "Icm20649Dmp3Driver.h"

#include <string.h>

static int inv_enable_sensor_internal(struct inv_icm20649 * s, unsigned char androidSensor, unsigned char enable, char * mems_put_to_sleep);
static int inv_set_hw_smplrt_dmp_odrs(struct inv_icm20649 * s);
static void inv_reGenerate_sensorControl(struct inv_icm20649 * s, const short *sen_num_2_ctrl, unsigned short *sensor_control, uint8_t header2_count);

unsigned long inv_icm20649_ctrl_androidSensor_enabled(struct inv_icm20649 * s, unsigned char androidSensor)
{
	return s->inv_androidSensorsOn_mask & (1L << (androidSensor&0x1F));
}

typedef	struct {
	enum ANDROID_SENSORS AndroidSensor;
	enum INV_SENSORS     InvSensor;
}	MinDelayGenElementT;

#define MinDelayGen(s, list) MinDelayGenActual(s, list, sizeof(list) / sizeof (MinDelayGenElementT))

static unsigned short MinDelayGenActual(struct inv_icm20649 *s, const MinDelayGenElementT *element, unsigned long elementQuan)
{
	unsigned short minDelay = (unsigned short) -1;

	while(elementQuan--) {
		if (inv_icm20649_ctrl_androidSensor_enabled(s, element->AndroidSensor)) {
			unsigned short odrDelay = s->inv_dmp_odr_delays[element->InvSensor];

			if (minDelay > odrDelay)
					minDelay = odrDelay;
		}
		element++;
	} // end while elements to process

	return	minDelay;
}

static int DividerRateSet(struct inv_icm20649 *s, unsigned short minDelay, unsigned short hwSampleRateDivider, enum INV_SENSORS InvSensor)
{
	int result = 0;
	
	if (minDelay != 0xFFFF) {
		unsigned short dmpOdrDivider = (minDelay * 1125L) / (hwSampleRateDivider * 1000L); // a divider from (1125Hz/hw_smplrt_divider).

		s->inv_dmp_odr_dividers[InvSensor] = hwSampleRateDivider * dmpOdrDivider;
		result |= dmp_set_sensor_rate_20649(s, InvSensor, (dmpOdrDivider - 1));
	}
	
	return result;
}

static unsigned short SampleRateDividerGet(unsigned short minDelay)
{
	unsigned short delay = min(INV_ODR_MIN_DELAY, minDelay); // because of GYRO_SMPLRT_DIV which relies on 8 bits, we can't have ODR value higher than 200ms
	return delay * 1125L / 1000L; // a divider from 1125Hz.
}



/** @brief Get minimum ODR to be applied to accel engine based on all accel-based enabled sensors.
* @return ODR in ms we expect to be applied to accel engine
*/
static unsigned short getMinDlyAccel(struct inv_icm20649 *s)
{
	const MinDelayGenElementT MinDelayGenAccelList[] ={
		{ANDROID_SENSOR_ACCELEROMETER,                      INV_SENSOR_ACCEL                },
		{ANDROID_SENSOR_RAW_ACCELEROMETER,                  INV_SENSOR_ACCEL                },
		{ANDROID_SENSOR_GAME_ROTATION_VECTOR,               INV_SENSOR_SIXQ_accel           }
	};
	
	if(inv_icm20649_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_ACCELEROMETER))
		if(inv_icm20649_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_ACCELEROMETER))
			s->inv_dmp_odr_delays[INV_SENSOR_ACCEL] = min(s->odr_acc_ms,s->odr_racc_ms);
		else
			s->inv_dmp_odr_delays[INV_SENSOR_ACCEL] = s->odr_acc_ms;
	else
		if(inv_icm20649_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_ACCELEROMETER))
			s->inv_dmp_odr_delays[INV_SENSOR_ACCEL] = s->odr_racc_ms;
		else
			if(inv_icm20649_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_WOM))
				return s->odr_acc_wom_ms;
	return MinDelayGen(s, MinDelayGenAccelList);
}

/** @brief Get minimum ODR to be applied to gyro engine based on all gyro-based enabled sensors.
* @return ODR in ms we expect to be applied to gyro engine
*/
static unsigned short getMinDlyGyro(struct inv_icm20649 *s)
{
	const MinDelayGenElementT MinDelayGenGyroList[] = {
		{ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED,        INV_SENSOR_GYRO              },
		{ANDROID_SENSOR_GYROSCOPE,                     INV_SENSOR_CALIB_GYRO        },
		{ANDROID_SENSOR_RAW_GYROSCOPE,                 INV_SENSOR_GYRO              },
		{ANDROID_SENSOR_GAME_ROTATION_VECTOR,          INV_SENSOR_SIXQ              },
	};

	if(inv_icm20649_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED))
		if(inv_icm20649_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_GYROSCOPE))
			s->inv_dmp_odr_delays[INV_SENSOR_GYRO] = min(s->odr_gyr_ms,s->odr_rgyr_ms);
		else
			s->inv_dmp_odr_delays[INV_SENSOR_GYRO] = s->odr_gyr_ms;
	else
		if(inv_icm20649_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_GYROSCOPE))
			s->inv_dmp_odr_delays[INV_SENSOR_GYRO] = s->odr_rgyr_ms;

	return MinDelayGen(s, MinDelayGenGyroList);
}

int inv_icm20649_base_control_init(struct inv_icm20649 * s)
{
	int result = 0;
	unsigned int i;

	memset(s->inv_dmp_odr_dividers, 0, sizeof(s->inv_dmp_odr_dividers));
	
	for(i = 0; i < (sizeof(s->inv_dmp_odr_delays)/sizeof(unsigned short)); i++) {
			s->inv_dmp_odr_delays[i] = INV_ODR_MIN_DELAY_OUT;
	}
	s->lLastHwSmplrtDividerAcc = 0;
	s->lLastHwSmplrtDividerGyr = 0;
	s->sBatchMode              = 0;
	s->header2_count           = 0;
	s->mems_put_to_sleep       = 1;

	s->odr_acc_ms = INV_ODR_MIN_DELAY_OUT;
	s->odr_acc_wom_ms = INV_ODR_MIN_DELAY_OUT;
	s->odr_racc_ms = INV_ODR_MIN_DELAY_OUT;
	s->odr_gyr_ms = INV_ODR_MIN_DELAY_OUT;
	s->odr_rgyr_ms = INV_ODR_MIN_DELAY_OUT;
	
	return result;
}

static int inv_set_hw_smplrt_dmp_odrs(struct inv_icm20649 * s)
{
	int result = 0;
	unsigned short minDly, minDly_accel, minDly_gyro;
	unsigned short hw_smplrt_divider = 0;
	
	const MinDelayGenElementT MinDelayGenAccel2List[] = {
		{ANDROID_SENSOR_ACCELEROMETER,                      INV_SENSOR_ACCEL                },
		{ANDROID_SENSOR_RAW_ACCELEROMETER,                  INV_SENSOR_ACCEL                },
	};
	const MinDelayGenElementT MinDelayGenGyro2List[] = {
		{ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED,             INV_SENSOR_GYRO                 },
		{ANDROID_SENSOR_GYROSCOPE,                          INV_SENSOR_CALIB_GYRO           },
		{ANDROID_SENSOR_RAW_GYROSCOPE,                      INV_SENSOR_GYRO           },
	};
	const MinDelayGenElementT MinDelayGenGyro3List[] = {
		{ANDROID_SENSOR_GYROSCOPE,                          INV_SENSOR_CALIB_GYRO           },
	};
	const MinDelayGenElementT MinDelayGenGyro4List[] = {
		{ANDROID_SENSOR_GAME_ROTATION_VECTOR,               INV_SENSOR_SIXQ                 },
	};

	// Engine ACCEL Based
	minDly_accel = getMinDlyAccel(s);

	// Engine Gyro Based
	minDly_gyro  = getMinDlyGyro(s);

	// get min delay of all enabled sensors of all sensor engine groups
	minDly = min(minDly_gyro, minDly_accel);

	// switch between low power and low noise at 500Hz boundary
	if (minDly != 0xFFFF) {
		// above 500Hz boundary, force LN mode
		if (minDly==1) {
			if (s->base_state.chip_lp_ln_mode == CHIP_LOW_POWER) {
				s->go_back_lp_when_odr_low = 1;
				inv_icm20649_enter_low_noise_mode(s);
			}
		} else { // below 500 Hz boundary, go back to originally requested mode
			if (s->go_back_lp_when_odr_low) {
				s->go_back_lp_when_odr_low = 0;
				inv_icm20649_enter_duty_cycle_mode(s);
			}	
		}
	} else // all sensors are turned OFF, force originally requested mode
	{
		if (s->go_back_lp_when_odr_low) {
			s->go_back_lp_when_odr_low = 0;
			inv_icm20649_enter_duty_cycle_mode(s);
		}
	}
	if (minDly_accel != 0xFFFF)    minDly_accel = minDly;
	if (minDly_gyro  != 0xFFFF)    minDly_gyro  = minDly;

	// set odrs for each enabled sensors

	// Engine ACCEL Based
	if (minDly_accel != 0xFFFF)	{ // 0xFFFF -- none accel based sensor enable
		hw_smplrt_divider = SampleRateDividerGet(minDly_accel);

		if (hw_smplrt_divider != s->lLastHwSmplrtDividerAcc) {
			
			result |= inv_icm20649_ctrl_set_accel_quaternion_gain(s, hw_smplrt_divider);
			result |= inv_icm20649_set_accel_divider(s, hw_smplrt_divider);
			s->lLastHwSmplrtDividerAcc = hw_smplrt_divider;
		}

		result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenAccel2List), hw_smplrt_divider, INV_SENSOR_ACCEL);
	}

	// Engine Gyro Based
	if (minDly_gyro != 0xFFFF) { // 0xFFFF -- none gyro based sensor enable
		hw_smplrt_divider = SampleRateDividerGet(minDly_gyro);

		if (hw_smplrt_divider != s->lLastHwSmplrtDividerGyr) {
			result |= inv_icm20649_set_gyro_divider(s, (unsigned char)(hw_smplrt_divider));
			s->lLastHwSmplrtDividerGyr = hw_smplrt_divider;
		}

		result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenGyro2List), hw_smplrt_divider, INV_SENSOR_GYRO);
		result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenGyro3List), hw_smplrt_divider, INV_SENSOR_CALIB_GYRO);
		result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenGyro4List), hw_smplrt_divider, INV_SENSOR_SIXQ);
	}

	return result;
}

int inv_icm20649_set_odr(struct inv_icm20649 * s, unsigned char androidSensor, unsigned short delayInMs)
{
	int result;

	inv_icm20649_prevent_lpen_control(s);

	// Avoid LP/LN silent switch, max frequency is 562.5Hz
	if (delayInMs < 2) delayInMs = 2; //ms
	if (delayInMs > INV_ODR_MIN_DELAY_OUT) delayInMs = INV_ODR_MIN_DELAY_OUT;
	
	switch (androidSensor) {
		case ANDROID_SENSOR_ACCELEROMETER:
			if(inv_icm20649_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_ACCELEROMETER))
				s->inv_dmp_odr_delays[INV_SENSOR_ACCEL] = min(delayInMs,s->odr_racc_ms);
			else
				s->inv_dmp_odr_delays[INV_SENSOR_ACCEL] = delayInMs;
			s->odr_acc_ms = delayInMs;
			break;
		case ANDROID_SENSOR_RAW_ACCELEROMETER:
			if(inv_icm20649_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_ACCELEROMETER))
				s->inv_dmp_odr_delays[INV_SENSOR_ACCEL] = min(delayInMs,s->odr_acc_ms);
			else
				s->inv_dmp_odr_delays[INV_SENSOR_ACCEL] = delayInMs;
			s->odr_racc_ms = delayInMs;
			break;
		case ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED:
			if(inv_icm20649_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_GYROSCOPE))
				s->inv_dmp_odr_delays[INV_SENSOR_GYRO] = min(delayInMs,s->odr_rgyr_ms);
			else
				s->inv_dmp_odr_delays[INV_SENSOR_GYRO] = delayInMs;
			s->odr_gyr_ms = delayInMs;
			break;
		case ANDROID_SENSOR_RAW_GYROSCOPE:
			if(inv_icm20649_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED))
				s->inv_dmp_odr_delays[INV_SENSOR_GYRO] = min(delayInMs,s->odr_gyr_ms);
			else
				s->inv_dmp_odr_delays[INV_SENSOR_GYRO] = delayInMs;
			s->odr_rgyr_ms = delayInMs;
			break;
		case ANDROID_SENSOR_GYROSCOPE:
			s->inv_dmp_odr_delays[INV_SENSOR_CALIB_GYRO] = delayInMs;
			break;
		case ANDROID_SENSOR_GAME_ROTATION_VECTOR:
			s->inv_dmp_odr_delays[INV_SENSOR_SIXQ] = delayInMs;
			s->inv_dmp_odr_delays[INV_SENSOR_SIXQ_accel] = delayInMs;
			break;
		case ANDROID_SENSOR_WOM:
			s->odr_acc_wom_ms = INV_ODR_MIN_DELAY_OUT;
			break;
		default:
			break;
	}

	result = inv_set_hw_smplrt_dmp_odrs(s);
	result |= inv_icm20649_set_gyro_sf(s, inv_icm20649_get_gyro_divider(s), inv_icm20649_get_gyro_fullscale(s));

	// debug get odr
	// result should be SAME as you entered in Ms in the Rolldice console
	// i.e. If you use: O a 63 [ Press capital O then 'a' then 63 then ENTER]
	// You should get the nearest number to 63 here if you debug  the 'test_odr'  

	//inv_icm20649_ctrl_get_odr( androidSensor, &test_odr );

	inv_icm20649_allow_lpen_control(s);
	return result;
}

/*
   inv_icm20649_ctrl_get_odr(s)
   Function to Query DMP3 DataRate (ODR)
   
   *odr = inv_icm20649_get_odr_in_units( );

    The result in odr_units saved in *odr param
*/
int inv_icm20649_ctrl_get_odr(struct inv_icm20649 * s, unsigned char SensorId, uint32_t *odr, enum INV_ODR_TYPE odr_units)
{
	int result=0;

	if(!odr) // sanity
		return -1;

	*odr = 0;

	/*
	You can obtain the odr in Milliseconds, Micro Seconds or Ticks.
	Use the enum values: ODR_IN_Ms, ODR_IN_Us or ODR_IN_Ticks,
	when calling inv_icm20649_get_odr_in_units().
	*/

	switch (SensorId) {
		case ANDROID_SENSOR_ACCELEROMETER:
		case ANDROID_SENSOR_RAW_ACCELEROMETER:
			*odr = inv_icm20649_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_ACCEL] , odr_units );
			break;

		case ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED:
		case ANDROID_SENSOR_RAW_GYROSCOPE:
			*odr = inv_icm20649_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_GYRO] , odr_units );
			break;

		case ANDROID_SENSOR_GYROSCOPE:
			*odr = inv_icm20649_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_CALIB_GYRO] , odr_units );
			break;

		case ANDROID_SENSOR_GAME_ROTATION_VECTOR:
			*odr = inv_icm20649_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_SIXQ] , odr_units );
			break;

		default:
			*odr=0;
	}

	return result;
}

static void inv_reGenerate_sensorControl(struct inv_icm20649 * s, const short *sen_num_2_ctrl, unsigned short *sensor_control, uint8_t header2_count)
{
	short delta;
	int cntr;
	unsigned long tmp_androidSensorsOn_mask;

	//check if only header2 still remaining
	if(header2_count)
		*sensor_control = HEADER2_SET;
	else
		*sensor_control = 0;
	cntr = 0;
	tmp_androidSensorsOn_mask = s->inv_androidSensorsOn_mask;
	while (tmp_androidSensorsOn_mask) {
		if (tmp_androidSensorsOn_mask & 1) {
			delta = sen_num_2_ctrl[cntr];
			if (delta != -1) *sensor_control |= delta;
		}
		tmp_androidSensorsOn_mask >>= 1;
		cntr++;
	}
}

/** Computes the sensor control register that needs to be sent to the DMP
* @param[in] androidSensor A sensor number, the numbers correspond to sensors.h definition in Android
* @param[in] enable non-zero to turn sensor on, 0 to turn sensor off
* @param[in] sen_num_2_ctrl Table matching android sensor number to bits in DMP control register
* @param[in,out] sensor_control Sensor control register to write to DMP to enable/disable sensors
*/
static void inv_convert_androidSensor_to_control(struct inv_icm20649 * s, unsigned char androidSensor, unsigned char enable, const short *sen_num_2_ctrl, unsigned short *sensor_control)
{
	short delta = 0;

	if (androidSensor >= ANDROID_SENSOR_NUM_MAX)
		return; // Sensor not supported

	delta = sen_num_2_ctrl[androidSensor];
	if (delta == -1)
		return; // This sensor not supported

	if (enable) {
		s->inv_androidSensorsOn_mask |= 1L << (androidSensor & 0x1F); // Set bit
		*sensor_control |= delta;
	}
	else {
		s->inv_androidSensorsOn_mask &= ~(1L << (androidSensor & 0x1F)); // Clear bit
		// control has to be regenerated when removing sensors because of overlap
		inv_reGenerate_sensorControl(s, sen_num_2_ctrl, sensor_control, s->header2_count);
	}

	return;
}

int inv_icm20649_ctrl_enable_sensor(struct inv_icm20649 * s, unsigned char androidSensor, unsigned char enable)
{
	int result = 0;

	inv_icm20649_prevent_lpen_control(s);
	if( s->mems_put_to_sleep ) {
		s->mems_put_to_sleep = 0;
		result |= inv_icm20649_wakeup_mems(s);
	}
	result |= inv_enable_sensor_internal(s, androidSensor, enable, &s->mems_put_to_sleep);
	inv_icm20649_allow_lpen_control(s);
	return result;
}

static int inv_enable_sensor_internal(struct inv_icm20649 * s, unsigned char androidSensor, unsigned char enable, char * mems_put_to_sleep)
{
	int result = 0;
	unsigned short inv_event_control = 0;
	unsigned short data_rdy_status = 0;
	const short inv_androidSensor_to_control_bits[ANDROID_SENSOR_NUM_MAX]=
	{
		// Unsupported Sensors are -1
		-1, // Meta Data
		0x8008, // Accelerometer
		0x4048, // Gyroscope
		0x0808, // Game Rotation Vector
		0x4008, // Gyroscope Uncalibrated
		0x8008, // Raw Acc
		0x4048, // Raw Gyr
		0,// WOM
	};
	
	if(enable && !inv_icm20649_ctrl_androidSensor_enabled(s, androidSensor))
		s->skip_sample[inv_icm20649_sensor_android_2_sensor_type(androidSensor)] = 1;
	
	inv_convert_androidSensor_to_control(s, androidSensor, enable, inv_androidSensor_to_control_bits, &s->inv_sensor_control);
	result = dmp_set_data_output_control1_20649(s, s->inv_sensor_control);
	result |= dmp_set_data_interrupt_control_20649(s, s->inv_sensor_control);

	if (s->inv_sensor_control & ACCEL_SET)
		s->inv_sensor_control2 |= ACCEL_ACCURACY_SET;
	else
		s->inv_sensor_control2 &= ~ACCEL_ACCURACY_SET;

	if ((s->inv_sensor_control & GYRO_CALIBR_SET) || (s->inv_sensor_control & GYRO_SET))
		s->inv_sensor_control2 |= GYRO_ACCURACY_SET;
	else
		s->inv_sensor_control2 &= ~GYRO_ACCURACY_SET;

	result |= dmp_set_data_output_control2_20649(s, s->inv_sensor_control2);

	// sets DATA_RDY_STATUS in DMP based on which sensors are on
	if (s->inv_androidSensorsOn_mask & INV_NEEDS_GYRO_MASK)
		data_rdy_status |= GYRO_AVAILABLE;
	
	if (s->inv_androidSensorsOn_mask & INV_NEEDS_ACCEL_MASK)
		data_rdy_status |= ACCEL_AVAILABLE;

	// turn on gyro cal only if gyro is available
	if (data_rdy_status & GYRO_AVAILABLE)
		inv_event_control |= INV_GYRO_CAL_EN;

	result |= dmp_set_motion_event_control_20649(s, inv_event_control);
	
	result |= inv_set_hw_smplrt_dmp_odrs(s);
	result |= inv_icm20649_set_gyro_sf(s, inv_icm20649_get_gyro_divider(s), inv_icm20649_get_gyro_fullscale(s));

	if (!s->inv_sensor_control) {
		*mems_put_to_sleep =1 ;
		result |= inv_icm20649_sleep_mems(s);
	}

	// DMP no longer controls PWR_MGMT_2 because of hardware bug, 0x80 set to override default behaviour of inv_icm20649_enable_hw_sensors()
	result |= inv_icm20649_enable_hw_sensors(s, (int)data_rdy_status | 0x80);

	result |= dmp_set_data_rdy_status_20649(s, data_rdy_status);
	
	return result;
}

int inv_icm20649_ctrl_enable_batch(struct inv_icm20649 * s, unsigned char enable)
{
	int ret = 0;

	if(enable)
		s->inv_sensor_control2 |= BATCH_MODE_EN;
	else
		s->inv_sensor_control2 &= ~BATCH_MODE_EN;

	ret = dmp_set_data_output_control2_20649(s, s->inv_sensor_control2);

	/* give batch mode status to mems transport layer 
	to allow disable/enable LP_EN when reading FIFO in batch mode */
	inv_icm20649_ctrl_set_batch_mode_status(s, enable);

	return ret;
}

void inv_icm20649_ctrl_set_batch_mode_status(struct inv_icm20649 * s, unsigned char enable)
{
	if(enable)
		s->sBatchMode=1;
	else
		s->sBatchMode=0;
}

unsigned char inv_icm20649_ctrl_get_batch_mode_status(struct inv_icm20649 * s)
{
	return s->sBatchMode;
}

int inv_icm20649_ctrl_set_batch_timeout(struct inv_icm20649 * s, unsigned short batch_time_in_seconds)
{
	unsigned int timeout = 0;

	if(    s->inv_sensor_control & GYRO_CALIBR_SET 
		|| s->inv_sensor_control & QUAT6_SET 
		|| s->inv_sensor_control & GYRO_SET ) { // If Gyro based sensor is enabled.
		timeout = (unsigned int) (batch_time_in_seconds * (BASE_SAMPLE_RATE/ (inv_icm20649_get_gyro_divider(s) + 1)));
		return dmp_set_batchmode_params_20649(s, timeout, GYRO_AVAILABLE);
	}

	if(    s->inv_sensor_control & ACCEL_SET ) { // If Accel is enabled and no Gyro based sensor is enabled.
		timeout = (unsigned int) (batch_time_in_seconds * (BASE_SAMPLE_RATE/ (inv_icm20649_get_accel_divider(s) + 1)));
		return dmp_set_batchmode_params_20649(s, timeout, ACCEL_AVAILABLE);
	}

	return -1;  // Call batch only when a sensor is enabled.
}    

int inv_icm20649_ctrl_set_batch_timeout_ms(struct inv_icm20649 * s, unsigned short batch_time_in_ms)
{
	unsigned int timeout = 0;

	if(    s->inv_sensor_control & GYRO_CALIBR_SET 
		|| s->inv_sensor_control & QUAT6_SET 
		|| s->inv_sensor_control & GYRO_SET ) { // If Gyro based sensor is enabled.
		timeout = (unsigned int) ((batch_time_in_ms * (BASE_SAMPLE_RATE/ (inv_icm20649_get_gyro_divider(s) + 1)))/1000);
		return dmp_set_batchmode_params_20649(s, timeout, GYRO_AVAILABLE);
	}

	if(    s->inv_sensor_control & ACCEL_SET ) { // If Accel is enabled and no Gyro based sensor is enabled.
		timeout = (unsigned int) ((batch_time_in_ms * (BASE_SAMPLE_RATE/ (inv_icm20649_get_accel_divider(s) + 1)))/1000);
		return dmp_set_batchmode_params_20649(s, timeout, ACCEL_AVAILABLE);
	}

	return -1; // Call batch only when a sensor is enabled.
}

/** Each bit corresponds to a sensor being on (Sensors 0 to 21)
*/
unsigned long *inv_icm20649_ctrl_get_androidSensorsOn_mask(struct inv_icm20649 * s)
{
	return &s->inv_androidSensorsOn_mask;
}

/** @brief Sets accel quaternion gain according to accel engine rate.
* @param[in] hw_smplrt_divider  hardware sample rate divider such that accel engine rate = 1125Hz/hw_smplrt_divider
* @return 0 in case of success, -1 for any error
*/
int inv_icm20649_ctrl_set_accel_quaternion_gain(struct inv_icm20649 * s, unsigned short hw_smplrt_divider)
{
	int accel_gain = 15252014L; //set 225Hz gain as default

	switch (hw_smplrt_divider) {
		case 5: //1125Hz/5 = 225Hz
			accel_gain = 15252014L;
			break;
		case 10: //1125Hz/10 = 112Hz
			accel_gain = 30504029L;
			break;
		case 11: //1125Hz/11 = 102Hz
			accel_gain = 33554432L;
			break;
		case 22: //1125Hz/22 = 51Hz
			accel_gain = 67108864L;
			break;
		default:
			accel_gain = 15252014L;
			break;
	}

	return dmp_set_accel_feedback_gain_20649(s, accel_gain);
}