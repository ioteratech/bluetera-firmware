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
#include "Icm20649SelfTest.h"
 
#include "Icm20649Defs.h"
#include "Icm20649DataBaseDriver.h"

/* full scale and LPF setting */
#define SELFTEST_GYRO_FS                ((0 << 3) | 1)
#define SELFTEST_ACCEL_FS               ((7 << 3) | 1)

/* register settings */
#define SELFTEST_GYRO_SMPLRT_DIV        10
#define SELFTEST_GYRO_AVGCFG            3
#define SELFTEST_ACCEL_SMPLRT_DIV       10
#define SELFTEST_ACCEL_DEC3_CFG         2

/* wait time in ms between 2 data collection */
#define WAIT_TIME_BTW_2_SAMPLESREAD     10
/* wait time in ms after sensor self-test enabling for oscillations to stabilize */
#define DEF_ST_STABLE_TIME              20 //ms
/* number of times self test reading should be done until abord */
#define DEF_ST_TRY_TIMES                2
/* number of samples to be read to be averaged */
#define DEF_ST_SAMPLES                  200

#define LOWER_BOUND_CHECK(value) ((value)>>1) // value * 0.5
#define UPPER_BOUND_CHECK(value) ((value) + ((value)>>1) ) // value * 1.5

struct recover_regs {
	// Bank#0
	uint8_t fifo_cfg;           // REG_FIFO_CFG
	uint8_t user_ctrl;          // REG_USER_CTRL
	uint8_t lp_config;          // REG_LP_CONFIG
	uint8_t int_enable;         // REG_INT_ENABLE
	uint8_t int_enable_1;       // REG_INT_ENABLE_1
	uint8_t int_enable_2;       // REG_INT_ENABLE_2
	uint8_t fifo_en;            // REG_FIFO_EN
	uint8_t fifo_en_2;          // REG_FIFO_EN_2
	uint8_t fifo_rst;           // REG_FIFO_RST

	// Bank#2
	uint8_t gyro_smplrt_div;     // REG_GYRO_SMPLRT_DIV
	uint8_t gyro_config_1;       // REG_GYRO_CONFIG_1
	uint8_t gyro_config_2;       // REG_GYRO_CONFIG_2
	uint8_t accel_smplrt_div_1;  // REG_ACCEL_SMPLRT_DIV_1
	uint8_t accel_smplrt_div_2;  // REG_ACCEL_SMPLRT_DIV_2
	uint8_t accel_config;        // REG_ACCEL_CONFIG
	uint8_t accel_config_2;      // REG_ACCEL_CONFIG_2
};

// Table for list of results for factory self-test value equation
// st_otp = 1310/2^FS * 1.01^(st_value - 1)
// for gyro and accel FS = 0 so 1310 * 1.01^(st_value - 1)
// st_value = 1 => 1310
// st_value = 2 => 1310 * 1.01 = 1323
// etc../
static const uint16_t sSelfTestEquation[256] = {
	1310, 1323, 1336, 1349, 1363, 1376, 1390, 1404,
	1418, 1432, 1447, 1461, 1476, 1490, 1505, 1520,
	1536, 1551, 1566, 1582, 1598, 1614, 1630, 1646,
	1663, 1679, 1696, 1713, 1730, 1748, 1765, 1783,
	1801, 1819, 1837, 1855, 1874, 1893, 1911, 1931,
	1950, 1969, 1989, 2009, 2029, 2049, 2070, 2091,
	2112, 2133, 2154, 2176, 2197, 2219, 2241, 2264,
	2287, 2309, 2332, 2356, 2379, 2403, 2427, 2451,
	2476, 2501, 2526, 2551, 2577, 2602, 2628, 2655,
	2681, 2708, 2735, 2762, 2790, 2818, 2846, 2875,
	2903, 2932, 2962, 2991, 3021, 3052, 3082, 3113,
	3144, 3175, 3207, 3239, 3272, 3304, 3337, 3371,
	3405, 3439, 3473, 3508, 3543, 3578, 3614, 3650,
	3687, 3724, 3761, 3798, 3836, 3875, 3914, 3953,
	3992, 4032, 4072, 4113, 4154, 4196, 4238, 4280,
	4323, 4366, 4410, 4454, 4499, 4544, 4589, 4635,
	4681, 4728, 4775, 4823, 4871, 4920, 4969, 5019,
	5069, 5120, 5171, 5223, 5275, 5328, 5381, 5435,
	5489, 5544, 5600, 5656, 5712, 5769, 5827, 5885,
	5944, 6004, 6064, 6124, 6185, 6247, 6310, 6373,
	6437, 6501, 6566, 6632, 6698, 6765, 6833, 6901,
	6970, 7040, 7110, 7181, 7253, 7326, 7399, 7473,
	7548, 7623, 7699, 7776, 7854, 7933, 8012, 8092,
	8173, 8255, 8337, 8421, 8505, 8590, 8676, 8763,
	8850, 8939, 9028, 9118, 9210, 9302, 9395, 9489,
	9583, 9679, 9776, 9874, 9973, 10072, 10173, 10275,
	10378, 10481, 10586, 10692, 10799, 10907, 11016, 11126,
	11237, 11350, 11463, 11578, 11694, 11811, 11929, 12048,
	12169, 12290, 12413, 12537, 12663, 12789, 12917, 13046,
	13177, 13309, 13442, 13576, 13712, 13849, 13988, 14127,
	14269, 14411, 14556, 14701, 14848, 14997, 15147, 15298,
	15451, 15606, 15762, 15919, 16078, 16239, 16402, 16566
};

static int inv_save_setting(struct inv_icm20649 * s, struct recover_regs * saved_regs)
{
	int result = 0;

	result |= inv_icm20649_read_mems_reg(s, REG_FIFO_CFG, 1, &saved_regs->fifo_cfg);

	result |= inv_icm20649_read_mems_reg(s, REG_USER_CTRL, 1, &saved_regs->user_ctrl);

	result = inv_icm20649_read_mems_reg(s, REG_LP_CONFIG, 1, &saved_regs->lp_config);

	result |= inv_icm20649_read_mems_reg(s, REG_INT_ENABLE, 1, &saved_regs->int_enable);

	result |= inv_icm20649_read_mems_reg(s, REG_INT_ENABLE_1, 1, &saved_regs->int_enable_1);

	result |= inv_icm20649_read_mems_reg(s, REG_INT_ENABLE_2, 1, &saved_regs->int_enable_2);

	result |= inv_icm20649_read_mems_reg(s, REG_FIFO_EN, 1, &saved_regs->fifo_en);

	result |= inv_icm20649_read_mems_reg(s, REG_FIFO_EN_2, 1, &saved_regs->fifo_en_2);

	result |= inv_icm20649_read_mems_reg(s, REG_FIFO_RST, 1, &saved_regs->fifo_rst);

	result |= inv_icm20649_read_mems_reg(s, REG_GYRO_SMPLRT_DIV, 1, &saved_regs->gyro_smplrt_div);

	result |= inv_icm20649_read_mems_reg(s, REG_GYRO_CONFIG_1, 1, &saved_regs->gyro_config_1);

	result |= inv_icm20649_read_mems_reg(s, REG_GYRO_CONFIG_2, 1, &saved_regs->gyro_config_2);

	result |= inv_icm20649_read_mems_reg(s, REG_ACCEL_SMPLRT_DIV_1, 1, &saved_regs->accel_smplrt_div_1);

	result |= inv_icm20649_read_mems_reg(s, REG_ACCEL_SMPLRT_DIV_2, 1, &saved_regs->accel_smplrt_div_2);

	result |= inv_icm20649_read_mems_reg(s, REG_ACCEL_CONFIG, 1, &saved_regs->accel_config);

	result |= inv_icm20649_read_mems_reg(s, REG_ACCEL_CONFIG_2, 1, &saved_regs->accel_config_2);

	return result;
}

static int inv_recover_setting(struct inv_icm20649 * s, const struct recover_regs * saved_regs)
{
	int result = 0;

	// Stop sensors
	result |= inv_icm20649_write_single_mems_reg(s, REG_PWR_MGMT_2, 
			BIT_PWR_PRESSURE_STBY | BIT_PWR_ACCEL_STBY | BIT_PWR_GYRO_STBY);

	// Restore sensor configurations
	result |= inv_icm20649_write_single_mems_reg(s, REG_GYRO_SMPLRT_DIV, saved_regs->gyro_smplrt_div);

	result |= inv_icm20649_write_single_mems_reg(s, REG_GYRO_CONFIG_1, saved_regs->gyro_config_1);

	result |= inv_icm20649_write_single_mems_reg(s, REG_GYRO_CONFIG_2, saved_regs->gyro_config_2);

	result |= inv_icm20649_write_single_mems_reg(s, REG_ACCEL_SMPLRT_DIV_1, saved_regs->accel_smplrt_div_1);

	result |= inv_icm20649_write_single_mems_reg(s, REG_ACCEL_SMPLRT_DIV_2, saved_regs->accel_smplrt_div_2);

	result |= inv_icm20649_write_single_mems_reg(s, REG_ACCEL_CONFIG, saved_regs->accel_config);

	result |= inv_icm20649_write_single_mems_reg(s, REG_ACCEL_CONFIG_2, saved_regs->accel_config_2);

	// Restore FIFO configurations
	result |= inv_icm20649_write_single_mems_reg(s, REG_FIFO_CFG, saved_regs->fifo_cfg);

	result |= inv_icm20649_write_single_mems_reg(s, REG_LP_CONFIG, saved_regs->lp_config);

	result |= inv_icm20649_write_single_mems_reg(s, REG_INT_ENABLE, saved_regs->int_enable);

	result |= inv_icm20649_write_single_mems_reg(s, REG_INT_ENABLE_1, saved_regs->int_enable_1);

	result |= inv_icm20649_write_single_mems_reg(s, REG_FIFO_EN, saved_regs->fifo_en);

	result |= inv_icm20649_write_single_mems_reg(s, REG_FIFO_EN_2, saved_regs->fifo_en_2);

	result |= inv_icm20649_write_single_mems_reg(s, REG_FIFO_RST, MAX_5_BIT_VALUE);

	result |= inv_icm20649_write_single_mems_reg(s, REG_FIFO_RST, saved_regs->fifo_rst);

	// Reset DMP
	result |= inv_icm20649_write_single_mems_reg(s, REG_USER_CTRL, 
			(saved_regs->user_ctrl & (~BIT_FIFO_EN)) | BIT_DMP_RST);
	inv_icm20649_sleep_us(DMP_RESET_TIME * 1000);

	result |= inv_icm20649_set_dmp_address(s);
	// result |= inv_icm20649_set_secondary(s);
	// result |= inv_icm20649_setup_compass_akm(s);
	result |= inv_icm20649_sleep_mems(s);

	return result;
}

/**
*  @brief check accel or gyro self test
*  @param[in] sensorType type of sensor to be tested
*  @param[in] selfTestValuesReadFromReg self test written in register at production time.
*  @param[in] meanNormalTestValues average value of normal test.
*  @param[in] meanSelfTestValues   average value of self test
*  @return zero as success. A non-zero return value indicates failure in self test.
*/
static int inv_check_accelgyro_self_test(enum INV_SENSORS sensorType, uint8_t * selfTestValuesReadFromReg, int *meanNormalTestValues, int *meanSelfTestValues) 
{
	int ret_val;
	int lIsStOtpReadZero = 0;
	int l_st_otp_read[3], lDiffNormalStValues[3], i;

	ret_val = 0;

	// Calculate factory Self-Test value (ST_OTP) based on the following equation:
	// The factory Self-Test value (ST_OTP) is calculated from the ST_Code (the SELF_TEST values read)
	// using the following equation, where “FS” is the full scale value code:
	// st_otp = 1310/2^FS * 1.01^(st_value - 1)
	// the result of the equation is in sSelfTestEquation array
	for (i = 0; i < 3; i++) {
		if (selfTestValuesReadFromReg[i] != 0) {
			l_st_otp_read[i] = sSelfTestEquation[selfTestValuesReadFromReg[i] - 1];
		} else {
			l_st_otp_read[i] = 0;
			lIsStOtpReadZero = 1;
		}
	}

	// Calculate the Self-Test response as follows:
	// - GXST = GX_ST_OS - GX_OS
	// - GYST = GY_ST_OS - GY_OS
	// - GZST = GZ_ST_OS - GZ_OS
	// - AXST = AX_ST_OS - AX_OS
	// - AYST = AY_ST_OS - AY_OS
	// - AZST = AZ_ST_OS - AZ_OS
	for (i = 0; i < 3; i++) {
		lDiffNormalStValues[i] = meanSelfTestValues[i] - meanNormalTestValues[i];
		
		// Ensure the factory Self-Test values ST_OTP are not 0
		if (!lIsStOtpReadZero) {
			// Compare the current Self-Test response (GXST, GYST, GZST, AXST, AYST and AZST) to the factory Self-Test values (ST_OTP)
			// and report Self-Test is passing if all the following criteria are fulfilled:
			// (GXST / GXST_OTP)  > 0.5
			if (lDiffNormalStValues[i] < LOWER_BOUND_CHECK(l_st_otp_read[i]) )
				ret_val = 1;
			if (sensorType != INV_SENSOR_GYRO)
				// (AXST / AXST_OTP)  < 1.5
				if (lDiffNormalStValues[i] > UPPER_BOUND_CHECK(l_st_otp_read[i]) )
					ret_val = 1;
		} else
			ret_val = 1;
	}

	return ret_val;
}

static int inv_setup_selftest(struct inv_icm20649 * s, struct recover_regs * recover_regs)
{
	int result = 0;

	// reset static value
	memset(s->gyro_st_data, 0, sizeof(s->gyro_st_data));
	memset(s->accel_st_data, 0, sizeof(s->accel_st_data));
	
	// Wake up
	result |= inv_icm20649_write_single_mems_reg(s, REG_PWR_MGMT_1, BIT_CLK_PLL);

	// Save the current settings
	result |= inv_save_setting(s, recover_regs);

	// Stop sensors
	result |= inv_icm20649_write_single_mems_reg(s, REG_PWR_MGMT_2, BIT_PWR_ACCEL_STBY | BIT_PWR_GYRO_STBY);

	/* Perform a soft-reset of the chip by setting the MSB of PWR_MGMT_1 register
	 * This will clear any prior states in the chip
	 */
	result |= inv_icm20649_write_single_mems_reg(s, REG_PWR_MGMT_1, BIT_H_RESET);               
	inv_icm20649_sleep_us(100000); //100ms delay after soft reset--yd

	// Wake up
	result |= inv_icm20649_write_single_mems_reg(s, REG_PWR_MGMT_1, BIT_CLK_PLL);
	if (result)
		return result;

	// Set cycle mode
	result |= inv_icm20649_write_single_mems_reg(s, REG_LP_CONFIG, 
			BIT_I2C_MST_CYCLE | BIT_ACCEL_CYCLE | BIT_GYRO_CYCLE);

	// Configure FSR and DLPF for gyro
	result |= inv_icm20649_write_single_mems_reg(s, REG_GYRO_SMPLRT_DIV, SELFTEST_GYRO_SMPLRT_DIV);

	result |= inv_icm20649_write_single_mems_reg(s, REG_GYRO_CONFIG_1, SELFTEST_GYRO_FS);

	result |= inv_icm20649_write_single_mems_reg(s, REG_GYRO_CONFIG_2, SELFTEST_GYRO_AVGCFG);

	// Configure FSR and DLPF for accel
	result |= inv_icm20649_write_single_mems_reg(s, REG_ACCEL_SMPLRT_DIV_1, 0);

	result |= inv_icm20649_write_single_mems_reg(s, REG_ACCEL_SMPLRT_DIV_2, SELFTEST_ACCEL_SMPLRT_DIV);

	result |= inv_icm20649_write_single_mems_reg(s, REG_ACCEL_CONFIG, SELFTEST_ACCEL_FS);

	result |= inv_icm20649_write_single_mems_reg(s, REG_ACCEL_CONFIG_2, SELFTEST_ACCEL_DEC3_CFG);

	// Read selftest values
	// Retrieve factory Self-Test code (ST_Code) from SELF_TEST registers  (User Bank 1): 
	result |= inv_icm20649_read_mems_reg(s, REG_SELF_TEST1, 1, &s->gyro_st_data[0]);

	result |= inv_icm20649_read_mems_reg(s, REG_SELF_TEST2, 1, &s->gyro_st_data[1]);

	result |= inv_icm20649_read_mems_reg(s, REG_SELF_TEST3, 1, &s->gyro_st_data[2]);

	result |= inv_icm20649_read_mems_reg(s, REG_SELF_TEST4, 1, &s->accel_st_data[0]);

	result |= inv_icm20649_read_mems_reg(s, REG_SELF_TEST5, 1, &s->accel_st_data[1]);

	result |= inv_icm20649_read_mems_reg(s, REG_SELF_TEST6, 1, &s->accel_st_data[2]);

	// Restart sensors
	inv_icm20649_sleep_us(GYRO_ENGINE_UP_TIME * 1000);

	return result;
}

static int inv_selftest_read_samples(struct inv_icm20649 * self, enum INV_SENSORS type, int *sum_result, int *s)
{
	uint8_t w;
	int16_t vals[3];
	uint8_t d[BYTES_PER_SENSOR];
	int j;

	// Average 200 readings and save the averaged values as GX_OS, GY_OS, GZ_OS, AX_OS, AY_OS and AZ_OS. 
	// - GX_OS = Average (GYRO_XOUT_H | GYRO_XOUT_L)
	// - GY_OS = Average (GYRO_YOUT_H | GYRO_YOUT_L)
	// - GZ_OS = Average (GYRO_ZOUT_H | GYRO_ZOUT_L)
	// - AX_OS = Average (ACCEL_XOUT_H | ACCEL_XOUT_L)
	// - AY_OS = Average (ACCEL_YOUT_H | ACCEL_YOUT_L)
	// - AZ_OS = Average (ACCEL_ZOUT_H | ACCEL_ZOUT_L)

	if (INV_SENSOR_GYRO == type)
		w = REG_GYRO_XOUT_H_SH;
	else
		w = REG_ACCEL_XOUT_H_SH;

	while (*s < DEF_ST_SAMPLES) {
			if(inv_icm20649_read_mems_reg(self, w, BYTES_PER_SENSOR, d))
				return -1;
			
			for (j = 0; j < THREE_AXES; j++) {
				vals[j] = (d[(2*j)]<<8) | (d[(2*j)+ 1] & 0xff);
				sum_result[j] += vals[j];
			}
			
			(*s)++;
			
			inv_icm20649_sleep_us(WAIT_TIME_BTW_2_SAMPLESREAD * 1000);
	}

	return 0;
}

/*
*  inv_do_test_accelgyro() - do the actual test of self testing
*/
static int inv_do_test_accelgyro(struct inv_icm20649 * s, enum INV_SENSORS sensorType, int *meanValue, int *stMeanValue)
{
	int result, i, j;
	int lNbSamples = 0;

	// initialize output to be 0
	for (i = 0; i < THREE_AXES; i++) {
		meanValue[i] = 0;
		stMeanValue[i] = 0;
	}
	
	// read the accel/gyro output
	// the output values are 16 bits wide and in 2’s complement
	// Average 200 readings and save the averaged values
	result = inv_selftest_read_samples(s, sensorType, meanValue, &lNbSamples);
	if (result)
		return result;
	for (j = 0; j < THREE_AXES; j++) {
		meanValue[j] /= lNbSamples;
	}

	// Set Self-Test Bit
	if (sensorType == INV_SENSOR_GYRO)
	{
		// Enable gyroscope Self-Test by setting register User Bank 2, Register Address 02 (02h) Bit [5:3] to b111
		result = inv_icm20649_write_single_mems_reg(s, REG_GYRO_CONFIG_2, BIT_GYRO_CTEN | SELFTEST_GYRO_AVGCFG);
	} else
	{
		result = inv_icm20649_write_single_mems_reg(s, REG_ACCEL_CONFIG_2, BIT_ACCEL_CTEN | SELFTEST_ACCEL_DEC3_CFG);
	}
	if (result)
		return result;

	// Wait 20ms for oscillations to stabilize. 
	inv_icm20649_sleep_us(DEF_ST_STABLE_TIME * 1000);

	// Read the accel/gyro output and average 200 readings
	// These readings are in units of LSBs
	lNbSamples = 0; 
	result = inv_selftest_read_samples(s, sensorType, stMeanValue, &lNbSamples);
	if (result)
		return result;
	for (j = 0; j < THREE_AXES; j++) {
		stMeanValue[j] /= lNbSamples;
	}

	return 0;
}




int inv_icm20649_run_selftest(struct inv_icm20649 * s)
{
	int result;
	int gyro_bias_st[THREE_AXES], gyro_bias_regular[THREE_AXES];
	int accel_bias_st[THREE_AXES], accel_bias_regular[THREE_AXES];
	int test_times;
	char accel_result, gyro_result;
	struct recover_regs recover_regs;

	accel_result = 0;
	gyro_result = 0;

	// save original state of the chip, initialize registers, configure sensors and read ST values
	result = inv_setup_selftest(s, &recover_regs);
	if (result)
		goto test_fail;

	// perform self test for gyro
	test_times = DEF_ST_TRY_TIMES;
	while (test_times > 0) {
		result = inv_do_test_accelgyro(s, INV_SENSOR_GYRO, gyro_bias_regular, gyro_bias_st);
		if (result)
			test_times--;
		else
			break;
	}
	if (result)
		goto test_fail;

	// perform self test for accel
	test_times = DEF_ST_TRY_TIMES;
	while (test_times > 0) {
		result = inv_do_test_accelgyro(s, INV_SENSOR_ACCEL, accel_bias_regular, accel_bias_st);
		if (result)
			test_times--;
		else
			break;
	}
	if (result)
		goto test_fail;

	// check values read at various steps
	accel_result = !inv_check_accelgyro_self_test(INV_SENSOR_ACCEL, s->accel_st_data, accel_bias_regular, accel_bias_st);
	gyro_result = !inv_check_accelgyro_self_test(INV_SENSOR_GYRO, s->gyro_st_data, gyro_bias_regular, gyro_bias_st);

test_fail:
	// restore original state of the chips
	inv_recover_setting(s, &recover_regs);

	return (accel_result << 1) | gyro_result;
}
 /**
 * @}
 */
 
