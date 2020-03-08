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

#include "imu_manager.h"

#include <app_timer.h>
#include <nrfx_spim.h>
#include <nrfx_gpiote.h>
#include <nrf_log.h>
#include <nrf_delay.h>
#include <app_scheduler.h>
#include <app_util_platform.h>
#include <nrfx_timer.h>

#include "bluetera_boards.h"
#include "utils.h"
#include "bluetera_err.h"
#include "MadgwickAHRS.h"

#define BANK0				0
#define	BANK1				1
#define BANK2				2
#define BANK3				3
#define BANK4				4

#define REG_BANK_SEL		0x76

#define WHO_AM_I_VALUE		0x42

// bank 0
#define REG_DEVICE_CONFIG 			0x11
#define REG_INT_CONFIG				0x14
#define REG_FIFO_CONFIG				0x16
#define REG_ACCEL_DATA_X1			0x1F
#define REG_ACCEL_DATA_X0			0x20
#define REG_ACCEL_DATA_Y1			0x21
#define REG_ACCEL_DATA_Y0			0x22
#define REG_ACCEL_DATA_Z1			0x23
#define REG_ACCEL_DATA_Z0			0x24
#define REG_GYRO_DATA_X1			0x25
#define REG_GYRO_DATA_X0			0x26
#define REG_GYRO_DATA_Y1			0x27
#define REG_GYRO_DATA_Y0			0x28
#define REG_GYRO_DATA_Z1			0x29
#define REG_GYRO_DATA_Z0			0x2A
#define REG_INT_STATUS				0x2D
#define REG_PWR_MGMT0				0x4E
#define REG_GYRO_CONFIG0			0x4F
#define REG_ACCEL_CONFIG0			0x50
#define REG_GYRO_CONFIG1			0x51
#define REG_GYRO_ACCEL_CONFIG0		0x52
#define REG_INT_CONFIG0				0x63
#define REG_INT_CONFIG1				0x64
#define REG_INT_SOURCE0				0x65
#define REG_INT_SOURCE3				0x68
#define REG_WHO_AM_I				0x75

// bank 4
#define REG_APEX_CONFIG5			0x44

static nrfx_spim_t _spi = NRFX_SPIM_INSTANCE(ICM_SPI_INSTANCE);
static bltr_imu_data_handler_t _imu_data_handler;
static bool _data_ready;
static bool _calibrating;
static uint32_t _calibration_samples;
static float _gyro_bias[3];
static bltr_imu_config_t _config;

static void _on_pin_event_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
//static void _imu_irq_data_handler(const bltr_imu_sensor_data_t* data);
//static void _synced_irq_data_handler(void * p_event_data, uint16_t event_size);
static int _read_reg(uint8_t reg, uint8_t* data, uint32_t len);
static int _write_reg(uint8_t reg, const uint8_t* data, uint32_t len);
//static void _enter_critical_section();
//static void _leave_critical_section();

static uint32_t _sample_rate = 50;

static void _set_bank(uint8_t bank)
{
	uint8_t data = 0;
	_read_reg(REG_BANK_SEL, &data, 1);
	data = data & 0xF8;
	data = data | bank;
	_write_reg(REG_BANK_SEL, &data, 1);
}

static ret_code_t _init()
{
	uint8_t data;
	
	_set_bank(BANK0);

	// reset
	data = 0x01;
	_write_reg(REG_DEVICE_CONFIG, &data, 1);

	// after writing 1 to this bitfield, wait 1ms for soft reset to be effective, before
    // attempting any other register access

	nrf_delay_ms(1);

	// after reset bank0 is selected, so no need to select it again

	data = 0x00;
	_read_reg(REG_WHO_AM_I, &data, 1);

	if(data != WHO_AM_I_VALUE)
		return BLTR_IMU_NOT_FOUND;

	// setup gyro and acc

	_read_reg(REG_PWR_MGMT0, &data, 1);
	// clear relevant bits, leave reserved bits untouched
	data = data & 0xC0;
	// enable TEMP_DIS
	// disable IDLE
	// GYRO_MODE = low noise mode
	// ACCEL_MODE = low noise mode
	data = data | 0x2F;
	_write_reg(REG_PWR_MGMT0, &data, 1);

	nrf_delay_ms(100);

	_read_reg(REG_GYRO_CONFIG0, &data, 1);
	data = data & 0x10;
	// GYRO_FS_SEL[7:5] = 2000dps
	// GYRO_ODR[3:0] = 1kHz
	data = data | 0b00000110;
	_write_reg(REG_GYRO_CONFIG0, &data, 1);

	_read_reg(REG_ACCEL_CONFIG0, &data, 1);
	data = data & 0x10;
	// ACCEL_FS_SEL[7:5] = 16g
	// ACCEL_ODR[3:0] = 1kHz
	data = data | 0b00000110;
	_write_reg(REG_ACCEL_CONFIG0, &data, 1);

	_read_reg(REG_GYRO_CONFIG1, &data, 1);
	data = data & 0x10;
	// TEMP_FILT_BW[7:5] = DLPF BW = 5Hz; DLPF Latency = 32ms
	// GYRO_UI_FILT_ORD[3:2] = 1st order
	// GYRO_DEC2_M2_ORD[1:0] = 3rd order
	data = data | 0xc2;
	_write_reg(REG_GYRO_CONFIG1, &data, 1);

	_read_reg(REG_INT_CONFIG, &data, 1);
	data = data & 0b11000000;
	// INT2_MODE[5] = Pulsed mode
	// INT2_DRIVE_CIRCUIT[4] = Push pull
	// INT2_POLARITY[3] = Active high
	// INT1_MODE[2] = Pulsed mode
	// INT1_DRIVE_CIRCUIT[1] = Push pull
	// INT1_POLARITY[0] = Active high
	data = data | 0b00011011;
	_write_reg(REG_INT_CONFIG, &data, 1);

	_read_reg(REG_INT_CONFIG1, &data, 1);
	data = data & 0b10001111;
	// INT_TPULSE_DURATION[6] = Interrupt pulse duration is 100µs. Use only if ODR < 4kHz
	// INT_TDEASSERT_DISABLE[5] = The interrupt de-assertion duration is set to a minimum of 100µs. Use only if ODR < 4kHz.
	// INT_ASYNC_RESET[4] = 0 (User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation)
	data = data | 0b00000000;
	_write_reg(REG_INT_CONFIG1, &data, 1);

	_read_reg(REG_INT_SOURCE0, &data, 1);
	data = data & 0b10000000;
	// UI_FSYNC_INT1_EN[6] = UI FSYNC interrupt not routed to INT1
	// PLL_RDY_INT1_EN[5] = PLL ready interrupt not routed to INT1
	// RESET_DONE_INT1_EN[4] = Reset done interrupt not routed to INT1
	// ------------> UI_DRDY_INT1_EN[3] = UI data ready interrupt routed to INT1
	// FIFO_THS_INT1_EN[2] = FIFO threshold interrupt not routed to INT1
	// FIFO_FULL_INT1_EN[1] = FIFO full interrupt not routed to INT1
	// UI_AGC_RDY_INT1_EN[0] = UI AGC ready interrupt not routed to INT1
	data = data | 0b00001000;
	_write_reg(REG_INT_SOURCE0, &data, 1);

	_read_reg(REG_INT_SOURCE3, &data, 1);
	data = data & 0b10000000;
	// UI_FSYNC_INT2_EN[6] = UI FSYNC interrupt not routed to INT2
	// PLL_RDY_INT2_EN[5] = PLL ready interrupt not routed to INT2
	// RESET_DONE_INT2_EN[4] = Reset done interrupt not routed to INT2
	// UI_DRDY_INT2_EN[3] = UI data ready interrupt not routed to INT2
	// FIFO_THS_INT2_EN[2] = FIFO threshold interrupt not routed to INT2
	// FIFO_FULL_INT2_EN[1] = FIFO full interrupt not routed to INT2
	// ------------> UI_AGC_RDY_INT2_EN[0] = UI AGC ready interrupt routed to INT2
	data = data | 0b00000001;
	_write_reg(REG_INT_SOURCE3, &data, 1);

	_set_bank(BANK4);

	_read_reg(REG_APEX_CONFIG5, &data, 1);
	data = data & 0b11111000;
	// MOUNTING_MATRIX[2:0] = [ 1 0 0; 0 1 0; 0 0 1]
	data = data | 0b00000000;
	_write_reg(REG_APEX_CONFIG5, &data, 1);

	_set_bank(BANK0);

	return BLTR_SUCCESS;
}

ret_code_t bltr_imu_init(const bltr_imu_init_t* init)
{
	APP_ERROR_CHECK_BOOL(init != NULL);

	_imu_data_handler = init->imu_data_handler;

	// Initialize SPI
	nrfx_spim_config_t spi_cfg =
	{
		.sck_pin = ICM_SPI_SCK_PIN,
		.mosi_pin = ICM_SPI_MOSI_PIN,
		.miso_pin = ICM_SPI_MISO_PIN,
		.ss_pin = ICM_SPI_CS_PIN,
		.ss_active_high = false,
		.irq_priority = NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY,
		.orc = 0xFF,
		.frequency = NRF_SPIM_FREQ_2M,
		.mode = NRF_SPIM_MODE_3,
		.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST,
		NRFX_SPIM_DEFAULT_EXTENDED_CONFIG        
	};

	APP_ERROR_CHECK(nrfx_spim_init(&_spi, &spi_cfg, NULL, NULL));

	// Initialize interrupts
	nrfx_gpiote_in_config_t icm_int1 = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	nrfx_gpiote_in_init(ICM_INT1_PIN, &icm_int1, _on_pin_event_handler);
	nrfx_gpiote_in_event_enable(ICM_INT1_PIN, true);
	nrfx_gpiote_in_config_t icm_int2 = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	nrfx_gpiote_in_init(ICM_INT2_PIN, &icm_int2, _on_pin_event_handler);

	_data_ready = false;
	_calibrating = true;

	// TODO: move this out of here!
	nrfx_gpiote_out_clear(LED_RED_PIN);

	_calibration_samples = 0;
	_gyro_bias[0] = 0.0f;
	_gyro_bias[1] = 0.0f;
	_gyro_bias[2] = 0.0f;

	memset(&_config, 0, sizeof(_config));

	return _init();
}

ret_code_t bltr_imu_handle_uplink_message(const bluetera_uplink_message_t* message)
{
	if(message->which_payload != BLUETERA_UPLINK_MESSAGE_IMU_TAG)
		return BLTR_SUCCESS;

	ret_code_t err = BLTR_SUCCESS;
	const bluetera_imu_command_t* cmd = (const bluetera_imu_command_t*)&message->payload.imu;
	switch(cmd->which_payload)
	{		
		case BLUETERA_IMU_COMMAND_START_TAG:
			{
				bltr_imu_config_t config = 
				{
					.data_types = cmd->payload.start.data_types,
					.odr = cmd->payload.start.odr,
					.acc_fsr = cmd->payload.start.acc_fsr,
					.gyro_fsr = cmd->payload.start.gyro_fsr
				};

				_sample_rate = (int)((1.0f / cmd->payload.start.odr) * 1000.0f);

				err = bltr_imu_start(&config);
			}

			break;

		case BLUETERA_IMU_COMMAND_STOP_TAG:
			err = bltr_imu_stop();						
			break;

		default:
			err = BLTR_IMU_ERROR_INVALID_COMMAND;
			break;
	}

	return err;
}

ret_code_t bltr_imu_start(const bltr_imu_config_t* config)
{
	_config = *config;

	return BLTR_SUCCESS;
}

ret_code_t bltr_imu_stop()
{
	return BLTR_SUCCESS;
}

#define PI 3.14159265359f

void bltr_imu_poll()
{
	static int counter = 0;

	if(_data_ready)
	{
		uint8_t buf[12];
		_read_reg(REG_ACCEL_DATA_X1, buf, 12);	

		uint64_t timestamp = bltr_utils_get_timestamp();

		float acc_fsr = (16.0f / 32768.0f);
		float ax = (int16_t)((buf[0] << 8) | buf[1]) * acc_fsr;
		float ay = (int16_t)((buf[2] << 8) | buf[3]) * acc_fsr;
		float az = (int16_t)((buf[4] << 8) | buf[5]) * acc_fsr;

		float gyro_fsr = (2000.0f / 32768.0f);
		float gx = (int16_t)((buf[6] << 8) | buf[7]) * gyro_fsr;
		float gy = (int16_t)((buf[8] << 8) | buf[9]) * gyro_fsr;
		float gz = (int16_t)((buf[10] << 8) | buf[11]) * gyro_fsr;

		if(_calibrating)
		{
			if(_calibration_samples < 1000)
			{
				_gyro_bias[0] += gx;
				_gyro_bias[1] += gy;
				_gyro_bias[2] += gz;
			}
			else
			{
				_gyro_bias[0] /= _calibration_samples;
				_gyro_bias[1] /= _calibration_samples;
				_gyro_bias[2] /= _calibration_samples;
				_calibrating = false;

				// TODO: move this out of here!
				nrfx_gpiote_out_set(LED_RED_PIN);

				NRF_LOG_INFO("gyro cal: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(_gyro_bias[0]));
				NRF_LOG_INFO(NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(_gyro_bias[1]));
				NRF_LOG_INFO(NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(_gyro_bias[2]));
			}

			_calibration_samples++;
		}
		else
		{
			gx -= _gyro_bias[0];
			gy -= _gyro_bias[1];
			gz -= _gyro_bias[2];
			MadgwickAHRSupdateIMU(gx * (PI / 180.0f), gy * (PI / 180.0f), gz * (PI / 180.0f), ax * (PI / 180.0f), ay * (PI / 180.0f), az * (PI / 180.0f));

			if(counter % _sample_rate == 0)
			{
				// bltr_imu_sensor_data_t sensor_data;
				// sensor_data.sensor = BLTR_IMU_SENSOR_TYPE_RAW;
				// sensor_data.timestamp = timestamp;		
				// sensor_data.raw.acceleration[0] = (buf[0] << 8) | buf[1];
				// sensor_data.raw.acceleration[1] = (buf[2] << 8) | buf[3];
				// sensor_data.raw.acceleration[2] = (buf[4] << 8) | buf[5];
				// sensor_data.raw.gyroscope[0] = (buf[6] << 8) | buf[7];
				// sensor_data.raw.gyroscope[1] = (buf[8] << 8) | buf[9];
				// sensor_data.raw.gyroscope[2] = (buf[10] << 8) | buf[11];

				if(_config.data_types & BLTR_IMU_DATA_TYPE_QUATERNION)
				{
					bltr_imu_sensor_data_t sensor_data;
					sensor_data.sensor = BLTR_IMU_SENSOR_TYPE_ROTATION_VECTOR;
					sensor_data.timestamp = timestamp;		
					sensor_data.quaternion[0] = q0;
					sensor_data.quaternion[1] = q1;
					sensor_data.quaternion[2] = q2;
					sensor_data.quaternion[3] = q3;
				
					_imu_data_handler(&sensor_data);
				}
				
				if(_config.data_types & BLTR_IMU_DATA_TYPE_ACCELEROMETER)
				{
					bltr_imu_sensor_data_t sensor_data;
					sensor_data.sensor = BLTR_IMU_SENSOR_TYPE_ACCELEROMETER;
					sensor_data.timestamp = timestamp;		
					sensor_data.acceleration[0] = ax;
					sensor_data.acceleration[1] = ay;
					sensor_data.acceleration[2] = az;
				
					_imu_data_handler(&sensor_data);
				}

				if(_config.data_types & BLTR_IMU_DATA_TYPE_GYROSCOPE)
				{
					bltr_imu_sensor_data_t sensor_data;
					sensor_data.sensor = BLTR_IMU_SENSOR_TYPE_GYROSCOPE;
					sensor_data.timestamp = timestamp;		
					sensor_data.gyroscope[0] = gx;
					sensor_data.gyroscope[1] = gy;
					sensor_data.gyroscope[2] = gz;
				
					_imu_data_handler(&sensor_data);
				}
			}

			counter++;
		}

		_data_ready = false;
	}
}

static void _on_pin_event_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	if(pin == ICM_INT1_PIN)
	{
		//nrfx_gpiote_out_set(DEBUG_GPIO_TIMING);
	
		// read data
		uint8_t int_status = 0;
		
		//_enter_critical_section();

		_read_reg(REG_INT_STATUS, &int_status, 1);

		if(int_status & 0x08)	// DATA_RDY_INT
			_data_ready = true;

		//_leave_critical_section();

		//nrfx_gpiote_out_clear(DEBUG_GPIO_TIMING);
	}
	else if (pin == ICM_INT2_PIN)
	{
		
	}
}

static int _read_reg(uint8_t reg, uint8_t* data, uint32_t len)
{
	uint8_t tx[64] = { reg | 0x80 };
	uint8_t rx[64] = { 0 };

	nrfx_spim_xfer_desc_t xfer =
	{
		.p_tx_buffer = tx,
		.tx_length = len + 1,
		.p_rx_buffer = rx,
		.rx_length= len + 1,
	};

	ret_code_t err = nrfx_spim_xfer(&_spi, &xfer, 0);

	memcpy(data, rx + 1, len);

	if(err != NRF_SUCCESS)
		NRF_LOG_INFO("READ ERROR");
	
	if (err == NRF_SUCCESS)
		return 0;
	else
		return -1;
}

static int _write_reg(uint8_t reg, const uint8_t* data, uint32_t len)
{
	uint8_t tx[64] = { reg & 0x7F };
	memcpy(tx + 1, data, len);
	uint8_t rx[64] = { 0 };

	nrfx_spim_xfer_desc_t xfer =
	{
		.p_tx_buffer = tx,
		.tx_length = len + 1,
		.p_rx_buffer = rx,
		.rx_length= len + 1,
	};

	ret_code_t err = nrfx_spim_xfer(&_spi, &xfer, 0);
	
	if(err != NRF_SUCCESS)
		NRF_LOG_INFO("WRITE ERROR");

	if (err == NRF_SUCCESS)
		return 0;
	else
		return -1;
}

// static void _enter_critical_section()
// {
// 	nrfx_gpiote_in_event_disable(ICM_INT1_PIN);
// 	nrfx_gpiote_in_event_disable(ICM_INT2_PIN);
// }

// static void _leave_critical_section()
// {
// 	nrfx_gpiote_in_event_enable(ICM_INT1_PIN, true);
// 	nrfx_gpiote_in_event_enable(ICM_INT2_PIN, true);
// }
