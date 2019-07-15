#include "bltr_invn_wrapper.h"
#include <Invn/Devices/SensorTypes.h>
#include <Invn/Devices/DeviceIcm20649.h>
#include <Invn/Devices/Drivers/Icm20649/Icm20649.h>
#include <stdbool.h>

#define EXCLUDE_BLUETERA_HEADERS
#include "bluetera_err.h"

#define ICM_REG_REG_BANK_SEL       		0x7F

// bank 0
#define ICM_REG_WHO_AM_I           		0x00
#define ICM_REG_PWR_MGMT_1        		0x06
#define ICM_REG_PWR_MGMT_2         		0x07
#define ICM_REG_INT_PIN_CFG        		0x0F
#define ICM_REG_INT_ENABLE_1       		0x11
#define ICM_REG_GYRO_XOUT_H        		0x33
#define ICM_REG_ACCEL_XOUT_H       		0x2D

// bank 2
#define ICM_REG_GYRO_SMPLRT_DIV    		0x00
#define ICM_REG_GYRO_CONFIG_1      		0x01
#define ICM_REG_ACCEL_CONFIG       		0x14
#define ICM_REG_ACCEL_SMPLRT_DIV_1		0x10
#define ICM_REG_ACCEL_SMPLRT_DIV_2		0x11


#define BANK0 (0 << 4)
#define BANK1 (1 << 4)
#define BANK2 (2 << 4)
#define BANK3 (3 << 4)

typedef enum {
    GYRO_FSR_500DPS = 0,
    GYRO_FSR_1000DPS,
    GYRO_FSR_2000DPS,
    GYRO_FSR_4000DPS,
	GYRO_FSR_INVALID
} GyroFullscaleRange;

typedef enum  {
    ACC_FSR_4G = 0,
    ACC_FSR_8G,
    ACC_FSR_16G,
    ACC_FSR_30G,
	ACC_FSR_INVALID
} AccFullscaleRange;

static const uint8_t dmp3_image[] = {
	#include <Invn/Images/icm20649_img.dmp3a.h>
};

static const float _icm_mounting_matrix[9] =
{
	-1.0f, 0.0f, 0.0f,
	0.0f, 1.0f, 0.0f,
	0.0f, 0.0f, -1.0f
};

static inv_device_t* _device;
static inv_device_icm20649_t _device_icm20649 = { 0 };
static bltr_imu_data_handler_t _imu_data_handler;
static bltr_imu_data_handler_t _imu_irq_data_handler;
static int(*_read_reg)(void* spi, uint8_t reg, uint8_t* data, uint32_t len);
static int (*_write_reg)(void* spi, uint8_t reg, const uint8_t* data, uint32_t len);
static void(*_delay_ms_func)(int ms);
static void(*_delay_us_func)(int us);
static uint64_t(*_get_timestamp_us)(void);
static void (*_enter_critical_section)(void);
static void (*_leave_critical_section)(void);
static void* _spi;

static volatile bool _irq_from_device = false;
static inv_sensor_listener_t _sensor_listener = { 0 };
static uint8_t _freq_div;

static bltr_imu_config_t _current_config;

void _init_direct(uint8_t div, uint8_t acc_fsr, uint8_t gyro_fsr);
static void _init_dmp();
static void _set_fsr_imu_mode_direct();
static void _uninit_dmp();
static void _inv_sensor_listener_event_cb(const inv_sensor_event_t * event, void * context);
static uint16_t _gyro_fsr_enum_to_num(GyroFullscaleRange fsr);
static GyroFullscaleRange _num_to_gyro_fsr_enum(uint16_t fsr);
static uint16_t _acc_fsr_enum_to_num(AccFullscaleRange fsr);
static AccFullscaleRange _num_to_acc_fsr_enum(uint16_t fsr);

// methods
uint32_t bltr_invn_init(const bltr_invn_init_t* init)
{
	_freq_div = 0;
	_imu_data_handler = init->imu_data_handler;
	_imu_irq_data_handler = init->imu_irq_data_handler;
	_read_reg = init->read_reg;
	_write_reg = init->write_reg;
	_delay_ms_func = init->delay_ms;
	_delay_us_func = init->delay_us;
	_get_timestamp_us = init->get_timestamp_us;
	_enter_critical_section = init->enter_critical_section;
	_leave_critical_section = init->leave_critical_section;
	_spi = init->spi;

	_sensor_listener.event_cb = _inv_sensor_listener_event_cb;
	_sensor_listener.context = _spi;

	_init_dmp();

	// TODO(tomer) verify that "init" contains all what we need?
	// TODO(tomer) verify that IMU is physically connected, and return an error based on that

	return 0;
}

uint32_t bltr_invn_start(const bltr_imu_config_t* config)
{
	// verify fsr's

	AccFullscaleRange real_acc_fsr = _num_to_acc_fsr_enum(config->acc_fsr);

	if(real_acc_fsr == ACC_FSR_INVALID)
		return BLTR_IMU_ERROR_INVALID_CONFIG;

	GyroFullscaleRange real_gyro_fsr = _num_to_gyro_fsr_enum(config->gyro_fsr);

	if(real_gyro_fsr == GYRO_FSR_INVALID)
		return BLTR_IMU_ERROR_INVALID_CONFIG;

	if(config->odr < 1 || config->odr > 200)
		return BLTR_IMU_ERROR_INVALID_CONFIG;

	// TODO(tomer) verify "config.odr"
	// TODO(tomer) check for allowed combinations of "config.data_types"

	_current_config = *config;

	// TODO(tomer) implement the rest of the data types, and select the correct operation mode (dmp or direct) according to different combinations and rates

	// NOTICE INV_ICM20649_SENSOR_RAW_ACCELEROMETER sets for both raw and regular
	inv_device_set_sensor_config(_device, INV_ICM20649_SENSOR_RAW_ACCELEROMETER, INV_DEVICE_ICM20649_CONFIG_FSR, &_current_config.acc_fsr, 0);

	// NOTICE INV_ICM20649_SENSOR_RAW_GYROSCOPE sets for both raw and regular	
	inv_device_set_sensor_config(_device, INV_ICM20649_SENSOR_RAW_GYROSCOPE, INV_DEVICE_ICM20649_CONFIG_FSR, &_current_config.gyro_fsr, 0);	

	// because of a bug in the invensense driver library, we have to enable both quaternion and accelerometer togheter, 
	// otherwise it will ignore our ODR (sample rate) settings
	if(_current_config.data_types & BLTR_IMU_DATA_TYPE_QUATERNION || _current_config.data_types & BLTR_IMU_DATA_TYPE_ACCELEROMETER)
	{
		inv_device_set_sensor_period(_device, INV_SENSOR_TYPE_GAME_ROTATION_VECTOR, 1000 / _current_config.odr);
		inv_device_set_sensor_period(_device, INV_SENSOR_TYPE_ACCELEROMETER, 1000 / _current_config.odr);
		
		inv_device_start_sensor(_device, INV_SENSOR_TYPE_GAME_ROTATION_VECTOR);
		inv_device_start_sensor(_device, INV_SENSOR_TYPE_ACCELEROMETER);
	}

	return BLTR_SUCCESS;
}

uint32_t bltr_invn_stop()
{
	if(_current_config.data_types & BLTR_IMU_DATA_TYPE_QUATERNION || _current_config.data_types & BLTR_IMU_DATA_TYPE_ACCELEROMETER)
	{
		inv_device_stop_sensor(_device, INV_SENSOR_TYPE_GAME_ROTATION_VECTOR);
		inv_device_stop_sensor(_device, INV_SENSOR_TYPE_ACCELEROMETER);
	}

	return BLTR_IMU_ERROR_CRITICAL;
}

void bltr_invn_poll()
{
	_irq_from_device = false;
	inv_device_poll(_device);
	
	// no need to pull in direct mode, because data reading takes place in interrupt, and then transfered to the application queue,
	// which is then deqeued and exceuted from the main thead
}

void bltr_invn_irq_handler()
{
	_irq_from_device = true;

	// uint64_t timestamp = inv_icm20649_get_time_us();
	
	// // read data
	// uint8_t buf[12];
	// _enter_critical_section();	
	// _read_reg(_spi, 0x1A, buf, 1);

	// if(buf[0] & 0x01)	// RAW_DATA_0_RDY_INT
	// 	_read_reg(_spi, ICM_REG_ACCEL_XOUT_H, buf, 12);	

	// _leave_critical_section();

	// // callback
	// if(buf[0] & 0x01)
	// {		
	// 	bltr_imu_sensor_data_t sensor_data;
	// 	sensor_data.sensor = BLTR_IMU_SENSOR_TYPE_RAW;
	// 	sensor_data.timestamp = timestamp;		
	// 	sensor_data.raw.acceleration[0] = (buf[0] << 8) | buf[1];
	// 	sensor_data.raw.acceleration[1] = (buf[2] << 8) | buf[3];
	// 	sensor_data.raw.acceleration[2] = (buf[4] << 8) | buf[5];
	// 	sensor_data.raw.gyroscope[0] = (buf[6] << 8) | buf[7];
	// 	sensor_data.raw.gyroscope[1] = (buf[8] << 8) | buf[9];
	// 	sensor_data.raw.gyroscope[2] = (buf[10] << 8) | buf[11];

	// 	_imu_data_handler(&sensor_data);
	// }
}

// static methods
void _init_direct(uint8_t div, uint8_t acc_fsr, uint8_t gyro_fsr)
{
	_enter_critical_section();

	// select bank 0
	uint8_t data = BANK0;
	_write_reg(_spi, ICM_REG_REG_BANK_SEL, &data, 1);
	_delay_ms_func(15);

	data = 0x80; // reset bit
	_write_reg(_spi, ICM_REG_PWR_MGMT_1, &data, 1);

	_delay_ms_func(150);

	// no need to select bank again because the default after reset is BANK0

	uint8_t whoami = 0xff;
	_read_reg(_spi, ICM_REG_WHO_AM_I, &whoami, 1);

	if(whoami != 0xE1)
		return;

	data = 0x01; // Auto selects the best available clock source – PLL if ready, else use the Internal oscillator 6 Internal 20 MHz oscillator
	_write_reg(_spi, ICM_REG_PWR_MGMT_1, &data, 1);
	_delay_ms_func(15);

	// select bank 2
	data = BANK2;
	_write_reg(_spi, ICM_REG_REG_BANK_SEL, &data, 1);
	_delay_ms_func(15);

	 // If hardware_lpf is either GYRO_HARDWARE_LPF_NORMAL or GYRO_HARDWARE_LPF_EXPERIMENTAL then the
    // gyro is running in 9KHz sample mode and GYRO_FCHOICE should be 0, otherwise we're in 1.1KHz sample
    // mode and GYRO_FCHOICE = 1.  When in 1.1KHz mode select the 196.6Hz DLPF (GYRO_DLPFCFG = 0)
    // Unfortunately we can't configure any difference in DLPF based on NORMAL vs. EXPERIMENTAL because
    // the ICM20649 only has a single 9KHz DLPF cutoff.

	data = 0x01; // FCHOICE = 1; Enable gyro DLPF.
    data |= gyro_fsr << 1;
	_write_reg(_spi, ICM_REG_GYRO_CONFIG_1, &data, 1);
	_delay_ms_func(15);

	data = div; // GYRO_SMPLRT_DIV = 0; 1.1KHz/(1+GYRO_SMPLRT_DIV)
	_write_reg(_spi, ICM_REG_GYRO_SMPLRT_DIV, &data, 1);
	_delay_ms_func(100);

	data = 0x01; // FCHOICE = 1; Enable accel DLPF.
    data |= acc_fsr << 1;
	_write_reg(_spi, ICM_REG_ACCEL_CONFIG, &data, 1);
	_delay_ms_func(15);

	// ACCEL_SMPLRT_DIV = 0; 1.1KHz/(1+ACCEL_SMPLRT_DIV)

	data = 0; 
	_write_reg(_spi, ICM_REG_ACCEL_SMPLRT_DIV_1, &data, 1);
	_delay_ms_func(100);

	data = div;
	_write_reg(_spi, ICM_REG_ACCEL_SMPLRT_DIV_2, &data, 1);
	_delay_ms_func(100);

	// select bank 0
	data = BANK0;
	_write_reg(_spi, ICM_REG_REG_BANK_SEL, &data, 1);
	_delay_ms_func(15);

	data = 0x10; // INT_ANYRD_2CLEAR = 1;  Interrupt status in INT_STATUS is cleared (set to 0) if any read operation is performed.
	_write_reg(_spi, ICM_REG_INT_PIN_CFG, &data, 1);
	_delay_ms_func(15);

	data = 0x01; // RAW_DATA_0_RDY_EN = 1; Enable raw data ready interrupt from any sensor to propagate to interrupt pin 1.
	_write_reg(_spi, ICM_REG_INT_ENABLE_1, &data, 1);

	_leave_critical_section();
}

// Make sure to disable ICM interrupts before calling this method
static void _init_dmp()
{
	inv_serif_hal_t serif_spi_instance =
	{
		.read_reg = _read_reg,
		.write_reg = _write_reg,
		.context = _spi,
		.max_read_transaction_size = 1024 * 32,
		.max_write_transaction_size = 1024 * 32,
		.serif_type = INV_SERIF_HAL_TYPE_SPI
	};

	_enter_critical_section();

	inv_device_icm20649_init2(&_device_icm20649, &serif_spi_instance, &_sensor_listener, dmp3_image, sizeof(dmp3_image));
	_device = inv_device_icm20649_get_base(&_device_icm20649);

	inv_device_setup(_device);
	inv_device_load(_device, 0, dmp3_image, sizeof(dmp3_image), true, false);

	// calling this should apply the same mouting matrix for all sensors
	inv_device_set_sensor_mounting_matrix(_device, INV_ICM20649_SENSOR_RAW_GYROSCOPE, _icm_mounting_matrix);

	uint8_t is_high_power = 1;
	inv_device_set_sensor_config(_device, 0, INV_DEVICE_ICM20649_CONFIG_POWER_MODE, &is_high_power, 0);

	_leave_critical_section();
}

static void _uninit_dmp()
{
	_enter_critical_section();

	inv_device_cleanup(_device);
	icm20649_instance = 0; // TODO for some reason the dmp does not set it to 0 so reinit later causes assert, might need to verify further if this is ok
	memset(&_device_icm20649, 0, sizeof(_device_icm20649));

	_leave_critical_section();
}

static void _inv_sensor_listener_event_cb(const inv_sensor_event_t* event, void* context)
{
	bltr_imu_sensor_data_t sensor_data;
	sensor_data.timestamp = event->timestamp;
	bool send_data = false;

	if (event->status == INV_SENSOR_STATUS_DATA_UPDATED)
	{
		switch (INV_SENSOR_ID_TO_TYPE(event->sensor))
		{
			case INV_SENSOR_TYPE_ACCELEROMETER:
				if(_current_config.data_types & BLTR_IMU_DATA_TYPE_ACCELEROMETER)
				{
					sensor_data.sensor = BLTR_IMU_SENSOR_TYPE_ACCELEROMETER;				
					memcpy(sensor_data.acceleration, event->data.acc.vect, sizeof( event->data.acc.vect));
					send_data = true;
				}
				break;

			case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
			case INV_SENSOR_TYPE_ROTATION_VECTOR:
				if(_current_config.data_types & BLTR_IMU_DATA_TYPE_QUATERNION)
				{
					sensor_data.sensor = BLTR_IMU_SENSOR_TYPE_ROTATION_VECTOR;
					memcpy(sensor_data.quaternion, event->data.quaternion.quat, sizeof(event->data.quaternion.quat));
					send_data = true;
				}
				break;

			case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
			case INV_SENSOR_TYPE_RAW_GYROSCOPE:
			default:
				return;
		}
	}

	if(send_data)
		_imu_data_handler(&sensor_data);
}

static uint16_t _gyro_fsr_enum_to_num(GyroFullscaleRange fsr)
{
	switch(fsr)
	{
		case GYRO_FSR_500DPS:
			return 500;
		case GYRO_FSR_1000DPS:
			return 1000;
		case GYRO_FSR_2000DPS:
			return 2000;
		case GYRO_FSR_4000DPS:
			return 4000;
	}

	return 0;
}

static GyroFullscaleRange _num_to_gyro_fsr_enum(uint16_t fsr)
{
	switch(fsr)
	{
		case 500:
			return GYRO_FSR_500DPS;
		case 1000:
			return GYRO_FSR_1000DPS;
		case 2000:
			return GYRO_FSR_2000DPS;
		case 4000:
			return GYRO_FSR_4000DPS;
	}

	return GYRO_FSR_INVALID;
}

static uint16_t _acc_fsr_enum_to_num(AccFullscaleRange fsr)
{
	switch(fsr)
	{
		case ACC_FSR_4G:
			return 4;
		case ACC_FSR_8G:
			return 8;
		case ACC_FSR_16G:
			return 16;
		case ACC_FSR_30G:
			return 30;
	}

	return 0;
}

static AccFullscaleRange _num_to_acc_fsr_enum(uint16_t fsr)
{
	switch(fsr)
	{
		case 4:
			return ACC_FSR_4G;
		case 8:
			return ACC_FSR_8G;
		case 16:
			return ACC_FSR_16G;
		case 30:
			return ACC_FSR_30G;
	}

	return ACC_FSR_INVALID;
}

// The following methods are used in Icm20649.c and must be implemented by native code. See Icm20649.h
void inv_icm20649_sleep_us(int us)
{
	_delay_us_func(us);
}

uint64_t inv_icm20649_get_time_us()
{
	return _get_timestamp_us();
}

uint64_t inv_icm20649_get_dataready_interrupt_time_us()
{
	return _get_timestamp_us();
}

