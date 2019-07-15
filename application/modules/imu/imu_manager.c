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
#include "bltr_invn_wrapper.h"

static nrfx_spim_t _spi = NRFX_SPIM_INSTANCE(ICM_SPI_INSTANCE);
static bltr_imu_data_handler_t _imu_data_handler;

static void _on_pin_event_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
//static void _imu_irq_data_handler(const bltr_imu_sensor_data_t* data);
//static void _synced_irq_data_handler(void * p_event_data, uint16_t event_size);
static int _read_reg(void* context, uint8_t reg, uint8_t* data, uint32_t len);
static int _write_reg(void* context, uint8_t reg, const uint8_t* data, uint32_t len);
static void _delay_ms(int ms);
static void _delay_us(int us);
static uint64_t _get_timestamp_us();
static void _enter_critical_section();
static void _leave_critical_section();

void bltr_imu_init(const bltr_imu_init_t* init)
{
	APP_ERROR_CHECK_BOOL(init != NULL);
	NRF_LOG_DEBUG("ICM20649: initializing");

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
	nrfx_gpiote_in_init(ICM_INTERRUPT_PIN, &icm_int1, _on_pin_event_handler);

	// Initialize Invensense
	_imu_data_handler = init->imu_data_handler;
	bltr_invn_init_t invn_init = 
	{
		.imu_data_handler = init->imu_data_handler,
		.read_reg = _read_reg,
		.write_reg = _write_reg,
		.delay_ms = _delay_ms,
		.delay_us = _delay_us,
		.get_timestamp_us = _get_timestamp_us,
		.enter_critical_section = _enter_critical_section,
		.leave_critical_section = _leave_critical_section,
		.spi = &_spi
	};
	bltr_invn_init(&invn_init);
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
	return bltr_invn_start(config);
}

ret_code_t bltr_imu_stop()
{
	return bltr_invn_stop();
}

void bltr_imu_poll()
{
	bltr_invn_poll();
}

static void _on_pin_event_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	if(pin == ICM_INTERRUPT_PIN)
		bltr_invn_irq_handler();
}

// static void _imu_irq_data_handler(const bltr_imu_sensor_data_t* data)
// {
// 	app_sched_event_put(&data, sizeof(data), _synced_irq_data_handler);
// }

// static void _synced_irq_data_handler(void* p_event_data, uint16_t event_size)
// {
// 	bltr_imu_sensor_data_t* sensor_data = (bltr_imu_sensor_data_t*)p_event_data;
// 	_imu_data_handler(sensor_data);
// }

static int _read_reg(void* context, uint8_t reg, uint8_t* data, uint32_t len)
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

	ret_code_t err = nrfx_spim_xfer((nrfx_spim_t*)context, &xfer, 0);

	memcpy(data, rx + 1, len);

	if(err != NRF_SUCCESS)
		NRF_LOG_INFO("READ ERROR");
	
	if (err == NRF_SUCCESS)
		return 0;
	else
		return -1;
}

static int _write_reg(void* context, uint8_t reg, const uint8_t* data, uint32_t len)
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

	ret_code_t err = nrfx_spim_xfer((nrfx_spim_t*)context, &xfer, 0);
	
	if(err != NRF_SUCCESS)
		NRF_LOG_INFO("WRITE ERROR");

	if (err == NRF_SUCCESS)
		return 0;
	else
		return -1;
}

static void _delay_ms(int ms)
{
	nrf_delay_ms(ms);
}

static void _delay_us(int us)
{
	nrf_delay_us(us);
}

static uint64_t _get_timestamp_us()
{
	return bltr_utils_get_timestamp();
}

static void _enter_critical_section()
{
	nrfx_gpiote_in_event_disable(ICM_INTERRUPT_PIN);
}

static void _leave_critical_section()
{
	nrfx_gpiote_in_event_enable(ICM_INTERRUPT_PIN, true);
}
