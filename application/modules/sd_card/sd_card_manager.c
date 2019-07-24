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

#include "bluetera_config.h"

#include <string.h>
#include <stdlib.h>
#include <nrf_block_dev_sdc.h>
#include <nrf_delay.h>
#include <nrfx_gpiote.h>
#include <app_timer.h>
#include <app_scheduler.h>
#include <ff.h>
#include <diskio_blkdev.h>

#include "bluetera_err.h"
#include "sd_card_manager.h"
#include "bluetera_boards.h"
#include "utils.h"

#if BLTR_SD_CARD_ENABLED
#define DISK_INIT_RETRIES	3
#define DEBOUCE_INTERVAL 	APP_TIMER_TICKS(500)
#define LOG_DIR				"/"

// configure logging
#define NRF_LOG_MODULE_NAME bltr_sd_card
#if BLTR_SD_CARD_CONFIG_LOG_ENABLED
    #define NRF_LOG_LEVEL       BLTR_SD_CARD_CONFIG_LOG_LEVEL
	#define NRF_LOG_INFO_COLOR  BLTR_SD_CARD_CONFIG_INFO_COLOR
	#define NRF_LOG_DEBUG_COLOR BLTR_SD_CARD_CONFIG_DEBUG_COLOR
#else
    #define NRF_LOG_LEVEL       0
#endif
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_strerror.h"

// instantiate block device descriptor and FATFS drive
NRF_BLOCK_DEV_SDC_DEFINE(
        _block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_SPI_MOSI_PIN, SDC_SPI_MISO_PIN, SDC_SPI_SCK_PIN, SDC_SPI_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);

static diskio_blkdev_t _drives[] =
{
		DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(_block_dev_sdc, block_dev), NULL)
};

// instantiate pin debounce timer
APP_TIMER_DEF(_debounce_timer);

// static fields
static char _log_filename[] = "LOG";
static FATFS _filesystem;
static FIL _file;
static bltr_sd_card_status_handler_t _sd_card_status_handler = NULL;

// module state
static bool _is_card_present;
static bool _is_module_initialized;
static bool _is_file_open;
static bool _should_log_imu_data;
static uint64_t _last_flush;

// static methods
static bool _try_init_module();
static void _deinit_module();
static bool _try_get_last_log_index(uint16_t* index);
static void _flush();
static void _debounce_handler(void* context);
static void _synced_debounce_handler(void* p_event_data, uint16_t event_size);
static void _on_pin_event_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);


// methods
void bltr_sd_card_init(const bltr_sd_card_init_t* init)
{
	_sd_card_status_handler = init->sd_card_status_handler;	
	_is_module_initialized = false;
	_is_file_open = false;	
	_should_log_imu_data = false;
	_last_flush = 0;

	diskio_blockdev_register(_drives, ARRAY_SIZE(_drives));

#if BLTR_SD_CARD_DETECT_CARD_ENABLED
	uint32_t err_code = app_timer_create(&_debounce_timer, APP_TIMER_MODE_SINGLE_SHOT, _debounce_handler);
	APP_ERROR_CHECK(err_code);

	nrfx_gpiote_in_config_t sdcd_int = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
	err_code = nrfx_gpiote_in_init(SDC_CD_PIN, &sdcd_int, _on_pin_event_handler);
	APP_ERROR_CHECK(err_code);

	nrfx_gpiote_in_event_enable(SDC_CD_PIN, true);
	_is_card_present = nrfx_gpiote_in_is_set(SDC_CD_PIN);
#else
	_is_card_present = true;
#endif		
}

ret_code_t bltr_sd_card_handle_uplink_message(const bluetera_uplink_message_t* message)
{
	// IMU start command may define the SD as its data sink - if so, we create a new file for the data
	if(message->which_payload != BLUETERA_UPLINK_MESSAGE_IMU_TAG)
		return BLTR_SUCCESS;
	
	const bluetera_imu_command_t* cmd = (const bluetera_imu_command_t*)&message->payload.imu;
	ret_code_t err = BLTR_SUCCESS;

	switch(cmd->which_payload)
	{
		case BLUETERA_IMU_COMMAND_START_TAG:
		{
			_should_log_imu_data = (cmd->payload.start.data_sink == BLUETERA_DATA_SINK_TYPE_DATA_SINK_TYPE_SDCARD);

			if(_should_log_imu_data)
			{
				NRF_LOG_INFO("bltr_sd_card_handle_uplink_message() - configuring SD card as IMU sink");
				err = bltr_sd_card_open_log();
			}

			break;
		}
		case BLUETERA_IMU_COMMAND_STOP_TAG:
		{
			if(_is_file_open)
				err = bltr_sd_card_close_log();

			break;
		}
		default:
			err = BLTR_SUCCESS;
			break;
	}

	return err;
}

ret_code_t bltr_sd_card_handle_imu_sensor_data(const bltr_imu_sensor_data_t* data) 
{
	if(!_should_log_imu_data)
		return BLTR_SUCCESS;

	// to downlink message
	ret_code_t err;
	bluetera_downlink_message_t message;
	err = bltr_msg_imu_sensor_data_to_downlink_message(data, &message);
	BLTR_RETURN_CODE_IF_ERROR(err);

	// encode 
	uint16_t bytes_written = 0;
	uint8_t buffer[BLTR_MAX_DOWNLINK_MESSAGE_SIZE];
	err = bltr_msg_encode_downlink_message(&message, buffer, &bytes_written);
	BLTR_RETURN_CODE_IF_ERROR(err);

	// write
	err = bltr_sd_card_write_log(buffer, bytes_written);

	if(err != BLTR_SUCCESS)
	{
		if(_is_file_open)
			_deinit_module();
		
		return err;
	}

	return BLTR_SUCCESS;
}

ret_code_t bltr_sd_card_open_log()
{
	// validate state
	if(_is_file_open)
		return BLTR_SD_CARD_ERROR_INVALID_STATE;

	// initialize module
	if(!_is_module_initialized)
	{
		if(!_try_init_module())
			goto init_failed;		// yes, it is actually good practice to use 'goto' here. See https://stackoverflow.com/a/245761/499721
	}

	// open file
	uint16_t index = 0;
	if(!_try_get_last_log_index(&index))
		goto init_failed;

 	// we want to create a new log file, so increase the LOG file with the biggest number by 1
	index++;

	char filename[64] = { 0 };
	int len = strlen(_log_filename);
	memcpy(filename, _log_filename, len);
	
	snprintf(filename + len, sizeof(filename) - len, "%d", index);

    FRESULT result = f_open(&_file, filename, FA_READ | FA_WRITE | FA_CREATE_NEW);
    if (result != FR_OK)
    {
        NRF_LOG_INFO("unable to open or create file: %s, reason: %d", filename, result);
		goto init_failed;
    }

	NRF_LOG_INFO("new log created: %s", filename);    

	_is_file_open = true;
	_last_flush = bltr_utils_get_timestamp();

	return BLTR_SUCCESS;

// init failed - deinit module and return error code
init_failed:
	_deinit_module();
	return BLTR_SD_CARD_ERROR_INIT_FAILED;
}

ret_code_t bltr_sd_card_close_log()
{
	if(!_is_file_open)
		return BLTR_SUCCESS;

	NRF_LOG_INFO("closing file");

	f_close(&_file);
	_deinit_module();

	return BLTR_SUCCESS;
}

ret_code_t bltr_sd_card_write_log(uint8_t* data, uint16_t len)
{
	// check status
	if(!_is_module_initialized || !_is_file_open)
		return BLTR_SD_CARD_ERROR_INVALID_STATE;

	// write
	ret_code_t err = BLTR_SUCCESS;
	UINT bytes_written;
	FRESULT result = f_write(&_file, data, len, (UINT*)&bytes_written);

	if(result != FR_OK)
		err = (bytes_written < len) ? BLTR_SD_CARD_ERROR_CARD_FULL : BLTR_SD_CARD_ERROR_WRITE_FAILED;

	uint64_t now = bltr_utils_get_timestamp();
	float dt = (now - _last_flush) / 1e6f;

	if(dt >= BLTR_SD_CARD_FLUSH_INTERVAL)
	{
		_flush();
		_last_flush = now;
	}

	return err;
}

bool bltr_sd_card_is_present()
{
#if BLTR_SD_CARD_DETECT_CARD_ENABLED
	return _is_card_present;
#else
	return true;
#endif
}

bool _try_init_module()
{
	NRF_LOG_INFO("initializing disk 0 (SDC)...");

	// check SD card presense
	if(!bltr_sd_card_is_present())
	{
		NRF_LOG_INFO("SD card missing");
		return false;
	}

	// initialize hardware	
	DSTATUS disk_state = STA_NOINIT;

    for (uint32_t retries = DISK_INIT_RETRIES; retries && disk_state; --retries)
        disk_state = disk_initialize(0);
    
    if (disk_state)
    {
        NRF_LOG_INFO("disk initialization failed.");
        return false;
    }

	NRF_LOG_INFO("disk initialized");

	// initialize FatFS (mounting)
	uint32_t blocks_per_mb = (1024uL * 1024uL) / _block_dev_sdc.block_dev.p_ops->geometry(&_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = _block_dev_sdc.block_dev.p_ops->geometry(&_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    NRF_LOG_INFO("mounting volume. Capacity: %d MB", capacity);
    FRESULT result = f_mount(&_filesystem, "", 1);
	
    if (result)
    {
        NRF_LOG_INFO("mount failed: %d", result);
		disk_uninitialize(0);
        return false;
    }

	NRF_LOG_INFO("mounted volume...");

	// done
	_is_module_initialized = true;
	
	return true;
}

void _deinit_module()
{
	NRF_LOG_INFO("deinitializing module");

	// deinitialize FatFS (unmounting), ignoring all errors
    f_mount(0, "", 0);
	memset(&_filesystem, 0, sizeof(_filesystem));
	
	// deinitialize hardware, ignoring all errors
	disk_uninitialize(0);

	// done
	_is_module_initialized = false;
	_is_file_open = false;
}

void _flush()
{
	if(!_is_module_initialized || !_is_file_open)
		return;

	FRESULT result = f_sync(&_file);

	NRF_LOG_INFO("1234flushed");

	if(result != FR_OK)
		NRF_LOG_INFO("f_sync error: %d", result);
}

bool _try_get_last_log_index(uint16_t* index)
{
	// open log directory
	DIR dir;
	FILINFO file_info;
	uint16_t largest_log_index = 0;

	FRESULT fresult = f_opendir(&dir, LOG_DIR);
    if (fresult != FR_OK)
        return false;

	// read files in log directory
    do
    {
       	FRESULT result = f_readdir(&dir, &file_info);

        if (result != FR_OK)
        {
            NRF_LOG_INFO("directory read failed.");
            break; // still need to close dir
        }

        if (file_info.fname[0] && !(file_info.fattrib & AM_DIR))
        {
			char buf[4] = { 0 };
			memcpy(buf, file_info.fname, 3);

			if(strcmp(buf, _log_filename) != 0)
				continue;
			
			if(strlen(file_info.fname) < 4)
				continue;

			int idx = atoi(file_info.fname + 3);

			if(idx > largest_log_index)
				largest_log_index = idx;
        }
    }
    while (file_info.fname[0]); // f_readdir returns null when there are no more items

	f_closedir(&dir);

	*index = largest_log_index;

	return true;
}

// Card insertion/removal detection
static void _on_pin_event_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	if(pin == SDC_CD_PIN)
	{
		app_timer_stop(_debounce_timer);
		app_timer_start(_debounce_timer, DEBOUCE_INTERVAL, NULL);
	}
}

static void _debounce_handler(void* context)
{
	bool is_set = nrfx_gpiote_in_is_set(SDC_CD_PIN);
	app_sched_event_put(&is_set, 1, _synced_debounce_handler);
}

static void _synced_debounce_handler(void* p_event_data, uint16_t event_size)
{
	bool new_state = *((bool*)p_event_data);

	if(new_state == _is_card_present) // nothing changed
		return;
	
	_is_card_present = new_state;

	if(_is_card_present) // sd card inserted
	{
		NRF_LOG_INFO("insert detected");
	}
	else
	{
		NRF_LOG_INFO("remove detected");

		if(_is_module_initialized)
			_deinit_module();
	}

	if(_sd_card_status_handler != NULL)
		_sd_card_status_handler(_is_card_present);
}

#else // #if BLTR_SD_CARD_ENABLED

void bltr_sd_card_init(const bltr_sd_card_init_t* init) { /* ignore */}

ret_code_t bltr_sd_card_handle_uplink_message(const bluetera_uplink_message_t* message)
{
	if(message->which_payload != BLUETERA_UPLINK_MESSAGE_IMU_TAG)
		return BLTR_SUCCESS;
	
	const bluetera_imu_command_t* cmd = (const bluetera_imu_command_t*)&message->payload.imu;
	if(cmd->which_payload != BLUETERA_IMU_COMMAND_START_TAG)
		return BLTR_SUCCESS;

	if(cmd->payload.start.data_sink == BLUETERA_DATA_SINK_TYPE_DATA_SINK_TYPE_SDCARD)
		return BLTR_SD_CARD_ERROR_UNSUPPORTED;

	return BLTR_SUCCESS;	
}

ret_code_t bltr_sd_card_imu_data_handler(const bltr_imu_sensor_data_t* data) { return BLTR_SUCCESS; }

ret_code_t bltr_sd_card_open_log() { return BLTR_SD_CARD_ERROR_UNSUPPORTED; }
ret_code_t bltr_sd_card_close_log() { return BLTR_SD_CARD_ERROR_UNSUPPORTED; }
ret_code_t bltr_sd_card_write_log(uint8_t* data, uint16_t len) { return BLTR_SD_CARD_ERROR_UNSUPPORTED; }
bool bltr_sd_card_is_inserted() { return false; }

#endif // #if BLTR_SD_CARD_ENABLED