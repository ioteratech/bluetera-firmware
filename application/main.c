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

#include <stdint.h>
#include <string.h>

#include <nordic_common.h>
#include <nrf.h>
#include <nrf_sdm.h>
#include <app_error.h>
#include <ble.h>
#include <ble_err.h>
#include <ble_hci.h>
#include <ble_srv_common.h>
#include <ble_advdata.h>
#include <ble_advertising.h>
#include <ble_dis.h>
#include <ble_conn_params.h>
#include <nrf_sdh.h>
#include <nrf_sdh_ble.h>
#include <nrf_sdh_soc.h>
#include <peer_manager.h>
#include <fds.h>
#include <nrf_ble_gatt.h>
#include <nrf_ble_qwr.h>
#include <ble_conn_state.h>
#include <nrf_pwr_mgmt.h>
#include <app_timer.h>

#include <nrf_log.h>
#include <nrf_log_ctrl.h>
#include <nrf_log_default_backends.h>

#include <nrfx_gpiote.h>
#include <nrf_delay.h>

#include <app_scheduler.h>

#include <nrfx_timer.h>

#include "bluetera_boards.h"
#include "icm_driver.h"
#include "imu_service.h"
#include "bluetera_messages.h"
#include "utils.h"
#include "bluetera_constants.h"

// The advertising interval (in units of 0.625 ms)
#define	APP_ADV_INTERVAL 					300

// The advertising duration in units of 10 milliseconds
#define APP_ADV_DURATION 					BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED

// A tag identifying the SoftDevice BLE configuration
#define APP_BLE_CONN_CFG_TAG 				1 

// application's BLE observer priority, you shouldn't need to modify this value
#define APP_BLE_OBSERVER_PRIO 				3

// minimum acceptable connection interval
#define MIN_CONN_INTERVAL 					MSEC_TO_UNITS(20, UNIT_1_25_MS)

// maximum acceptable connection interval
#define MAX_CONN_INTERVAL 					MSEC_TO_UNITS(20, UNIT_1_25_MS)

// slave latency
#define SLAVE_LATENCY 						0

// connection supervisory timeout
#define CONN_SUP_TIMEOUT 					MSEC_TO_UNITS(4000, UNIT_10_MS)

// time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called
#define FIRST_CONN_PARAMS_UPDATE_DELAY 		APP_TIMER_TICKS(5000)

// time between each call to sd_ble_gap_conn_param_update after the first call
#define NEXT_CONN_PARAMS_UPDATE_DELAY 		APP_TIMER_TICKS(30000)

// number of attempts before giving up the connection parameter negotiation
#define MAX_CONN_PARAMS_UPDATE_COUNT 		3

// perform bonding
#define SEC_PARAM_BOND                      1

// Man In The Middle protection
#define SEC_PARAM_MITM                      0

// LE Secure Connections
#define SEC_PARAM_LESC                      0

// keypress notifications
#define SEC_PARAM_KEYPRESS                  0

// I/O capabilities
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE

// out-of-band data
#define SEC_PARAM_OOB                       0

// minimum encryption key size
#define SEC_PARAM_MIN_KEY_SIZE              7

// maximum encryption key size
#define SEC_PARAM_MAX_KEY_SIZE              16

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define LED_TIMER_INTERVAL_ADV		        APP_TIMER_TICKS(2000)
#define LED_TIMER_INTERVAL_CONNECTED        APP_TIMER_TICKS(500)

BLE_IMU_DEF(_imu_service);
NRF_BLE_GATT_DEF(_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(_advertising);                                 /**< Advertising module instance. */
APP_TIMER_DEF(_led_timer_id);

#define SCHED_MAX_EVENT_DATA_SIZE		MAX(32, sizeof(ble_evt_t))
#define SCHED_QUEUE_SIZE				256

typedef enum
{
	IMU_BLE_UPDATE_RATE_LOW	= 50,	// 20 Hz
	IMU_BLE_UPDATE_RATE_NORMAL = 20,	// 50 Hz
	IMU_BLE_UPDATE_RATE_HIGH = 10   // 100 Hz
} ImuBleUpdateRate;

static ImuBleUpdateRate _selected_imu_ble_update_rate = IMU_BLE_UPDATE_RATE_NORMAL;

// handle of the current connection
static uint16_t _conn_handle = BLE_CONN_HANDLE_INVALID; 

// UUID of services to advertise
static ble_uuid_t _adv_uuids[] =
{
	// {IMU_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN}
};

static void timers_init();
static void ble_stack_init();
static void gap_params_init();
static void services_init();
static void advertising_init();
static void conn_params_init();
static void peer_manager_init();
static ret_code_t bluetera_messages_init();

static void imu_data_handler(const bltr_imu_sensor_data_t* data);
static void advertising_start(bool erase_bonds);
static void on_sensor_enable(void* p_event_data, uint16_t event_size);
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt);
static void on_ble_disconnected(void*, uint16_t);
static void bluetera_uplink_message_handler(bluetera_uplink_message_t* msg);

int main()
{
	// note: initialization order is important - rearrange with care!

	// general initialization
	ret_code_t err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);
	NRF_LOG_DEFAULT_BACKENDS_INIT();
	NRF_LOG_INFO("startup");	
	timers_init();

	// power management
	err_code = nrf_pwr_mgmt_init();
	APP_ERROR_CHECK(err_code);

	// BLE
	ble_stack_init();
	gap_params_init();	
	err_code = nrf_ble_gatt_init(&_gatt, gatt_evt_handler);
	APP_ERROR_CHECK(err_code);
	services_init();
	bluetera_messages_init();
	advertising_init();
	conn_params_init();
	peer_manager_init();	

	// nrfx_gpiote_out_config_t debug_pin = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(false);
	// nrfx_gpiote_out_init(DEBUG_GPIO_TIMING, &debug_pin);

	// application scheduler (synchronizes IRQ events to main thread)
	APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

	// Bluetera general initialization
	bltr_utils_init();
	
	// Leds
	APP_ERROR_CHECK(nrfx_gpiote_init());
	nrfx_gpiote_out_config_t led_cfg = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
	nrfx_gpiote_out_init(LED_BLUE_PIN, &led_cfg);
	err_code = app_timer_start(_led_timer_id, LED_TIMER_INTERVAL_ADV, NULL);
    APP_ERROR_CHECK(err_code);

	// IMU initialization
	nrf_delay_ms(1000);		// account for IMU wakeup delay

	bltr_imu_init_t imu_init;	
	memset(&imu_init, 0, sizeof(imu_init));
	imu_init.imu_data_handler = imu_data_handler;
	bltr_imu_init(&imu_init);
	bltr_imu_set_mode(BLTR_IMU_MODE_DMP);

	uint16_t acc_fsr;
	uint16_t gyro_fsr;
	bltr_imu_get_fsr(&acc_fsr, &gyro_fsr);
	bltr_imu_service_update_fsr(&_imu_service, acc_fsr, gyro_fsr);

	// go!
	advertising_start(false);

	while(true)
	{
		bltr_imu_update();		
		app_sched_execute();

		// if there are no pending log operations, then sleep until next the next event occurs
		if (NRF_LOG_PROCESS() == false)
			nrf_pwr_mgmt_run();
		
		NRF_LOG_PROCESS();
	}
}

// this function will be called in case of an assert in the SoftDevice.
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{	
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void delete_bonds()
{
	ret_code_t err_code;

	NRF_LOG_INFO("erasing bonds");
	err_code = pm_peers_delete();
	APP_ERROR_CHECK(err_code);
}

static void advertising_start(bool erase_bonds)
{
	if (erase_bonds == true)
	{
		delete_bonds();
		// advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
	}
	else
	{
		ret_code_t err_code;

		err_code = ble_advertising_start(&_advertising, BLE_ADV_MODE_FAST);
		APP_ERROR_CHECK(err_code);
	}
}

static void fds_evt_handler(fds_evt_t const * const p_evt)
{
	if (p_evt->id == FDS_EVT_GC)
		NRF_LOG_DEBUG("GC completed");
}

static void pm_evt_handler(pm_evt_t const * p_evt)
{
	ret_code_t err_code;

	switch (p_evt->evt_id)
	{
		case PM_EVT_BONDED_PEER_CONNECTED:
			NRF_LOG_INFO("connected to a previously bonded device.");
			break;

		case PM_EVT_CONN_SEC_SUCCEEDED:		
			NRF_LOG_INFO("connection secured, procedure: %d", p_evt->params.conn_sec_succeeded.procedure);
			break;
		case PM_EVT_CONN_SEC_FAILED:
			// 	Often, when securing fails, it shouldn't be restarted, for security reasons.
			//  Other times, it can be restarted directly.
			//  Sometimes it can be restarted, but only after changing some Security Parameters.
			//  Sometimes, it cannot be restarted until the link is disconnected and reconnected.
			//  Sometimes it is impossible, to secure the link, or the peer device does not support it.
			//  How to handle this error is highly application dependent.
			break;
		case PM_EVT_CONN_SEC_CONFIG_REQ:
			{
				// reject pairing request from an already bonded peer.
				pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
				pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
			}
			break;
		case PM_EVT_STORAGE_FULL:
			{
				// run garbage collection on the flash.
				err_code = fds_gc();
				if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
				{
					// retry
				}
				else
				{
					APP_ERROR_CHECK(err_code);
				}
			}
			break;
		case PM_EVT_PEERS_DELETE_SUCCEEDED:
			NRF_LOG_DEBUG("PM_EVT_PEERS_DELETE_SUCCEEDED");
			advertising_start(false);
			break;
		case PM_EVT_PEER_DATA_UPDATE_FAILED:
			APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
			break;
		case PM_EVT_PEER_DELETE_FAILED:
			APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
			break;
		case PM_EVT_PEERS_DELETE_FAILED:
			APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
			break;
		case PM_EVT_ERROR_UNEXPECTED:
			APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
			break;
		case PM_EVT_CONN_SEC_START:
		case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
		case PM_EVT_PEER_DELETE_SUCCEEDED:
		case PM_EVT_LOCAL_DB_CACHE_APPLIED:
		case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
			// This can happen when the local DB has changed.
		case PM_EVT_SERVICE_CHANGED_IND_SENT:
		case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
		default:
			break;
	}
}

static void led_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    
	nrfx_gpiote_out_toggle(LED_BLUE_PIN);
}

static void timers_init()
{
	ret_code_t err_code;

	// Initialize timer module.
	err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_create(&_led_timer_id, APP_TIMER_MODE_REPEATED, led_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

static void gap_params_init()
{
	ret_code_t              err_code;
	ble_gap_conn_params_t   gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.slave_latency     = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);
}

static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
	if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
		NRF_LOG_INFO("mtu changed to: %d", p_evt->params.att_mtu_effective);
}

static void nrf_qwr_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}

static void services_init()
{
	ret_code_t err_code;
	nrf_ble_qwr_init_t qwr_init = {0};
	static char hw_version_str[16] = {0};
	static char fw_version_str[16] = {0};

	// initialize Queued Write Module.
	qwr_init.error_handler = nrf_qwr_error_handler;

	err_code = nrf_ble_qwr_init(&_qwr, &qwr_init);
	APP_ERROR_CHECK(err_code);

	// initialize Device Information service
	ble_dis_init_t dis_init = { 0 };
	snprintf(hw_version_str, sizeof(hw_version_str), "%d.%d", MSB_16(HARDWARE_VERSION), LSB_16(HARDWARE_VERSION));
	snprintf(fw_version_str, sizeof(fw_version_str), "%d.%d", MSB_16(FIRMWARE_VERSION), LSB_16(FIRMWARE_VERSION));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char*)MANUFACTURER_NAME);
	ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, hw_version_str);
	ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, fw_version_str);

	dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

	// initialize IMU service
    bltr_imu_service_init_t imu_service_init = { 0 };

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&imu_service_init.custom_value_char_attr_md.cccd_write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&imu_service_init.custom_value_char_attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&imu_service_init.custom_value_char_attr_md.write_perm);

	_imu_service.on_sensor_enable = on_sensor_enable;

    err_code = bltr_imu_service_init(&_imu_service, &imu_service_init);
    APP_ERROR_CHECK(err_code);
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
	ret_code_t err_code;

	if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
	{
		err_code = sd_ble_gap_disconnect(_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
		APP_ERROR_CHECK(err_code);
	}
}

static void conn_params_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}

static void conn_params_init()
{
	ret_code_t err_code;
	ble_conn_params_init_t cp_init;

	memset(&cp_init, 0, sizeof(cp_init));

	cp_init.p_conn_params                  = NULL;
	cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
	cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
	cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
	cp_init.disconnect_on_fail             = false;
	cp_init.evt_handler                    = on_conn_params_evt;
	cp_init.error_handler                  = conn_params_error_handler;

	err_code = ble_conn_params_init(&cp_init);
	APP_ERROR_CHECK(err_code);
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
	switch (ble_adv_evt)
	{
		case BLE_ADV_EVT_FAST:
			NRF_LOG_INFO("advertising: fast");
			break;
		case BLE_ADV_EVT_IDLE:
			NRF_LOG_INFO("advertising: idle");
			break;
		default:
			break;
	}
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
	ret_code_t err_code;

	switch (p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_CONNECTED:
			NRF_LOG_INFO("connected");
			_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			err_code = nrf_ble_qwr_conn_handle_assign(&_qwr, _conn_handle);
			APP_ERROR_CHECK(err_code);
			app_timer_stop(_led_timer_id);
			app_timer_start(_led_timer_id, LED_TIMER_INTERVAL_CONNECTED, NULL);
			break;
		case BLE_GAP_EVT_DISCONNECTED:
			NRF_LOG_INFO("disconnected, reason %d.", p_ble_evt->evt.gap_evt.params.disconnected.reason);
			_conn_handle = BLE_CONN_HANDLE_INVALID;
			app_sched_event_put(NULL, 0, on_ble_disconnected);
			app_timer_stop(_led_timer_id);
			app_timer_start(_led_timer_id, LED_TIMER_INTERVAL_ADV, NULL);
			break;
		case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
			{
				NRF_LOG_DEBUG("phy update request");
				ble_gap_phys_t const phys =
				{
					.rx_phys = BLE_GAP_PHY_AUTO,
					.tx_phys = BLE_GAP_PHY_AUTO,
				};
				err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
				APP_ERROR_CHECK(err_code);
			}
			break;
		case BLE_GATTC_EVT_TIMEOUT:
			NRF_LOG_DEBUG("GATT client timeout");
			err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			APP_ERROR_CHECK(err_code);
			break;
		case BLE_GATTS_EVT_TIMEOUT:
			NRF_LOG_DEBUG("GATT server timeout");
			err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			APP_ERROR_CHECK(err_code);
			break;
		default:
			// No implementation needed.
			break;
	}
}

static void ble_stack_init()
{
	ret_code_t err_code;

	err_code = nrf_sdh_enable_request();
	APP_ERROR_CHECK(err_code);

	// Configure the BLE stack using the default settings.
	// Fetch the start address of the application RAM.
	uint32_t ram_start = 0;
	err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
	APP_ERROR_CHECK(err_code);

	ble_cfg_t ble_cfg = { 0 };
	ble_cfg.conn_cfg.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
	ble_cfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = HVN_TX_QUEUE_SIZE;
	err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &ble_cfg, ram_start);
	APP_ERROR_CHECK(err_code);

	// Enable BLE stack.
	err_code = nrf_sdh_ble_enable(&ram_start);
	APP_ERROR_CHECK(err_code);

	// Register a handler for BLE events.
	NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

static void peer_manager_init()
{
	ble_gap_sec_params_t sec_param;
	ret_code_t           err_code;

	err_code = pm_init();
	APP_ERROR_CHECK(err_code);

	memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

	// Security parameters to be used for all security procedures.
	sec_param.bond           = SEC_PARAM_BOND;
	sec_param.mitm           = SEC_PARAM_MITM;
	sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
	sec_param.oob            = SEC_PARAM_OOB;
	sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
	sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
	sec_param.kdist_own.enc  = 1;
	sec_param.kdist_own.id   = 1;
	sec_param.kdist_peer.enc = 1;
	sec_param.kdist_peer.id  = 1;

	err_code = pm_sec_params_set(&sec_param);
	APP_ERROR_CHECK(err_code);

	err_code = pm_register(pm_evt_handler);
	APP_ERROR_CHECK(err_code);

	err_code = fds_register(fds_evt_handler);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init()
{
	ret_code_t             err_code;
	ble_advertising_init_t init;

	memset(&init, 0, sizeof(init));

	// construct advertising packet - includes full name and device descriptor (as manufacturer-specific data)
	init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
	init.advdata.include_appearance = false;
	init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	//init.advdata.uuids_complete.uuid_cnt = sizeof(_adv_uuids) / sizeof(_adv_uuids[0]);
	//init.advdata.uuids_complete.p_uuids  = _adv_uuids;

	// configure advertising
	init.config.ble_adv_fast_enabled  = true;
	init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
	init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
	init.evt_handler = on_adv_evt;

	err_code = ble_advertising_init(&_advertising, &init);
	APP_ERROR_CHECK(err_code);

	ble_advertising_conn_cfg_tag_set(&_advertising, APP_BLE_CONN_CFG_TAG);
}

// synced with main thread
static void imu_data_handler(const bltr_imu_sensor_data_t* data)
{
	if (_conn_handle == BLE_CONN_HANDLE_INVALID)
		return;

	uint32_t timestamp = (uint32_t)(data->timestamp / 1000.0f);
	switch (data->sensor)
	{
		case BLTR_IMU_SENSOR_TYPE_ACCELEROMETER:
		{
			// characteristic format:
			// [4 bytes]:   timestamp (milliseconds)
			// [12 bytes]:	accelerometer data [x, y, z]
			uint8_t _acc_data[CHAR_ACC_LENGTH] = { 0 };			
			memcpy(_acc_data, &timestamp, 4);
			memcpy(_acc_data + 4, data->acceleration, 12);			
			bltr_imu_service_update_acc(&_imu_service, _acc_data);
		}
			break;
	
		case BLTR_IMU_SENSOR_TYPE_ROTATION_VECTOR:
		{
			// characteristic format:
			// [4 bytes]:   timestamp (milliseconds)
			// [16 bytes]:  quaternion data [w, x, y, z]
			uint8_t _quat_data[CHAR_QUAT_LENGTH] = { 0 };
			memcpy(_quat_data, &timestamp, 4);
			memcpy(_quat_data + 4, data->quaternion, 16);
			bltr_imu_service_update_quat(&_imu_service, _quat_data);
		}
			break;

		default:
			break;
	}
}

// synced with scheduler
static void on_ble_disconnected(void* p_event_data, uint16_t event_size)
{
	// p_event_data should be always null here and event_size is always 0

	bltr_imu_stop();
}

// called from scheduler
void on_sensor_enable(void* p_event_data, uint16_t event_size)
{
	uint8_t* cbdata = (uint8_t*)p_event_data;

	switch(cbdata[0])
	{
		case IMU_SENSOR_SET_ENABLED:
			if(cbdata[1])
			{
				bltr_imu_set_mode(BLTR_IMU_MODE_DMP);
				bltr_imu_start(_selected_imu_ble_update_rate);
			}
			else
			{
				bltr_imu_stop();
			}
			break;
		case IMU_SENSOR_RAW:
			if(cbdata[1])
			{
				bltr_imu_set_freq_divider(11); // ~102 Hz
				bltr_imu_set_mode(BLTR_IMU_MODE_DIRECT);
			}
			break;
		case IMU_SENSOR_SET_FSR:
			{
				uint16_t acc_fsr = *(uint16_t*)&cbdata[1];
				uint16_t gyro_fsr = *(uint16_t*)&cbdata[3];

				// TODO validate those values, currently if they are not valid
				// bltr_imu_set_fsr will ignore them but the GATT table will show
				// the new invalid values

				bltr_imu_set_fsr(acc_fsr, gyro_fsr);
			}
			break;
	}
}

// Bluetera uplink message handler
static void bluetera_uplink_message_handler(bluetera_uplink_message_t* msg)
{
	NRF_LOG_DEBUG("bluetera_uplink_message_handler(): msg->which_payload = %d", msg->which_payload);
	switch(msg->which_payload)
	{
		case BLUETERA_UPLINK_MESSAGE_ECHO_TAG:
			bltr_msg_send_echo(msg->payload.echo.value);
		break;

		default:
			break;
	}
}

static ret_code_t bluetera_messages_init()
{
	bltr_msg_init_t context = 
	{
		.message_handler = bluetera_uplink_message_handler
	};

	return bltr_msg_init(&context); 
}