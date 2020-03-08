#include <pb_encode.h>
#include <pb_decode.h>
#include <nordic_common.h>
#include <nrf_queue.h>
#include <app_scheduler.h>
#include <ble.h>
#include <ble_srv_common.h>

#include "bluetera_messages.h"
#include "ble_bus.h"

// Configure logging
#define NRF_LOG_MODULE_NAME bltr_msg
#if BLTR_MSG_CONFIG_LOG_ENABLED
    #define NRF_LOG_LEVEL       BLTR_MSG_CONFIG_LOG_LEVEL
	#define NRF_LOG_INFO_COLOR  BLTR_MSG_CONFIG_INFO_COLOR
	#define NRF_LOG_DEBUG_COLOR BLTR_MSG_CONFIG_DEBUG_COLOR
#else
    #define NRF_LOG_LEVEL       0
#endif // BLTR_MSG_LOG_ENABLED
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_strerror.h"

// static fields
static bltr_uplink_message_handler_t _message_handler = NULL;
static uint8_t _ibuffer[BLTR_MAX_UPLINK_MESSAGE_SIZE];
static uint8_t _obuffer[BLTR_MAX_DOWNLINK_MESSAGE_SIZE];
BLE_BUS_DEF(_bus);
static volatile bool _can_send_message = false;

// static methods
static ret_code_t _try_send_message(const bluetera_downlink_message_t* message);
static void _bus_data_handler(ble_bus_evt_t * p_evt);
static void _on_ble_event(ble_evt_t const * p_ble_evt, void* p_context);

// register BLE events handler
NRF_SDH_BLE_OBSERVER(bltr_msg_observer, BLTR_MSG_BLE_OBSERVER_PRIO, _on_ble_event, NULL);

// implementation
ret_code_t bltr_msg_init(const bltr_msg_init_t* init)
{
	ASSERT(init != NULL);
	
	ret_code_t err_code;	

	// Init locals
	_message_handler = init->message_handler;
	memset(_obuffer, 0, sizeof(_obuffer));
	memset(_ibuffer, 0, sizeof(_ibuffer));

	// Init BUS service	
	ble_bus_init_t bus_init = { 0 };
    bus_init.data_handler = _bus_data_handler;
    err_code = ble_bus_init(&_bus, &bus_init);
    APP_ERROR_CHECK(err_code);

	return BLTR_SUCCESS;
}

ret_code_t bltr_msg_send_sensor_data(const bltr_imu_sensor_data_t* data)
{	
	bluetera_downlink_message_t message;
	ret_code_t err = bltr_msg_imu_sensor_data_to_downlink_message(data, &message);
	BLTR_RETURN_CODE_IF_ERROR(err);

	err = _try_send_message(&message);
	return err;
}

ret_code_t bltr_msg_send_echo(uint32_t value)
{
	// build message
	bluetera_downlink_message_t message;
	message.which_payload = BLUETERA_DOWNLINK_MESSAGE_ECHO_TAG;
	message.payload.echo.value = value;
	message.payload.echo.has_value = true;

	// try sending message
	ret_code_t err = _try_send_message(&message);
	return err;
}

ret_code_t bltr_msg_send_error(bluetera_bluetera_modules_type_t module, uint32_t code)
{
	// build message
	bluetera_downlink_message_t message;
	message.which_payload = BLUETERA_DOWNLINK_MESSAGE_ERROR_TAG;	
	message.payload.error.module = module;
	message.payload.error.has_module = true;
	message.payload.error.code = code;
	message.payload.error.has_code = true;

	// try sending message
	ret_code_t err = _try_send_message(&message);
	return err;
}

ret_code_t bltr_msg_encode_downlink_message(const bluetera_downlink_message_t* msg, uint8_t* target, uint16_t* bytes_written)
{
	pb_ostream_t ostream = pb_ostream_from_buffer(target, BLTR_MAX_DOWNLINK_MESSAGE_SIZE);
	if(!pb_encode_delimited(&ostream, bluetera_downlink_message_fields, msg) || (ostream.bytes_written == 0))
	{
		NRF_LOG_ERROR("bltr_msg_encode_downlink_message() - pb_encode_delimited() failed with error: %s", PB_GET_ERROR(ostream));
		return BLTR_MSG_ERROR_OP_FAILED;
	}

	*bytes_written = ostream.bytes_written;

	return BLTR_SUCCESS;
}

static ret_code_t _try_send_message(const bluetera_downlink_message_t* message)
{	
	ASSERT(message != NULL);

	ret_code_t err_code;

	// make sure we can send the message
	if(!_can_send_message)
		return BLTR_MSG_ERROR_NO_TRANSPORT;
		
	// encode message
	uint16_t data_length;
	err_code = bltr_msg_encode_downlink_message(message, _obuffer, &data_length);
	if(err_code != BLTR_SUCCESS)
		return err_code;

	// send message (will fail if not enough space)	
	//NRF_LOG_HEXDUMP_INFO(_obuffer, data_length);
	err_code = ble_bus_data_send(&_bus, _obuffer, &data_length);

	return err_code;
}

// try to parse RX queue data - must run in application context
static void _try_parse_rx_data(void * p_event_data, uint16_t event_size)
{
	NRF_LOG_DEBUG("_try_parse_rx_data() - event_size = %d", event_size);

	// verify
	if(p_event_data == NULL)
		return;

	// copy buffer
	bltr_app_scheduler_message_t* app_message = (bltr_app_scheduler_message_t*)p_event_data;
	memcpy(_ibuffer, app_message->data, app_message->length);

	// try parsing
	bluetera_uplink_message_t message;
	pb_istream_t istream = pb_istream_from_buffer(_ibuffer, sizeof(_ibuffer));
	if(pb_decode_delimited(&istream, bluetera_uplink_message_fields, &message))
	{		
		if(_message_handler != NULL)
			_message_handler(&message);
	}
	else
	{
		NRF_LOG_WARNING("_try_parse_rx_data() - pb_decode_delimited() failed with error: %s", PB_GET_ERROR(ostream));
	}	
}

// Transport handlers
// Carful! This method is called from an IRQ
// TODO: expand to include other transport like USB, UART, RTT etc
static void _bus_data_handler(ble_bus_evt_t * p_evt)
{	
	bltr_app_scheduler_message_t app_message;	
    switch(p_evt->type)
    {
		case BLE_BUS_EVT_RX_DATA:
			NRF_LOG_DEBUG("_bus_data_handler() - RX data");
			NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);			
			app_message.length = MIN(p_evt->params.rx_data.length, sizeof(app_message.data));
			memcpy(app_message.data, p_evt->params.rx_data.p_data, app_message.length);
			app_sched_event_put(&app_message, sizeof(app_message), _try_parse_rx_data);
			break;

		case BLE_BUS_EVT_COMM_STARTED:
			NRF_LOG_INFO("_bus_data_handler() - Notifications enabled");
			_can_send_message = true;
			break;

		case BLE_BUS_EVT_COMM_STOPPED:
			NRF_LOG_INFO("_bus_data_handler() - Notifications disabled");
			_can_send_message = false;
			break;

		case BLE_BUS_EVT_TX_RDY:    	
		default:
			/* do nothing */
			break;
    }
}

static void _on_ble_event(ble_evt_t const * p_ble_evt, void* p_context)
{
	if (p_ble_evt->header.evt_id == BLE_GAP_EVT_DISCONNECTED)
	{
		NRF_LOG_INFO("_on_ble_event() - disconnected");
		_can_send_message = false;
	}
}

ret_code_t bltr_msg_imu_sensor_data_to_downlink_message(const bltr_imu_sensor_data_t* data, bluetera_downlink_message_t* message)
{
	ret_code_t err = BLTR_SUCCESS;

	switch (data->sensor)
	{
		case BLTR_IMU_SENSOR_TYPE_ACCELEROMETER:
			message->which_payload = BLUETERA_DOWNLINK_MESSAGE_ACCELERATION_TAG;
			message->payload.acceleration.timestamp = (uint32_t)(data->timestamp / 1000.0f);
			message->payload.acceleration.has_timestamp = true;
			message->payload.acceleration.x = data->acceleration[0];
			message->payload.acceleration.has_x = true;
			message->payload.acceleration.y = data->acceleration[1];
			message->payload.acceleration.has_y = true;
			message->payload.acceleration.z = data->acceleration[2];
			message->payload.acceleration.has_z = true;
			break;
	
		case BLTR_IMU_SENSOR_TYPE_ROTATION_VECTOR:
			message->which_payload = BLUETERA_DOWNLINK_MESSAGE_QUATERNION_TAG;
			message->payload.quaternion.timestamp = (uint32_t)(data->timestamp / 1000.0f);
			message->payload.quaternion.has_timestamp = true;
			message->payload.quaternion.w = data->quaternion[0];
			message->payload.quaternion.has_w = true;
			message->payload.quaternion.x = data->quaternion[1];
			message->payload.quaternion.has_x = true;
			message->payload.quaternion.y = data->quaternion[2];
			message->payload.quaternion.has_y = true;
			message->payload.quaternion.z = data->quaternion[3];
			message->payload.quaternion.has_z = true;
			break;
		
		case BLTR_IMU_SENSOR_TYPE_GYROSCOPE:
			message->which_payload = BLUETERA_DOWNLINK_MESSAGE_GYROSCOPE_TAG; // TODO add support for raw data in .proto file!
			message->payload.gyroscope.timestamp = (uint32_t)(data->timestamp / 1000.0f);
			message->payload.gyroscope.has_timestamp = true;
			message->payload.gyroscope.x = data->gyroscope[0];
			message->payload.gyroscope.has_x = true;
			message->payload.gyroscope.y = data->gyroscope[1];
			message->payload.gyroscope.has_y = true;
			message->payload.gyroscope.z = data->gyroscope[2];
			message->payload.gyroscope.has_z = true;
			break;

		case BLTR_IMU_SENSOR_TYPE_RAW:
			message->which_payload = BLUETERA_DOWNLINK_MESSAGE_ACCELERATION_TAG; // TODO add support for raw data in .proto file!
			message->payload.acceleration.timestamp = (uint32_t)(data->timestamp / 1000.0f);
			message->payload.acceleration.has_timestamp = true;
			message->payload.acceleration.x = data->raw.acceleration[0];
			message->payload.acceleration.has_x = true;
			message->payload.acceleration.y = data->raw.acceleration[1];
			message->payload.acceleration.has_y = true;
			message->payload.acceleration.z = data->raw.acceleration[2];
			message->payload.acceleration.has_z = true;
			break;

		default:
			err = BLTR_MSG_ERROR_UNSUPPORTED;
			break;
	}

	return err;	
}