#include <pb_encode.h>
#include <pb_decode.h>
#include <nordic_common.h>
#include <nrf_queue.h>
#include <app_scheduler.h>
#include <nrf_log.h>

#include "bluetera_messages.h"
#include "ble_bus.h"

static bltr_init_t _init_context;

typedef struct
{	
	uint8_t obuffer[BLTR_MAX_DOWNLINK_MESSAGE_SIZE];
	uint8_t ibuffer[BLTR_MAX_UPLINK_MESSAGE_SIZE];
	pb_ostream_t ostream;
	pb_istream_t istream;
} bltr_msg_context_t;

BLE_BUS_DEF(_bus);

static bltr_msg_context_t _context;
static ret_code_t _try_send_message(const uint8_t* buffer, uint32_t length);

static void nrf_qwr_error_handler(uint32_t nrf_error);
static void bus_data_handler(ble_bus_evt_t * p_evt);

ret_code_t bltr_msg_init(const bltr_init_t* context)
{
	ret_code_t err_code;	

	// Init message handlers
	_init_context = *context;

	// Init BUS service	
	ble_bus_init_t bus_init = { 0 };
    bus_init.data_handler = bus_data_handler;
    err_code = ble_bus_init(&_bus, &bus_init);
    APP_ERROR_CHECK(err_code);

	// Init protobuf
	_context.ostream = pb_ostream_from_buffer(_context.obuffer, sizeof(_context.obuffer));
	_context.istream = pb_istream_from_buffer(_context.ibuffer, sizeof(_context.ibuffer));

	return BLTR_SUCCESS;
}

ret_code_t bltr_msg_send_acceleration(uint16_t timestamp, float acc[3])
{
	// build message
	bluetera_bluetera_downlink_message_t message;
	message.timestamp = (uint32_t)timestamp;
	message.which_payload = BLUETERA_BLUETERA_DOWNLINK_MESSAGE_ACCELERATION_TAG;
	message.payload.acceleration.x = acc[0];
	message.payload.acceleration.y = acc[1];
	message.payload.acceleration.z = acc[2];

	// encode
	if(!pb_encode_delimited(&_context.ostream, bluetera_bluetera_downlink_message_fields, &message) || (_context.ostream.bytes_written == 0))
		return BLTR_MSG_ERROR_OP_FAILED;

	// try sending
	ret_code_t err = _try_send_message(_context.obuffer, _context.ostream.bytes_written);

	return BLTR_SUCCESS;
}

static ret_code_t _try_send_message(const uint8_t* buffer, uint32_t length)
{	
	// make sure there is enough space	
	if(ble_bus_get_num_free_tx_bytes(&_bus) < length)
		return BLTR_MSG_ERROR_RESOURCES;

	// send using 'tx_buffer_size' chuncks
	uint32_t tx_buffer_size = ble_bus_get_tx_buffer_size(&_bus);
	while(length > 0)
	{
		uint16_t packet_len = MIN(length, tx_buffer_size);
		if(ble_bus_data_send(&_bus, (uint8_t*)buffer, &packet_len) != NRF_SUCCESS)
			return BLTR_MSG_ERROR_OP_FAILED;

		buffer += packet_len;
		length -= packet_len;		
	}

	return BLTR_SUCCESS;
}


// try to parse RX queue data - must run in application context
static void _try_parse_rx_data(void * p_event_data, uint16_t event_size)
{
	// verify
	if(p_event_data == NULL)
		return;

	// copy
	bltr_app_scheduler_message_t* app_message = (bltr_app_scheduler_message_t*)p_event_data;
	memcpy(_context.ibuffer, app_message->data, app_message->length);

	// try parsing
	bluetera_bluetera_uplink_message_t message;
	if(pb_decode_delimited(&_context.istream, bluetera_bluetera_uplink_message_fields, &message))
	{
		if(_init_context.message_handler != NULL)
			_init_context.message_handler(&message);
	}
}

// Transport handlers
// TODO: expand to include other transport like USB, UART, RTT etc
static void bus_data_handler(ble_bus_evt_t * p_evt)
{
	bltr_app_scheduler_message_t app_message;	
    switch(p_evt->type)
    {
		case BLE_BUS_EVT_RX_DATA:
			NRF_LOG_INFO("BLE_BUS_EVT_RX_DATA");

			app_message.length = MIN(p_evt->params.rx_data.length, sizeof(app_message.data));
			memcpy(app_message.data, p_evt->params.rx_data.p_data, app_message.length);
			app_sched_event_put(&app_message, sizeof(app_message), _try_parse_rx_data);
			break;

		case BLE_BUS_EVT_TX_RDY:
		case BLE_BUS_EVT_COMM_STARTED:
    	case BLE_BUS_EVT_COMM_STOPPED:
			break;
    }
}
