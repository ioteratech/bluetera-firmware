/*
 * bluetera_messages.h
 *
 *  Created on: 15 ????? 2019
 *      Author: Boaz
 */

#ifndef BLUETERA_MESSAGES_H_
#define BLUETERA_MESSAGES_H_

#include "bluetera_messages.pb.h"
#include "bluetera_err.h"
#include "pb.h"

#define BLTR_MAX_MESSAGE_LENGTH_VARINT_BYTES			4
#define BLTR_MAX_DOWNLINK_MESSAGE_SIZE					(BLUETERA_BLUETERA_DOWNLINK_MESSAGE_SIZE + BLTR_MAX_MESSAGE_LENGTH_VARINT_BYTES)
#define BLTR_MAX_UPLINK_MESSAGE_SIZE					(BLUETERA_BLUETERA_UPLINK_MESSAGE_SIZE + BLTR_MAX_MESSAGE_LENGTH_VARINT_BYTES)

#define BLTR_APP_SCHEDULER_DATA_SIZE					BLTR_MAX_UPLINK_MESSAGE_SIZE

typedef void (* bltr_uplink_message_handler_t) (bluetera_bluetera_uplink_message_t * msg);

typedef struct
{
	bltr_uplink_message_handler_t message_handler;
} bltr_init_t;

typedef struct
{
	uint8_t data[BLTR_APP_SCHEDULER_DATA_SIZE];
	uint16_t length;
} bltr_app_scheduler_message_t;


ret_code_t bltr_msg_init(const bltr_init_t* context);
ret_code_t bltr_msg_send_acceleration( uint16_t timestamp, float acc[3]);

#endif /* BLUETERA_MESSAGES_H_ */
