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
#include "bltr_imu.h"

#define BLTR_MAX_MESSAGE_LENGTH_VARINT_BYTES			4
#define BLTR_MAX_DOWNLINK_MESSAGE_SIZE					(BLUETERA_DOWNLINK_MESSAGE_SIZE + BLTR_MAX_MESSAGE_LENGTH_VARINT_BYTES)
#define BLTR_MAX_UPLINK_MESSAGE_SIZE					(BLUETERA_UPLINK_MESSAGE_SIZE + BLTR_MAX_MESSAGE_LENGTH_VARINT_BYTES)

#define BLTR_APP_SCHEDULER_DATA_SIZE					BLTR_MAX_UPLINK_MESSAGE_SIZE

typedef void (* bltr_uplink_message_handler_t) (bluetera_uplink_message_t * msg);

typedef struct
{
	bltr_uplink_message_handler_t message_handler;
} bltr_msg_init_t;

// TODO(boaz): move bltr_app_scheduler_message_t to c-file
typedef struct
{
	uint8_t data[BLTR_APP_SCHEDULER_DATA_SIZE];
	uint16_t length;
} bltr_app_scheduler_message_t;


ret_code_t bltr_msg_init(const bltr_msg_init_t* init);
ret_code_t bltr_msg_send_sensor_data(const bltr_imu_sensor_data_t* data);
ret_code_t bltr_msg_send_echo(uint32_t value);
ret_code_t bltr_msg_send_error(bluetera_bluetera_modules_type_t module, uint32_t code);

// helper methods
ret_code_t bltr_msg_encode_downlink_message(const bluetera_downlink_message_t* msg, uint8_t* target, uint16_t* bytes_written);
ret_code_t bltr_msg_imu_sensor_data_to_downlink_message(const bltr_imu_sensor_data_t* data, bluetera_downlink_message_t* message);

void bltr_msg_debug();

#endif /* BLUETERA_MESSAGES_H_ */
