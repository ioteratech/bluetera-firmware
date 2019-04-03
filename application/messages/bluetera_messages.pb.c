/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.6-dev at Tue Feb  5 17:06:07 2019. */

#include "bluetera_messages.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t bluetera_imu_acceleration_payload_fields[4] = {
    PB_FIELD(  1, FLOAT   , REQUIRED, STATIC  , FIRST, bluetera_imu_acceleration_payload_t, x, x, 0),
    PB_FIELD(  2, FLOAT   , REQUIRED, STATIC  , OTHER, bluetera_imu_acceleration_payload_t, y, x, 0),
    PB_FIELD(  3, FLOAT   , REQUIRED, STATIC  , OTHER, bluetera_imu_acceleration_payload_t, z, y, 0),
    PB_LAST_FIELD
};

const pb_field_t bluetera_imu_quaternion_payload_fields[5] = {
    PB_FIELD(  1, FLOAT   , REQUIRED, STATIC  , FIRST, bluetera_imu_quaternion_payload_t, w, w, 0),
    PB_FIELD(  2, FLOAT   , REQUIRED, STATIC  , OTHER, bluetera_imu_quaternion_payload_t, x, w, 0),
    PB_FIELD(  3, FLOAT   , REQUIRED, STATIC  , OTHER, bluetera_imu_quaternion_payload_t, y, x, 0),
    PB_FIELD(  4, FLOAT   , REQUIRED, STATIC  , OTHER, bluetera_imu_quaternion_payload_t, z, y, 0),
    PB_LAST_FIELD
};

const pb_field_t bluetera_bluetera_downlink_message_fields[4] = {
    PB_FIELD(  1, UINT32  , REQUIRED, STATIC  , FIRST, bluetera_bluetera_downlink_message_t, timestamp, timestamp, 0),
    PB_ONEOF_FIELD(payload,   2, MESSAGE , ONEOF, STATIC  , OTHER, bluetera_bluetera_downlink_message_t, acceleration, timestamp, &bluetera_imu_acceleration_payload_fields),
    PB_ONEOF_FIELD(payload,   3, MESSAGE , ONEOF, STATIC  , OTHER, bluetera_bluetera_downlink_message_t, quaternion, timestamp, &bluetera_imu_quaternion_payload_fields),
    PB_LAST_FIELD
};

const pb_field_t bluetera_echo_payload_fields[2] = {
    PB_FIELD(  1, STRING  , REQUIRED, STATIC  , FIRST, bluetera_echo_payload_t, value, value, 0),
    PB_LAST_FIELD
};

const pb_field_t bluetera_bluetera_uplink_message_fields[2] = {
    PB_ONEOF_FIELD(payload,   1, MESSAGE , ONEOF, STATIC  , FIRST, bluetera_bluetera_uplink_message_t, echo, echo, &bluetera_echo_payload_fields),
    PB_LAST_FIELD
};


/* Check that field information fits in pb_field_t */
#if !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_32BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in 8 or 16 bit
 * field descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(bluetera_bluetera_downlink_message_t, payload.acceleration) < 65536 && pb_membersize(bluetera_bluetera_downlink_message_t, payload.quaternion) < 65536 && pb_membersize(bluetera_bluetera_downlink_message_t, payload.acceleration) < 65536 && pb_membersize(bluetera_bluetera_downlink_message_t, payload.quaternion) < 65536 && pb_membersize(bluetera_bluetera_uplink_message_t, payload.echo) < 65536 && pb_membersize(bluetera_bluetera_downlink_message_t, payload.acceleration) < 65536 && pb_membersize(bluetera_bluetera_downlink_message_t, payload.quaternion) < 65536 && pb_membersize(bluetera_bluetera_downlink_message_t, payload.acceleration) < 65536 && pb_membersize(bluetera_bluetera_downlink_message_t, payload.quaternion) < 65536 && pb_membersize(bluetera_bluetera_uplink_message_t, payload.echo) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_bluetera_imu_acceleration_payload_bluetera_imu_quaternion_payload_bluetera_bluetera_downlink_message_bluetera_echo_payload_bluetera_bluetera_uplink_message)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_16BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in the default
 * 8 bit descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(bluetera_bluetera_downlink_message_t, payload.acceleration) < 256 && pb_membersize(bluetera_bluetera_downlink_message_t, payload.quaternion) < 256 && pb_membersize(bluetera_bluetera_downlink_message_t, payload.acceleration) < 256 && pb_membersize(bluetera_bluetera_downlink_message_t, payload.quaternion) < 256 && pb_membersize(bluetera_bluetera_uplink_message_t, payload.echo) < 256 && pb_membersize(bluetera_bluetera_downlink_message_t, payload.acceleration) < 256 && pb_membersize(bluetera_bluetera_downlink_message_t, payload.quaternion) < 256 && pb_membersize(bluetera_bluetera_downlink_message_t, payload.acceleration) < 256 && pb_membersize(bluetera_bluetera_downlink_message_t, payload.quaternion) < 256 && pb_membersize(bluetera_bluetera_uplink_message_t, payload.echo) < 256), YOU_MUST_DEFINE_PB_FIELD_16BIT_FOR_MESSAGES_bluetera_imu_acceleration_payload_bluetera_imu_quaternion_payload_bluetera_bluetera_downlink_message_bluetera_echo_payload_bluetera_bluetera_uplink_message)
#endif


/* @@protoc_insertion_point(eof) */
