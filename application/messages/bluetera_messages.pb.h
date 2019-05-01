/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.6-dev at Wed May  1 15:57:09 2019. */

#ifndef PB_BLUETERA_MESSAGES_PB_H_INCLUDED
#define PB_BLUETERA_MESSAGES_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum
{
    BLUETERA_BLUETERA_MODULES_TYPE_SYSTEM = 0,
    BLUETERA_BLUETERA_MODULES_TYPE_IMU = 1,
    BLUETERA_BLUETERA_MODULES_TYPE_SDCARD = 2
} bluetera_bluetera_modules_type_t;
#define BLUETERA_BLUETERA_MODULES_TYPE_MIN BLUETERA_BLUETERA_MODULES_TYPE_SYSTEM
#define BLUETERA_BLUETERA_MODULES_TYPE_MAX BLUETERA_BLUETERA_MODULES_TYPE_SDCARD
#define BLUETERA_BLUETERA_MODULES_TYPE_ARRAYSIZE ((bluetera_bluetera_modules_type_t)(BLUETERA_BLUETERA_MODULES_TYPE_SDCARD+1))

typedef enum
{
    BLUETERA_IMU_DATA_TYPE_UNKNOWN = 0,
    BLUETERA_IMU_DATA_TYPE_QUATERNION = 1,
    BLUETERA_IMU_DATA_TYPE_ACCELEROMETER = 2,
    BLUETERA_IMU_DATA_TYPE_RAW = 4
} bluetera_imu_data_type_t;
#define BLUETERA_IMU_DATA_TYPE_MIN BLUETERA_IMU_DATA_TYPE_UNKNOWN
#define BLUETERA_IMU_DATA_TYPE_MAX BLUETERA_IMU_DATA_TYPE_RAW
#define BLUETERA_IMU_DATA_TYPE_ARRAYSIZE ((bluetera_imu_data_type_t)(BLUETERA_IMU_DATA_TYPE_RAW+1))

/* Struct definitions */
typedef struct {
    pb_callback_t value;
/* @@protoc_insertion_point(struct:bluetera_echo_payload_t) */
} bluetera_echo_payload_t;

typedef struct {
    bool has_module;
    bluetera_bluetera_modules_type_t module;
    bool has_code;
    uint32_t code;
/* @@protoc_insertion_point(struct:bluetera_error_t) */
} bluetera_error_t;

typedef struct {
    bool has_timestamp;
    uint32_t timestamp;
    bool has_x;
    float x;
    bool has_y;
    float y;
    bool has_z;
    float z;
/* @@protoc_insertion_point(struct:bluetera_imu_acceleration_payload_t) */
} bluetera_imu_acceleration_payload_t;

typedef struct {
    bool has_timestamp;
    uint32_t timestamp;
    bool has_w;
    float w;
    bool has_x;
    float x;
    bool has_y;
    float y;
    bool has_z;
    float z;
/* @@protoc_insertion_point(struct:bluetera_imu_quaternion_payload_t) */
} bluetera_imu_quaternion_payload_t;

typedef struct {
    bool has_data_types;
    uint32_t data_types;
    bool has_odr;
    uint32_t odr;
    bool has_acc_fsr;
    uint32_t acc_fsr;
    bool has_gyro_fsr;
    uint32_t gyro_fsr;
/* @@protoc_insertion_point(struct:bluetera_imu_start_t) */
} bluetera_imu_start_t;

typedef struct {
    pb_size_t which_payload;
    union {
        bluetera_imu_acceleration_payload_t acceleration;
        bluetera_imu_quaternion_payload_t quaternion;
        bluetera_error_t error;
        bluetera_echo_payload_t echo;
    } payload;
/* @@protoc_insertion_point(struct:bluetera_downlink_message_t) */
} bluetera_downlink_message_t;

typedef struct {
    pb_size_t which_payload;
    union {
        bluetera_imu_start_t start;
        bool stop;
    } payload;
/* @@protoc_insertion_point(struct:bluetera_imu_command_t) */
} bluetera_imu_command_t;

typedef struct {
    pb_size_t which_payload;
    union {
        bluetera_imu_command_t imu;
        bluetera_echo_payload_t echo;
    } payload;
/* @@protoc_insertion_point(struct:bluetera_uplink_message_t) */
} bluetera_uplink_message_t;

/* Default values for struct fields */

/* Initializer values for message structs */
#define BLUETERA_ECHO_PAYLOAD_INIT_DEFAULT       {{{NULL}, NULL}}
#define BLUETERA_ERROR_INIT_DEFAULT              {false, (bluetera_bluetera_modules_type_t)0, false, 0}
#define BLUETERA_IMU_ACCELERATION_PAYLOAD_INIT_DEFAULT {false, 0, false, 0, false, 0, false, 0}
#define BLUETERA_IMU_QUATERNION_PAYLOAD_INIT_DEFAULT {false, 0, false, 0, false, 0, false, 0, false, 0}
#define BLUETERA_DOWNLINK_MESSAGE_INIT_DEFAULT   {0, {BLUETERA_IMU_ACCELERATION_PAYLOAD_INIT_DEFAULT}}
#define BLUETERA_IMU_START_INIT_DEFAULT          {false, 0, false, 0, false, 0, false, 0}
#define BLUETERA_IMU_COMMAND_INIT_DEFAULT        {0, {BLUETERA_IMU_START_INIT_DEFAULT}}
#define BLUETERA_UPLINK_MESSAGE_INIT_DEFAULT     {0, {BLUETERA_IMU_COMMAND_INIT_DEFAULT}}
#define BLUETERA_ECHO_PAYLOAD_INIT_ZERO          {{{NULL}, NULL}}
#define BLUETERA_ERROR_INIT_ZERO                 {false, (bluetera_bluetera_modules_type_t)0, false, 0}
#define BLUETERA_IMU_ACCELERATION_PAYLOAD_INIT_ZERO {false, 0, false, 0, false, 0, false, 0}
#define BLUETERA_IMU_QUATERNION_PAYLOAD_INIT_ZERO {false, 0, false, 0, false, 0, false, 0, false, 0}
#define BLUETERA_DOWNLINK_MESSAGE_INIT_ZERO      {0, {BLUETERA_IMU_ACCELERATION_PAYLOAD_INIT_ZERO}}
#define BLUETERA_IMU_START_INIT_ZERO             {false, 0, false, 0, false, 0, false, 0}
#define BLUETERA_IMU_COMMAND_INIT_ZERO           {0, {BLUETERA_IMU_START_INIT_ZERO}}
#define BLUETERA_UPLINK_MESSAGE_INIT_ZERO        {0, {BLUETERA_IMU_COMMAND_INIT_ZERO}}

/* Field tags (for use in manual encoding/decoding) */
#define BLUETERA_ECHO_PAYLOAD_VALUE_TAG          1
#define BLUETERA_ERROR_MODULE_TAG                1
#define BLUETERA_ERROR_CODE_TAG                  2
#define BLUETERA_IMU_ACCELERATION_PAYLOAD_TIMESTAMP_TAG 1
#define BLUETERA_IMU_ACCELERATION_PAYLOAD_X_TAG  2
#define BLUETERA_IMU_ACCELERATION_PAYLOAD_Y_TAG  3
#define BLUETERA_IMU_ACCELERATION_PAYLOAD_Z_TAG  4
#define BLUETERA_IMU_QUATERNION_PAYLOAD_TIMESTAMP_TAG 1
#define BLUETERA_IMU_QUATERNION_PAYLOAD_W_TAG    2
#define BLUETERA_IMU_QUATERNION_PAYLOAD_X_TAG    3
#define BLUETERA_IMU_QUATERNION_PAYLOAD_Y_TAG    4
#define BLUETERA_IMU_QUATERNION_PAYLOAD_Z_TAG    5
#define BLUETERA_IMU_START_DATA_TYPES_TAG        1
#define BLUETERA_IMU_START_ODR_TAG               2
#define BLUETERA_IMU_START_ACC_FSR_TAG           3
#define BLUETERA_IMU_START_GYRO_FSR_TAG          4
#define BLUETERA_DOWNLINK_MESSAGE_ACCELERATION_TAG 1
#define BLUETERA_DOWNLINK_MESSAGE_QUATERNION_TAG 2
#define BLUETERA_DOWNLINK_MESSAGE_ERROR_TAG      16
#define BLUETERA_DOWNLINK_MESSAGE_ECHO_TAG       17
#define BLUETERA_IMU_COMMAND_START_TAG           1
#define BLUETERA_IMU_COMMAND_STOP_TAG            2
#define BLUETERA_UPLINK_MESSAGE_IMU_TAG          1
#define BLUETERA_UPLINK_MESSAGE_ECHO_TAG         17

/* Struct field encoding specification for nanopb */
extern const pb_field_t bluetera_echo_payload_fields[2];
extern const pb_field_t bluetera_error_fields[3];
extern const pb_field_t bluetera_imu_acceleration_payload_fields[5];
extern const pb_field_t bluetera_imu_quaternion_payload_fields[6];
extern const pb_field_t bluetera_downlink_message_fields[5];
extern const pb_field_t bluetera_imu_start_fields[5];
extern const pb_field_t bluetera_imu_command_fields[3];
extern const pb_field_t bluetera_uplink_message_fields[3];

/* Maximum encoded size of messages (where known) */
/* BLUETERA_ECHO_PAYLOAD_SIZE depends on runtime parameters */
#define BLUETERA_ERROR_SIZE                      8
#define BLUETERA_IMU_ACCELERATION_PAYLOAD_SIZE   21
#define BLUETERA_IMU_QUATERNION_PAYLOAD_SIZE     26
/* BLUETERA_DOWNLINK_MESSAGE_SIZE depends on runtime parameters */
#define BLUETERA_IMU_START_SIZE                  23
#define BLUETERA_IMU_COMMAND_SIZE                25
/* BLUETERA_UPLINK_MESSAGE_SIZE depends on runtime parameters */

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define BLUETERA_MESSAGES_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
