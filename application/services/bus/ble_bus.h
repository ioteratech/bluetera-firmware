 /**
 * Copyright (c) 2012 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**@file
 *
 * @defgroup ble_bus Bluetera UART Service
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    Bluetera UART Service implementation, heavily based on Nordic UART Service
 *
 * @details The Bluetera UART Service is a simple GATT-based service with TX and RX characteristics.
 *          Data received from the peer is passed to the application, and the data received
 *          from the application of this service is sent to the peer as Handle Value
 *          Notifications. This module demonstrates how to implement a custom GATT-based
 *          service and characteristics using the SoftDevice. The service
 *          is used by the application to send and receive ASCII text strings to and from the
 *          peer.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_bus_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_BUS_BLE_OBSERVER_PRIO,
 *                                   ble_bus_on_ble_evt, &instance);
 *          @endcode
 */
#ifndef BLE_BUS_H__
#define BLE_BUS_H__

#include <stdint.h>
#include <stdbool.h>
#include "bluetera_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_bus instance.
 *
 * @param     _name            Name of the instance.
 * @param[in] _bus_max_clients Maximum number of NUS clients connected at a time.
 * @hideinitializer
 */
#define BLE_BUS_DEF(_name)                      				  \
    static ble_bus_t _name;                                       \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,                           \
                         BLE_BUS_BLE_OBSERVER_PRIO,               \
                         ble_bus_on_ble_evt,                      \
                         &_name)

#define BLE_UUID_BUS_SERVICE 0x0001 /**< The UUID of the Bluetera UART Service. */

#define OPCODE_LENGTH        1
#define HANDLE_LENGTH        2

/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer by the Bluetera UART service module. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_BUS_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define BLE_BUS_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif


/**@brief   Bluetera UART Service event types. */
typedef enum
{
    BLE_BUS_EVT_RX_DATA,      /**< Data received. */
    BLE_BUS_EVT_TX_RDY,       /**< Service is ready to accept new data to be transmitted. */
    BLE_BUS_EVT_COMM_STARTED, /**< Notification has been enabled. */
    BLE_BUS_EVT_COMM_STOPPED, /**< Notification has been disabled. */
} ble_bus_evt_type_t;


/* Forward declaration of the ble_bus_t type. */
typedef struct ble_bus_s ble_bus_t;


/**@brief   Bluetera UART Service @ref BLE_BUS_EVT_RX_DATA event data.
 *
 * @details This structure is passed to an event when @ref BLE_BUS_EVT_RX_DATA occurs.
 */
typedef struct
{
    uint8_t const * p_data; /**< A pointer to the buffer with received data. */
    uint16_t        length; /**< Length of received data. */
} ble_bus_evt_rx_data_t;

typedef struct
{
	uint8_t tx_completed;
} ble_bus_evt_tx_ready;

/**@brief   Bluetera UART Service event structure.
 *
 * @details This structure is passed to an event coming from service.
 */
typedef struct
{
    ble_bus_evt_type_t         type;        /**< Event type. */
    ble_bus_t                * p_bus;       /**< A pointer to the instance. */

    union
    {
        ble_bus_evt_rx_data_t rx_data; /**< @ref BLE_BUS_EVT_RX_DATA event data. */
		ble_bus_evt_tx_ready tx_ready;
    } params;
} ble_bus_evt_t;


/**@brief Bluetera UART Service event handler type. */
typedef void (* ble_bus_data_handler_t) (ble_bus_evt_t * p_evt);


/**@brief   Bluetera UART Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_bus_init
 *          function.
 */
typedef struct
{
    ble_bus_data_handler_t data_handler; /**< Event handler to be called for handling received data. */
} ble_bus_init_t;


/**@brief   Bluetera UART Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_bus_s
{
    uint8_t                         uuid_type;          /**< UUID type for Bluetera UART Service Base UUID. */
    uint16_t                        service_handle;     /**< Handle of Bluetera UART Service (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        tx_handles;         /**< Handles related to the TX characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        rx_handles;         /**< Handles related to the RX characteristic (as provided by the SoftDevice). */
    ble_bus_data_handler_t          data_handler;       /**< Event handler to be called for handling received data. */
	uint16_t                      	conn_handle;
	bool 					   		is_notification_enabled;
	uint8_t 						free_buffers;
};


/**@brief   Function for initializing the Bluetera UART Service.
 *
 * @param[out] p_bus      Bluetera UART Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_bus_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_bus or p_bus_init is NULL.
 */
uint32_t ble_bus_init(ble_bus_t * p_bus, ble_bus_init_t const * p_bus_init);


/**@brief   Function for handling the Bluetera UART Service's BLE events.
 *
 * @details The Bluetera UART Service expects the application to call this function each time an
 * event is received from the SoftDevice. This function processes the event if it
 * is relevant and calls the Bluetera UART Service event handler of the
 * application if necessary.
 *
 * @param[in] p_ble_evt     Event received from the SoftDevice.
 * @param[in] p_context     Bluetera UART Service structure.
 */
void ble_bus_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief   Function for sending a data to the peer.
 *
 * @details This function sends the input string as an RX characteristic notification to the
 *          peer.
 *
 * @param[in]     p_bus       Pointer to the Bluetera UART Service structure.
 * @param[in]     p_data      String to be sent.
 * @param[in,out] p_length    Pointer Length of the string. Amount of sent bytes.
 * @param[in]     conn_handle Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_bus_data_send(ble_bus_t * p_bus, uint8_t   * p_data, uint16_t  * p_length);

/**@brief   Get the size of a TX buffer.
 *
 * @param[in]     p_bus       Pointer to the Bluetera UART Service structure.
 *
 * @retval Number of available buffers
 */
uint32_t ble_bus_get_tx_buffer_size(ble_bus_t * p_bus);

/**@brief   Get the number of available TX buffers.
 *
 * @details Returns the number of internal available TX buffers.
 *          Use this function instead of accessing ble_bus_s.free_buffers for thread-safety
 *
 * @param[in]     p_bus       Pointer to the Bluetera UART Service structure.
 *
 * @retval Number of available buffers
 */
uint8_t ble_bus_get_num_free_tx_buffers(ble_bus_t * p_bus);

/**@brief   Get the number of available TX bytes.
 *
 * @details Returns the number of internal available bytes for TX.
 *
 * @param[in]     p_bus       Pointer to the Bluetera UART Service structure.
 *
 * @retval Number of available buffers
 */
uint32_t ble_bus_get_num_free_tx_bytes(ble_bus_t * p_bus);

#ifdef __cplusplus
}
#endif

#endif // BLE_BUS_H__

/** @} */
