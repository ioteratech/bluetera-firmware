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
#include "sdk_common.h"
#include "ble.h"
#include "ble_bus.h"
#include "ble_srv_common.h"
#include "bluetera_constants.h"

#define NRF_LOG_MODULE_NAME ble_bus
#if BLE_BUS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       BLE_BUS_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  BLE_BUS_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BLE_BUS_CONFIG_DEBUG_COLOR
#else // BLE_BUS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       0
#endif // BLE_BUS_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


#define BLE_UUID_BUS_TX_CHARACTERISTIC 0x0003               /**< The UUID of the TX Characteristic. */
#define BLE_UUID_BUS_RX_CHARACTERISTIC 0x0002               /**< The UUID of the RX Characteristic. */

#define BLE_BUS_MAX_RX_CHAR_LEN        BLE_BUS_MAX_DATA_LEN /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_BUS_MAX_TX_CHAR_LEN        BLE_BUS_MAX_DATA_LEN /**< Maximum length of the TX Characteristic (in bytes). */

//e2532657-9ba2-4913-9cd4-ce6ee7b579e8
//actual: e2530001-9ba2-4913-9cd4-ce6ee7b579e8
#define BUS_BASE_UUID                  {{0xE8, 0x79, 0xB5, 0xE7, 0x6E, 0xCE, 0xD4, 0x9C, 0x13, 0x49, 0xA2, 0x9B, 0x57, 0x26, 0x53, 0xE2}} /**< Used vendor specific UUID. */


/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the SoftDevice.
 *
 * @param[in] p_bus     Bluetera UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_bus_t * p_bus, ble_evt_t const * p_ble_evt)
{
    ret_code_t                 err_code;
    ble_bus_evt_t              evt;
    ble_gatts_value_t          gatts_val;
    uint8_t                    cccd_value[2];

	p_bus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
	p_bus->is_notification_enabled = false;
	nrf_atomic_u32_store(&p_bus->free_buffers, HVN_TX_QUEUE_SIZE);

    /* Check the hosts CCCD value to inform of readiness to send data using the RX characteristic */
    memset(&gatts_val, 0, sizeof(ble_gatts_value_t));
    gatts_val.p_value = cccd_value;
    gatts_val.len     = sizeof(cccd_value);
    gatts_val.offset  = 0;

    err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle, p_bus->rx_handles.cccd_handle, &gatts_val);

    if ((err_code == NRF_SUCCESS) && (p_bus->data_handler != NULL) && ble_srv_is_notification_enabled(gatts_val.p_value))
    {
        memset(&evt, 0, sizeof(ble_bus_evt_t));
        evt.type        = BLE_BUS_EVT_COMM_STARTED;
        evt.p_bus       = p_bus;

        p_bus->data_handler(&evt);
    }
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the SoftDevice.
 *
 * @param[in] p_bus     Bluetera UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_bus_t * p_bus, ble_evt_t const * p_ble_evt)
{
    ret_code_t                    err_code;
    ble_bus_evt_t                 evt;
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gatts_evt.conn_handle);
    }

    memset(&evt, 0, sizeof(ble_bus_evt_t));
    evt.p_bus       = p_bus;

    if ((p_evt_write->handle == p_bus->tx_handles.cccd_handle) && (p_evt_write->len == 2))
    {
        if (p_bus->conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                p_bus->is_notification_enabled = true;
                evt.type                          = BLE_BUS_EVT_COMM_STARTED;
            }
            else
            {
                p_bus->is_notification_enabled = false;
                evt.type                          = BLE_BUS_EVT_COMM_STOPPED;
            }

            if (p_bus->data_handler != NULL)
            {
                p_bus->data_handler(&evt);
            }

        }
    }
    else if ((p_evt_write->handle == p_bus->rx_handles.value_handle) &&
             (p_bus->data_handler != NULL))
    {
        evt.type                  = BLE_BUS_EVT_RX_DATA;
        evt.params.rx_data.p_data = p_evt_write->data;
        evt.params.rx_data.length = p_evt_write->len;

        p_bus->data_handler(&evt);
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_HVN_TX_COMPLETE event from the SoftDevice.
 *
 * @param[in] p_bus     Bluetera UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_hvx_tx_complete(ble_bus_t * p_bus, ble_evt_t const * p_ble_evt)
{
    ret_code_t                 err_code;
    ble_bus_evt_t              evt;

	nrf_atomic_u32_add(&p_bus->free_buffers, p_ble_evt->evt.gatts_evt.params.hvn_tx_complete.count);
    if (p_bus->is_notification_enabled)
    {
        memset(&evt, 0, sizeof(ble_bus_evt_t));
        evt.type        = BLE_BUS_EVT_TX_RDY;
        evt.p_bus       = p_bus;
		evt.params.tx_ready.tx_completed = p_ble_evt->evt.gatts_evt.params.hvn_tx_complete.count;

        p_bus->data_handler(&evt);
    }
}


void ble_bus_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_bus_t * p_bus = (ble_bus_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_bus, p_ble_evt);
            break;
		case BLE_GAP_EVT_DISCONNECTED:
			p_bus->conn_handle = BLE_CONN_HANDLE_INVALID;
			break;
        case BLE_GATTS_EVT_WRITE:
            on_write(p_bus, p_ble_evt);
            break;
        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            on_hvx_tx_complete(p_bus, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_bus_init(ble_bus_t * p_bus, ble_bus_init_t const * p_bus_init)
{
    ret_code_t            err_code;
    ble_uuid_t            ble_uuid;
    ble_uuid128_t         bus_base_uuid = BUS_BASE_UUID;
    ble_add_char_params_t add_char_params;

    VERIFY_PARAM_NOT_NULL(p_bus);
    VERIFY_PARAM_NOT_NULL(p_bus_init);

    // Initialize the service structure.
    p_bus->data_handler = p_bus_init->data_handler;

    /**@snippet [Adding proprietary Service to the SoftDevice] */
    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&bus_base_uuid, &p_bus->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_bus->uuid_type;
    ble_uuid.uuid = BLE_UUID_BUS_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_bus->service_handle);
    /**@snippet [Adding proprietary Service to the SoftDevice] */
    VERIFY_SUCCESS(err_code);

    // Add the RX Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = BLE_UUID_BUS_RX_CHARACTERISTIC;
    add_char_params.uuid_type                = p_bus->uuid_type;
    add_char_params.max_len                  = BLE_BUS_MAX_RX_CHAR_LEN;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.write         = 1;
    add_char_params.char_props.write_wo_resp = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;

    err_code = characteristic_add(p_bus->service_handle, &add_char_params, &p_bus->rx_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the TX Characteristic.
    /**@snippet [Adding proprietary characteristic to the SoftDevice] */
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLE_UUID_BUS_TX_CHARACTERISTIC;
    add_char_params.uuid_type         = p_bus->uuid_type;
    add_char_params.max_len           = BLE_BUS_MAX_TX_CHAR_LEN;
    add_char_params.init_len          = sizeof(uint8_t);
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;

    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    return characteristic_add(p_bus->service_handle, &add_char_params, &p_bus->tx_handles);
    /**@snippet [Adding proprietary characteristic to the SoftDevice] */
}


uint32_t ble_bus_data_send(ble_bus_t* p_bus, uint8_t* p_data, uint16_t* p_length)
{
    ret_code_t                 err_code;
    ble_gatts_hvx_params_t     hvx_params;

    VERIFY_PARAM_NOT_NULL(p_bus);

    if (p_bus->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (!p_bus->is_notification_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }
	
	if(*p_length > ble_bus_get_num_free_tx_bytes(p_bus))
	{		
		return NRF_ERROR_BUSY;
	}

	uint16_t packet_length;
	uint16_t remaining_bytes = *p_length;	
	uint32_t used_buffers = 0;

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_bus->tx_handles.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
	hvx_params.p_len  = &packet_length;

	while(remaining_bytes > 0)
	{
		packet_length = MIN(remaining_bytes, BLE_BUS_MAX_TX_CHAR_LEN);
		hvx_params.p_data = (const uint8_t*)p_data;		
		err_code = sd_ble_gatts_hvx(p_bus->conn_handle, &hvx_params);

		if(err_code == NRF_SUCCESS)
		{
			used_buffers++;			
			p_data += packet_length;
			remaining_bytes -= packet_length;
		}
		else
		{
			break;
		}
	}

	nrf_atomic_u32_sub(&p_bus->free_buffers, used_buffers);
    return err_code;
}

uint32_t ble_bus_get_num_free_tx_bytes(ble_bus_t * p_bus)
{
	return (BLE_BUS_MAX_TX_CHAR_LEN * p_bus->free_buffers);
}