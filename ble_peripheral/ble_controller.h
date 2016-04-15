/**
 * Name: ble_controller.h
 * Author: Benedict R. Gaster
 * Desc:
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define LOCAL_SERVICE_UUID            0x1523                                        /**< Proprietary UUID for local service. */
#define LOCAL_CHAR_UUID               0x1524                                        /**< Proprietary UUID for local characteristic. */
#define LOCAL_VENDOR_UUID_INDEX       0                                             /**< Index of the 128-bit UUID base List (the structure m_base_uuid128) to be used for the UUIDs. */

#define BLE_CONTROLLER_UUID_BASE_REV { 0x33, 0x2f, 0x08, 0x7b, 0xb5, 0x1c, 0xd7, 0xae, 0xd3, 0x44, 0x3f, 0x08, 0x0b, 0xa8, 0x43, 0xa9}

#define BLE_CONTROLLER_UUID_BASE { 0xA9, 0x43, 0xA8, 0x0B, 0x08, 0x3F, 0x44, 0xD3, 0xAE, 0xD7, 0x1C, 0xB5, 0x7B, 0x08, 0x2F, 0x33 }

#define BLE_CONTROLLER_SERVICE 0xa80b

// Forward declaration of the ble_lbs_t type.
typedef struct ble_controller_s ble_controller_t;

typedef void (*ble_controller_write_handler_t) (ble_controller_t * p_lbs,
                                                uint8_t new_state);

typedef struct
{
    ble_controller_write_handler_t controller_write_handler; /**< Event handler to be called when the LED Characteristic is written. */
} ble_controller_init_t;


struct ble_controller_s
{
    uint16_t                    service_handle;              /**< Handle of Controller Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    controller_char_handles;     /**< Handles related to the Controller Characteristic. */
    ble_gatts_char_handles_t    controller_io_char_handles;  /**< Handles related to the ControllerCharacteristic. */
    uint8_t                     uuid_type;                   /**< UUID type for the Controller Service. */
    uint16_t                    conn_handle;                 /**< Handle of the current connection (as provided by the BLE stack). BLE_CONN_HANDLE_INVALID if not in a connection. */
    ble_controller_write_handler_t controller_write_handler;   /**< Event handler to be called when the Controller Characteristic is written. */
};

/**
 * [ble_controller_init description]
 * @param  p_cservice        [description]
 * @param  p_controller_init [description]
 * @return                   [description]
 */
uint32_t ble_controller_init(
    ble_controller_t * p_cservice,
    const ble_controller_init_t * p_controller_init);

void ble_controller_on_ble_evt(
    ble_controller_t * p_controller, ble_evt_t * p_ble_evt);

uint32_t ble_controller_send_click(
    ble_controller_t * p_controller,
    uint8_t button);
