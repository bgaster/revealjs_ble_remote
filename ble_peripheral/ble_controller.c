/**
 * @module ble_controller.c
 * @author: Benedict R. Gaster
 *
 * Implementation of the Reveal.js BLE Remote Peripheral Protocol.
 */
#include <stddef.h>
#include <string.h>
#include "nordic_common.h"
#include "app_error.h"

#include "ble_controller.h"

// Used to hope current packet to be transmitted
static uint8_t tmp[2] = {0};

/**
 * Called on connect to keep copy of evt handler.
 * @param p_controller Controller
 * @param p_ble_evt    Event
 */
static void on_connect(ble_controller_t * p_controller, ble_evt_t * p_ble_evt)
{
    p_controller->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**
 * Called on disconnect, stored invalid handle for controller event
 * as it should not be called once disconnected
 * @param p_controller Controller
 * @param p_ble_evt    Event
 */
static void on_disconnect(
  ble_controller_t * p_controller, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_controller->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**
 * Setup controller characterstic
 * @param  p_cservice        Controller service
 * @param  p_controller_init Any callback set during initialzation
 * @return                   NRF_SUCCESS is successful, otherwise an error code.
 */
static uint32_t controller_io_char_add(
  ble_controller_t * p_cservice,
  const ble_controller_init_t * p_controller_init)
{
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t    attr_char_value;
  ble_uuid_t          ble_uuid;
  ble_gatts_attr_md_t attr_md;

  // controller Characteristic UUID
  const ble_uuid128_t base_uuid = {
    {
      0xc4, 0x02, 0xcf, 0xef, 0x63, 0xd1, 0x0a, 0xbe,
      0xc1, 0x4e, 0x0f, 0x38, 0xce, 0xc6, 0xfb, 0xb4
    }
  };

#define CONTROLLER_IO_UUID_CHAR 0xc6ce

  uint32_t err_code;


  err_code = sd_ble_uuid_vs_add(&base_uuid, &ble_uuid.type);

  if (err_code != NRF_SUCCESS) {
    //return NRF_SUCCESS;
    return err_code;
  }

  memset(&cccd_md, 0, sizeof(cccd_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;

  memset(&char_md, 0, sizeof(char_md));

  char_md.char_props.read   = 1;
  char_md.char_props.notify = 1;
  //char_md.char_props.write_wo_resp = 1;
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = &cccd_md;
  char_md.p_sccd_md         = NULL;

  //ble_uuid.type = p_cservice->uuid_type;
  ble_uuid.uuid = CONTROLLER_IO_UUID_CHAR;

  memset(&attr_md, 0, sizeof(attr_md));

  //BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&attr_md.read_perm);
  //BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&attr_md.write_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
  //BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

  attr_md.vloc       = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth    = 0;
  attr_md.wr_auth    = 0;
  attr_md.vlen       = 0;

  memset(&attr_char_value, 0, sizeof(attr_char_value));

  attr_char_value.p_uuid       = &ble_uuid;
  attr_char_value.p_attr_md    = &attr_md;
  attr_char_value.init_len     = 2;//sizeof(uint8_t);
  attr_char_value.init_offs    = 0;
  attr_char_value.max_len      = 2;//sizeof(uint8_t) * 2;
  attr_char_value.p_value      = tmp;

  return sd_ble_gatts_characteristic_add(
    p_cservice->service_handle,
    &char_md,
    &attr_char_value,
    &p_cservice->controller_io_char_handles);
}

/**
 * Initialize controller service and add it to the BLE stack.
 * @param  p_cservice        Controller service struct
 * @param  p_controller_init Any callbacks and so on set during initialzation
 * @return                   NRF_SUCCESS if successful, otherwise an error code
 */
uint32_t ble_controller_init(
    ble_controller_t * p_cservice,
    const ble_controller_init_t * p_controller_init)
{
  uint32_t   err_code;
  ble_uuid_t ble_uuid;

  // Initialize service structure.
  p_cservice->conn_handle              = BLE_CONN_HANDLE_INVALID;
  p_cservice->controller_write_handler = p_controller_init->controller_write_handler;
  ble_uuid.type = p_cservice->uuid_type;
  ble_uuid.uuid = BLE_CONTROLLER_SERVICE;

  err_code = sd_ble_gatts_service_add(
    BLE_GATTS_SRVC_TYPE_PRIMARY,
    &ble_uuid,
    &p_cservice->service_handle);

  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  // Add characteristics.
  err_code = controller_io_char_add(p_cservice, p_controller_init);
  //if (err_code != NRF_SUCCESS) {
  //  return err_code;
  //}

  return err_code;
}

/**
 * Handle a BLE event forwarded from Softdevice
 * @param p_controller controller connection
 * @param p_ble_evt    Event to handle
 */
void ble_controller_on_ble_evt(
  ble_controller_t * p_controller,
  ble_evt_t * p_ble_evt)
{
  switch (p_ble_evt->header.evt_id)
  {
  case BLE_GAP_EVT_CONNECTED:
    on_connect(p_controller, p_ble_evt);
    break;

  case BLE_GAP_EVT_DISCONNECTED:
    on_disconnect(p_controller, p_ble_evt);
    break;

  case BLE_GATTS_EVT_WRITE:
    //what needs to go here?
    break;

  default:
   // No implementation needed.
   break;
  }
}

/**
 * Called when a button is pressed on the remote. Constructs protocol packet
 * and transmits to connected central.
 * @param  p_cservice Controller service
 * @param  button     Button pressed
 * @return            Result of sending packet
 */
uint32_t ble_controller_send_click(
     ble_controller_t * p_cservice,
     uint8_t button)
{
    uint8_t buf[2];
    buf[0] = 0;
    buf[1] = button;

    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(buf);
    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_cservice->controller_io_char_handles.value_handle;
    params.p_data = buf;
    params.p_len  = &len;
    return sd_ble_gatts_hvx(p_cservice->conn_handle, &params);
}
