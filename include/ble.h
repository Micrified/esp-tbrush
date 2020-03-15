#ifndef BLE_H
#define BLE_H


/*
 *******************************************************************************
 *                        (C) Copyright 2020 IoT Seminar                       *
 * Created: 15/03/2020                                                         *
 *                                                                             *
 * Programmer(s):                                                              *
 * - Charles Randolph                                                          *
 * - Sonnya Dellarosa                                                          *
 *                                                                             *
 * Description:                                                                *
 *  Wrapper for Bluedroid BLE                                                  *
 *                                                                             *
 *******************************************************************************
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "sdkconfig.h"

#include "config.h"


/*
 *******************************************************************************
 *                         External Symbolic Constants                         *
 *******************************************************************************
*/


// The name of the device as seen in the GAP advertising messages
#define BLE_DEVICE_NAME			                    DEVICE_BLE_SERVICE_NAME

// The Message-Transmission-Unit to use with clients
#define BLE_MTU_SIZE			                    512

// The maximum size (in bytes) of the response/indicate message (< MTU size) 
#define BLE_RSP_MSG_MAX_SIZE	                    20

// Macro: Shorthand for converting ESP error to string
#define E2S(err)		                            esp_err_to_name((err))


// The number of handles inside the App profile (four because they are)
// 1. Service handle
// 2. Characteristic declaration handle
// 3. Characteristic value handle
// 4. Characteristic descriptor handle
#define GATTS_HANDLE_COUNT                          4

// The UUID of the service in the app application profile
#define GATTS_SERVICE_UUID                          0x00FF

// The UUID for the app characteristic value
#define GATTS_CHARACTERISTIC_UUID                   0xFF01

// The UUID for the app characteristic descriptor (using the standard CCCD type)
#define GATTS_CHARACTERISTIC_DESCRIPTOR_UUID   \
	ESP_GATT_UUID_CHAR_CLIENT_CONFIG

// Describes the maximum buffer size for a write-event handler (+1 for null t?)
#define GATTS_WRITE_EVENT_MAX_BUF_SIZE              256 + 1

// The maximum length for a characteristic value (in bytes)
#define GATTS_CHARACTERISTIC_VALUE_LENGTH_MAX        64


/*
 *******************************************************************************
 *                              Type Definitions                               *
 *******************************************************************************
*/


// Enumeration of supported Application Profiles
enum {
	/*************************************************************************/
	APP_PROFILE_MAIN,

	/*************************************************************************/
	// The number of application profiles (do not remove)
	APP_PROFILE_COUNT
};


// Enumeration of state change events
typedef enum {
	BLE_STATE_EVENT_CONNECTED,
	BLE_STATE_EVENT_DISCONNECTED
} ble_state_event_t;


// Implementation of a profile: (Supports ONE service with ONE characteristic)
struct gatts_profile_t {
	esp_gatts_cb_t         gatts_cb;       // Callback function for events
	uint16_t               gatts_if;       // GATT interface ID
	uint16_t               app_id;         // Application ID
	uint16_t               conn_id;        // Connection ID
	uint16_t               service_handle; // Service Handle (index)
	esp_gatt_srvc_id_t     service_id;     // Service ID
	uint16_t               char_handle;    // Characteristic Handle (index)
	esp_bt_uuid_t          char_uuid;      // Characteristic UUID
	esp_gatt_perm_t        perm;           // Attribute permissions
	esp_gatt_char_prop_t   property;       // Properties of characteristic
	uint16_t               descr_handle;   // Config descriptor handle (index)
	esp_bt_uuid_t          descr_uuid;     // Config descriptor UUID
};


// Describes a buffer used for holding write-event data
typedef struct {
	uint8_t *buffer;
	size_t len;
} prepare_type_env_t;


// Callback type invoked on arrival of data
typedef esp_err_t (*ble_rx_cb_t)(size_t size, void *buffer);


// Callback type invoked on state change
typedef esp_err_t (*ble_state_event_cb_t)(ble_state_event_t state_event);


/*
 *******************************************************************************
 *                            Function Declarations                            *
 *******************************************************************************
*/


/* @brief Initializes the BT Controller in BLE mode, and Bluetooth Stack
 * @note NVS Flash storage should be initialized prior to calling this
 * 
 * @param 
 * - ble_rx_cb: The callback invoked when data is received over BLE
 * - ble_state_event_cb: The callback invoked when BLE state changes
 * @return esp_err_t: ESP_OK on success, otherwise unsuccessful
*/
esp_err_t ble_init (ble_rx_cb_t ble_rx_cb, 
	ble_state_event_cb_t ble_state_event_cb);


/* @brief Dispatches a message to the connected BLE device
 * @note Send this only after having called ble_init and knowing a device is 
 *       connected
 * @param
 * - len: The length (size) of the uint8_t byte buffer to send
 * - buffer: The uint8_t buffer pointer
 * @return
 * - ESP_OK: The message was sent
 * - ESP_ERR_INVALID_ARG: The give buffer was null, among other reasons
 * - ESP_ERR_INVALID_SIZE The message given is too large to fit in an MTU
 * - Other errors resulting from esp_ble_gatts_send_indicate are possible
*/
esp_err_t ble_send (size_t len, uint8_t *buffer);


#endif