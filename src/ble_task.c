#include "ble_task.h"


/*
 *******************************************************************************
 *                          Internal Global Variables                          *
 *******************************************************************************
*/


// Queue holding incoming data from the BLE driver
static xQueueHandle g_ble_rx_queue = NULL;


// Queue holding data to transmit via the BLE driver
static xQueueHandle g_ble_tx_queue = NULL;


/*
 *******************************************************************************
 *                        Internal Function Definitions                        *
 *******************************************************************************
*/


// Callback invoked when data arrives to the buffer
esp_err_t ble_rx_cb (size_t size, void *buffer) {

}


// Callback type invoked on bluetooth state change
esp_err_t ble_state_event_cb(ble_state_event_t state_event) {

}


/*
 *******************************************************************************
 *                            Function Definitions                             *
 *******************************************************************************
*/


void task_ble (void *args) {
	esp_err_t err = ESP_OK;


	// Initialize the RX queue
	if ((g_ble_rx_queue = xQueueCreate()) == NULL) {
		ERR("Unable to initialize RX queue!");
		goto esc;
	}

	// Initialize the TX queue
	if ((g_ble_tx_queue = xQueueCreate()) == NULL) {
		ERR("Unable to initialize TX queue!");
		goto esc;
	}

	// Run main loop
	do {

		// Wait for trigger


	} while (1);


esc:

	ERR(BLE_TASK_NAME " is suspended due to unexpected circumstances!");

	// Signal system reset
	xEventGroupSetBits(g_signal_group, SIGNAL_TASK_FAULT);
}