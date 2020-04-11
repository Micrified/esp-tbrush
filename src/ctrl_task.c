#include "ctrl_task.h"


/*
 *******************************************************************************
 *                          Internal Global Variables                          *
 *******************************************************************************
*/


// Queue holding incoming data from the BLE driver
static xQueueHandle g_ble_rx_queue = NULL;


// Queue holding data to transmit via the BLE driver
xQueueHandle g_ble_tx_queue = NULL;


/*
 *******************************************************************************
 *                        Internal Function Definitions                        *
 *******************************************************************************
*/


// Writes the given chunk of data to the queue (NON-BLOCKING)
esp_err_t enqueue_msg (xQueueHandle queue, uint8_t *data_p, size_t size) {
	size_t rem = size; off_t offset = 0;
	queue_msg_t msg;
	BaseType_t result = pdPASS;

	// While data remains to be sent and there is space in the queue
	while (rem > 0 && (result == pdPASS)) {

		// Compute message chunk size
		size_t z = CTRL_TASK_QUEUE_MSG_DATA_SIZE;
		if (rem < CTRL_TASK_QUEUE_MSG_DATA_SIZE) {
			z = rem;
		}

		// Build the message
		msg.size = z;
		memcpy(msg.data, data_p + offset, z * sizeof(uint8_t));

		// Push chunk to queue
		result = xQueueSendToBack(queue, (void *)&msg, 
			CTRL_TASK_QUEUE_MAX_TICKS);

		// Update remaining size
		rem -= z;

		// Update offset
		offset += z;

	}

	return (result == pdPASS) ? ESP_OK : ESP_ERR_TIMEOUT;
}

// Callback invoked when data arrives to the buffer
esp_err_t ble_rx_cb (size_t size, void *buffer) {
	esp_err_t err = ESP_OK;

	// Push it to the receival queue for processing
	err = enqueue_msg(g_ble_rx_queue, buffer, size);

	return err;
}


// Callback type invoked on bluetooth state change
esp_err_t ble_state_event_cb(ble_state_event_t state_event) {
	ESP_LOGI("CTRL-TASK", "Change in BLE state!");
	return ESP_OK;
}


static void buzzer_action (int pulses) {
	ui_action_t action = (ui_action_t) {
		.flags = UI_ACTION_BUZZER,
		.duration = 50,
		.periods = 1
	};

	if (xQueueSendToBack(g_ui_action_queue, &action, 0) != pdTRUE) {
		ERR("Could not send to action queue!");
	}
}


/*
 *******************************************************************************
 *                            Function Definitions                             *
 *******************************************************************************
*/


void task_ctrl (void *args) {
	esp_err_t err = ESP_OK;
	ctrl_mode_t state = CTRL_MODE_IDLE;
	queue_msg_t msg;
	uint32_t signals;
	const TickType_t ctrl_block_time = 32;


	// Initialize the RX queue
	if ((g_ble_rx_queue = xQueueCreate(CTRL_TASK_BLE_RX_QUEUE_SIZE, 
		sizeof(queue_msg_t))) == NULL) {
		ERR("Unable to initialize RX queue!");
		goto esc;
	}

	// Initialize the TX queue
	if ((g_ble_tx_queue = xQueueCreate(CTRL_TASK_BLE_TX_QUEUE_SIZE, 
		sizeof(queue_msg_t))) == NULL) {
		ERR("Unable to initialize TX queue!");
		goto esc;
	}

    // Start up BLE
    if ((err = ble_init(ble_rx_cb, ble_state_event_cb)) != ESP_OK) {
    	ERR("Unable to initialize BLE!");
    	goto esc;
    }

	// Run main loop
	do {

		// Wait for trigger
		signals = xEventGroupWaitBits(g_signal_group, CTRL_SIGNAL_MASK,
			pdTRUE, pdFALSE, ctrl_block_time);


		// Check if there is data in the BLE RX queue
		if ((signals & CTRL_SIGNAL_BLE_RX) != 0) {
			ESP_LOGI("CTRL-TASK", "No message processing implemented!");

			// Discard all enqueued data
			while (xQueueReceive(g_ble_rx_queue, &msg, 0) == pdPASS);
		}

		// Check if there was a button toggle
		if ((signals & CTRL_SIGNAL_BTN_TOGGLE) != 0) {
			if (state == CTRL_MODE_IDLE) {
				// Switching to live
				buzzer_action(1);
			} else {
				// Switching to idle
				buzzer_action(2);
			}
			state = (1 - state);
		}

		// Send everything in the BLE TX queue
		while (xQueueReceive(g_ble_tx_queue, &msg, 0) == pdPASS) {
			ble_send(msg.size, msg.data);
		}

		// If in idle mode do nothing
		if (state == CTRL_MODE_IDLE) {
			continue;
		}

		// Otherwise do something (probably some light data processing?)



	} while (1);


esc:

	ERR(CTRL_TASK_NAME " is suspended due to unexpected circumstances!");

	// Signal system reset
	xEventGroupSetBits(g_signal_group, SIGNAL_TASK_FAULT);
}