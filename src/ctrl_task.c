#include "ctrl_task.h"


/*
 *******************************************************************************
 *                          Internal Global Variables                          *
 *******************************************************************************
*/


// Queue holding incoming data from the BLE driver
static xQueueHandle g_ble_rx_queue = NULL;


// Queue holding data to transmit via the BLE driver
static xQueueHandle g_ble_tx_queue = NULL;


// Handle for the active mode timer
TimerHandle_t g_active_mode_timer = NULL;


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
	ESP_LOGI(CTRL_TASK_NAME, "ble_rx_cb()");

	// Push it to the receival queue for processing
	err = enqueue_msg(g_ble_rx_queue, buffer, size);

	return err;
}


// Callback type invoked on bluetooth state change
esp_err_t ble_state_event_cb(ble_state_event_t state_event) {
	EventBits_t bits = (state_event == BLE_STATE_EVENT_CONNECTED ? 
		SIGNAL_BLE_CONNECTED : SIGNAL_BLE_DISCONNECTED);

	// Signal all tasks with the new connection status
	xEventGroupSetBits(g_signal_group, bits);

	return ESP_OK;
}


// Creates and enqueues an action sounding the buzzer
static void buzzer_action (uint8_t repeats, uint16_t duration) {
	ui_action_t action = (ui_action_t) {
		.flags = UI_ACTION_BUZZER,
		.duration = duration,
		.periods = repeats
	};

	if (xQueueSendToBack(g_ui_action_queue, &action, 0) != pdTRUE) {
		ERR("Could not send to action queue!");
	}
}


// Procedure processing all BLE received messages
void process_rx_queue () {
	esp_err_t err;                          // Error tracking
	queue_msg_t queue_msg;                  // Received queue message data
	msg_data_t *rx_msg_data_p;              // Received (packed) message data
	msg_t rx_msg;                           // Received (unpacked) message
	off_t offset = 0;

	// While there are messages in the receival queue
	while (xQueueReceive(g_ble_rx_queue, &queue_msg, 0) == pdPASS) {

		// Call the state machine until no data remains to be parsed
		do {
			rx_msg_data_p = msg_parse_fsm(queue_msg.size - offset, 
				queue_msg.data + offset, &offset);

			// If a message was processed
			if (rx_msg_data_p != NULL) {

				// Unmarshall the message
				if ((err = msg_unpack(rx_msg_data_p, &rx_msg)) != ESP_OK) {
					ESP_LOGE(CTRL_TASK_NAME, "Error unpacking rx message!");
				} else {
					ESP_LOGI(CTRL_TASK_NAME, "Rx {.type = %d}", rx_msg.type);
				}
			}

		} while (offset < queue_msg.size);
	}
}


// Procedure processing all BLE transmission messages
void process_tx_queue (int isConnected) {
	esp_err_t err;
	queue_msg_t queue_msg;

	// While there exists tx queue data to transmit
	while (xQueueReceive(g_ble_tx_queue, &queue_msg, 0) == pdPASS) {

		// Dump messages if they can't be sent to keep queue free
		if (isConnected == 0) {
			continue;
		}

		// Otherwise transmit them
		if ((err = ble_send(queue_msg.size, queue_msg.data)) != ESP_OK) {
			ESP_LOGE(CTRL_TASK_NAME, "Unable to send data (check network)!");
		}
	}
}


// Procedure processing all processed data messages
void process_data_queue (brush_mode_t mode) {
	esp_err_t err;
	imu_proc_data_t proc_data;
	msg_t msg;
	uint8_t buffer[sizeof(msg_t)];

	// While there exists processed data to transmit
	while (xQueueReceive(g_processed_data_queue, &proc_data, 0) == pdPASS) {

		// Configure the message to send
		msg.type = MESSAGE_TYPE_STATUS;
		msg.data.status.mode = mode;
		msg.data.status.zone = proc_data.zone;
		msg.data.status.rate = proc_data.rate;
		msg.data.status.progress = 5;

		// Serialize the message
		size_t tx_size = msg_pack(&msg, buffer);

		// Write the message to the transmission queue
		if ((err = enqueue_msg(g_ble_tx_queue, buffer, tx_size)) != ESP_OK) {
			ESP_LOGE(CTRL_TASK_NAME, "Unable to enqueue msg to tx queue!");
		}
	}
}




/*
 *******************************************************************************
 *                            Function Definitions                             *
 *******************************************************************************
*/


void task_ctrl (void *args) {
	esp_err_t err = ESP_OK;                 // Error tracking
	ctrl_mode_t mode = CTRL_MODE_IDLE;     // Current control mode
	uint32_t signals;                       // Signal group flags
	const TickType_t ctrl_block_time = 32;  // Poll time before control loop
	int isConnected = 0;                    // Connected flag


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


		// Update connection status
		if (signals & SIGNAL_BLE_CONNECTED) {
			isConnected = 1;
		}
		if (signals & SIGNAL_BLE_DISCONNECTED) {
			isConnected = 0;
		}

		// Check if brushing is complete
		if ((signals & CTRL_SIGNAL_BRUSH_DONE) != 0) {

			ESP_LOGW(CTRL_TASK_NAME, "Brushing finished!");

			// Dispatch report if connected
			ESP_LOGW(CTRL_TASK_NAME, "Whhoo whee sent report!!");

			// Set mode back to idle
			mode = CTRL_MODE_IDLE;
		}

		// Check if training is complete
		if ((signals & CTRL_SIGNAL_TRAIN_DONE) != 0) {
			ESP_LOGW(CTRL_TASK_NAME, "Training signalled finished!");

			// Update mode
			mode = CTRL_MODE_BRUSH;
		}

		// Check if there was a button toggle
		if ((signals & CTRL_SIGNAL_BTN_TOGGLE) != 0) {
			if (mode > CTRL_MODE_IDLE) {

				// Log reset request
				ESP_LOGW(CTRL_TASK_NAME, "Resetting");

				// Signal the other services
				xEventGroupSetBits(g_signal_group, IMU_SIGNAL_RESET);

				// Update mode
				mode = CTRL_MODE_IDLE;
			} else {

				// Signal to begin training
				xEventGroupSetBits(g_signal_group, IMU_SIGNAL_TRAIN_START);

				// Update mode
				mode = CTRL_MODE_TRAIN;
			}
		}

		// Process everything in the recieve message queue if not busy
		if (mode == CTRL_MODE_IDLE) {
			process_rx_queue();
		}

		// Process everything in the processed data queue
		process_data_queue(mode);

		// Process everything in the transmit message queue queue
		process_tx_queue(isConnected);

	} while (1);

esc:

	ERR(CTRL_TASK_NAME " is suspended due to unexpected circumstances!");

	// Signal system reset
	xEventGroupSetBits(g_signal_group, SIGNAL_TASK_FAULT);
}