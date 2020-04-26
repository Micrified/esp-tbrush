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


// The results for a brushing session (4 buckets allowing x/400)
static uint16_t g_results[4][4];


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
	mpu6050_data_t data;
	uint8_t buffer[7] = {MSG_START_BYTE, MSG_START_BYTE, MESSAGE_TYPE_STATUS, 
		0x0, 0x0, 0x0, 0x0}; // Mode, Zone, Rate, Progress
	static uint16_t n = 0;

	// If receiving train mode, then zero out the counter in prep for brush mode
	if (n == CTRL_MODE_TRAIN) {
		n = 0;
	}

	// While there exists processed data to transmit
	while (xQueueReceive(g_raw_data_queue, &data, 0) == pdPASS) {

		// Classify the sample
		brush_zone_t zone = classify_rt(&data);

		// Update results
		g_results[n / IMU_BRUSHING_SAMPLE_SIZE][zone]++;

		// Increment the message count 
		n++;

		// Consider only every 4th sample
		if ((n % 4) != 0) {
			continue;
		}

		// Prepare the message
		buffer[3] = mode;
		buffer[4] = zone;
		buffer[6] = n / ((IMU_BRUSHING_SAMPLE_SIZE * 4) / 100);

		// Enqueue the message
		enqueue_msg(g_ble_tx_queue, buffer, 7);
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

	uint8_t result_buffer[12] = {MSG_START_BYTE, MSG_START_BYTE, MESSAGE_TYPE_RESULT,
		0x0,                // ID
		0x0, 0x0, 0x0, 0x0, // P_ll, R_ll, P_lr, R_lr
		0x0, 0x0, 0x0, 0x0  // P_tl, R_tl, P_tr, R_tr
	};

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

		// Check if brushing is complete
		if ((signals & CTRL_SIGNAL_BRUSH_DONE) != 0) {

			ESP_LOGW(CTRL_TASK_NAME, "Brushing finished!");

			// Display results
			printf("Zone 0 :: %d / %d\n", g_results[0][0], IMU_BRUSHING_SAMPLE_SIZE);
			printf("Zone 1 :: %d / %d\n", g_results[1][1], IMU_BRUSHING_SAMPLE_SIZE);
			printf("Zone 2 :: %d / %d\n", g_results[2][2], IMU_BRUSHING_SAMPLE_SIZE);
			printf("Zone 3 :: %d / %d\n", g_results[3][3], IMU_BRUSHING_SAMPLE_SIZE);

			// Configure the results message
			result_buffer[4]  = g_results[0][0] / (IMU_BRUSHING_SAMPLE_SIZE / 100);
			result_buffer[6]  = g_results[1][1] / (IMU_BRUSHING_SAMPLE_SIZE / 100);
			result_buffer[8]  = g_results[2][2] / (IMU_BRUSHING_SAMPLE_SIZE / 100);
			result_buffer[10] = g_results[3][3] / (IMU_BRUSHING_SAMPLE_SIZE / 100);

			//
			enqueue_msg(g_ble_tx_queue, result_buffer, 12);

			// Print the training data
			//display_training_data();

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