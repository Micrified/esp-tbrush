#include "msg.h"


/*
 *******************************************************************************
 *                          Internal Type Definitions                          *
 *******************************************************************************
*/


// States of the message processing FSM
typedef enum {
	FSM_STATE_START = 0,    // Must accept marker byte 1
	FSM_STATE_1,            // Must accept marker byte 2
	FSM_STATE_2,            // Must accept the type byte
	FSM_STATE_3,            // Must accept N bytes associated with type
	FSM_STATE_END           // Done - Reset and return data
} msg_fsm_state_t;


/*
 *******************************************************************************
 *                              Global Variables                               *
 *******************************************************************************
*/


// The message data (payload) sizes by type
static size_t g_msg_data_size_lookup_table[MESSAGE_TYPE_MAX] = {
	[MESSAGE_TYPE_STATUS]  = 4,
	[MESSAGE_TYPE_RESULT]  = 9,
	[MESSAGE_TYPE_REQUEST] = 0,
};


/*
 *******************************************************************************
 *                    Internal Packing Function Definitions                    *
 *******************************************************************************
*/


static size_t msg_pack_status (uint8_t *buffer_p, msg_t *msg_p) {
	off_t offset = 0;
	data_status_t d = msg_p->data.status;

	// Pack status fields
	buffer_p[offset++] = d.mode;
	buffer_p[offset++] = d.zone;
	buffer_p[offset++] = d.rate;
	buffer_p[offset++] = d.progress;

	return g_msg_data_size_lookup_table[msg_p->type];
}


static size_t msg_pack_result (uint8_t *buffer_p, msg_t *msg_p) {
	off_t offset = 0;
	data_result_t d = msg_p->data.result;

	// Pack result fields
	buffer_p[offset++] = d.id;
	buffer_p[offset++] = d.progress_zone_ll;
	buffer_p[offset++] = d.rate_zone_ll;
	buffer_p[offset++] = d.progress_zone_lr;
	buffer_p[offset++] = d.rate_zone_lr;
	buffer_p[offset++] = d.progress_zone_tl;
	buffer_p[offset++] = d.rate_zone_tl;
	buffer_p[offset++] = d.progress_zone_tr;
	buffer_p[offset++] = d.rate_zone_tr;

	return g_msg_data_size_lookup_table[msg_p->type];
}


static size_t msg_pack_request (uint8_t *buffer_p, msg_t *msg_p) {
	return 0;
}


/*
 *******************************************************************************
 *                   Internal Unpacking Function Definitions                   *
 *******************************************************************************
*/


// Unpack status (unimplemented on this end)
esp_err_t msg_unpack_status (uint8_t *buffer_p, msg_t *msg_p) {
	ESP_LOGE("msg", "Unpack status unimplemented!");
	return ESP_OK;
}


// Unpack result (unimplemented on this end)
esp_err_t msg_unpack_result (uint8_t *buffer_p, msg_t *msg_p) {
	ESP_LOGE("msg", "Unpack result unimplemented!");
	return ESP_OK;
}

// Unpack a request (no data, it's just the type)
esp_err_t msg_unpack_request (uint8_t *buffer_p, msg_t *msg_p) {
	return ESP_OK;
}




/*
 *******************************************************************************
 *                        External Function Definitions                        *
 *******************************************************************************
*/


/*\
 * @brief           Marshalls a message to a sequence of bytes for transmission
 * @param msg_p     Pointer to the message structure
 * @param buffer_p  Pointer to the buffer in which message will be marshalled
 * @return          The number of bytes written to the buffer
\*/
size_t msg_pack (msg_t *msg_p, uint8_t *buffer_p) {
	size_t len = 0;
	if (msg_p == NULL || buffer_p == NULL) {
		return 0;
	}

	// Insert first two marker bytes
	buffer_p[len++] = MSG_START_BYTE;
	buffer_p[len++] = MSG_START_BYTE;

	// Insert the type byte
	buffer_p[len++] = msg_p->type;

	// Pack the contents
	switch (msg_p->type) {
		case MESSAGE_TYPE_STATUS: {
			len += msg_pack_status(buffer_p + len, msg_p);
		}
		break;
		case MESSAGE_TYPE_RESULT: {
			len += msg_pack_result(buffer_p + len, msg_p);
		}
		break;
		case MESSAGE_TYPE_REQUEST: {
			len += msg_pack_request(buffer_p + len, msg_p);
		}
		break;
		default:
			ESP_LOGE("MSG", "Unknown type byte!!!");
			len = 0;
	}

	return len;
}


/*\
 * @brief           State machine processing messages out of a buffer
 * @note            This function persists state between calls
 * @param size      Number of bytes to process
 * @param buffer_p  Pointer to the buffer of size 'size' to process
 * @param off_p     Pointer to size_t variable holding number of bytes
 *                  processed
 * @return          NULL if no message, otherwise pointer to message datas
 *                  [Copy or use this message before calling again]
\*/
msg_data_t *msg_parse_fsm (size_t size, uint8_t *buffer_p, off_t *off_p) {

	// [Persisting] Initial state (expecting markers)
	static msg_fsm_state_t state = FSM_STATE_START;

	// [Persisting] Message data buffer
	static msg_data_t msg;

	// [Persisting] Message data left to read
	static size_t remaining = 0;

	// [Persistent] Message data pointer offset
	static size_t data_offset = 0;

	// Reset offset for processing new buffer
	off_t offset = 0;

	// While there remain bytes to process
	while (offset < size) {

		// Obtain the next byte
		uint8_t b = buffer_p[offset];

		switch (state) {
			case FSM_STATE_START: {
				if (b == MSG_START_BYTE) {
					state++;
				}
			}
			break;

			case FSM_STATE_1: {
				if (b == MSG_START_BYTE) {
					state++;
				}
			}
			break;

			case FSM_STATE_2: {
				if (b < MESSAGE_TYPE_MAX) {
					msg.type = b;

					// Set how many bytes need to be eaten
					remaining = g_msg_data_size_lookup_table[msg.type];
					state++;

					// Edge case: zero bytes in the payload
					if (remaining == 0) {
						state++;
					}
				}
			}
			break;

			case FSM_STATE_3: {
				if (remaining == 0) {
					state++;
					goto end;
				} else {
					msg.data[data_offset++] = b;
				}
				--remaining;
			}
			break;

			case FSM_STATE_END: {
				// Unimplemented - should jump out in state 3
			}
			break;
		}

		// Increment offset
		offset++;
	}

end:
	// It's possible we processed a message but the buffer contains more still
	if (off_p != NULL) {
		*off_p = offset;
	}

	// If reached max state then reset variables
	if (state == FSM_STATE_END) {
		remaining = 0;
		data_offset = 0;
		state = FSM_STATE_START;

		return &msg;
	}

	return NULL;
}


/*\
 * @brief             Unpacks message data structure to message type
 * @param msg_data_p  Message data structure to unpack
 * @param msg_p       Message structure to which fields will be set
 * @return            ESP_OK on success; ESP_ERR_INVALID_ARG on bad
 *                    parameters
\*/
esp_err_t msg_unpack (msg_data_t *msg_data_p, msg_t *msg_p) {
	esp_err_t err = ESP_OK;
	msg_t msg = {0};

	// Check arguments
	if (msg_data_p == NULL || msg_p == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	// Set type
	msg.type = msg_data_p->type;

	// Unpack depending on the message type
	switch (msg.type) {
		case MESSAGE_TYPE_STATUS: {
			err = msg_unpack_status(msg_data_p->data, &msg);
		}
		break;

		case MESSAGE_TYPE_RESULT: {
			err = msg_unpack_result(msg_data_p->data, &msg);
		}
		break;

		case MESSAGE_TYPE_REQUEST: {
			err = msg_unpack_request(msg_data_p->data, &msg);
		}
		break;

		default:
			ESP_LOGE("msg", "Cannot unpack unknown message type (%d)", msg.type);
	}

	// If no error, copy data structure to pointer location
	if (err == ESP_OK) {
		*msg_p = msg;
	}

	return err;
}