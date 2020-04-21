#if !defined(MSG_H)
#define MSG_H

/*
 *******************************************************************************
 *                        (C) Copyright 2020 IoT Seminar                       *
 * Created: 14/04/2020                                                         *
 *                                                                             *
 * Programmer(s):                                                              *
 * - Charles Randolph                                                          *
 * - Sonnya Dellarosa                                                          *
 *                                                                             *
 * Description:                                                                *
 *  Contains the data structures for networked messages, along with their seri *
 *  alization/deserialization functions                                        *
 *                                                                             *
 *******************************************************************************
*/


#include "esp_system.h"
#include "esp_log.h"
#include "config.h"


/*
 *******************************************************************************
 *                             Symbolic Constants                              *
 *******************************************************************************
*/


#define MSG_START_BYTE    0xFF


/*
 *******************************************************************************
 *                              Type Definitions                               *
 *******************************************************************************
*/


// Enumeration of possible message types
typedef enum {
	MESSAGE_TYPE_STATUS,     // Status message (streamed while brushing)
	MESSAGE_TYPE_RESULT,     // Result message (sent at end of session)
	MESSAGE_TYPE_REQUEST,    // Request message (ask to resent result)

	MESSAGE_TYPE_MAX
} msg_type_t;


// Enumeration of device modes
typedef enum {
	MODE_IDLE = 0,
	MODE_TRAIN,
	MODE_BRUSH,

	MODE_MAX
} brush_mode_t;


// Enumeration of brushing zones
typedef enum {
	BRUSH_ZONE_LL,
	BRUSH_ZONE_LR,
	BRUSH_ZONE_TL,
	BRUSH_ZONE_TR,

	BRUSH_MODE_MAX
} brush_zone_t;


// Structure describing status message data
typedef struct {
	brush_mode_t mode;        // Mode that toothbrush is in
	brush_zone_t zone;        // Zone being brushed (only set in mode brush)
	uint8_t      rate;        // Rate (Hz) at which brushing is occurring
	uint8_t      progress;    // Progress (0-100) of brushing session
} data_status_t;


// Structure describing session result data
typedef struct {
	uint8_t      id;                // Session identifier
	uint8_t      progress_zone_ll;  // Progress (0-100) of specific zone
	uint8_t      rate_zone_ll;      // Rate at which brushing occurred for zone
	uint8_t      progress_zone_lr;
	uint8_t      rate_zone_lr;
	uint8_t      progress_zone_tl;
	uint8_t      rate_zone_tl;
	uint8_t      progress_zone_tr;
	uint8_t      rate_zone_tr;
} data_result_t;


// Stucture describing a message
typedef struct {
	msg_type_t type;
	union {
		data_status_t status;
		data_result_t result;
	} data;
} msg_t;


// Structure describing packed message data (type labeled)
typedef struct {
	msg_type_t type;
	uint8_t data[sizeof(msg_t)];
} msg_data_t;


/*
 *******************************************************************************
 *                            Function Declarations                            *
 *******************************************************************************
*/


/*\
 * @brief           Marshalls a message to a sequence of bytes for transmission
 * @param msg_p     Pointer to the message structure
 * @param buffer_p  Pointer to the buffer in which message will be marshalled
 * @return          The number of bytes written to the buffer
\*/
size_t msg_pack (msg_t *msg_p, uint8_t *buffer_p);


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
msg_data_t *msg_parse_fsm (size_t size, uint8_t *buffer_p, off_t *off_p);


/*\
 * @brief             Unpacks message data structure to message type
 * @param msg_data_p  Message data structure to unpack
 * @param msg_p       Message structure to which fields will be set
 * @return            ESP_OK on success; ESP_ERR_INVALID_ARG on bad
 *                    parameters
\*/
esp_err_t msg_unpack (msg_data_t *msg_data_p, msg_t *msg_p);


#endif