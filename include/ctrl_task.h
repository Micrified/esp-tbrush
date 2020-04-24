#if !defined(CTRL_TASK_H)
#define CTRL_TASK_H

/*
 *******************************************************************************
 *                        (C) Copyright 2020 IoT Seminar                       *
 * Created: 18/03/2020                                                         *
 *                                                                             *
 * Programmer(s):                                                              *
 * - Sonnya Dellarosa                                                          *
 * - Charles Randolph                                                          *
 *                                                                             *
 * Description:                                                                *
 *  The control task interacts with the Bluetooth driver wrapper, parses instr *
 *  uctions, and adjusts system state                                          *
 *                                                                             *
 *******************************************************************************
*/


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "config.h"
#include "errors.h"
#include "signals.h"
#include "ble.h"
#include "msg.h"
#include "classifier.h"

#include "imu_task.h"
#include "ui_task.h"



/*
 *******************************************************************************
 *                             Symbolic Constants                              *
 *******************************************************************************
*/


// The task name
#define CTRL_TASK_NAME                    "CTRL_Task"


// The task priority level
#define CTRL_TASK_PRIORITY                tskIDLE_PRIORITY


// The task stack size (in bytes - not like traditional FreeRTOS)
#define CTRL_TASK_STACK_SIZE              4096


// The number of elements the internal RX queue can contain
#define CTRL_TASK_BLE_RX_QUEUE_SIZE       16


// The number of elements the internal TX queue can contain
#define CTRL_TASK_BLE_TX_QUEUE_SIZE       16


// The number of bytes an internal queue message can hold
#define CTRL_TASK_QUEUE_MSG_DATA_SIZE     32


// The number of ticks to wait when performing a queue operation
#define CTRL_TASK_QUEUE_MAX_TICKS         16


/*
 *******************************************************************************
 *                              Type Definitions                               *
 *******************************************************************************
*/


// Structure describing an internal queue message
typedef struct {
	size_t size;
	uint8_t data[CTRL_TASK_QUEUE_MSG_DATA_SIZE];
} queue_msg_t;


// State machine enumeration for the modes
typedef enum {
	CTRL_MODE_IDLE = 0,
	CTRL_MODE_TRAIN,
	CTRL_MODE_BRUSH
} ctrl_mode_t;


/*
 *******************************************************************************
 *                            Function Declarations                            *
 *******************************************************************************
*/


/*\
 * @brief The control task
 * @param args Generic task parameter pointer
 * @return void
\*/
void task_ctrl (void *args);


#endif