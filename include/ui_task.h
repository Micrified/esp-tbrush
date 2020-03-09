#if !defined(UI_TASK_H)
#define UI_TASK_H

/*
 *******************************************************************************
 *                        (C) Copyright 2020 IoT Seminar                       *
 * Created: 09/03/2020                                                         *
 *                                                                             *
 * Programmer(s):                                                              *
 * - Sonnya Dellarosa                                                          *
 * - Charles Randolph                                                          *
 *                                                                             *
 * Description:                                                                *
 *  Task that controls user-facing devices for giving feedback. Callable from  *
 *  other tasks                                                                *
 *                                                                             *
 *******************************************************************************
*/


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "config.h"
#include "errors.h"
#include "signals.h"


/*
 *******************************************************************************
 *                             Symbolic Constants                              *
 *******************************************************************************
*/


// The task name
#define UI_TASK_NAME              "UI_Task"


// The task priority level
#define UI_TASK_PRIORITY           tskIDLE_PRIORITY


// The task stack size (in bytes - not like traditional FreeRTOS)
#define UI_TASK_STACK_SIZE          2048


// The task poll period (in ms)
#define UI_TASK_POLL_PERIOD         200


// The size of the UI queue
#define UI_ACTION_QUEUE_SIZE        16


// Bit-flag for the buzzer action
#define UI_ACTION_BUZZER            (1 << 0)


// Bit-flag for the vibrating motor
#define UI_ACTION_VIBRATION          (1 << 1)


/*
 *******************************************************************************
 *                              Type Definitions                               *
 *******************************************************************************
*/


// Type describing a UI action 
typedef struct {
	uint8_t flags;              // Actions to perform
	uint16_t duration;          // Duration of the action (ms)
} ui_action_t;


/*
 *******************************************************************************
 *                            Function Declarations                            *
 *******************************************************************************
*/


/*\
 * @brief the UI task
 * @param args Generic task parameter pointer
 * @return void
\*/
void task_ui (void *args);


#endif