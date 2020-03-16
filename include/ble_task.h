#if !defined(BLE_TASK_H)
#define BLE_TASK_H

/*
 *******************************************************************************
 *                        (C) Copyright 2020 IoT Seminar                       *
 * Created: 16/03/2020                                                         *
 *                                                                             *
 * Programmer(s):                                                              *
 * - Sonnya Dellarosa                                                          *
 * - Charles Randolph                                                          *
 *                                                                             *
 * Description:                                                                *
 *  Task that communicates with the Bluetooth driver, and handles communicatio *
 *  ns                                                                         *
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


/*
 *******************************************************************************
 *                             Symbolic Constants                              *
 *******************************************************************************
*/


// The task name
#define BLE_TASK_NAME             "BLE_Task"


// The task priority level
#define BLE_TASK_PRIORITY         tskIDLE_PRIORITY


// The task stack size (in bytes - not like traditional FreeRTOS)
#define BLE_TASK_STACK_SIZE       4096


/*
 *******************************************************************************
 *                              Type Definitions                               *
 *******************************************************************************
*/



/*
 *******************************************************************************
 *                            Function Declarations                            *
 *******************************************************************************
*/


/*\
 * @brief The BLE task
 * @param args Generic task parameter pointer
 * @return void
\*/
void task_ble (void *args);


#endif