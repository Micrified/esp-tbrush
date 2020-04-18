#if !defined(SIGNALS_H)
#define SIGNALS_H


/*
 *******************************************************************************
 *                        (C) Copyright 2020 IoT Seminar                       *
 * Created: 01/03/2020                                                         *
 *                                                                             *
 * Programmer(s):                                                              *
 * - Sonnya Dellarosa                                                          *
 * - Charles Randolph                                                          *
 *                                                                             *
 * Description:                                                                *
 *  Holds global signal group                                                  *
 *                                                                             *
 *******************************************************************************
*/


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"


/*
 *******************************************************************************
 *                             Symbolic Constants                              *
 *******************************************************************************
*/


// General signal bits
#define SIGNAL_TASK_FAULT           (1 << 7)

// Bluetooth signal bits
#define SIGNAL_BLE_CONNECTED        (1 << 0)
#define SIGNAL_BLE_DISCONNECTED     (1 << 1)
#define SIGNAL_BLE_MASK             (SIGNAL_BLE_CONNECTED | SIGNAL_BLE_DISCONNECTED)

// IMU signal bits
#define IMU_SIGNAL_RESET            (1 << 2)
#define IMU_SIGNAL_TRAIN_START      (1 << 3)
#define IMU_SIGNAL_MASK           	(IMU_SIGNAL_RESET | IMU_SIGNAL_TRAIN_START)

// CTRL signal bits
#define CTRL_SIGNAL_BTN_TOGGLE      (1 << 4)
#define CTRL_SIGNAL_TRAIN_DONE      (1 << 5)
#define CTRL_SIGNAL_BRUSH_DONE      (1 << 6)
#define CTRL_SIGNAL_MASK            (CTRL_SIGNAL_BTN_TOGGLE | CTRL_SIGNAL_TRAIN_DONE | CTRL_SIGNAL_BRUSH_DONE | SIGNAL_BLE_MASK)


/*
 *******************************************************************************
 *                              Global Variables                               *
 *******************************************************************************
*/

/* INFO: All global data item must be init by main prior to task launch */


// [EXTERN] Global event signal group
extern EventGroupHandle_t g_signal_group;


// [EXTERN] Global UI action queue
extern xQueueHandle g_ui_action_queue;


// [EXTERN] Global processed data queue
extern xQueueHandle g_processed_data_queue;


#endif