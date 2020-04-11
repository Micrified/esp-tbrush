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


// IMU signal bits
#define IMU_SIGNAL_CALIBRATE        (1 << 0)
#define IMU_SIGNAL_MASK             IMU_SIGNAL_CALIBRATE

// CTRL signal bits
#define CTRL_SIGNAL_BLE_RX          (1 << 1)
#define CTRL_SIGNAL_BTN_TOGGLE      (1 << 2)
#define CTRL_SIGNAL_MASK            (CTRL_SIGNAL_BLE_RX | CTRL_SIGNAL_BTN_TOGGLE)


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


// [EXTERN] Global TX queue
extern xQueueHandle g_ble_tx_queue;


#endif