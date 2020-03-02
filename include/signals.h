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


/*
 *******************************************************************************
 *                              Global Variables                               *
 *******************************************************************************
*/


// [EXTERN] Global event signal group
extern EventGroupHandle_t g_signal_group;


#endif