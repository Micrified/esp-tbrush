#if !defined(IMU_TASK_H)
#define IMU_TASK_H

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
 *  IMU interface task                                                         *
 *                                                                             *
 *******************************************************************************
*/


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "mpu6050.h"
#include "config.h"
#include "errors.h"
#include "signals.h"
#include "ui_task.h"

#include "msg.h"

/*
 *******************************************************************************
 *                             Symbolic Constants                              *
 *******************************************************************************
*/


// The task name
#define IMU_TASK_NAME                     "IMU_Task"


// The task priority level
#define IMU_TASK_PRIORITY                  tskIDLE_PRIORITY                


// The task stack size (in bytes - not words like traditional FreeRTOS)
#define IMU_TASK_STACK_SIZE                4096


// Whether or not internal pullups should be enabled for SCL and SDA
#define IMU_ENABLE_INTERNAL_PULLUPS        false


// The number of interrupt events the internal xEventQueue can hold
#define IMU_INTERRUPT_QUEUE_SIZE           16


// The number of samples to average when calibrating the system
#define IMU_CALIBRATION_SAMPLE_SIZE        100


// Mode 1: Calibration
#define IMU_TASK_MODE_CALIBRATION          (1 << 0)


// Number of elements in the processed data queue
#define IMU_PROCESSED_DATA_QUEUE_SIZE		64


// Number of samples for training a section
#define IMU_TRAINING_SAMPLE_BUF_SIZE        150


/*
 *******************************************************************************
 *                              Type Definitions                               *
 *******************************************************************************
*/


// Enumeration of IMU modes
typedef enum {
	IMU_MODE_IDLE,
	IMU_MODE_CALIBRATION,
	IMU_MODE_TRAIN
} imu_mode_t;


// Type describing processed data
typedef struct {
	uint8_t rate;       // Brushing rate (in Hz)
	brush_zone_t zone;  // Brushing zone
} imu_proc_data_t;


/*
 *******************************************************************************
 *                            Function Declarations                            *
 *******************************************************************************
*/


/*\
 * @brief The IMU task. 
 * @param args Generic task parameter pointer
 * @return void
\*/
void task_imu (void *args);


#endif