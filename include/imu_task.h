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
#include "esp_system.h"
#include "mpu6050.h"
#include "config.h"
#include "errors.h"

/*
 *******************************************************************************
 *                             Symbolic Constants                              *
 *******************************************************************************
*/


// The task name
#define IMU_TASK_NAME                     "IMU_Task"


// The task priority level
#define IMU_TASK_PRIORITY                 


// The task stack size (in words, not bytes; word = 4 bytes for 32-bit system)
#define IMU_TASK_STACK_SIZE                1024


// Whether or not internal pullups should be enabled for SCL and SDA
#define IMU_ENABLE_INTERNAL_PULLUPS        false


// The number of interrupt events the internal xEventQueue can hold
#define IMU_INTERRUPT_QUEUE_SIZE           16


/*
 *******************************************************************************
 *                            Function Declarations                            *
 *******************************************************************************
*/



/*
 * @brief The IMU task. 
 * @param args Generic task parameter pointer
 * @return void
*/
void *task_imu (void *args);



#endif