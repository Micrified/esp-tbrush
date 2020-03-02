/*
 *******************************************************************************
 *                        (C) Copyright 2020 IoT Seminar                       *
 * Created: 20/02/2020                                                         *
 *                                                                             *
 * Programmer(s):                                                              *
 * - Sonnya Dellarosa                                                          *
 * - Charles Randolph                                                          *
 *                                                                             *
 * Description:                                                                *
 *  Main file for the IoT Toothbrush Accessory                                 *
 *                                                                             *
 *******************************************************************************
*/


#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "mpu6050.h"
#include "errors.h"
#include "imu_task.h"
#include "signals.h"
#include "config.h"


/*
 *******************************************************************************
 *                             Symbolic Constants                              *
 *******************************************************************************
*/


// CPU cores
#define APPLICATION_CORE          1
#define PROTOCOL_CORE             0


/*
 *******************************************************************************
 *                              Global Variables                               *
 *******************************************************************************
*/


// Global signal event group
EventGroupHandle_t g_signal_group;


/*
 *******************************************************************************
 *                            Function Definitions                             *
 *******************************************************************************
*/


void app_main (void) {
    esp_err_t err = ESP_OK;
    TaskHandle_t imu_task_handle = NULL;
    TaskHandle_t ble_task_handle = NULL;

    /*\  
     *   Cores (2)
     *   #0: Protocol CPU
     *   #1: Application CPU
    \*/

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");


    // Create the signal event group
    if ((g_signal_group = xEventGroupCreate()) == NULL) {
        ERR("Insufficient heap memory available to create event-group!");
        goto reboot;
    } else {
        ESP_LOGI("Startup", "Signal group initialized");
    }

    // Create the IMU task pinned to core
    if (xTaskCreatePinnedToCore(task_imu, IMU_TASK_NAME, IMU_TASK_STACK_SIZE, NULL,
        IMU_TASK_PRIORITY, &imu_task_handle, APPLICATION_CORE) != pdPASS) {
        vTaskDelete(imu_task_handle);
        ERR("Unable to create task: " IMU_TASK_NAME);
        goto reboot;
    } else {
        ESP_LOGI("Startup", "Launched " IMU_TASK_NAME);
    }


    // Log start
    ESP_LOGI("Startup", "Ready");
    return;


reboot:

    fflush(stdout);
    esp_restart();
}
