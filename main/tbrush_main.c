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
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/i2c.h"

#include "imu.h"
#include "errors.h"


#define I2C_SLAVE_ADDR      0x68
        

void app_main (void) {
    esp_err_t err = ESP_OK;

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

    ESP_LOGI("APP", "Attempting to read accelerometer z-axis data!");

    // Initialize I2C
    if ((err = imu_init(21, 22, I2C_SLAVE_ADDR)) != ESP_OK) {
        return;
    }

    // Read the IMU a bit
    imu_data_t data;
    for (int i = 0; i < 1000; ++i) {

        // Issue a request
        if ((err = i2c_request(I2C_SLAVE_ADDR)) != ESP_OK) {
            break;
        }

        ESP_LOGI("APP", "... recieving");

        // Receive back data
        if ((err = i2c_receive(I2C_SLAVE_ADDR, &data)) != ESP_OK) {
            break;
        }

        // Print results (signed/unsigned?)
        printf("az = %d\n", (int16_t)data.az);

        //vTaskDelay(portTICK_PERIOD_MS);
    }


    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
