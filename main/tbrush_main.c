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
#include "../include/imu.h"
#include "errors.h"


#define I2C_MASTER_PORT         

void app_main (void) {
    esp_err_t err = ESP_OK;

    // // Prepare I2C configuration for the driver
    // i2c_config_t cfg = (i2c_config_t) {
    //     .mode          = I2C_MODE_MASTER,
    //     .sda_io_num    = 42,
    //     .scl_io_num    = 39,
    //     .sda_pullup_en = true,
    //     .scl_pullup_en = true,
    //     {
    //         .master = {
    //             .clk_speed = APB_CLK_FREQ
    //         }
    //     }
    // };

    // // Configure I2C
    // if ((err = i2c_param_config()) != ESP_OK) {
    //     ESP_LOGE();
    // }

    ERR("Oh no!");

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

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
