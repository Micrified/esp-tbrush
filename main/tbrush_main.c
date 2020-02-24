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
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "imu.h"
#include "errors.h"

/*
 *******************************************************************************
 *                             Symbolic Constants                              *
 *******************************************************************************
*/


// SDA pin for I2C bus
#define I2C_SDA_PIN               21

// SCL pin for I2C bus
#define I2C_SCL_PIN               22

// Address of the IMU device on the I2C bus
#define I2C_SLAVE_ADDR            0x68

// GPIO pin configured for the interrupt
#define I2C_INTR_PIN              15

// Interrupt priority (default)
#define ESP_INTR_FLAG_DEFAULT     0


/*
 *******************************************************************************
 *                              Global Variables                               *
 *******************************************************************************
*/


// Event queue holding events from the GPIO interrupt
static xQueueHandle g_gpio_event_queue = NULL;


/*
 *******************************************************************************
 *                            Function Definitions                             *
 *******************************************************************************
*/


// Interrupt handler
static void IRAM_ATTR gpio_isr_handler (void *arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(g_gpio_event_queue, &gpio_num, NULL);
}


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


    /********************** EVENT-QUEUE CONFIGURATION ***********************/


    if ((g_gpio_event_queue = xQueueCreate(10, sizeof(uint32_t))) == NULL) {
        ERR("Insufficient memory to create queue!");
        goto esc;
    }


    /************************* GPIO CONFIGURATION ***************************/


    // Configure GPIO to allow interrupts
    uint64_t pin_bit_mask = ((uint64_t)1) << (uint64_t)(I2C_INTR_PIN);
    gpio_config_t gpio_cfg = (gpio_config_t) {
        .pin_bit_mask    = pin_bit_mask,
        .mode            = GPIO_MODE_INPUT,
        .pull_up_en      = GPIO_PULLUP_DISABLE,
        .pull_down_en    = GPIO_PULLDOWN_ENABLE,
        .intr_type       = GPIO_INTR_ANYEDGE 
    };

    // Apply GPIO configuration
    if ((err = gpio_config(&gpio_cfg)) != ESP_OK) {
        ERR("Couldn't configure GPIO!");
        goto esc;
    }

    // Install GPIO interrupt service 
    if ((err = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT)) != ESP_OK) {
        if (err == ESP_ERR_NO_MEM) {
            ERR("No memory available to install service!");
        } else if (err == ESP_ERR_INVALID_STATE) {
            ERR("Service is already installed!");
        } else if (err == ESP_ERR_NOT_FOUND) {
            ERR("No free interrupt found with specified flags!");
        } else if (err == ESP_ERR_INVALID_ARG) {
            ERR("GPIO error!");
        } else {
            ERR("Unknown error!");
        }
        goto esc;
    }

    // Attach an ISR handler for a specific pin
    if ((err = gpio_isr_handler_add(I2C_INTR_PIN, gpio_isr_handler, 
        (void *)I2C_INTR_PIN) // Ugly hack to pass PIN as "pointer"
        ) != ESP_OK) {
        if (err == ESP_ERR_INVALID_STATE) {
            ERR("Must initialize ISR service: gpio_install_isr_service()!");
        } else if (err == ESP_ERR_INVALID_ARG) {
            ERR("Bad parameter!");
        } else {
            ERR("Invalid error type!");
        }
        goto esc;
    }


    /************************** I2C CONFIGURATION ***************************/


    // Initialize I2C
    if ((err = imu_init(I2C_SDA_PIN, I2C_SCL_PIN, I2C_SLAVE_ADDR)) != ESP_OK) {
        goto esc;
    }

    // Set IMU to not sleep
    if ((err = imu_set_mode(I2C_SLAVE_ADDR, false)) != ESP_OK) {
        goto esc;
    }

    // Configure accelerometer sensitivity
    if ((err = imu_cfg_accelerometer(I2C_SLAVE_ADDR, 
        ACCEL_CFG_RANGE_8G)) != ESP_OK) {
        goto esc;
    }

    // Configure gyroscope sensitivity
    if ((err = imu_cfg_gyroscope(I2C_SLAVE_ADDR, 
        GYRO_CFG_RANGE_500)) != ESP_OK) {
        goto esc;
    }

    // Configure the DLFP 
    if ((err = imu_set_dlfp(I2C_SLAVE_ADDR, DLFP_FILTER_6)) != ESP_OK) {
        goto esc;
    }

    // Set the sampling rate
    if ((err = imu_set_sampling_rate(I2C_SLAVE_ADDR, 0x9)) != ESP_OK) {
        goto esc;
    }


    // Configure interrupt behaviour 
    uint8_t imu_cfg_flags = 0x0;
    if ((err = imu_cfg_intr(I2C_SLAVE_ADDR, imu_cfg_flags)) != ESP_OK) {
        goto esc;
    }

    // Enable interrupts from FIFO
    // IDEA: Wait until FIFO hits -> read everything -> reset interrupt
    uint8_t imu_intr_flags = INTR_DATA_RDY;
    if ((err = imu_set_intr(I2C_SLAVE_ADDR, imu_intr_flags)) != ESP_OK) {
        goto esc;
    }

    // Configure FIFO
    uint8_t imu_fifo_flags = FIFO_EN_GX | FIFO_EN_GY | FIFO_EN_GZ | FIFO_EN_ACCEL;
    if ((err = imu_cfg_fifo(I2C_SLAVE_ADDR, imu_fifo_flags)) != ESP_OK) {
        goto esc;
    }

    // Enable FIFO
    if ((err = imu_set_fifo(I2C_SLAVE_ADDR, true)) != ESP_OK) {
        goto esc;
    }


    // Read the IMU a bit
    imu_data_t data;
    uint32_t pin_number;
    while (1) {

        if (xQueueReceive(g_gpio_event_queue, &pin_number, 0x0)) {
            //printf("intr - GPIO [%d]\n", pin_number);


            // Clear the interrupt by setting the pin
            if ((err = imu_clr_intr(I2C_SLAVE_ADDR)) != ESP_OK) {
                break;
            }

            // Read the FIFO out
            if ((err = i2c_receive_fifo(I2C_SLAVE_ADDR, &data)) != ESP_OK) {
                break;
            }

            // Print readings
            printf("%5d %5d %5d | %5d %5d %5d\n", data.ax, data.ay, data.az, data.gx, data.gy, data.gz);
        }

        //printf("...\n");

        // // Print az
        // if (i2c_read_az(I2C_SLAVE_ADDR) != ESP_OK) {
        //     ERR("Bad read!");
        // }

        // Print gz
        // if (i2c_read_gz(I2C_SLAVE_ADDR) != ESP_OK) {
        //     ERR("Bad read!");
        // }

        //vTaskDelay(portTICK_PERIOD_MS);
    }

esc:

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
