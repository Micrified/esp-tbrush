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
        .intr_type       = GPIO_INTR_POSEDGE 
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

    // Configure MPU-6050 i2c data structure
    mpu6050_i2c_cfg_t i2c_cfg = (mpu6050_i2c_cfg_t) {
        .sda_pin        = I2C_SDA_PIN,
        .scl_pin        = I2C_SCL_PIN,
        .slave_addr     = I2C_SLAVE_ADDR,
        .i2c_port       = I2C_NUM_0,
        .clk_speed      = I2C_APB_CLK_FREQ / 200,    // Requires 400kHz
        .sda_pullup_en  = false,
        .scl_pullup_en  = false
    };

    // Initialize i2c
    if (mpu6050_init(&i2c_cfg) != MPU6050_ERR_OK) {
        ERR("init");
        goto esc;
    }

    // Initialize I2C
    uint8_t flags = 0x0;
    if (mpu6050_configure_power(&i2c_cfg, flags) != MPU6050_ERR_OK) {
        ERR("configure_power");
        goto esc;
    }

    // Configure accelerometer sensitivity
    if (mpu6050_configure_accelerometer(&i2c_cfg, A_CFG_8G) 
        != MPU6050_ERR_OK) {
        ERR("configure_accelerometer");
        goto esc;
    }

    // Configure gyroscope sensitivity
    if (mpu6050_configure_gyroscope(&i2c_cfg, G_CFG_500) != MPU6050_ERR_OK) {
        ERR("configure_gyroscope");
        goto esc;
    }

    // Configure the DLFP 
    if (mpu6050_configure_dlfp(&i2c_cfg, DLFP_CFG_FILTER_2) 
        != MPU6050_ERR_OK) {
        ERR("configure_dlfp");
        goto esc;
    }

    // Set the sampling rate (~100Hz)
    if (mpu6050_set_sample_rate_divider(&i2c_cfg, 0x9) != MPU6050_ERR_OK) {
        ERR("set_sample_rate_divider");
        goto esc;
    }


    // Configure interrupt behaviour
    flags = 0x0;
    if (mpu6050_configure_interrupt(&i2c_cfg, flags) != MPU6050_ERR_OK) {
        ERR("configure_interrupt");
        goto esc;
    }


    // Enable interrupts from full refresh of sensors
    flags = INTR_EN_DATA_RDY;
    if (mpu6050_enable_interrupt(&i2c_cfg, flags) != MPU6050_ERR_OK) {
        ERR("enable_interrupt");
        goto esc;
    }

    // Enable the FIFO and reset it
    flags = USER_CTRL_FIFO_EN | USER_CTRL_FIFO_RST;
    if (mpu6050_enable_fifo(&i2c_cfg, flags) != MPU6050_ERR_OK) {
        ERR("enable_fifo");
        goto esc;
    }

    // Configure FIFO
    flags = FIFO_CFG_GX | FIFO_CFG_GY | FIFO_CFG_GZ | FIFO_CFG_AXYZ;
    if (mpu6050_configure_fifo(&i2c_cfg, flags) != MPU6050_ERR_OK) {
        ERR("configure_fifo");
        goto esc;
    }


    ESP_LOGI("APP", "Setup done...");

    // Read the IMU a bit
    mpu6050_data_t data;

    uint32_t pin_number;
    while (1) {

        if (xQueueReceive(g_gpio_event_queue, &pin_number, 0x0)) {
            //printf("intr - GPIO [%d]\n", pin_number);

            // Read the FIFO out
            if (mpu6050_receive_fifo(&i2c_cfg, &data) != MPU6050_ERR_OK) {
                break;
            }

            // Print readings
            printf("%d, %d, %d, %d, %d, %d\n", data.ax, data.ay, data.az, data.gx, data.gy, data.gz);
        }
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
