#if !defined(CFG_H)
#define CFG_H

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
 *  System-wide symbolic constants and configuration types                     *
 *                                                                             *
 *******************************************************************************
*/


#include "driver/i2c.h"
#include "driver/gpio.h"


/*
 *******************************************************************************
 *                       I2C General Symbolic Constants                        *
 *******************************************************************************
*/


// SDA pin for I2C bus
#define I2C_SDA_PIN                     21

// SCL pin for I2C bus
#define I2C_SCL_PIN                     22


/*
 *******************************************************************************
 *                       GPIO Device Symbolic Constants                        *
 *******************************************************************************
*/


// GPIO for the buzzer
#define GPIO_BUZZER_PIN                 18


// GPIO for the vibration motor
#define GPIO_VIBRATION_PIN              4


// GPIO for the reset button
#define GPIO_RESET_BUTTON_PIN           19


// Interrupt priority (default)
#define GPIO_RESET_INTR_FLAG_DEFAULT    1


/*
 *******************************************************************************
 *                         I2C IMU Symbolic Constants                          *
 *******************************************************************************
*/


// Address of IMU device on I2C bus
#define I2C_IMU_SLAVE_ADDR              0x68

// IMU device GPIO pin
#define I2C_IMU_INTR_PIN                15

// Interrupt priority (default)
#define I2C_IMU_INTR_FLAG_DEFAULT        2

// I2C port for the IMU
#define I2C_IMU_PORT_NUM                  I2C_NUM_0


/*
 *******************************************************************************
 *                         I2C OLED Symbolic Constants                         *
 *******************************************************************************
*/


// Service name identifier for the BLE link
#define DEVICE_BLE_SERVICE_NAME           "ESP-Tbrush"


/*
 *******************************************************************************
 *                   Classification Model Symbolic Constants                   *
 *******************************************************************************
*/


// Number of samples for training a section
#define IMU_TRAINING_SAMPLE_BUF_SIZE        120


// Number of samples to skip when changing sections
#define IMU_TRAINING_SAMPLE_DROP_SIZE       20


// Number of samples for a brushing session section
#define IMU_BRUSHING_SAMPLE_SIZE            400


#endif