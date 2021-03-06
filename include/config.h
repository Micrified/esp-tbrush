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
#define I2C_SDA_PIN                   21

// SCL pin for I2C bus
#define I2C_SCL_PIN                   22


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
#define I2C_IMU_INTR_FLAG_DEFAULT        0

// I2C port for the IMU
#define I2C_IMU_PORT_NUM                  I2C_NUM_0


/*
 *******************************************************************************
 *                         I2C OLED Symbolic Constants                         *
 *******************************************************************************
*/


#endif