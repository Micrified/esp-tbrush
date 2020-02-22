/*
 *******************************************************************************
 *                        (C) Copyright 2020 IoT Seminar                       *
 * Created: 21/02/2020                                                         *
 *                                                                             *
 * Programmer(s):                                                              *
 * - Sonnya Dellarosa                                                          *
 * - Charles Randolph                                                          *
 *                                                                             *
 * Description:                                                                *
 *  Defines the IMU task, which processes gyroscope and accelerometer data     *
 *                                                                             *
 *******************************************************************************
*/


#if !defined(IMU_H)
#define IMU_H


#include "driver/i2c.h"
#include "esp_log.h"
#include "errors.h"


/*
 *******************************************************************************
 *                              Type-Definitions                               *
 *******************************************************************************
*/


// Structure describing interesting IMU data
typedef struct {
	uint16_t ax, ay, az;
	uint16_t gx, gy, gz;
} imu_data_t;


// Type describing the IMU accelerometer modes
typedef uint8_t accel_cfg_t;


/*
 *******************************************************************************
 *                             Symbolic Constants                              *
 *******************************************************************************
*/


/*\
 * Register map for the MPU-6050 device. 
 * Note: Data is written to the FIFO in order of 
 * register number (from lowest to highest)
 * 
 * E.G: Say you enabled the register values
 * - FIFO_EN_ACCEL
 * - FIFO_EN_GX
 * - FIFO_EN_GY
 * - FIFO_EN_GZ
 *
 * This would mean the following FIFO stream
 * would be made available when reading:
 * -----------------------------------------
 *     BYTE #    |         VALUE
 * -----------------------------------------
 *       0       |     REG_ACCEL_X_H
 *       1       |     REG_ACCEL_X_L
 * -----------------------------------------
 *       2       |     REG_ACCEL_Y_H
 *       3       |     REG_ACCEL_Y_L
 * -----------------------------------------
 *       4       |     REG_ACCEL_Z_H
 *       5       |     REG_ACCEL_Z_L
 * -----------------------------------------
 *       6       |     REG_GYRO_X_H
 *       7       |     REG_GYRO_X_L
 * -----------------------------------------
 *       8       |     REG_GYRO_Y_H
 *       9       |     REG_GYRO_Y_L
 * -----------------------------------------
 *      10       |     REG_GYRO_Z_H
 *      11       |     REG_GYRO_Z_L
 * -----------------------------------------
 *
 * Note: The register map isn't 100% complete,
 * only the registers related to the accelerometer
 * and gyroscope were included, along with control
 * features of the MPU. There are additional external
 * sensor and temperature registers in the datasheet.
\*/


// MPU-6050 [R] Accelerometer register values (Hex)
#define REG_ACCEL_X_L            0x3C
#define REG_ACCEL_X_H            0x3B
#define REG_ACCEL_Y_L            0x3E
#define REG_ACCEL_Y_H            0x3D
#define REG_ACCEL_Z_L            0x40
#define REG_ACCEL_Z_H            0x3F


// MPU-6050 [R] Gyroscope register values (Hex)
#define REG_GYRO_X_L             0x44
#define REG_GYRO_X_H             0x43
#define REG_GYRO_Y_L             0x46
#define REG_GYRO_Y_H             0x45
#define REG_GYRO_Z_L             0x48
#define REG_GYRO_Z_H             0x47


// MPU-6050 Accelerometer configuration values (Hex)
#define REG_ACCEL_CFG            0x1C
#define ACCEL_CFG_RANGE_2G       0x0
#define ACCEL_CFG_RANGE_4G       (1 << 2)
#define ACCEL_CFG_RANGE_8G       (1 << 3)
#define ACCEL_CFG_RANGE_16G      ((1 << 2) | (1 << 3))


// MPU-6050 [R/W] Power management register values (Hex)
#define REG_PWR_MGMT_1           0x6B
#define PWR_DEV_RESET            (1 << 6)
#define PWR_DEV_SLEEP            (1 << 5)


// MPU-6050 [R/W] FIFO enable register values (Hex)
#define REG_FIFO_EN              0x23
#define FIFO_EN_TEMP             (1 << 6)
#define FIFO_EN_GX               (1 << 5)
#define FIFO_EN_GY               (1 << 4)
#define FIFO_EN_GZ               (1 << 3)
#define FIFO_EN_ACCEL            (1 << 2)


// MPU-6050 [R] FIFO count register values (Hex)
#define REG_FIFO_COUNT_L         0x73
#define REG_FIFI_COUNT_H         0x72


// MPU-6050 [R/W] FIFO value registers (Hex)
#define REG_FIFO_VALUE           0x74


// MPU-6050 [R/W] Interrupt enable register value (Hex)
#define REG_INTR_EN              0x38
#define INTR_DATA_RDY            (1 << 0)
#define INTR_FIFO_OFL            (1 << 3)


// MPU-6050 [R] Interrupt status register value (Hex)
#define REG_INTR_STATUS          0x3A


// MPU-6050 [R/W] User control register values (Hex)
#define REG_USER_CTRL            0x6A
#define CTRL_FIFO_EN             (1 << 5)


/*
 *******************************************************************************
 *                            Function Declarations                            *
 *******************************************************************************
*/


/*\ 
 *@brief Initializes I2C for the ESP on one of the available ports.
 *        The ESP is configured to be the master.
 * @param sda_pin     Value of the SDA I2C pin
 * @param scl_pin     Value of the SCL I2C pin
 * @param slave_addr  Address of the slave device
 * @return esp_err_t  ESP error value (ESP_OK if none)
\*/
esp_err_t imu_init (uint8_t sda_pin, uint8_t scl_pin, uint8_t slave_addr);


/*\
 * @brief Wakes or puts the IMU to sleep
 * @prarm slave_addr  Address of the slave device
 * @param sleeping    Boolean value. If true, device is put in sleep.
 * @return esp_err_t  ESP error value (ESP_OK if none)
\*/
esp_err_t imu_set_mode (uint8_t slave_addr, bool sleeping);


/*\
 * @brief Configures the IMU accelerometer
 * @param slave_addr  Address of the slave device
 * @param accel_cfg_t  Accelerometer configuration value
 * @return esp_err_t   ESP error value (ESP_OK if none)
\*/
esp_err_t imu_cfg_accelerometer (uint8_t slave_addr, accel_cfg_t cfg);


// Attempts to read the az register of the device at given slave_addr
esp_err_t i2c_read_az (uint8_t slave_addr);


#endif