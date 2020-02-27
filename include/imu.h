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
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
} imu_data_t;


// Type describing the IMU accelerometer modes
typedef uint8_t accel_cfg_t;


// Type describing the IMU gyroscope modes
typedef uint8_t gyro_cfg_t;


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
 * ----------------------------------------------------------- *
 *     BYTE #    |         VALUE          |    Register (dec)  *
 * ----------------------------------------------------------- *
 *       0       |     ACCEL_XOUT[15:8]   |         59         *
 *       1       |     ACCEL_XOUT[7:0]    |         60         *
 * ----------------------------------------------------------- *
 *       2       |     ACCEL_YOUT[15:8]   |         61         *
 *       3       |     ACCEL_YOUT[7:0]    |         62         *
 * ----------------------------------------------------------- *
 *       4       |     ACCEL_ZOUT[15:8]   |         63         *
 *       5       |     ACCEL_ZOUT[7:0]    |         64         *
 * ----------------------------------------------------------- *
 *       6       |     ---------------    |         65         *
 *       7       |     ---------------    |         66         *
 * ----------------------------------------------------------- *
 *       8       |     GYRO_XOUT[15:8]    |         67         *
 *       9       |     GYRO_XOUT[7:0]     |         68         *
 * ----------------------------------------------------------- *
 *      10       |     GYRO_YOUT[15:8]    |         69         *
 *      11       |     GYRO_YOUT[7:0]     |         70         *
 * ----------------------------------------------------------- *
 *      12       |     GYRO_ZOUT[15:8]    |         71         *
 *      13       |     GYRO_ZOUT[7:0]     |         72         *
 * ----------------------------------------------------------- *
 *      ...      |     ...............    |         ...        *
 * ----------------------------------------------------------- *
 *               |    -----------------   |         96         *
 * ----------------------------------------------------------- *
 * In total 38 bytes per total mapping. Thus one should only 
 * read bursts of 38 bytes and use only the first 14
 * -------------- ^^ Theory ------------------------------
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
#define ACCEL_CFG_RANGE_4G       (1 << 3)
#define ACCEL_CFG_RANGE_8G       (1 << 4)
#define ACCEL_CFG_RANGE_16G      ((1 << 3) | (1 << 4))


// MPU-6050 Gyroscope configuration values (Hex)
#define REG_GYRO_CFG             0x1B
#define GYRO_CFG_RANGE_250       0x0
#define GYRO_CFG_RANGE_500       (1 << 3)
#define GYRO_CFG_RANGE_1000      (1 << 4)
#define GYRO_CFG_RANGE_2000      ((1 << 3) | (1 << 4))


// MPU-6050 [R/W] Power management register values (Hex)
#define REG_PWR_MGMT_1           0x6B
#define PWR_DEV_RESET            (1 << 7)
#define PWR_DEV_SLEEP            (1 << 6)


// MPU-6050 [R/W] FIFO enable register values (Hex)
#define REG_FIFO_EN              0x23
#define FIFO_EN_TEMP             (1 << 7)
#define FIFO_EN_GX               (1 << 6)
#define FIFO_EN_GY               (1 << 5)
#define FIFO_EN_GZ               (1 << 4)
#define FIFO_EN_ACCEL            (1 << 3)


// MPU-6050 [R/W] Sample rate divider (Hex)
#define REG_SAMPLE_RATE          0x19


// MPU-6050 [R/W] DLFP configuration (Hex)
#define REG_DLFP_CFG             0x1A
#define DLFP_FILTER_0            0x0 // A{260Hz,0.0ms} G{256Hz 0.98ms} Fs=8kHz
#define DLFP_FILTER_1            0x1 // A{184Hz,2.0ms} G{188Hz 1.9ms}  Fs=1kHz
#define DLFP_FILTER_2            0x2 // A{94Hz, 3.0ms} G{98Hz  2.8ms}  Fs=1kHz
#define DLFP_FILTER_3            0x3 // A{44Hz, 4.9ms} G{42Hz, 4.8ms}  Fs=1kHz
#define DLFP_FILTER_4            0x4 // A{21Hz, 8.5ms} G{20Hz, 8.3ms}  Fs=1kHz
#define DLFP_FILTER_5            0x5 // A{10Hz,13.8ms} G{10Hz,13.4ms}  Fs=1kHz
#define DLFP_FILTER_6            0x6 // A{ 5Hz,19.0ms} G{ 5Hz,18.6ms}  Fs=1kHz


// MPU-6050 [R] FIFO count register values (Hex)
#define REG_FIFO_COUNT_L         0x73
#define REG_FIFI_COUNT_H         0x72


// MPU-6050 [R/W] FIFO value registers (Hex)
#define REG_FIFO_VALUE           0x74


// MPU-6050 [R/W] Interrupt enable register value (Hex)
#define REG_INTR_EN              0x38
#define INTR_DATA_RDY            (1 << 0)
#define INTR_FIFO_OFL            (1 << 4)


// MPU-6050[R/W] Interrupt configuration register value (Hex)
#define REG_INTR_CFG             0x37
#define INTR_CFG_ACTIVE_LOW      (1 << 7)
#define INTR_CFG_OPEN_DRAIN      (1 << 6)
#define INTR_CFG_LATCHING        (1 << 5)
#define INTR_CFG_ANY_CLR         (1 << 4)
#define INTR_CFG_FSYNC_LVL       (1 << 3)
#define INTR_CFG_FSYNC_EN        (1 << 2)
#define INTR_CFG_FSYNC_BYPASS    (1 << 1)


// MPU-6050 [R] Interrupt status register value (Hex)
#define REG_INTR_STATUS          0x3A


// MPU-6050 [R/W] User control register values (Hex)
#define REG_USER_CTRL            0x6A
#define CTRL_FIFO_EN             (1 << 6)
#define CTRL_FIFO_RST            (1 << 2)


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
 * @param slave_addr   Address of the slave device
 * @param accel_cfg_t  Accelerometer configuration value
 * @return esp_err_t   ESP error value (ESP_OK if none)
\*/
esp_err_t imu_cfg_accelerometer (uint8_t slave_addr, accel_cfg_t cfg);


/*\
 * @brief Configures the IMU gyroscope
 * @param slave_addr  Address of the slave device
 * @param gyro_cfg_t  Gyroscope configuration value
 * @return esp_err_t  ESP error value (ESP_OK if none)
\*/
esp_err_t imu_cfg_gyroscope (uint8_t slave_addr, gyro_cfg_t cfg);


/*\
 * @brief Enables FIFO for the IMU.
 * @param slave_addr  Address of the slave device
 * @param enabled     Boolean value. If true, FIFO is enabled. 
 * @return esp_err_t  ESP error value (ESP_OK) if none)
\*/
esp_err_t imu_set_fifo (uint8_t slave_addr, bool enabled);


/*\
 * @brief Configures FIFO for the IMU. 
 * @param slave_addr  Address of the slave device
 * @param flags       Byte (flag) indicating what FIFO will contain
 * @return esp_err_t  ESP error value (ESP_OK if none)
\*/
esp_err_t imu_cfg_fifo (uint8_t slave_addr, uint8_t flags);



/*\
 * @brief Enables the IMU to generate interrupts on interrupt pin
 * @param slave_addr  Address of the slave device
 * @param flags       Byte (flag) indicating what will generate
 *                    an interrupt (see datasheet for all)
 * @return esp_err_t  ESP error value (ESP_OK if none)
\*/
esp_err_t imu_set_intr (uint8_t slave_addr, uint8_t flags);


/*\
 * @brief Configures the behaviour of the interrupt pin
 * @param slave_addr  Address of the slave device
 * @param flags       Byte (flags) with ISR configuration
 *                    bits
 * @return esp_err_t  ESP error value (ESP_OK if none)
\*/
esp_err_t imu_cfg_intr (uint8_t slave_addr, uint8_t flags);


/*\
 * @brief Clears an interrupt by reading register 58
 * @param slave_addr  Address of the slave device
 * @return esp_err_t ESP error value (ESP_OK if none)
\*/
esp_err_t imu_clr_intr (uint8_t slave_addr);


/*\
 * @brief Sets the sampling rate. See notes (important)
 * @note  Gyro output rate = 8kHz when DLFP disabled
 * @note  Gyro output rate = 1kHz when DLFP enabled
 * @param slave_addr  Address of the slave device
 * @param divider     An 8-bit value dividing the Gyro output rate
 *                    sample_rate = (gyro_output_rate / (1 + divider))
 * @return esp_err_t ESP error value (ESP_OK if none)
\*/
esp_err_t imu_set_sampling_rate (uint8_t slave_addr, uint8_t divider);


/*\
 * @brief Sets the DLFP setting.
 * @note  Sampling rate divider is adjusted if DLFP enabled
 * @note  DLFP only disabled for input: DLFP_FILTER_0
 * @param slave_addr  Address of the slave device
 * @param filter      Byte representing the filter to apply
 * @return esp_err_t  ESP error value (ESP_OK if none)
\*/
esp_err_t imu_set_dlfp (uint8_t slave_addr, uint8_t filter);


/*\
 * @brief Reads data from the FIFO by individually requesting each byte
 * @param slave_addr  Address of the slave device
 * @param data_p      Non-null pointer to data-structure in which data
 *                    is written.
 * @return esp_err_t  ESP error value (ESP_OK if none)
\*/
esp_err_t i2c_receive_fifo (uint8_t slave_addr, imu_data_t *data_p);


/*\
 * @brief Returns the number of bytes in the FIFO queue by reading
 *        the value of the FIFO register
 * @param slave_addr  Address of the slave device
 * @param len_p      Non-null pointer to integer where value is 
 *                    written
 * @return esp_err_t  ESP error value (ESP_OK if none)
\*/
esp_err_t i2c_get_fifo_length (uint8_t slave_addr, uint16_t *len_p);


/*\
 * @brief Resets the FIFO queue
 * @param slave_addr  Address of the slave device
 * @return esp_err_t ESP error value (ESP_OK if none)
\*/
esp_err_t i2c_fifo_reset (uint8_t slave_addr);


// Attempts to read the az register of the device at given
esp_err_t i2c_read_az (uint8_t slave_addr);


// Attempts to read the gz register of the device
esp_err_t i2c_read_gz (uint8_t slave_addr);


#endif