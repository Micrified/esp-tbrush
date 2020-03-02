/*
 *******************************************************************************
 *                         (C) Copyright 2020 Micrified                        *
 * Created: 27/02/2020                                                         *
 *                                                                             *
 * Programmer(s):                                                              *
 * - Charles Randolph                                                          *
 *                                                                             *
 * Description:                                                                *
 *  MPU-6050 interface for the ESP32 I2C driver                                *
 *                                                                             *
 *******************************************************************************
*/


#if !defined(MPU_6050_H)
#define MPU_6050_H

#include "driver/i2c.h"
#include "esp_log.h"


/*
 *******************************************************************************
 *                              Register Symbols                               *
 *******************************************************************************
*/


// BIT_ masks
#define BIT_0                   (1 << 0)
#define BIT_1                   (1 << 1)
#define BIT_2                   (1 << 2)
#define BIT_3                   (1 << 3)
#define BIT_4                   (1 << 4)
#define BIT_5                   (1 << 5)
#define BIT_6                   (1 << 6)
#define BIT_7                   (1 << 7)

// [R] Accelerometer registers (X,Y,Z)
#define REG_AX_L                0x3C
#define REG_AX_H                0x3B
#define REG_AY_L                0x3E
#define REG_AY_H                0x3D
#define REG_AZ_L                0x40
#define REG_AZ_H                0x3F

// [R] Gyroscope registers (X,Y,Z)
#define REG_GX_L                0x44
#define REG_GX_H                0x43
#define REG_GY_L                0x46
#define REG_GY_H                0x45
#define REG_GZ_L                0x48
#define REG_GZ_H                0x47

// [R+W] Accelerometer configuration register + mode masks
#define REG_A_CFG               0x1C
#define A_CFG_2G                0x0
#define A_CFG_4G                BIT_3
#define A_CFG_8G                BIT_4
#define A_CFG_16G               (BIT_3 | BIT_4)

// [R+W] Gyroscope configuration register + mode masks
#define REG_G_CFG               0x1B
#define G_CFG_250               0x0
#define G_CFG_500               BIT_3
#define G_CFG_1000              BIT_4
#define G_CFG_2000              (BIT_3 | BIT_4)

// [R+W] Power management (1) register + mode masks
#define REG_PWR_MGMT_1           0x6B
#define PWR_MGMT_1_RESET         BIT_7
#define PWR_MGMT_1_SLEEP         BIT_6

// [R+W] FIFO configuration register + mode masks
#define REG_FIFO_CFG             0x23
#define FIFO_CFG_TEMP            BIT_7
#define FIFO_CFG_GX              BIT_6
#define FIFO_CFG_GY              BIT_5
#define FIFO_CFG_GZ              BIT_4
#define FIFO_CFG_AXYZ            BIT_3

// [R] FIFO count registers
#define REG_FIFO_COUNT_L         0x73
#define REG_FIFO_COUNT_H         0x72

// [R/W] FIFO output register
#define REG_FIFO                 0x74

// [R+W] Sample-rate divider register
#define REG_SAMPLE_RATE_DIV      0x19

// [R+W] DLFP configuration register + masks
#define REG_DLFP_CFG             0x1A
#define DLFP_CFG_FILTER_0        0x0 // A{260Hz,0.0ms} G{256Hz 0.98ms} Fs=8kHz
#define DLFP_CFG_FILTER_1        0x1 // A{184Hz,2.0ms} G{188Hz 1.9ms}  Fs=1kHz
#define DLFP_CFG_FILTER_2        0x2 // A{94Hz, 3.0ms} G{98Hz  2.8ms}  Fs=1kHz
#define DLFP_CFG_FILTER_3        0x3 // A{44Hz, 4.9ms} G{42Hz, 4.8ms}  Fs=1kHz
#define DLFP_CFG_FILTER_4        0x4 // A{21Hz, 8.5ms} G{20Hz, 8.3ms}  Fs=1kHz
#define DLFP_CFG_FILTER_5        0x5 // A{10Hz,13.8ms} G{10Hz,13.4ms}  Fs=1kHz
#define DLFP_CFG_FILTER_6        0x6 // A{ 5Hz,19.0ms} G{ 5Hz,18.6ms}  Fs=1kHz

// [R+W] Interrupt enable register + masks
#define REG_INTR_EN              0x38
#define INTR_EN_DATA_RDY         BIT_0
#define INTR_EN_FIFO_OFL         BIT_4

// [R+W] Interrupt configuration register + masks
#define REG_INTR_CFG             0x37
#define INTR_CFG_ACTIVE_LOW      BIT_7
#define INTR_CFG_OPEN_DRAIN      BIT_6
#define INTR_CFG_LATCHING        BIT_5
#define INTR_CFG_ANY_CLR         BIT_4
#define INTR_CFG_FSYNC_LVL       BIT_3
#define INTR_CFG_FSYNC_EN        BIT_2
#define INTR_CFG_FSYNC_BYPASS    BIT_1

// [R] Interrupt status register
#define REG_INTR_STATUS          0x3A

// [R+W] User control register + masks
#define REG_USER_CTRL            0x6A
#define USER_CTRL_FIFO_EN        BIT_6
#define USER_CTRL_FIFO_RST       BIT_2


/*
 *******************************************************************************
 *                             Symbolic Constants                              *
 *******************************************************************************
*/


// Number of elements to read from FIFO at a time (dependent on configuration)
#define FIFO_BURST_LEN           12


/*
 *******************************************************************************
 *                              Type Definitions                               *
 *******************************************************************************
*/


// Structure containing interesting sensor data
typedef struct {
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
} mpu6050_data_t;


// Structure containing I2C configuration information
typedef struct {
	uint8_t    sda_pin, scl_pin;  // Data and clock pin #s
	uint8_t    slave_addr;        // I2C address of MPU-6050
	i2c_port_t i2c_port;          // I2C port # (I2C_NUM_0 ~ (I2C_NUM_MAX-1))
	uint32_t   clk_speed;         // I2C clock frequency for master
	bool       sda_pullup_en;     // Enable internal pullup for SDA
	bool       scl_pullup_en;     // Enable internal pullup for SCL
} mpu6050_i2c_cfg_t;


// Enumeration of error types
typedef enum {
	MPU6050_ERR_OK,                  // No error
	MPU6050_ERR_PARAM_CFG_FAIL,      // i2c_param_config() error
	MPU6050_ERR_DRIVER_INSTALL_FAIL, // i2c_driver_install() error
	MPU6050_ERR_INVALID_ARGUMENT,    // invalid parameter to function
	MPU6050_ERR_NO_SLAVE_ACK,        // No acknowledgment from slave
	MPU6050_ERR_INVALID_STATE,       // Driver not installed / not i2c master
	MPU6050_ERR_OPERATION_TIMEOUT,   // Bus busy,
	MPU6050_ERR_UNKNOWN,             // Unknown error
	MPU6050_ERR_MAX
} mpu6050_err_t;


/*
 *******************************************************************************
 *                        General Function Declarations                        *
 *******************************************************************************
*/


/*\
 * @brief Initializes I2C for the ESP using the given configuration
 *        structure pointer. 
 * @note  Recommended to configure clk_speed to 400kHz:
 *        I2C_APB_CLK_FREQ / 200 for example since I2C source clock
 *        is 80MHz APB clock (i.e. 200x too fast)
 * @param cfg  Configuration structure for I2C
 * @return mpu6050_err_t Error value (MPU6050_ERR_OK if none)
\*/
mpu6050_err_t mpu6050_init (mpu6050_i2c_cfg_t *cfg);


/*\
 * @brief Reads a byte from the I2C bus
 * @param cfg           The I2C device configuration
 * @param value_p       Address where byte value will be saved
 * @param mpu6050_err_t Error value (MPU6050_ERR_OK if none)
\*/
mpu6050_err_t mpu6050_receive_byte (mpu6050_i2c_cfg_t *cfg, uint8_t *value_p);


/*\
 * @brief Writes value (byte) to given register at I2C device configuration
 * @param cfg            The I2C device configuration
 * @param reg            The register to write to 
 * @param value          The byte value to write
 * @param request        If true, 'value' is not written (considered req)
 * @return mpu6050_err_t Error value (MPU6050_ERR_OK if none)
\*/
mpu6050_err_t mpu6050_write_register (mpu6050_i2c_cfg_t *cfg, uint8_t reg, 
	uint8_t value, bool request);


/*\
 * @brief Returns a string describing the given error value
 * @note  Returns NULL if error is out of bounds
 * @param err  The error enumeral
 * @return const char * Pointer to descriptive string
\*/
const char *mpu6050_err_to_str (mpu6050_err_t err);


/*
 *******************************************************************************
 *                       IMU-6050 Interface Declarations                       *
 *******************************************************************************
*/



/*\
 * @brief Configure power mode and clock source (See datasheet 4.30)
 * @param cfg             The device configuration structure
 * @param flags           The configuration flags
 * @return mpu6050_err_t  Error value (MPU6050_ERR_OK if none)
\*/
mpu6050_err_t mpu6050_configure_power (mpu6050_i2c_cfg_t *cfg, uint8_t flags);


/*\
 * @brief Configure accelerometer self test and full-scale range
 *        (See datasheet 4.5)
 * @param cfg             The device configuration structure
 * @param flags           The configuration flags
 * @return mpu6050_err_t  Error value (MPU6050_ERR_OK if none)
\*/
mpu6050_err_t mpu6050_configure_accelerometer (mpu6050_i2c_cfg_t *cfg, 
	uint8_t flags);


/*\
 * @brief Configure gyroscope self test and full-scale range
 *        (See datasheet 4.4)
 * @param cfg             The device configuration structure
 * @param flags           The configuration flags
 * @return mpu6050_err_t  Error value (MPU6050_ERR_OK if none)
\*/
mpu6050_err_t mpu6050_configure_gyroscope (mpu6050_i2c_cfg_t *cfg, 
	uint8_t flags);


/*\
 * @brief Allows the FIFO to be enabled or disabled by writing
 *        to the user control register (See datasheet 4.29)
 * @param cfg             The device configuration structure
 * @param flags           Configurations flags. If you want
 *                        to enable the FIFO correctly, you
 *                        should set the following: 
 *                        (USER_CTRL_FIFO_EN | USER_CTRL_FIFO_RST)
 * @return mpu6050_err_t  Error value (MPU6050_ERR_OK if none)
\*/
mpu6050_err_t mpu6050_enable_fifo (mpu6050_i2c_cfg_t *cfg, uint8_t flags);


/*\
 * @brief Configure what sensor registers are written into the
 *        FIFO (See datasheet 4.7). The register is called
 *        FIFO Enable, but FIFO is actually enabled in the 
 *        user-control register
 * @param cfg             The device configuration structure
 * @param flags           The configuration flags
 * @return mpu6050_err_t  Error value (MPU6050_ERR_OK if none)
\*/
mpu6050_err_t mpu6050_configure_fifo (mpu6050_i2c_cfg_t *cfg, uint8_t flags);


/*\
 * @brief Enables interrupt generation by interrupt sources
 *       (See datasheet 4.16)
 * @param cfg             The device configuration structure
 * @param flags           The configuration flags
 * @return mpu6050_err_t  Error value (MPU6050_ERR_OK if none)
\*/
mpu6050_err_t mpu6050_enable_interrupt (mpu6050_i2c_cfg_t *cfg, uint8_t flags);


/*\
 * @brief Configure behavior of interrupt signals at the INT
 *        pin (See datasheet 4.15)
 * @param cfg             The device configuration structure
 * @param flags           The configuration flags
 * @return mpu6050_err_t  Error value (MPU6050_ERR_OK if none)
\*/
mpu6050_err_t mpu6050_configure_interrupt (mpu6050_i2c_cfg_t *cfg, 
	uint8_t flags);


/*\
 * @brief Actually the interrupt-status register, reading this
 *        register will clear a latching interrupt
 *        (See datasheet 4.17)
 * @param cfg             The device configuration structure
 * @return mpu6050_err_t  Error value (MPU6050_ERR_OK if none)
\*/
mpu6050_err_t mpu6050_clear_interrupt (mpu6050_i2c_cfg_t *cfg);


/*\
 * @brief Specifies the divider that sets the sample rate
 *       (See datasheet 4.2).
 * @note  Sample_rate = (RATE / (1 + divident))
 * @note  When DLFP is enabled, RATE is 1kHz; else 8kHz
 * @param cfg             The device configuration structure
 * @param divident        Value to divide gyro output RATE
 * @return mpu6050_err_t  Error value (MPU6050_ERR_OK if none)
\*/
mpu6050_err_t mpu6050_set_sample_rate_divider (mpu6050_i2c_cfg_t *cfg, 
	uint8_t divident);


/*\
 * @brief Configures frame synchronization (FSYNC) and 
 *        DLFP settings for both gyroscope and accelerometer
 *        (See datasheet 4.3)
 * @param cfg             The device configuration structure
 * @param filter          The DLFP filter choice (see macros)
 * @return mpu6050_err_t  Error value (MPU6050_ERR_OK if none)
\*/
mpu6050_err_t mpu6050_configure_dlfp (mpu6050_i2c_cfg_t *cfg, uint8_t filter);


/*\
 * @brief Reads gyro and accelerometer data from the FIFO queue
 * @note  ASSUMES FIFO CONFIGURED WITH GYRO + ACCEL ONLY!!!
 * @param cfg             The device configuration structure
 * @param data_p          Pointer to struct to save sample data
 * @return mpu6050_err_t  Error value (MPU6050_ERR_OK if none)
\*/
mpu6050_err_t mpu6050_receive_fifo (mpu6050_i2c_cfg_t *cfg, 
	mpu6050_data_t *data_p);


/*\
 * @brief Number of samples currently in the FIFO buffer
 *        (See datasheet 4.32)
 * @param cfg             The device configuration structure
 * @param len_p           Pointer at which length will be written
 * @return mpu6050_err_t  Error value (MPU6050_ERR_OK if none)
\*/
mpu6050_err_t mpu6050_get_fifo_length (mpu6050_i2c_cfg_t *cfg, 
	uint16_t *len_p);


#endif