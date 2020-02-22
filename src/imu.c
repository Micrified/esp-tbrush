#include "imu.h"


/*
 *******************************************************************************
 *                        External Function Definitions                        *
 *******************************************************************************
*/


esp_err_t imu_init (uint8_t sda_pin, uint8_t scl_pin, uint8_t slave_addr) {
	esp_err_t err = ESP_OK;

	// Prepare the I2C configuration for the driver (master)
	i2c_config_t cfg = (i2c_config_t) {
		.mode             = I2C_MODE_MASTER,
		.sda_io_num       = sda_pin,
		.scl_io_num       = scl_pin, 
		.sda_pullup_en    = false,    // Using external 10K resistor
		.scl_pullup_en    = false,    // Using external 10K resistor
		{
			.master = {
				// I2C source clock is 80MHz APB clock, but IMU requires 400kHz
				// Thus I2C_APB_CLK_FREQ is 200x too large
				.clk_speed = I2C_APB_CLK_FREQ / 200
			}
		}
	};

	// Configure the I2C master port
	if ((err = i2c_param_config(I2C_NUM_0, &cfg)) != ESP_OK) {
		ERR("I2C param configuration failed!");
		goto esc;
	}

	// Configure the I2C driver (only LOWMED can be handled in C, not HIGH)
	if ((err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0x0, 0x0, 
		ESP_INTR_FLAG_LOWMED)) != ESP_OK) {
		if (err == ESP_ERR_INVALID_ARG) {
			ERR("I2C driver given invalid argument!");
		} else {
			ERR("I2C driver install error!");
		}
		goto esc;
	}

	// Set the timeout [DEBUG ATTEMPT]
	// if ((err = i2c_set_timeout(I2C_NUM_0, 1048575)) != ESP_OK) {
	// 	ERR("I2C set timeout invalid parameter!");
	// 	goto esc;
	// }

esc:
	return err;
}


esp_err_t i2c_request (uint8_t slave_addr) {
	esp_err_t err = ESP_OK;

	// Create the command link
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	// Queue start signal command
	if ((err = i2c_master_start(cmd)) != ESP_OK) {
		ERR("I2C master start invalid parameter!");
		goto esc;
	}

	// Queue the slave address (first byte)
	if ((err = i2c_master_write_byte(cmd, 
		(slave_addr << 1) | I2C_MASTER_WRITE,  // Read OP
		true)) != ESP_OK) {
		ERR("I2C master write byte invalid parameter!");
		goto esc;
	}

	// Either sequence of register data values (to read/write from)
	uint8_t reg_read_sequence[2] = {REG_ACCEL_Z_H, REG_ACCEL_Z_L};
	if ((err = i2c_master_write(cmd, reg_read_sequence, 2, true)) != ESP_OK) {
		ERR("I2C master read invalid parameter!");
		goto esc;
	}

	// Queue the the master stop command
	if ((err = i2c_master_stop(cmd)) != ESP_OK) {
		ERR("I2C master stop invalid parameter!");
		goto esc;
	}

	// Issue the command to the I2C driver
	if ((err = i2c_master_cmd_begin(I2C_NUM_0, cmd, portTICK_PERIOD_MS)) != ESP_OK) {
		if (err == ESP_ERR_INVALID_ARG) {
			ERR("I2C master cmd invalid parameter!");
		} else if (err == ESP_FAIL) {
			ERR("I2C master cmd no slave ack!");
		} else if (err == ESP_ERR_INVALID_STATE) {
			ERR("I2C master cmd driver not installed or not in master mode!");
		} else if (err == ESP_ERR_TIMEOUT) {
			ERR("I2C master cmd operation timeout (bus busy)!");
		} else {
			ERR("I2C master cmd unknown error!");
		}
		goto esc;
	}

esc:

	// Destroy the command link
	i2c_cmd_link_delete(cmd);

	return err;
}


esp_err_t i2c_receive (uint8_t slave_addr, imu_data_t *imu_data_p) {
	esp_err_t err = ESP_OK;

	// Verify data pointer
	if (imu_data_p == NULL) {
		ERR("imu_read issued with invalid data pointer!");
		return ESP_ERR_INVALID_ARG;
	}

	// Create the command link
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	// Queue start signal command
	if ((err = i2c_master_start(cmd)) != ESP_OK) {
		ERR("I2C master start invalid parameter!");
		goto esc;
	}

	// Queue the slave address (first byte)
	if ((err = i2c_master_write_byte(cmd, 
		(slave_addr << 1) | I2C_MASTER_READ, 
		true)) != ESP_OK) {
		ERR("I2C master write byte invalid parameter!");
		goto esc;
	}

	// Either sequence of register data values (to read/write from)
	uint8_t reg_read_sequence[2] = {0};
	if ((err = i2c_master_read(cmd, reg_read_sequence, 2, true)) != ESP_OK) {
		ERR("I2C master read invalid parameter!");
		goto esc;
	}

	// Queue the the master stop command
	if ((err = i2c_master_stop(cmd)) != ESP_OK) {
		ERR("I2C master stop invalid parameter!");
		goto esc;
	}

	// Issue the command to the I2C driver
	if ((err = i2c_master_cmd_begin(I2C_NUM_0, cmd, portTICK_PERIOD_MS)) != ESP_OK) {
		if (err == ESP_ERR_INVALID_ARG) {
			ERR("I2C master cmd invalid parameter!");
		} else if (err == ESP_FAIL) {
			ERR("I2C master cmd no slave ack!");
		} else if (err == ESP_ERR_INVALID_STATE) {
			ERR("I2C master cmd driver not installed or not in master mode!");
		} else {
			ERR("I2C master cmd operation timeout (bus busy)!");
		}
		goto esc;
	}

	// Update the given buffer
	imu_data_p->az = (reg_read_sequence[0] << 8) | (reg_read_sequence[1]);

esc:

	// Destroy the command link
	i2c_cmd_link_delete(cmd);

	return err;
}
