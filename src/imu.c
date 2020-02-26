#include "imu.h"


/*
 *******************************************************************************
 *                        Internal Function Definitions                        *
 *******************************************************************************
*/


/*\
 * @brief Writes a register address to read on I2C_NUM_0 from a slave device
 * @note  Assumes all operations will require acknowledgements
 * @param slave_addr  The slave address which should receive the write
 * @param reg_addr    The single-byte register value to read 
 * @return esp_err_t  Returns ESP_OK on success, otherwise error
\*/
esp_err_t i2c_reg_request (uint8_t slave_addr, uint8_t reg_addr) {
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
		(slave_addr << 1) | I2C_MASTER_WRITE,  // op bit
		true)) != ESP_OK) {
		ERR("I2C master write byte invalid parameter!");
		goto esc;
	}

	// Either sequence of register data values (to read/write from)
	if ((err = i2c_master_write(cmd, &reg_addr, 1, true)) != ESP_OK) {
		ERR("I2C master write invalid parameter!");
		goto esc;
	}

	// Queue the the master stop command
	if ((err = i2c_master_stop(cmd)) != ESP_OK) {
		ERR("I2C master stop invalid parameter!");
		goto esc;
	}

	// Issue the command to the I2C driver
	if ((err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 
		portTICK_PERIOD_MS)) != ESP_OK) {
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


/*\
 * @brief Reads single byte from I2C_NUM_0, places value in given pointer
 * @note  Assumes all operations will require acknowledgements
 * @param slave_addr  The source of the data to read
 * @param data_p      Pointer at which the byte will be saved
 * @return esp_err_t  Error value. ESP_OK on success, else error. 
\*/
esp_err_t i2c_reg_receive (uint8_t slave_addr, uint8_t *data_p) {
	esp_err_t err = ESP_OK;

	// Verify data pointer
	if (data_p == NULL) {
		ERR("i2c_receive issued with invalid data pointer!");
		return ESP_ERR_INVALID_ARG;
	}

	// Create the command link
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	// Queue the start signal
	if ((err = i2c_master_start(cmd)) != ESP_OK) {
		ERR("I2C master start invalid parameter!");
		goto esc;
	}

	// Queue adjusted slave address (op = read)
	if ((err = i2c_master_write_byte(cmd,
		(slave_addr << 1) | I2C_MASTER_READ, // op bit
		true)) != ESP_OK) {
		ERR("I2C master write byte invalid parameter!");
		goto esc;
	}

	// Read single byte data value
	uint8_t rx_byte; 
	if ((err = i2c_master_read(cmd, &rx_byte, 1, true)) != ESP_OK) {
		ERR("I2C master read invalid parameter!");
		goto esc;
	}

	// Queue the master stop command
	if ((err = i2c_master_stop(cmd)) != ESP_OK) {
		ERR("I2C master stop invalid parameter!");
		goto esc;
	}

	// Instruct I2C driver to process the queued commands
	if ((err = i2c_master_cmd_begin(
		I2C_NUM_0, // Assuming using I2C port zero
		cmd, 
		portTICK_PERIOD_MS)) != ESP_OK) {
		if (err == ESP_ERR_INVALID_ARG) {
			ERR("I2C master cmd invalid parameter!");
		} else if (err == ESP_FAIL) {
			ERR("I2C master cmd no slave ack!");
		} else if (err == ESP_ERR_INVALID_STATE) {
			ERR("I2C master cmd driver not installed / not in master mode!");
		} else {
			ERR("I2C master cmd operation timeout (bus busy)!");
		}
		goto esc;
	}

	// Write to the data pointer
	*data_p = rx_byte;

esc:

	// Destroy the command link
	i2c_cmd_link_delete(cmd);

	return err;
}


/*\
 * @brief Performs a write on port I2C_NUM_0 to a register with the given
 *        register value.
 * @note  Buffer pointer may be NULL, in which case no data is written
 * @note  Assumes acknowledgments are required
 * @param slave_addr  The address of the device to send to
 * @param reg_addr    The register to write to
 * @param buffer      The byte buffer to write
 * @param len         The length to write from the buffer (in bytes)
 * @return esp_err_t  Returns ESP_OK on success, otherwise error
\*/
esp_err_t i2c_write_register (uint8_t slave_addr, uint8_t reg_addr, 
	uint8_t *buffer, size_t len) {
	esp_err_t err = ESP_OK;

	// Create the command link
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	// Queue the start signal
	if ((err = i2c_master_start(cmd)) != ESP_OK) {
		ERR("I2C master start invalid parameter!");
		goto esc;
	}

	// Queue adjusted slave address (op = write)
	if ((err = i2c_master_write_byte(cmd,
		(slave_addr << 1) | I2C_MASTER_WRITE, // op bit
		true)) != ESP_OK) {
		ERR("I2C master write byte invalid parameter!");
		goto esc;
	}

	// Write the register address first
	if ((err = i2c_master_write(cmd, &reg_addr, 1, true)) != ESP_OK) {
		ERR("I2C master write (reg) invalid parameter!");
		goto esc;
	}

	// Write the data buffer next (only if valid pointer)
	if (buffer != NULL && 
		(err = i2c_master_write(cmd, buffer, len, true)) != ESP_OK) {
		ERR("I2C master write (data) invalid parameters!");
		goto esc;
	}

	// Queue the master stop command
	if ((err = i2c_master_stop(cmd)) != ESP_OK) {
		ERR("I2C master stop invalid parameter!");
		goto esc;
	}

	// Instruct I2C driver to process the queued commands
	if ((err = i2c_master_cmd_begin(
		I2C_NUM_0, // Assuming using I2C port zero
		cmd, 
		portTICK_PERIOD_MS)) != ESP_OK) {
		if (err == ESP_ERR_INVALID_ARG) {
			ERR("I2C master cmd invalid parameter!");
		} else if (err == ESP_FAIL) {
			ERR("I2C master cmd no slave ack!");
		} else if (err == ESP_ERR_INVALID_STATE) {
			ERR("I2C master cmd driver not installed / not in master mode!");
		} else {
			ERR("I2C master cmd operation timeout (bus busy)!");
		}
		goto esc;
	}

esc:

	// Destroy the command link
	i2c_cmd_link_delete(cmd);

	return err;
}

// Performs a request/receive sequence to FIFO (assuming signed 16-bit in order MSB then LSB)
esp_err_t read_pair (uint8_t slave_addr, int16_t *data_p) {
	uint8_t lsb, msb;
	esp_err_t err = ESP_OK;

	if (data_p == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	// Request MSB
	if ((err = i2c_reg_request(slave_addr, REG_FIFO_VALUE)) != ESP_OK) {
		return err;
	}
	// Read MSB
	if ((err = i2c_reg_receive(slave_addr, &msb)) != ESP_OK) {
		return err;
	}

	// Request LSB
	if ((err = i2c_reg_request(slave_addr, REG_FIFO_VALUE)) != ESP_OK) {
		return err;
	}
	// Read LSB
	if ((err = i2c_reg_receive(slave_addr, &lsb)) != ESP_OK) {
		return err;
	}

	// Write data to given pointer
	*data_p = (int16_t)((((uint16_t)msb) << 8) | ((uint16_t)lsb));

	return err;
}


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


esp_err_t imu_set_mode (uint8_t slave_addr, bool sleeping) {
	esp_err_t err = ESP_OK;
	uint8_t mode = 0x0;

	// Set (or don't) the sleep mode flag
	if (sleeping) {
		mode |= PWR_DEV_SLEEP;
	}

	// Attempt to write to the register
	if ((err = i2c_write_register(slave_addr, REG_PWR_MGMT_1,
		&mode, 1)) != ESP_OK) {
		return err;
	}

	return err;
}


esp_err_t imu_cfg_accelerometer (uint8_t slave_addr, accel_cfg_t cfg) {
	esp_err_t err = ESP_OK;
	uint8_t mode = 0x0 | cfg;


	// Attempt to write to the register
	if ((err = i2c_write_register(slave_addr, REG_ACCEL_CFG, 
		&mode, 1)) != ESP_OK) {
		return err;
	}

	return err;
}


esp_err_t imu_cfg_gyroscope (uint8_t slave_addr, gyro_cfg_t cfg) {
	esp_err_t err = ESP_OK;
	uint8_t mode = 0x0 | cfg;

	// Attempt to write to the register
	if ((err = i2c_write_register(slave_addr, REG_GYRO_CFG, 
		&mode, 1)) != ESP_OK) {
		return err;
	}

	return err;
}


esp_err_t imu_set_fifo (uint8_t slave_addr, bool enabled) {
	esp_err_t err = ESP_OK;
	uint8_t mode = 0x0;

	if (enabled) {
		mode |= CTRL_FIFO_EN;
	} 

	if ((err = i2c_write_register(slave_addr, REG_USER_CTRL,
		&mode, 1)) != ESP_OK) {
		return err;
	}

	return err;
}


esp_err_t imu_cfg_fifo (uint8_t slave_addr, uint8_t flags) {
	esp_err_t err = ESP_OK;

	// Attempt to write to the register
	if ((err = i2c_write_register(slave_addr, REG_FIFO_EN,
		&flags, 1)) != ESP_OK) {
		return err;
	}

	return err;
}


esp_err_t imu_set_intr (uint8_t slave_addr, uint8_t flags) {
	esp_err_t err = ESP_OK;

	// Attempt to write to the register
	if ((err = i2c_write_register(slave_addr, REG_INTR_EN,
		&flags, 1)) != ESP_OK) {
		return err;
	}

	return err;
}


esp_err_t imu_cfg_intr (uint8_t slave_addr, uint8_t flags) {
	esp_err_t err = ESP_OK;

	// Attempt to write to the register
	if ((err = i2c_write_register(slave_addr, REG_INTR_CFG,
		&flags, 1)) != ESP_OK) {
		return err;
	}

	return err;
}


esp_err_t imu_clr_intr (uint8_t slave_addr) {
	esp_err_t err = ESP_OK;
	uint8_t data;

	// Attempt to just read the register (sufficient to clear it)
	if ((err = i2c_reg_request(slave_addr, REG_INTR_STATUS)) != ESP_OK) {
		return err;
	}
	if ((err = i2c_reg_receive(slave_addr, &data)) != ESP_OK) {
		return err;
	}

	return err;
}


esp_err_t imu_set_sampling_rate (uint8_t slave_addr, uint8_t divider) {
	esp_err_t err = ESP_OK;

	// Attempt to write to the register
	if ((err = i2c_write_register(slave_addr, REG_SAMPLE_RATE,
		&divider, 1)) != ESP_OK) {
		return err;
	}

	return err;
}


esp_err_t imu_set_dlfp (uint8_t slave_addr, uint8_t filter) {
	esp_err_t err = ESP_OK;

	// Attempt to write to the register
	if ((err = i2c_write_register(slave_addr, REG_DLFP_CFG,
		&filter, 1)) != ESP_OK) {
		return err;
	}

	return err;
}


esp_err_t i2c_receive_fifo (uint8_t slave_addr, imu_data_t *data_p) {
	esp_err_t err = ESP_OK;
	int16_t value;

	if (data_p == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	// Read ax
	if ((err = read_pair(slave_addr, &value)) != ESP_OK) {
		return err;
	} else {
		data_p->ax = value;
	}

	// Read ay
	if ((err = read_pair(slave_addr, &value)) != ESP_OK) {
		return err;
	} else {
		data_p->ay = value;
	}

	// Read az
	if ((err = read_pair(slave_addr, &value)) != ESP_OK) {
		return err;
	} else {
		data_p->az = value;
	}

	// Read gx
	if ((err = read_pair(slave_addr, &value)) != ESP_OK) {
		return err;
	} else {
		data_p->gx = value;
	}

	// Read gy
	if ((err = read_pair(slave_addr, &value)) != ESP_OK) {
		return err;
	} else {
		data_p->gy = value;
	}

	// Read gz
	if ((err = read_pair(slave_addr, &value)) != ESP_OK) {
		return err;
	} else {
		data_p->gz = value;
	}


	return err;
}


esp_err_t i2c_get_fifo_length (uint8_t slave_addr, uint16_t *len_p) {
	esp_err_t err = ESP_OK;
	uint8_t msb, lsb;

	if (len_p == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	// Request MSB
	if ((err = i2c_reg_request(slave_addr, REG_FIFI_COUNT_H)) != ESP_OK) {
		return err;
	}
	// Receive MSB
	if ((err = i2c_reg_receive(slave_addr, &msb)) != ESP_OK) {
		return err;
	}

	// Request LSB
	if ((err = i2c_reg_request(slave_addr, REG_FIFO_COUNT_L)) != ESP_OK) {
		return err;
	}
	// Receive LSB
	if ((err = i2c_reg_receive(slave_addr, &lsb)) != ESP_OK) {
		return err;
	}

	// Write to data pointer
	*len_p = (((uint16_t)msb) << 8) | ((uint16_t)lsb);

	return err;
}


esp_err_t i2c_read_az (uint8_t slave_addr) {
	esp_err_t err = ESP_OK;
	uint8_t az_l, az_h;

	// Request register LSB
	if ((err = i2c_reg_request(slave_addr, REG_ACCEL_Z_L)) != ESP_OK) {
		return err;
	}
	// Read register LSB
	if ((err = i2c_reg_receive(slave_addr, &az_l)) != ESP_OK) {
		return err;
	}

	// Request register MSB
	if ((err = i2c_reg_request(slave_addr, REG_ACCEL_Z_H)) != ESP_OK) {
		return err;
	}
	// Read register MSB
	if ((err = i2c_reg_receive(slave_addr, &az_h)) != ESP_OK) {
		return err;
	}

	// Print z-axis value
	int16_t az = (int16_t)((((uint16_t)az_h) << 8) | ((uint16_t)az_l));
	printf("az = %d\n", az);

	return err;
}


esp_err_t i2c_read_gz (uint8_t slave_addr) {
	esp_err_t err = ESP_OK;
	uint8_t gz_l, gz_h;

	// Request register LSB
	if ((err = i2c_reg_request(slave_addr, REG_GYRO_Z_L)) != ESP_OK) {
		return err;
	}
	// Read register LSB
	if ((err = i2c_reg_receive(slave_addr, &gz_l)) != ESP_OK) {
		return err;
	}

	// Request register HSB
	if ((err = i2c_reg_request(slave_addr, REG_GYRO_Z_H)) != ESP_OK) {
		return err;
	}
	// Read register HSB
	if ((err = i2c_reg_receive(slave_addr, &gz_h)) != ESP_OK) {
		return err;
	}

	// Print z-axis value
	int16_t gz = (((uint16_t)gz_h) << 8) | (((uint16_t)gz_l));
	printf("gz = %d\n", gz);

	return err;
}
