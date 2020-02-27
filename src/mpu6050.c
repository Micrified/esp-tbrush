#include "mpu6050.h"


/*
 *******************************************************************************
 *                              Global Variables                               *
 *******************************************************************************
*/


// Table mapping mpu6050_err_t enumerals to strings
static const char *mpu6050_err_str[MPU6050_ERR_MAX] = {
	[MPU6050_ERR_OK]                  = "No error",
	[MPU6050_ERR_PARAM_CFG_FAIL]      = "i2c_param_config() error",
	[MPU6050_ERR_DRIVER_INSTALL_FAIL] = "i2c_driver_install() error",
	[MPU6050_ERR_INVALID_ARGUMENT]    = "invalid parameter to function",
	[MPU6050_ERR_NO_SLAVE_ACK]        = "No acknowledgment from slave",
	[MPU6050_ERR_INVALID_STATE]       = "Driver not installed / not i2c master",
	[MPU6050_ERR_OPERATION_TIMEOUT]   = "Timeout; Bus busy",
	[MPU6050_ERR_UNKNOWN]             = "Unknown error"
};


/*
 *******************************************************************************
 *                        Internal Function Definitions                        *
 *******************************************************************************
*/



/*
 *******************************************************************************
 *                        External Function Definitions                        *
 *******************************************************************************
*/


mpu6050_err_t mpu6050_init (mpu6050_i2c_cfg_t *cfg) {
	esp_err_t err = ESP_OK;

	// Setup I2C master configuration for the driver
	i2c_config_t i2c_cfg = (i2c_config_t) {
		.mode             = I2C_MODE_MASTER,
		.sda_io_num       = cfg->sda_pin,
		.scl_io_num       = cfg->scl_pin, 
		.sda_pullup_en    = cfg->sda_pullup_en,
		.scl_pullup_en    = cfg->scl_pullup_en,
		{
			.master = {
				.clk_speed = cfg->clk_speed
			}
		}
	};

	// Configure I2C master port
	if ((err = i2c_param_config(cfg->i2c_port, &i2c_cfg)) != ESP_OK) {
		return MPU6050_ERR_PARAM_CFG_FAIL;
	}

	// Install I2C driver (only LOWMED can be handled in C, not HIGH)
	if ((err = i2c_driver_install(cfg->i2c_port, I2C_MODE_MASTER, 0x0, 0x0,
		ESP_INTR_FLAG_LOWMED)) != ESP_OK) {
		if (err = ESP_ERR_INVALID_ARG) {
			return MPU6050_ERR_DRIVER_INSTALL_FAIL;
		} else {
			return MPU6050_ERR_INVALID_ARGUMENT;
		}
	}

	return MPU6050_ERR_OK;
}


/*\
 * @brief Reads a byte from the I2C bus
 * @param cfg           The I2C device configuration
 * @param value_p       Address where byte value will be saved
 * @param mpu6050_err_t Error value (MPU6050_ERR_OK if none)
\*/
mpu6050_err_t mpu6050_receive_byte (mpu6050_i2c_cfg_t *cfg, uint8_t *value_p) {
	esp_err_t err     = ESP_OK;
	mpu6050_err_t ret = MPU6050_ERR_OK;

	// Create I2C command buffer
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	// Queue start signal
	if (i2c_master_start(cmd) != ESP_OK) {
		ret = MPU6050_ERR_INVALID_ARGUMENT;
		goto esc;
	}

	// Queue the slave address (operation = read)
	if (i2c_master_write_byte(
		cmd,
		(cfg->slave_addr << 1) | I2C_MASTER_READ,
		true) != ESP_OK) {
		ret = MPU6050_ERR_INVALID_ARGUMENT;
		goto esc;
	}

	// Read from the bus
	if (i2c_master_read_byte(cmd, value_p, true) != ESP_OK) {
		ret = MPU6050_ERR_INVALID_ARGUMENT;
		goto esc;
	}

	// Queue the master stop command
	if (i2c_master_stop(cmd) != ESP_OK) {
		ret = MPU6050_ERR_INVALID_ARGUMENT;
		goto esc;
	}

	// Execute the command queue
	if ((err = i2c_master_cmd_begin(
		cfg->i2c_port,
		cmd,
		portTICK_PERIOD_MS)) != ESP_OK) {
		if (err == ESP_ERR_INVALID_ARG) {
			ret = MPU6050_ERR_INVALID_ARGUMENT;
		} else if (err == ESP_FAIL) {
			ret = MPU6050_ERR_NO_SLAVE_ACK;
		} else if (err == ESP_ERR_INVALID_STATE) {
			ret = MPU6050_ERR_INVALID_STATE;
		} else if (err = ESP_ERR_TIMEOUT) {
			ret = MPU6050_ERR_OPERATION_TIMEOUT;
		} else {
			ret = MPU6050_ERR_UNKNOWN;
		}
		goto esc;
	}

esc:

	// Destroy (recycle) command queue
	i2c_cmd_link_create(cmd);

	return ret;
}


mpu6050_err_t mpu6050_write_register (mpu6050_i2c_cfg_t *cfg, uint8_t reg, 
	uint8_t value, bool request) {
	esp_err_t err      = ESP_OK;
	mpu6050_err_t ret  = MPU6050_ERR_OK;
	uint8_t tx_data[2] = [reg, value];

	// Create I2C command buffer
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	// Queue start signal
	if (i2c_master_start(cmd) != ESP_OK) {
		ret = MPU6050_ERR_INVALID_ARGUMENT;
		goto esc;
	}

	// Queue adjusted slave address (operation = write)
	if (i2c_master_write_byte(
		cmd,
		(cfg->slave_addr << 1) | I2C_MASTER_WRITE,
		true) != ESP_OK) {
		ret = MPU6050_ERR_INVALID_ARGUMENT;
		goto esc;
	}

	// Queue transmission data (if not a request)
	if (request == false && 
		i2c_master_write(cmd, tx_data, 2, true) != ESP_OK) {
		ret = MPU6050_ERR_INVALID_ARGUMENT;
		goto esc;
	}

	// Queue stop command
	if (i2c_master_stop(cmd) != ESP_OK) {
		ret = MPU6050_ERR_INVALID_ARGUMENT;
		goto esc;
	}

	// Execute the command queue
	if ((err = i2c_master_cmd_begin(
		cfg->i2c_port,
		cmd, 
		portTICK_PERIOD_MS)) != ESP_OK) {
		if (err == ESP_ERR_INVALID_ARG) {
			ret = MPU6050_ERR_INVALID_ARGUMENT;
		} else if (err == ESP_FAIL) {
			ret = MPU6050_ERR_NO_SLAVE_ACK;
		} else if (err == ESP_ERR_INVALID_STATE) {
			ret = MPU6050_ERR_INVALID_STATE;
		} else if (err = ESP_ERR_TIMEOUT) {
			ret = MPU6050_ERR_OPERATION_TIMEOUT;
		} else {
			ret = MPU6050_ERR_UNKNOWN;
		}
		goto esc;
	}

esc:
	
	// Destroy (recycle) command queue
	i2c_cmd_link_create(cmd);

	return ret;
}


mpu6050_err_t mpu6050_configure_power (mpu6050_i2c_cfg_t *cfg, uint8_t flags) {
	return mpu6050_write_register(cfg, REG_PWR_MGMT_1, flags, false);
}


mpu6050_err_t mpu6050_configure_accelerometer (mpu6050_i2c_cfg_t *cfg, 
	uint8_t flags) {
	return mpu6050_write_register(cfg, REG_A_CFG, flags, false);
}


mpu6050_err_t mpu6050_configure_gyroscope (mpu6050_i2c_cfg_t *cfg, 
	uint8_t flags) {
	return mpu6050_write_register(cfg, REG_G_CFG, flags, false)
}


mpu6050_err_t mpu6050_enable_fifo (mpu6050_i2c_cfg_t *cfg, bool enable) {
	return mpu6050_write_register(cfg, );
}


mpu6050_err_t mpu6050_configure_fifo (mpu6050_i2c_cfg_t *cfg, uint8_t flags) {

}


mpu6050_err_t mpu6050_enable_interrupt (mpu6050_i2c_cfg_t *cfg, uint8_t flags) {

}


mpu6050_err_t mpu6050_configure_interrupt (mpu6050_i2c_cfg_t *cfg, 
	uint8_t flags) {

}


mpu6050_err_t mpu6050_clear_interrupt (mpu6050_i2c_cfg_t *cfg) {

}


mpu6050_err_t mpu6050_set_sample_rate_divider (mpu6050_i2c_cfg_t *cfg, 
	uint8_t divident) {

}


mpu6050_err_t mpu6050_configure_dlfp (mpu6050_i2c_cfg_t *cfg, uint8_t filter) {

}


mpu6050_err_t mpu6050_receive_fifo (mpu6050_i2c_cfg_t *cfg, 
	mpu6050_data_t *data_p) {

}


mpu6050_err_t mpu6050_get_fifo_length (mpu6050_i2c_cfg_t *cfg, 
	uint16_t *len_p) {

}


mpu6050_err_t mpu6050_fifo_reset (mpu6050_i2c_cfg_t *cfg) {

}