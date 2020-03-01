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
 *                        General Function Definitions                         *
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
		if (err == ESP_ERR_INVALID_ARG) {
			return MPU6050_ERR_DRIVER_INSTALL_FAIL;
		} else {
			return MPU6050_ERR_INVALID_ARGUMENT;
		}
	}

	return MPU6050_ERR_OK;
}


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
		} else if (err == ESP_ERR_TIMEOUT) {
			ret = MPU6050_ERR_OPERATION_TIMEOUT;
		} else {
			ret = MPU6050_ERR_UNKNOWN;
		}
		goto esc;
	}

esc:

	// Destroy (recycle) command queue
	i2c_cmd_link_delete(cmd);

	return ret;
}


mpu6050_err_t mpu6050_write_register (mpu6050_i2c_cfg_t *cfg, uint8_t reg, 
	uint8_t value, bool request) {
	esp_err_t err      = ESP_OK;
	mpu6050_err_t ret  = MPU6050_ERR_OK;

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

	// Queue the register
	if (i2c_master_write_byte(cmd, reg, true) != ESP_OK) {
		ret = MPU6050_ERR_INVALID_ARGUMENT;
		goto esc;
	}

	// Queue data (if not a request, but a write)
	if (request == false && i2c_master_write_byte(cmd, value, true) 
		!= ESP_OK) {
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
		} else if (err == ESP_ERR_TIMEOUT) {
			ret = MPU6050_ERR_OPERATION_TIMEOUT;
		} else {
			ret = MPU6050_ERR_UNKNOWN;
		}
		goto esc;
	}

esc:
	
	// Destroy (recycle) command queue
	i2c_cmd_link_delete(cmd);

	return ret;
}


const char *mpu6050_err_to_str (mpu6050_err_t err) {
	if (err > MPU6050_ERR_UNKNOWN) {
		return NULL; 
	} else {
		return mpu6050_err_str[err];
	}
}


/*
 *******************************************************************************
 *                   MPU-6050 Interface Function Definitions                   *
 *******************************************************************************
*/


mpu6050_err_t mpu6050_configure_power (mpu6050_i2c_cfg_t *cfg, uint8_t flags) {
	return mpu6050_write_register(cfg, REG_PWR_MGMT_1, flags, false);
}


mpu6050_err_t mpu6050_configure_accelerometer (mpu6050_i2c_cfg_t *cfg, 
	uint8_t flags) {
	return mpu6050_write_register(cfg, REG_A_CFG, flags, false);
}


mpu6050_err_t mpu6050_configure_gyroscope (mpu6050_i2c_cfg_t *cfg, 
	uint8_t flags) {
	return mpu6050_write_register(cfg, REG_G_CFG, flags, false);
}


mpu6050_err_t mpu6050_enable_fifo (mpu6050_i2c_cfg_t *cfg, uint8_t flags) {
	return mpu6050_write_register(cfg, REG_USER_CTRL, flags, false);
}


mpu6050_err_t mpu6050_configure_fifo (mpu6050_i2c_cfg_t *cfg, uint8_t flags) {
	return mpu6050_write_register(cfg, REG_FIFO_CFG, flags, false);
}


mpu6050_err_t mpu6050_enable_interrupt (mpu6050_i2c_cfg_t *cfg, 
	uint8_t flags) {
	return mpu6050_write_register(cfg, REG_INTR_EN, flags, false);
}


mpu6050_err_t mpu6050_configure_interrupt (mpu6050_i2c_cfg_t *cfg, 
	uint8_t flags) {
	return mpu6050_write_register(cfg, REG_INTR_CFG, flags, false);
}


mpu6050_err_t mpu6050_clear_interrupt (mpu6050_i2c_cfg_t *cfg) {
	return mpu6050_write_register(cfg, REG_INTR_STATUS, 
		INTR_EN_FIFO_OFL, false);
}


mpu6050_err_t mpu6050_set_sample_rate_divider (mpu6050_i2c_cfg_t *cfg, 
	uint8_t divident) {
	return mpu6050_write_register(cfg, REG_SAMPLE_RATE_DIV, divident,
		false);
}


mpu6050_err_t mpu6050_configure_dlfp (mpu6050_i2c_cfg_t *cfg, uint8_t filter) {
	return mpu6050_write_register(cfg, REG_DLFP_CFG, filter, false);
}


mpu6050_err_t mpu6050_receive_fifo (mpu6050_i2c_cfg_t *cfg, 
	mpu6050_data_t *data_p) {
	uint8_t fifo[FIFO_BURST_LEN];
	mpu6050_err_t err = MPU6050_ERR_OK;

	// Read specified FIFO buffer size (depends on configuration set)
	for (uint8_t i = 0; i < FIFO_BURST_LEN; ++i) {
		if ((err = mpu6050_write_register(cfg, REG_FIFO, 0x0, 
			true)) != MPU6050_ERR_OK) {
			break;
		}
		if ((err = mpu6050_receive_byte(cfg, fifo + i)) 
			!= MPU6050_ERR_OK) {
			break;
		}
	}

	// Configure data structure
	data_p->ax = (int16_t)fifo[0]  << 8  | (int16_t)fifo[1];
	data_p->ay = (int16_t)fifo[2]  << 8  | (int16_t)fifo[3];
	data_p->az = (int16_t)fifo[4]  << 8  | (int16_t)fifo[5];
	data_p->gx = (int16_t)fifo[6]  << 8  | (int16_t)fifo[7];
	data_p->gy = (int16_t)fifo[8]  << 8  | (int16_t)fifo[9];
	data_p->gz = (int16_t)fifo[10] << 8  | (int16_t)fifo[11];

	return err;
}


mpu6050_err_t mpu6050_get_fifo_length (mpu6050_i2c_cfg_t *cfg, 
	uint16_t *len_p) {
	uint16_t len;
	uint8_t temp;
	mpu6050_err_t err = MPU6050_ERR_OK;

	// Must request MSB first (see datasheet)
	if ((err = mpu6050_write_register(cfg, REG_FIFO_COUNT_H, 0x0, true))
		!= MPU6050_ERR_OK) {
		return err;
	}

	// Read MSB
	if ((err = mpu6050_receive_byte(cfg, &temp)) != MPU6050_ERR_OK) {
		return err;
	} 

	// Shift MSB
	len = (uint16_t)temp << 8;

	// Now request LSB second 
	if ((err = mpu6050_write_register(cfg, REG_FIFO_COUNT_L, 0x0, true))
		!= MPU6050_ERR_OK) {
		return err;
	}

	// Read LSB
	if ((err = mpu6050_receive_byte(cfg, &temp)) != MPU6050_ERR_OK) {
		return err;
	}

	// Construct value and save
	*len_p = (len |= temp);


	return err;
}
