#include "imu_task.h"


/*
 *******************************************************************************
 *                              Global Variables                               *
 *******************************************************************************
*/


// Event queue holding GPIO interrupt events
static xQueueHandle g_gpio_event_queue = NULL;


// Internal calibration data structure
mpu6050_data_t g_calibration_data = (mpu6050_data_t) {
	.ax = 0, .ay = 0, .az = 0, .gx = 0, .gy = 0, .gz = 0
};


/*
 *******************************************************************************
 *                        Internal Function Definitions                        *
 *******************************************************************************
*/


// GPIO interrupt handler
static void IRAM_ATTR g_gpio_isr_handler (void *arg) {
	uint32_t gpio_id = (uint32_t)arg;

	// Push interrupt event to queue
	xQueueSendFromISR(g_gpio_event_queue, &gpio_id, NULL);
}


// Performs the I2C configuration for the MPU-6050 IMU. Saves handle
static mpu6050_err_t init_imu (mpu6050_i2c_cfg_t **handle) {
	mpu6050_err_t err = MPU6050_ERR_OK;
	uint8_t flags;

    // Configure the MPU-6050 I2C data structure
    static mpu6050_i2c_cfg_t i2c_cfg = (mpu6050_i2c_cfg_t) {
        .sda_pin        = I2C_SDA_PIN,
        .scl_pin        = I2C_SCL_PIN,
        .slave_addr     = I2C_IMU_SLAVE_ADDR,
        .i2c_port       = I2C_IMU_PORT_NUM,
        .clk_speed      = I2C_APB_CLK_FREQ / 200,    // Requires 400kHz
        .sda_pullup_en  = IMU_ENABLE_INTERNAL_PULLUPS,
        .scl_pullup_en  = IMU_ENABLE_INTERNAL_PULLUPS
    };

    // Initialize I2C
    if ((err = mpu6050_init(&i2c_cfg)) != MPU6050_ERR_OK) {
        return err;
    }

    // Configure Power Management 1 to wake the IMU (don't reset)
    flags = 0x0;
    if ((err = mpu6050_configure_power(&i2c_cfg, flags)) != MPU6050_ERR_OK) {
    	return err;
    }

    // Configure accelerometer sensitivity
    flags = A_CFG_8G;
    if ((err = mpu6050_configure_accelerometer(&i2c_cfg, flags)) 
    	!= MPU6050_ERR_OK) {
    	return err;
    }

    // Configure gyro sensitivity
    flags = G_CFG_500;
    if ((err = mpu6050_configure_gyroscope(&i2c_cfg, flags)) 
    	!= MPU6050_ERR_OK) {
    	return err;
    }

    // Configure the Digital-Low-Pass-Filter
    flags = DLFP_CFG_FILTER_2;
    if ((err = mpu6050_configure_dlfp(&i2c_cfg, flags)) 
    	!= MPU6050_ERR_OK) {
    	return err;
    }

    // Set the sampling rate to ~50Hz
    flags = 19;
    if ((err = mpu6050_set_sample_rate_divider(&i2c_cfg, flags)) 
    	!= MPU6050_ERR_OK) {
    	return err;
    }

    // Configure interrupt behavior
    flags = 0x0;
    if ((err = mpu6050_configure_interrupt(&i2c_cfg, flags)) 
    	!= MPU6050_ERR_OK) {
    	return err;
    }

    // Enable interrupts after every sensor refresh
    flags = INTR_EN_DATA_RDY;
    if ((err = mpu6050_enable_interrupt(&i2c_cfg, flags)) 
    	!= MPU6050_ERR_OK) {
    	return err;
    }

    // Enable + Reset the FIFO
    flags = USER_CTRL_FIFO_EN | USER_CTRL_FIFO_RST;
    if ((err = mpu6050_enable_fifo(&i2c_cfg, flags)) 
    	!= MPU6050_ERR_OK) {
    	return err;
    }

    // Configure the data pushed to the FIFO
    flags = FIFO_CFG_GX | FIFO_CFG_GY | FIFO_CFG_GZ | FIFO_CFG_AXYZ;
    if ((err = mpu6050_configure_fifo(&i2c_cfg, flags)) != MPU6050_ERR_OK) {
    	return err;
    }

    // Save the configuration
    *handle = &i2c_cfg;

    return err;
}


/*
 *******************************************************************************
 *                            Function Definitions                             *
 *******************************************************************************
*/


void task_imu (void *args) {
	esp_err_t err = ESP_OK;
	mpu6050_data_t data;
	uint32_t signals, pin_id;
	int16_t calibration_ticks = 0;
	uint8_t mode = 1;
	uint16_t len;
	mpu6050_i2c_cfg_t *i2c_cfg_p = NULL;
	uint64_t pin_bit_mask = ((uint64_t)1) << (uint64_t)(I2C_IMU_INTR_PIN);
	const TickType_t event_block_time = 1;

	// Initialize the event-queue
	if ((g_gpio_event_queue = xQueueCreate(IMU_INTERRUPT_QUEUE_SIZE,
		sizeof(uint32_t))) == NULL) {
		ERR("Insufficient memory to create interrupt queue!");
		goto esc;
	}

	// Prepare GPIO configuration
    gpio_config_t gpio_cfg = (gpio_config_t) {
        .pin_bit_mask    = pin_bit_mask,
        .mode            = GPIO_MODE_INPUT,
        .pull_up_en      = GPIO_PULLUP_DISABLE,
        .pull_down_en    = GPIO_PULLDOWN_ENABLE,
        .intr_type       = GPIO_INTR_POSEDGE 
    };

	// Apply GPIO configuration
	if ((err = gpio_config(&gpio_cfg)) != ESP_OK) {
		ERR("Unable to configure GPIO!");
		goto esc;
	}

	// Install the GPIO interrupt service
	if ((err = gpio_install_isr_service(I2C_IMU_INTR_FLAG_DEFAULT))
	 != ESP_OK) {
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

	// Attach the ISR handler for the configured pin
    if ((err = gpio_isr_handler_add(I2C_IMU_INTR_PIN, g_gpio_isr_handler, 
        (void *)I2C_IMU_INTR_PIN) // Ugly hack to pass PIN as "pointer"
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

    // Initialize and configure the MPU-6050 IMU
    mpu6050_err_t res;
    if ((res = init_imu(&i2c_cfg_p)) != MPU6050_ERR_OK) {
    	ERR(mpu6050_err_to_str(res));
    	goto esc;
    }

    do {

    	// Check for signals within the given block time
    	signals = xEventGroupWaitBits(g_signal_group, IMU_SIGNAL_MASK, 
    		pdTRUE, pdFALSE, event_block_time);


    	// Check if calibration is necessary
    	if (signals & IMU_SIGNAL_CALIBRATE) {
    		mode |= IMU_TASK_MODE_CALIBRATION;
    	}


    	// Process a sample
    	if (xQueueReceive(g_gpio_event_queue, &pin_id, 0x0)) {

    		// Check the FIFO length
    		if (mpu6050_get_fifo_length(i2c_cfg_p, &len) != MPU6050_ERR_OK) {
    			ERR("FIFO length fetch error!");
    			break;
    		}
    		if (len < FIFO_BURST_LEN) {
    			continue;
    		}


    		// Fetch data from FIFO
    		if (mpu6050_receive_fifo(i2c_cfg_p, &data) != MPU6050_ERR_OK) {
    			ERR("FIFO data fetch error!");
    			break;
    		}

    		// If not in calibration mode, just print the data for now
    		if ((mode & IMU_TASK_MODE_CALIBRATION) == 0) {
    			printf("%d, %d, %d, %d, %d, %d\n", 
    				data.ax - g_calibration_data.ax,
    				data.ay - g_calibration_data.ay,
    				data.az - g_calibration_data.az,
    				data.gx - g_calibration_data.gx,
    				data.gy - g_calibration_data.gy,
    				data.gz - g_calibration_data.gz);
    			continue;
    		}

    		// If calibration is complete, unset flag and counter
    		if (calibration_ticks >= IMU_CALIBRATION_SAMPLE_SIZE) {
    			mode &= ~(IMU_TASK_MODE_CALIBRATION);
    			calibration_ticks = 0;
    			ESP_LOGI(IMU_TASK_NAME, "Calibration Complete");
    			printf("ax: %d, ay: %d, az: %d, gx: %d, gy: %d, gz: %d\n", 
    				g_calibration_data.ax,
    				g_calibration_data.ay,
    				g_calibration_data.az,
    				g_calibration_data.gx,
    				g_calibration_data.gy,
    				g_calibration_data.gz);

    			// Configure an action to acknowledge startup
    			ui_action_t action = (ui_action_t) {
    				.flags = UI_ACTION_VIBRATION | UI_ACTION_BUZZER,
    				.duration = 50,
    				.periods = 2
    			};

    			// Request a UI feedback event for startup
    			if (xQueueSendToBack(g_ui_action_queue, &action, 0) != pdTRUE) {
    				ERR("UI Action Queue overfull!");
    			}

    			continue;
    		}

    		// Increment calibration ticks
    		calibration_ticks++;

    		// Otherwise continue calibration: AVG(n) = [(n-1)*Avg(n-1) + X(n)]/n
    		g_calibration_data.ax = g_calibration_data.ax + ((data.ax - g_calibration_data.ax) / calibration_ticks);
    		g_calibration_data.ay = g_calibration_data.ay + ((data.ay - g_calibration_data.ay) / calibration_ticks);
    		g_calibration_data.az = g_calibration_data.az + ((data.az - g_calibration_data.az) / calibration_ticks);

    		g_calibration_data.gx = g_calibration_data.gx + ((data.gx - g_calibration_data.gx) / calibration_ticks);
    		g_calibration_data.gy = g_calibration_data.gy + ((data.gy - g_calibration_data.gy) / calibration_ticks);
    		g_calibration_data.gz = g_calibration_data.gz + ((data.gz - g_calibration_data.gz) / calibration_ticks);

    	}


    } while (1);


	// Escape label: Requires system reset
esc:
	ERR(IMU_TASK_NAME " is suspended due to unexpected circumstances!");

	// Signal system reset
	xEventGroupSetBits(g_signal_group, SIGNAL_TASK_FAULT);
}
