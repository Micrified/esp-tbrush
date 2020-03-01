#include "imu_task.h"


/*
 *******************************************************************************
 *                              Global Variables                               *
 *******************************************************************************
*/


// Event queue holding GPIO interrupt events
static xQueueHandle g_gpio_event_queue = NULL;


// The GPIO pin bit-mask
uint16_t g_gpio_pin_mask = ((uint64_t)1) << (uint64_t)(I2C_IMU_INTR_PIN);


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


// Performs the I2C configuration for the MPU-6050 IMU
static mpu6050_err_t init_imu (void) {
	mpu6050_err_t err = MPU6050_ERR_OK;
	uint8_t flags;

    // Configure the MPU-6050 I2C data structure
    mpu6050_i2c_cfg_t i2c_cfg = (mpu6050_i2c_cfg_t) {
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

    // Set the sampling rate to ~100Hz
    flags = 0x9;
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

    return err;
}


/*
 *******************************************************************************
 *                            Function Definitions                             *
 *******************************************************************************
*/


void *task_imu (void *args) {
	esp_err_t err = ESP_OK;
	mpu6050_data_t data;
	uint32_t signals;
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
    if ((res = init_imu()) != MPU6050_ERR_OK) {
    	ERR("%s", mpu6050_err_to_str(res));
    	goto esc;
    }

    do {

    	signals = xEventGroupWaitBits();


    } while (1);


	// Escape label: Requires system reset
esc:
	ERR("%s is suspended due to unexpected circumstances!", IMU_TASK_NAME);
	// Signal system reset
	// xEventGroupSetBits
}