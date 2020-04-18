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


// Handle for the training mode timer
TimerHandle_t g_training_mode_timer = NULL;


// The training buffers
static mpu6050_data_t g_train_data_ll[IMU_TRAINING_SAMPLE_BUF_SIZE];
static mpu6050_data_t g_train_data_lr[IMU_TRAINING_SAMPLE_BUF_SIZE]; 
static mpu6050_data_t g_train_data_tl[IMU_TRAINING_SAMPLE_BUF_SIZE];
static mpu6050_data_t g_train_data_tr[IMU_TRAINING_SAMPLE_BUF_SIZE];

// Training buffer pointer - buffer
static mpu6050_data_t *g_train_data[4] = {
    g_train_data_ll,
    g_train_data_lr,
    g_train_data_tl,
    g_train_data_tr
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


// Creates and enqueues an action sounding the buzzer
static void buzzer_action (uint8_t repeats, uint16_t duration) {
    ui_action_t action = (ui_action_t) {
        .flags = UI_ACTION_BUZZER,
        .duration = duration,
        .periods = repeats
    };

    if (xQueueSendToBack(g_ui_action_queue, &action, 0) != pdTRUE) {
        ERR("Could not send to action queue!");
    }
}


/*
 *******************************************************************************
 *                Classification/Training Function Definitions                 *
 *******************************************************************************
*/


// Called after all training samples are gathered
void on_training_data_complete (void) {

    // Training data is located in these arrays (IMU_TRAINING_SAMPLE_BUF_SIZE)
    // The size is set to 150 (50Hz ~ 3 seconds worth per zone)
    mpu6050_data_t *data_ll = g_train_data_ll;
    mpu6050_data_t *data_lr = g_train_data_lr;
    mpu6050_data_t *data_tl = g_train_data_tl;
    mpu6050_data_t *data_tr = g_train_data_tr;

    // Do whatever with the data 
}


// Called to classify a sample (brush_zone_t is defined in msg.h)
brush_zone_t on_data_classification (mpu6050_data_t *data_p) {

    // Do classification on data provided

    // Return type (max is not allowed - but is a placeholder)
    return BRUSH_MODE_MAX;
}


/*
 *******************************************************************************
 *                            Function Definitions                             *
 *******************************************************************************
*/


void task_imu (void *args) {
	esp_err_t err = ESP_OK;                    // Error tracking
	mpu6050_data_t data;                       // Instantaneous IMU data
	uint32_t signals, pin_id;                  // Signal bits, interrupt pin
	int16_t calibration_ticks = 0;             // Number of elapsed calib ticks
	imu_mode_t mode = IMU_MODE_CALIBRATION;    // Current IMU mode
	uint16_t len;                              // Holds IMU FIFO size
	mpu6050_i2c_cfg_t *i2c_cfg_p = NULL;       // I2C configuration
	uint64_t pin_bit_mask =                    // Interrupt pin mask
    ((uint64_t)1) << (uint64_t)(I2C_IMU_INTR_PIN);
	const TickType_t event_block_time = 1;     // Waiting time for events
    uint16_t sample_counter = 0;               // Counter for calib/train
    uint8_t training_zone = 0;                 // Zone currently being trained
    uint8_t is_pending_training = 0;           // Set to 1 if training pending
    uint8_t is_pending_reset = 0;              // Set to 1 if reset pending

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


        // Check if there was a training mode request
        if ((signals & IMU_SIGNAL_TRAIN_START) != 0) {
            is_pending_training = 1;
        }

        // check if there was a reset mode request
        if ((signals & IMU_SIGNAL_RESET) != 0) {
            is_pending_reset = 1;
        }

    	// Read a new sample from the queue
    	if (xQueueReceive(g_gpio_event_queue, &pin_id, 0x0)) {

    		// Check the FIFO length
    		if (mpu6050_get_fifo_length(i2c_cfg_p, &len) != MPU6050_ERR_OK) {
    			ERR("FIFO length fetch error!");
    			break;
    		}

            // Defer check to next cycle if insufficient data
    		if (len < FIFO_BURST_LEN) {
    			continue;
    		}

    		// Otherwise extract data from the IMU
    		if (mpu6050_receive_fifo(i2c_cfg_p, &data) != MPU6050_ERR_OK) {
    			ERR("FIFO data fetch error!");
    			break;
    		}

    	} else {

            // No sample was ready - defer check to next cycle
            continue;
        }

        // A sample certainly is ready - increment general counter
        sample_counter++;

        // If in calibration mode
        if (mode == IMU_MODE_CALIBRATION) {

            // Increment calibration ticks
            calibration_ticks++;

            // If calibration is complete
            if (calibration_ticks >= IMU_CALIBRATION_SAMPLE_SIZE) {

                // Output calibration data
                printf("ax: %d, ay: %d, az: %d, gx: %d, gy: %d, gz: %d\n", 
                    g_calibration_data.ax,
                    g_calibration_data.ay,
                    g_calibration_data.az,
                    g_calibration_data.gx,
                    g_calibration_data.gy,
                    g_calibration_data.gz);

                // Sound a buzzer to mark end of calibration
                buzzer_action(2, 50);

                // Change mode back to idle
                mode = IMU_MODE_IDLE;

            } else {

                // Otherwise update calibration values
                g_calibration_data.ax = g_calibration_data.ax + ((data.ax - g_calibration_data.ax) / calibration_ticks);
                g_calibration_data.ay = g_calibration_data.ay + ((data.ay - g_calibration_data.ay) / calibration_ticks);
                g_calibration_data.az = g_calibration_data.az + ((data.az - g_calibration_data.az) / calibration_ticks);

                g_calibration_data.gx = g_calibration_data.gx + ((data.gx - g_calibration_data.gx) / calibration_ticks);
                g_calibration_data.gy = g_calibration_data.gy + ((data.gy - g_calibration_data.gy) / calibration_ticks);
                g_calibration_data.gz = g_calibration_data.gz + ((data.gz - g_calibration_data.gz) / calibration_ticks);
            }


            // Defer to next cycle so that we don't reuse current sample across modes
            continue;
        }

        // If in training mode
        if (mode == IMU_MODE_TRAIN) {

            // Check if a reset occurred - this can only affect training mode
            if (is_pending_reset) {

                // Unset the flag
                is_pending_reset = 0;

                // Change modes
                mode = IMU_MODE_IDLE;

                // Defer to next cycle (variables reset from idle -> train)
                continue;
            }

            // If all zones are trained, go back to idle and signal
            if (training_zone >= 4) {

                // Apply training function here
                // TODO
                on_training_data_complete();

                // Signal that training is complete
                xEventGroupSetBits(g_signal_group, CTRL_SIGNAL_TRAIN_DONE);

                // Reset the mode
                mode = IMU_MODE_IDLE;

            } else {

                // If filled buffer, switch training zone and reset
                if (sample_counter > IMU_TRAINING_SAMPLE_BUF_SIZE) {
                    training_zone++;
                    sample_counter = 0;

                    // Sound buzzer
                    buzzer_action(1, 50);

                } else {

                    // Store data in training set
                    g_train_data[training_zone][sample_counter - 1] = data;
                }
            }

            // Defer to next cycle to not resuse current sample across modes
            continue;
        }

        // If in idle mode
        if (mode == IMU_MODE_IDLE) {

            // Transition to training mode if requested
            if (is_pending_training) {

                // Update the task mode
                mode = IMU_MODE_TRAIN;

                // Unset pending flag (now being handled)
                is_pending_training = 0;

                // Reset counter
                sample_counter = 0;

                // Reset training zone to zero
                training_zone = 0;

                // Defer to next cycle
                continue;
            }

            // Otherwise in normal mode

            // [DEBUG] Print normal data
            /* printf("%d, %d, %d, %d, %d, %d\n", 
                data.ax - g_calibration_data.ax,
                data.ay - g_calibration_data.ay,
                data.az - g_calibration_data.az,
                data.gx - g_calibration_data.gx,
                data.gy - g_calibration_data.gy,
                data.gz - g_calibration_data.gz); */

            // Push data to the processed queue at a rate of 5Hz
            if ((sample_counter % 10) == 0) {
                imu_proc_data_t proc_data = (imu_proc_data_t){
                    .rate = sample_counter,
                    .zone = on_data_classification(&data)
                };
                if (xQueueSendToBack(g_processed_data_queue, &proc_data, 0)
                    != pdTRUE) {
                    ERR("Processed Data Queue overfull!");
                }
            }

        }


    } while (1);


	// Escape label: Requires system reset
esc:
	ERR(IMU_TASK_NAME " is suspended due to unexpected circumstances!");

	// Signal system reset
	xEventGroupSetBits(g_signal_group, SIGNAL_TASK_FAULT);
}
