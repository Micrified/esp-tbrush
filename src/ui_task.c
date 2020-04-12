#include "ui_task.h"


/*
 *******************************************************************************
 *                              Global Variables                               *
 *******************************************************************************
*/


// Access control semaphore for the busy task status
SemaphoreHandle_t g_sem_access_control;


/*
 *******************************************************************************
 *                        Internal Function Definitions                        *
 *******************************************************************************
*/



// GPIO reset button interrupt handler
static void IRAM_ATTR g_gpio_reset_btn_isr_handler (void *arg) {
	// uint32_t gpio_id = (uint32_t)arg;
	BaseType_t isHighPrioWaiting, xResult;


	// Try to take the semaphore. If cannot - ignore input
	if (xSemaphoreTakeFromISR(g_sem_access_control, &isHighPrioWaiting)) {

		// Set the reset group flag
		xResult = xEventGroupSetBitsFromISR(g_signal_group, CTRL_SIGNAL_BTN_TOGGLE,
			&isHighPrioWaiting);

		// Give back to semaphore
		xSemaphoreGiveFromISR(g_sem_access_control, &isHighPrioWaiting);

		// Yield to any high priority tasks
		portYIELD_FROM_ISR();
	}
}


/*
 *******************************************************************************
 *                            Function Definitions                             *
 *******************************************************************************
*/


void task_ui (void *args) {
	esp_err_t err = ESP_OK;
	uint64_t output_pin_bit_mask = ((uint64_t)1) << (uint64_t)(GPIO_BUZZER_PIN);
	uint64_t input_pin_bit_mask 
		= ((uint64_t)1) << (uint64_t)(GPIO_RESET_BUTTON_PIN);
	const TickType_t poll_time = UI_TASK_POLL_PERIOD / portTICK_PERIOD_MS;
	ui_action_t action;

	// Initialize the semaphore
	if ((g_sem_access_control = xSemaphoreCreateBinary()) == NULL) {
		ERR("Unable to initialize access semaphore!");
		goto esc;
	}

	// Give to the semaphore (required since it initializes with zero)
	if (xSemaphoreGive(g_sem_access_control) != pdTRUE) {
		ERR("Unable to give to the access semaphore!");
		goto esc;
	}

	// Prepare GPIO configuration for output
	gpio_config_t gpio_output_cfg = (gpio_config_t) {
			.pin_bit_mask  = output_pin_bit_mask,
			.mode          = GPIO_MODE_OUTPUT,
			.pull_up_en    = GPIO_PULLUP_DISABLE,
			.pull_down_en  = GPIO_PULLDOWN_DISABLE,
			.intr_type     = GPIO_PIN_INTR_DISABLE
	};

	// Prepare GPIO configuration for input
	gpio_config_t gpio_input_cfg = (gpio_config_t) {
		.pin_bit_mask     = input_pin_bit_mask,
		.mode             = GPIO_MODE_INPUT,
		.pull_up_en       = GPIO_PULLUP_DISABLE,
		.pull_down_en     = GPIO_PULLDOWN_ENABLE,
		.intr_type        = GPIO_INTR_POSEDGE
	};

	// Apply output GPIO configuration
	if ((err = gpio_config(&gpio_output_cfg)) != ESP_OK) {
		ERR("Unable to output configure GPIO!");
		goto esc;
	}

	// Apply input GPIO configuration
	if ((err = gpio_config(&gpio_input_cfg)) != ESP_OK) {
		ERR("Unable to configure input GPIO!");
		goto esc;
	}

	// Attach the ISR handler for the configured pin
	if ((err = gpio_isr_handler_add(GPIO_RESET_BUTTON_PIN,
		g_gpio_reset_btn_isr_handler, (void *)GPIO_RESET_BUTTON_PIN
		)) != ESP_OK) {
        if (err == ESP_ERR_INVALID_STATE) {
            ERR("Must initialize ISR service: gpio_install_isr_service()!");
        } else if (err == ESP_ERR_INVALID_ARG) {
            ERR("Bad parameter!");
        } else {
            ERR("Invalid error type!");
        }
        goto esc;
	}

	// Run main loop
	do {

		// Wait for work to do
		if (xQueueReceive(g_ui_action_queue, &action, poll_time)) {

			// Take the semaphore
			if (xSemaphoreTake(g_sem_access_control, (TickType_t)16) != pdTRUE) {
				continue;
			}

			// Process periods
			do {

				// Enable UI devices
				if (action.flags & UI_ACTION_BUZZER) {
					gpio_set_level(GPIO_BUZZER_PIN, 1);
				}
				if (action.flags & UI_ACTION_VIBRATION) {
					// TODO: enable vibration
				}

				// Leave on
				vTaskDelay(action.duration / portTICK_PERIOD_MS);

				// If no periods, break
				if (action.periods <= 0) {
					break;
				} else {
					action.periods--;
				}

				// Shut off output for equal duration
				gpio_set_level(GPIO_BUZZER_PIN, 0);
				// TODO: shut off vibration

				// Delay
				vTaskDelay(action.duration / portTICK_PERIOD_MS);
				
			} while (1);

			// Release the semaphore
			xSemaphoreGive(g_sem_access_control);
		}

		// Clear any actions
		gpio_set_level(GPIO_BUZZER_PIN, 0);
		// TODO: clear vibration

	} while (1);


esc:

	ERR(UI_TASK_NAME " is suspended due to unexpected circumstances!");

	// Signal system reset
	xEventGroupSetBits(g_signal_group, SIGNAL_TASK_FAULT);
}