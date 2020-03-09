#include "ui_task.h"


/*
 *******************************************************************************
 *                              Global Variables                               *
 *******************************************************************************
*/



/*
 *******************************************************************************
 *                            Function Definitions                             *
 *******************************************************************************
*/


void task_ui (void *args) {
	esp_err_t err = ESP_OK;
	uint64_t pin_bit_mask = ((uint64_t)1) << (uint64_t)(GPIO_BUZZER_PIN);
	const TickType_t poll_time = UI_TASK_POLL_PERIOD / portTICK_PERIOD_MS;
	ui_action_t action;

	// Prepare GPIO configuration
	gpio_config_t gpio_cfg = (gpio_config_t) {
			.pin_bit_mask  = pin_bit_mask,
			.mode          = GPIO_MODE_OUTPUT,
			.pull_up_en    = GPIO_PULLUP_DISABLE,
			.pull_down_en  = GPIO_PULLDOWN_DISABLE,
			.intr_type     = GPIO_PIN_INTR_DISABLE
	};

	// Apply GPIO configuration
	if ((err = gpio_config(&gpio_cfg)) != ESP_OK) {
		ERR("Unable to configure GPIO!");
		goto esc;
	}


	// Run main loop
	do {

		// Wait for work to do
		if (xQueueReceive(g_ui_action_queue, &action, poll_time)) {

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