#include "ble_task.h"


/*
 *******************************************************************************
 *                            Function Definitions                             *
 *******************************************************************************
*/


void task_ble (void *args) {
	esp_err_t err = ESP_OK;

	// Run main loop
	do {


	} while (1);


esc:

	ERR(BLE_TASK_NAME " is suspended due to unexpected circumstances!");

	// Signal system reset
	xEventGroupSetBits(g_signal_group, SIGNAL_TASK_FAULT);
}