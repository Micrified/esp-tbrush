/*
 *******************************************************************************
 *                        (C) Copyright 2020 IoT Seminar                       *
 * Created: 21/02/2020                                                         *
 *                                                                             *
 * Programmer(s):                                                              *
 * - Sonnya Dellarosa                                                          *
 * - Charles Randolph                                                          *
 *                                                                             *
 * Description:                                                                *
 *  Defines useful error-handling macros                                       *
 *                                                                             *
 *******************************************************************************
*/

#if !defined(ERRORS)
#define ERRORS

#include "esp_log.h"

/*
 *******************************************************************************
 *                             Symbolic Constants                              *
 *******************************************************************************
*/


// Error macro including metadata
#define ERR(msg) ESP_LOGE("Error", "%s | %s:%d", (msg), __FILE__, __LINE__)



#endif