#if !defined(SIGNALS_H)
#define SIGNALS_H

/*
 *******************************************************************************
 *                        (C) Copyright 2020 IoT Seminar                       *
 * Created: 01/03/2020                                                         *
 *                                                                             *
 * Programmer(s):                                                              *
 * - Sonnya Dellarosa                                                          *
 * - Charles Randolph                                                          *
 *                                                                             *
 * Description:                                                                *
 *  Holds global signal group                                                  *
 *                                                                             *
 *******************************************************************************
*/


#include "freertos/FreeRTOS.h"


/*
 *******************************************************************************
 *                              Global Variables                               *
 *******************************************************************************
*/


// [EXTERN] Global event signal group
extern EventGroupHandle_t g_signal_group;


#endif