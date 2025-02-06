/*
 * globals.h
 *
 *  Created on: Feb 5, 2025
 *      Author: syeadz
 */

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

#include "cmsis_os2.h"

extern volatile uint32_t g_echoStartTime;  // Stores the timestamp at the rising edge
extern volatile uint32_t g_echoEndTime;  // Stores the timestamp at the falling edge

#endif /* INC_GLOBALS_H_ */
