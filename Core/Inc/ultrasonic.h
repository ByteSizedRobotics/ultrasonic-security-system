/*
 * ultrasonic.h
 *
 *  Created on: Feb 5, 2025
 *      Author: syeadz
 *
 *  This header file contains the definitions and function declarations
 *  for interfacing with the HC-SR04 ultrasonic sensor.
 *  The functions in this file initialize the sensor and calculate the
 *  distance based on the time it takes for the ultrasonic pulse to
 *  return to the sensor.
 */

#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_

#include "cmsis_os.h"  // RTOS API
#include "main.h"      // GPIO definitions

// Maximum and Minimum distances for valid readings (in cm)
#define U_MAX_DISTANCE 400
#define U_MIN_DISTANCE 3

/**
 * @brief Initializes the ultrasonic sensor with the provided configuration.
 *
 * @param timer Pointer to timer handle used as trigger.
 * @param echo_start_time Pointer to a volatile variable to store the start time of the echo pulse.
 * @param echo_end_time Pointer to a volatile variable to store the end time of the echo pulse.
 * @param flag_os_thread The flag used to indicate when the echo measurement is complete in the RTOS.
 */
void ultrasonic_init(TIM_HandleTypeDef *timer, volatile uint32_t *echo_start_time, volatile uint32_t *echo_end_time, uint32_t flag_os_thread);

/**
 * @brief Gets the distance measurement from the ultrasonic sensor.
 *
 * The function triggers the ultrasonic sensor to send a pulse, then waits for the echo. It calculates
 * the distance based on the time it takes for the echo to return and ensures that the distance is within
 * a valid range.
 *
 * @return The calculated distance in centimeters, or -1 if the distance is out of range.
 */
float ultrasonic_get_distance();

#endif /* INC_ULTRASONIC_H_ */
