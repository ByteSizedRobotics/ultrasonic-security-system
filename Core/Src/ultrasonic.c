/*
 * ultrasonic.c
 *
 *  Created on: Feb 5, 2025
 *      Author: syeadz
 *
 *  This source file contains the implementation of the functions for interfacing with the HC-SR04 ultrasonic sensor.
 *  It includes functions for initializing the sensor, triggering measurements, and calculating the distance based
 *  on the timing of the echo pulse.
 */

#include "ultrasonic.h"

// Structure to hold the configuration and state of the ultrasonic sensor
struct ultrasonic_conf {
	TIM_HandleTypeDef *timer;			 /**< Pointer for timer handle used for trigger pulse. */
    volatile uint32_t *echo_start_time;  /**< Pointer to the start time of the echo pulse. */
    volatile uint32_t *echo_end_time;    /**< Pointer to the end time of the echo pulse. */
    uint32_t flag_os_thread;             /**< OS flag used to signal completion of the echo measurement. */
} conf;


void ultrasonic_init(TIM_HandleTypeDef *timer, volatile uint32_t *echo_start_time, volatile uint32_t *echo_end_time, uint32_t flag_os_thread) {
    conf.timer = timer;						  // Set pointer to timer handle
	conf.echo_start_time = echo_start_time;   // Set pointer to echo start time
    conf.echo_end_time = echo_end_time;       // Set pointer to echo end time
    conf.flag_os_thread = flag_os_thread;     // Set the OS flag to notify completion
}


float ultrasonic_get_distance() {
    // Trigger the ultrasonic pulse
	__HAL_TIM_ENABLE(conf.timer);

    // Wait for the OS flag to indicate that the echo measurement is done
    osThreadFlagsWait(conf.flag_os_thread, osFlagsWaitAny, osWaitForever);

    // Reset the OS flag after the measurement is done
    osThreadFlagsClear(conf.flag_os_thread);

    float distance;

    // Calculate the distance by checking the time difference between the start and end times of the echo
    if (*conf.echo_start_time < *conf.echo_end_time) {
        // Normal case: echo_end_time is after echo_start_time
        distance = (*conf.echo_end_time - *conf.echo_start_time) * 0.034 / 2;
    } else {
        // Edge case: echo_end_time might have wrapped around
        distance = ((0xFFFFFF - *conf.echo_start_time) - *conf.echo_end_time) * 0.034 / 2;
    }

    // Check if the distance is within the valid range
    if (distance > U_MAX_DISTANCE || distance < U_MIN_DISTANCE) return -1;

    return distance;
}
