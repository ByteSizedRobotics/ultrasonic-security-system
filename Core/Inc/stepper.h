/*
 * stepper.h
 *
 *  Created on: Feb 17, 2025
 *      Author: syeadz
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#include "main.h"
#include "stdint.h"
#include "math.h"

// Set ports and pins for motor
#define S_IN1_PORT M_A_GPIO_Port
#define S_IN2_PORT M_B_GPIO_Port
#define S_IN3_PORT M_C_GPIO_Port
#define S_IN4_PORT M_D_GPIO_Port
#define S_IN1_PIN M_A_Pin
#define S_IN2_PIN M_B_Pin
#define S_IN3_PIN M_C_Pin
#define S_IN4_PIN M_D_Pin

/// @brief Initializes the stepper motor configuration
/// @param timer A pointer to the timer handle that will be used for stepper control
/// @param timer_int The interrupt number for the timer
/// @details This function sets the timer and interrupt number for the stepper motor control.
void stepper_init(TIM_HandleTypeDef *timer, uint8_t timer_int);

/// @brief Performs a half-step drive sequence for the stepper motor
/// @details This function sets the GPIO pins according to the current step in the half-step drive sequence and increments the step.
void stepper_half_drive(void);

/// @brief Performs a full-step drive sequence for the stepper motor
/// @details This function sets the GPIO pins according to the current step in the full-step drive sequence and increments the step.
void stepper_full_drive(void);

/// @brief Sets the stepper motor's speed by adjusting the timer's auto-reload value
/// @param counter The value to set the timer's auto-reload register to (determines the speed)
/// @details This function adjusts the timer's counter to control the stepper motor's speed.
void stepper_set_speed(uint16_t counter);

float stepper_get_angle();

void stepper_change_direction(void);

/// @brief Enables the stepper motor control by starting the timer and enabling interrupts
/// @details This function initializes the timer and allows interrupts to handle stepper motor steps.
void stepper_enable(void);

/// @brief Disables the stepper motor control by stopping the timer and disabling interrupts
/// @details This function stops the timer and disables interrupts for stepper motor control.
void stepper_disable(void);

#endif /* INC_STEPPER_H_ */
