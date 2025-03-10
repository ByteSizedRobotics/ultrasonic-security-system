#include "stepper.h"

/// @brief Full drive sequence for stepper motor control
/// @details This array contains the 4-step sequence used for controlling the stepper motor in full drive mode.
uint8_t full_drive_sequence[4][4] = {
    {1, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 1},
    {1, 0, 0, 1},
};

/// @brief Half drive sequence for stepper motor control
/// @details This array contains the 8-step sequence used for controlling the stepper motor in half drive mode.
uint8_t half_drive_sequence[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1},
};

/// @brief Current step index for the stepper motor
/// @details Holds the current step of the stepper motor for controlling rotation.
volatile uint8_t cur_step = 0;

struct stepper_conf {
    TIM_HandleTypeDef *timer;
    uint8_t timer_int;
    uint8_t direction;
    float angle;
} stepper_conf;


void stepper_init(TIM_HandleTypeDef *timer, uint8_t timer_int) {
    stepper_conf.timer = timer;
    stepper_conf.timer_int = timer_int;
    stepper_conf.direction = 0;
    stepper_conf.angle = 0;
}


void stepper_half_drive()
{
    HAL_GPIO_WritePin(S_IN1_PORT, S_IN1_PIN, half_drive_sequence[cur_step][0]);
    HAL_GPIO_WritePin(S_IN2_PORT, S_IN2_PIN, half_drive_sequence[cur_step][1]);
    HAL_GPIO_WritePin(S_IN3_PORT, S_IN3_PIN, half_drive_sequence[cur_step][2]);
    HAL_GPIO_WritePin(S_IN4_PORT, S_IN4_PIN, half_drive_sequence[cur_step][3]);

    if (stepper_conf.direction == 0)
        cur_step = (cur_step + 1) % 8;
    else
        cur_step = (cur_step == 0) ? 7 : cur_step - 1;
}


void stepper_full_drive(void)
{
    // Drive the stepper motor pins using the current full-drive sequence
    HAL_GPIO_WritePin(S_IN1_PORT, S_IN1_PIN, full_drive_sequence[cur_step][0]);
    HAL_GPIO_WritePin(S_IN2_PORT, S_IN2_PIN, full_drive_sequence[cur_step][1]);
    HAL_GPIO_WritePin(S_IN3_PORT, S_IN3_PIN, full_drive_sequence[cur_step][2]);
    HAL_GPIO_WritePin(S_IN4_PORT, S_IN4_PIN, full_drive_sequence[cur_step][3]);

    // Update current step and angle based on the desired direction.
    if (stepper_conf.direction == 0) {
        // Clockwise rotation: increment step and add angle
        cur_step = (cur_step + 1) % 4;
        stepper_conf.angle += 0.17578125;
    } else {
        // Counter-clockwise rotation: decrement step and subtract angle
        cur_step = (cur_step == 0) ? 3 : cur_step - 1;
        stepper_conf.angle -= 0.17578125;
    }

    // Normalize the angle so that it stays within 0 to 360 degrees
    stepper_conf.angle = fmod(stepper_conf.angle, 360.0);
    if (stepper_conf.angle < 0)
        stepper_conf.angle += 360.0;
}


void stepper_change_direction() {
    if (stepper_conf.direction == 0) {
        stepper_conf.direction = 1;
        cur_step = (cur_step == 0) ? (sizeof(half_drive_sequence) / sizeof(half_drive_sequence[0])) - 1 : cur_step - 1;
    } else {
        stepper_conf.direction = 0;
        cur_step = (cur_step + 1) % (sizeof(half_drive_sequence) / sizeof(half_drive_sequence[0]));
    }
}


void stepper_set_speed(uint16_t counter) {
	__HAL_TIM_SET_AUTORELOAD(stepper_conf.timer, counter-1);
	stepper_conf.timer->Instance->EGR |= TIM_EGR_UG;
}


float stepper_get_angle() {
	return stepper_conf.angle;
}


void stepper_enable()
{
	__HAL_TIM_SET_COUNTER(stepper_conf.timer, 0);
	__HAL_TIM_ENABLE_IT(stepper_conf.timer, TIM_IT_UPDATE);  // Allow the timer to generate the interrupt
	HAL_NVIC_EnableIRQ(stepper_conf.timer_int);            // Allow the CPU to handle it
}


void stepper_disable()
{
	__HAL_TIM_DISABLE_IT(stepper_conf.timer, TIM_IT_UPDATE);  // Stop generating the interrupt
	HAL_NVIC_DisableIRQ(stepper_conf.timer_int);           // Stop the CPU from handling it
}
