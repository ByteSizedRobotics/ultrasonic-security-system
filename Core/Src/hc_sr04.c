/*
 * hc_sr04.c
 *
 *  Created on: Feb 5, 2025
 *      Author: syeadz
 */

#include "hc_sr04.h"

float hc_sr04_get_distance() {
	HAL_GPIO_WritePin(S_TRIG_GPIO_Port, S_TRIG_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(S_TRIG_GPIO_Port, S_TRIG_Pin, GPIO_PIN_RESET);

	float distance;
	if (g_echoStartTime < g_echoEndTime) {
		distance = (g_echoEndTime - g_echoStartTime) * 0.034 / 2;
	} else {
		distance = ((0xFFFFFF - g_echoStartTime) - g_echoEndTime) * 0.034 / 2;
	}

	if (distance > 800) return -1; // timeout, no echo

	return distance;
}
