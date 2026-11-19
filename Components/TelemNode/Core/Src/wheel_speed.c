/*
 * wheel_speed.c
 *
 *  Created on: Apr 17, 2024
 *      Author: leoja
 */


#include "wheel_speed.h"

void WheelSpeed_Init(WheelSpeed_t* ws, TIM_HandleTypeDef* tm)
{
	ws->last_count = 0;
	ws->count = 0;
	ws->last_tick = HAL_GetTick();
	ws->tim = tm;
}

uint32_t WheelSpeed_GetCPS(WheelSpeed_t* ws)
{
	uint32_t tick = HAL_GetTick();
	uint32_t dif = (tick - ws->last_tick);

	ws->count = __HAL_TIM_GET_COUNTER(ws->tim);

	// prevent divide by 0
	if(dif == 0) dif = 1;
	//cps = clicks per second
	// for a 60 slot sensor ring, clicks per second == RPM
	// clicks / second = clicks / (ticks / 1000) = 1000 * clicks / ticks
	uint32_t cps = 1000 * (ws->count - ws->last_count) / dif;

	ws->last_count = ws->count;
	ws->last_tick = tick;

	return cps;
}
