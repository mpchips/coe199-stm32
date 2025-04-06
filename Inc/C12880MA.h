/*
 * C12880MA.h
 *
 *  Created on: Feb 6, 2025
 *      Author: moreypiatos
 */

#include <stdio.h>
#include <stm32f4xx.h>
#include <stm32f411xe.h>

#ifndef C12880MA_H_
#define C12880MA_H_



void Init_C12880MA();

void Init_C12880MA_CLKOUT();
void Init_C12880MA_GPIO();
void Init_C12880MA_EXTI();
void Init_C12880MA_ADC();

void C12880MA_measure(uint16_t channel_readings[288], uint32_t tint_ms);

void C12880MA_start(uint16_t channel_readings[288], uint32_t on_time, uint32_t sensor_clk_cycles);

void Init_TIM2();


#endif /* C12880MA_H_ */
