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
void Init_C12880MA_TIM();

/**
 * @brief Sets the integration time in µs, from 50µs to 1000000µs only.
 *        Input of below 50 or above 10000000 will clamp the integration
 *        time to the minimum or maximum, respectively.
 * @param tint (uint32_t) integration time in µs
 */
void C12880MA_set_tint(uint32_t tint);

void C12880MA_measure(uint16_t channel_readings[288], uint32_t tint_ms);

/**
 * @brief  Initiates measurement with C12880MA spectrometer.
 * @param  st_pulse_width (uint32_t) [6, 0xFFFFF] On-time of start pulse in terms of
 * 		   sensor clock cycles.
 * @retval none
 **/
void C12880MA_ST(uint32_t st_pulse_width);

/**
 * @brief  DEPRECATED. Initiates measurement with C12880MA spectrometer.
 * @param  channel_readings (uint16_t) 288-long array where each pixel's output value will
 * 		   be stored.
 * @param  on_time (uint32_t) [6, 0xFFFFF] On-time of start pulse in terms of
 * 		   sensor clock cycles.
 * @param  sensor_clk_cycles (uint32_t) variable counting number of sensor clock cycles that
 * 		   has passed. Will be modified by the function (reset) and the
 * 		   interrupt handler (increment).
 * @retval none
 **/
void _C12880MA_start(uint16_t channel_readings[288], uint32_t on_time, uint32_t sensor_clk_cycles);



#endif /* C12880MA_H_ */
