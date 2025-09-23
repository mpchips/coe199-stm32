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

// coefficients to convert pixel number to wavelength
// obtained from sensor's final inspection sheet (provided with sensor)
// wavelength(px) = A0 + B1*px + B2*px^2 + B3*px^3 + B4*px^4 + B5*px^5
#define A0  3.195480337e+02
#define B1  2.690839446e+00
#define B2 -1.092565127e-03
#define B3 -7.882067953e-06
#define B4  8.663338298e-09
#define B5  7.654515822e-12

#define DMA_Config_DisableDMA 	0x00022C14
#define DMA_Config_EnableDMA  	0x00022C15

#define DMA_S0_XferCpltFlag			0x00000020
#define DMA_S0_HalfXferCpltFlag	0x00000010
#define DMA_S0_XferErrFlag			0x00000008
#define DMA_S0_DirModeErrFlag		0x00000004
#define DMA_S0_FIFOErrFlag			0x00000001


/**
 * @brief Initializes all STM peripherals and pins used by
 *        the C12880MA hyperspectral sensor.
 * 
 * @param[in] C12880MA_readings - points to first element of
 *        array of type uint16_t where the raw output readings
 *        will be saved.
 */
void Init_C12880MA(uint16_t C12880MA_readings[288]);

void Init_C12880MA_CLKOUT();
void Init_C12880MA_GPIO();
void Init_C12880MA_EXTI();
void Init_C12880MA_ADC();
void Init_C12880MA_TIM();

/**
 * @brief Configures DMA to perform
 *        peripheral-to-memory transactions at the 
 *        request of the ADC peripheral. Called only
 *        by Init_C12880MA().
 * 
 * @param[in] C12880MA_readings - address of the first element of
 *        array where ADC output will be stored. This is incremented
 *        automatically by the DMA module after every transaction.
 */
void Init_C12880MA_DMA(uint16_t C12880MA_readings[288]);

/**
 * @brief Loads the destination address to be used for a DMA2 
 *        Stream 0 periphal-to-memory transaction.
 * 
 * @param[in] DstAddr - 32-bit destination address
 */
void DMA2_S0_Set_DstAddr(uint32_t DstAddr);

/**
 * @brief Loads the source address to be used for a DMA2 
 *        Stream 0 periphal-to-memory transaction.
 * 
 * @param[in] SrcAddr - 32-bit source address
 */
void DMA2_S0_Set_SrcAddr(uint32_t SrcAddr);

/**
 * @brief Sets the number of data transfers to be done
 *        before the DMA automatically disables itself.
 * 
 * @param[in] NumofDataTransfers - number of transfers
 */
void DMA2_S0_Set_NumofDataTransfers(uint32_t NumofDataTransfers);

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
