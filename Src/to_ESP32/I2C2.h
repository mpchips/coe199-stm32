/*
 * I2C2.h
 *
 *  Created on: Mar 21, 2025
 *      Author: moreypiatos
 */

#include <stdint.h>

#ifndef I2C2_H_
#define I2C2_H_

/**
 * @brief Initializes I2C Peripheral
 * */
void Init_I2C2();

/**
 * @brief Initiate start condition. This automatically
 * 		  sets STM as I2C master.
 * */
void I2C2_start();

/**
 * @brief Closes a transaction
 * */
void I2C2_stop();

/**
 * @brief transmits one byte of data
 * */
void I2C2_transmit(uint8_t data);

void I2C2_transmit_addr(uint8_t addr);

int I2C2_transmit_done();

uint8_t I2C2_receive();

/**
 * @brief Reads 2 bytes of data (upper + lower) from slave device
 * */
uint16_t I2C2_read_2b(uint8_t slave_addr, uint8_t reg_upper_addr, uint8_t reg_lower_addr);

/**
 * @brief Reads one byte of data from slave device
 * */
uint8_t I2C2_read(uint8_t slave_addr, uint8_t reg_addr);

#endif /* I2C2_H_ */
