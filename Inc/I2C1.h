/*
 * I2C1.h
 *
 *  Created on: Feb 6, 2025
 *      Author: moreypiatos
 */

#include <stdint.h>

#ifndef I2C1_H_
#define I2C1_H_

/**
 * @brief Initializes I2C Peripheral
 * */
void Init_I2C();

/**
 * @brief Initiate start condition. This automatically
 * 		  sets STM as I2C master.
 * */
void I2C_start();

/**
 * @brief Closes a transaction
 * */
void I2C_stop();

/**
 * @brief transmits one byte of data
 * */
void I2C_transmit(uint8_t data);

void I2C_transmit_addr(uint8_t addr);

int I2C_transmit_done();

uint8_t I2C_receive();

/**
 * @brief Writes data to slave's register address.
 *        May be followed by another data transfer
 *        or a STOP (call I2C_stop()).
 *
 * */
void I2C_write_open(uint8_t slave_addr, uint8_t reg_addr, uint8_t data);

/**
 * @brief Reads one byte of data from slave device
 * */
uint8_t I2C_read(uint8_t slave_addr, uint8_t reg_addr);

/**
 * @brief Reads 2 bytes of data (upper + lower) from slave device
 * */
uint16_t I2C_read_2b(uint8_t slave_addr, uint8_t reg_upper_addr, uint8_t reg_lower_addr);


#endif /* I2C1_H_ */
