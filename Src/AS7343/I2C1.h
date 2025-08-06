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
void Init_I2C1();

/**
 * @brief Initiate start condition. This automatically
 * 		  sets STM as I2C master.
 * */
void I2C1_start();

/**
 * @brief Closes a transaction
 * */
void I2C1_stop();

/**
 * @brief Sends a series of consecutive 8-bit data to a slave.
 * 
 * @param SLAVE_WRITE_ADDR (uint8_t) write address of slave
 * @param buf (char *) pointer to start of buffer
 * @param len (uint32_t) length of buffer
 * */
void I2C1_send_buf(uint8_t SLAVE_WRITE_ADDR, uint8_t *buf, uint32_t len);

/**
 * @brief transmits one byte of data
 * */
void I2C1_transmit(uint8_t data);

void I2C1_transmit_addr(uint8_t addr);

int I2C1_transmit_done();

uint8_t I2C1_receive();

/**
 * @brief Writes data to slave's register address.
 *        May be followed by another data transfer
 *        or a STOP (call I2C1_stop()).
 *
 * */
void I2C1_write_open(uint8_t slave_addr, uint8_t reg_addr, uint8_t data);

/**
 * @brief Reads one byte of data from slave device
 * */
uint8_t I2C1_read(uint8_t slave_addr, uint8_t reg_addr);

/**
 * @brief Reads 2 bytes of data (upper + lower) from slave device
 * */
uint16_t I2C1_read_2b(uint8_t slave_addr, uint8_t reg_upper_addr, uint8_t reg_lower_addr);


#endif /* I2C1_H_ */
