/*
 * MCP4725.c
 *
 *  Created on: Mar 21, 2025
 *      Author: trishamarieherras
 *
 * Uses I2C2
 *
 * PIN LAYOUT
 *    PB10  : SDA
 *    PB11  : SCL
 *    		: out
 */


#include <MCP4725.h>
#include <stdio.h>
#include <stdint.h>
#include <stm32f4xx.h>
#include <stm32f411xe.h>


/**
 * Constructor for MCP4725 struct.
 *
 * @param[in] addr I2C address for MCP4725.d
 *
 * @return Returns a MCP4725 instance with given values.
 *
 * */
MCP4725 MCP4725_init(MCP4725Ax_ADDRESS addr, float refV) {
	MCP4725 _MCP4725;

	_MCP4725._i2cAddress = addr;
	MCP4725_setReferenceVoltage(&_MCP4725, refV); //set _refVoltage & _bitsPerVolt variables

	return _MCP4725;
}

void MCP4725_setReferenceVoltage(MCP4725* _MCP4725, float value) {

}
float MCP4725_getReferenceVoltage(MCP4725* _MCP4725);
