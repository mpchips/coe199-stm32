/*
 * to_ESP32.h
 *
 *  Created on: May 20, 2025
 *      Author: trishamarieherras
 */

#ifndef SRC_TO_ESP32_TO_ESP32_H_
#define SRC_TO_ESP32_TO_ESP32_H_

#include <stdint.h>
#include <cJSON.h>
#include <I2C2.h>

#define ESP32_SLAVE_ADDR 0x08
#define ESP32_SLAVE_ADDR_WRITE (ESP32_SLAVE_ADDR << 1)
#define ESP32_SLAVE_ADDR_READ  (ESP32_SLAVE_ADDR << 1) + 0x01
#define LOCAL_DEVICE_ID "spectral_node" // REPLACE THIS WITH ACTUAL CODENAME (to be requested)

cJSON build_json_from_spectrum(float *wavelengths, float *absorbances, char *source_sensor);

void send_to_esp32(cJSON json_spectrum, char *txb_ptr);


#endif /* SRC_TO_ESP32_TO_ESP32_H_ */
