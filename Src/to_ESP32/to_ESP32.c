/*
 * to_ESP32.c
 *
 *  Created on: May 20, 2025
 *      Author: trishamarieherras
 */


#include <stdint.h>
#include <time.h>
#include <cJSON.h>
#include <I2C2.h>
#include <to_ESP32.h>
#include <stm32f4xx.h>
#include <stm32f411xe.h>



cJSON build_json_from_spectrum(float *wavelengths, float *absorbances, char *source_sensor) {
	time_t now;
	time(&now);

	// convert to time string
	char local_time[32];
	struct tm timeinfo;
	gmtime_r(&now, &timeinfo);
	strftime(local_time, sizeof(local_time), "%Y-%m-%dT%H:%M:%S", &timeinfo);

	// get number of elements in spectrum
	size_t len = sizeof(absorbances)/sizeof(absorbances[0]);

	// Build JSON spectrum array
	cJSON *spectrum = cJSON_CreateArray();
	for (int i; i < len; i++) {
		// entry = { "wavelength" : 400, "absorbance" : 2.123 }
		cJSON *entry = cJSON_CreateObject();
		cJSON_AddNumberToObject(entry, "wavelength", wavelengths[i]);
		cJSON_AddNumberToObject(entry, "absorbance", absorbances[i]);
		cJSON_AddItemToArray(spectrum, entry);
	}
	// spectrum = { "spectrum" : [
	// 		entry,
	// 		entry, ]
	// };

	// Build JSON
	cJSON *json = cJSON_CreateObject();
	cJSON_AddNumberToObject(json, "unix_time", (double) now);
	cJSON_AddStringToObject(json, "local_time", local_time);
	cJSON_AddStringToObject(json, "type", "data_array");
	cJSON_AddItemToObject(json, "spectrum", LOCAL_DEVICE_ID);
	cjSON_AddStringToObject(json, "sensor", source_sensor);

	return *json;
}

void send_to_esp32(cJSON json_spectrum, char *txb_ptr) {
	char *spectrum_str = cJSON_PrintUnformatted(&json_spectrum);

	I2C2_start();
	I2C2_transmit_addr(ESP32_SLAVE_ADDR_WRITE);
	while (!(I2C2->SR1 & (1 << 7))); // TxE: wait for Tx to Empty

	for (; txb_ptr != "\0"; ++txb_ptr) { // let null terminator mark end of transmission
		I2C2_transmit(txb_ptr);
	}

	I2C2_stop();
}
