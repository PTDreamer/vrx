/*
 * settings.h
 *
 *  Created on: Sep 13, 2017
 *      Author: jose
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include "stm32f1xx_hal_flash.h"

#define SETTINGSVERSION 1 /*Change this if you change the struct below to prevent people getting out of sync*/

typedef struct channel_t {
	uint32_t frequency;
	char name[4];
} channel_t;

struct systemSettings {
	uint8_t version;				//Used to track if a reset is needed on firmware upgrade
	uint8_t contrast;
	channel_t channels[20];
	uint8_t used_channels;
	uint16_t rssi_min;
	uint16_t rssi_max;
	uint16_t rssi_min2;
	uint16_t rssi_max2;
	uint32_t start_scan_freq;
	uint32_t end_scan_freq;
	uint8_t diversity;
	uint32_t last_used_frequency;
} systemSettings;

void saveSettings();
void restoreSettings();
void resetSettings();

#endif /* SETTINGS_H_ */
