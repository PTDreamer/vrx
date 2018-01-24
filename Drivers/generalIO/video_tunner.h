/*
 * video_tunner.h
 *
 *  Created on: Jan 22, 2018
 *      Author: jose
 */

#ifndef GENERALIO_VIDEO_TUNNER_H_
#define GENERALIO_VIDEO_TUNNER_H_

#include "stm32f1xx_hal.h"

uint32_t getCurrentFrequencyX10();
void setCurrentFrequencyX10(uint32_t);
void startScan();
void stopScan();
uint8_t isScanRunning();
uint16_t getRSSI();
void handle_scan();
int8_t getCurrentPreset();
void setCurrentPreset(uint8_t);
void tunner_init(I2C_HandleTypeDef *hi2c,uint8_t tunner_adr);
#endif /* GENERALIO_VIDEO_TUNNER_H_ */
