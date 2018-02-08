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
void setCurrentFrequency(uint32_t);
void startScan();
void stopScan();
uint8_t isScanRunning();
uint16_t getRSSI();
uint16_t getRSSI2();
void handle_scan();
int8_t getCurrentPreset();
uint16_t getRSSI_ADC();
void setCurrentPreset(uint8_t);
uint8_t getActiveTunner();
uint16_t getRSSI_ADC();
uint16_t getRSSI2_ADC();
void tunnerReloadSettings();
void tunner_init(I2C_HandleTypeDef *hi2c,uint8_t tunner_adr, uint8_t tunner_adr2, ADC_HandleTypeDef * adc, ADC_HandleTypeDef * adc2);
#endif /* GENERALIO_VIDEO_TUNNER_H_ */
