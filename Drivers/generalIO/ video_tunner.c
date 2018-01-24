/*
 *  video_tunner.c
 *
 *  Created on: Jan 22, 2018
 *      Author: jose
 */

#include "video_tunner.h"
#include  "../Src/settings.h"
#include "adc_global.h"

static uint32_t currentFrequency = 1280125;
static int8_t currentPreset = -1;
static I2C_HandleTypeDef *m_i2c;
static uint8_t m_tunner_adr;
void setTunnerFrequency(uint32_t freq);

void tunner_init(I2C_HandleTypeDef *hi2c,uint8_t tunner_adr) {
	m_i2c = hi2c;
	m_tunner_adr = tunner_adr;
}
uint32_t getCurrentFrequencyX10(){
	return currentFrequency;
}
void setCurrentFrequencyX10(uint32_t value){
	currentFrequency = value;
	currentPreset = -1;
	for(uint8_t x = 0; x < systemSettings.used_channels; ++x) {
		if(systemSettings.channels[x].frequency == value) {
			currentPreset = x;
			break;
		}
	}
	setTunnerFrequency(currentFrequency);
}
void startScan(){

}
void stopScan(){
}
uint8_t isScanRunning(){
	return 0;
}
uint16_t getRSSI(){
	uint16_t rssi, min;
	if(systemSettings.rssi_min < systemSettings.rssi_max)
		min = systemSettings.rssi_min;
	else
		min =systemSettings.rssi_max;
	rssi = 100.0f / ((int32_t)systemSettings.rssi_max - (int32_t)systemSettings.rssi_min) * rssi_adc_avg + min;
	return rssi;
}
void handle_scan(){}
int8_t getCurrentPreset(){return currentPreset;}
void setCurrentPreset(uint8_t preset) {
	if(preset >= systemSettings.used_channels)
		return;
	currentFrequency = systemSettings.channels[preset].frequency;
	currentPreset = preset;
	setTunnerFrequency(currentFrequency);
}

void setTunnerFrequency(uint32_t freq) {
	uint16_t val;
	val = (currentFrequency + 480000)/(2*62.5);
	uint8_t buf[4];
	buf[0] = val >> 8;
	buf[1] = val & 0x00FF;
	buf[2] = 0xc2;
	buf[3] = 0x01;
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(m_i2c, m_tunner_adr, buf, 4, 100);
}
