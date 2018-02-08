/*
 *  video_tunner.c
 *
 *  Created on: Jan 22, 2018
 *      Author: jose
 */

#include "video_tunner.h"
#include  "../Src/settings.h"

static uint32_t currentFrequency = 1280125;
static int8_t currentPreset = -1;
static I2C_HandleTypeDef *m_i2c;
static ADC_HandleTypeDef *hadc1;
static uint8_t m_tunner_adr;
static ADC_HandleTypeDef *hadc2;
static uint8_t m_tunner_adr2;
static uint16_t scan_higher_rssi;
static uint32_t scan_higher_rssi_frequency, scan_higher_rssi_frequency_course;
static uint32_t scan_current_frequency;
static uint8_t isScanning = 0, current_scan_type, current_tunner = 0;
static uint32_t adc, adc2, acumulator1, acumulator2, samples = 0;
static uint32_t tick = 0;
static double tun0_m, tun0_b, tun1_m, tun1_b;
static uint32_t lastSetFrequencyTime = 0;

enum scan_type { SCAN_COURSE, SCAN_FINE};
void setTunnerFrequency(uint32_t freq);

void tunner_init(I2C_HandleTypeDef *hi2c,uint8_t tunner_adr, uint8_t tunner_adr2, ADC_HandleTypeDef *adc, ADC_HandleTypeDef *adc2) {
	m_i2c = hi2c;
	m_tunner_adr = tunner_adr;
	m_tunner_adr2 = tunner_adr2;
	hadc1 = adc;
	hadc2 = adc2;
}
uint32_t getCurrentFrequencyX10(){
	return currentFrequency;
}
void setCurrentFrequency(uint32_t value){
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
	setCurrentFrequency(systemSettings.start_scan_freq);
	if(systemSettings.rssi_max > systemSettings.rssi_min)
		scan_higher_rssi = 0;
	else
		scan_higher_rssi = 0xFFFF;
	scan_higher_rssi_frequency = systemSettings.start_scan_freq;
	scan_current_frequency = systemSettings.start_scan_freq;
	isScanning = 1;
	current_scan_type = SCAN_COURSE;
}
void stopScan(){
	isScanning = 0;
}
uint8_t isScanRunning(){
	return isScanning;
}
uint16_t getRSSI_ADC() {
	HAL_ADC_Start(hadc1);
    HAL_ADC_PollForConversion(hadc1, 100);
    adc = HAL_ADC_GetValue(hadc1);

	HAL_ADC_Start(hadc2);
    HAL_ADC_PollForConversion(hadc2, 100);
    adc2 = HAL_ADC_GetValue(hadc2);
    if(systemSettings.diversity) {
    	if(HAL_GetTick() - tick > 1000) {
    		uint32_t val1 = acumulator1 / samples;
    		uint32_t val2 = acumulator2 / samples;
    		double r1 = tun0_m * val1 + tun0_b;
    		double r2 = tun1_m * val2 + tun1_b;
    		if(r1 > r2) {
    			HAL_GPIO_WritePin(TUN1_SELECT_Port, TUN1_SELECT, GPIO_PIN_RESET);
    			HAL_GPIO_WritePin(TUN0_SELECT_Port, TUN0_SELECT, GPIO_PIN_SET);
    			current_tunner = 0;
    		}
    		else {
     			HAL_GPIO_WritePin(TUN0_SELECT_Port, TUN0_SELECT, GPIO_PIN_RESET);
       			HAL_GPIO_WritePin(TUN1_SELECT_Port, TUN1_SELECT, GPIO_PIN_SET);
       			current_tunner = 1;
    		}
    		acumulator1 = 0;
    		acumulator2 = 0;
    		samples = 0;
    		tick = HAL_GetTick();
    	}
    	else {
    		acumulator1 += adc;
    		acumulator2 += adc2;
    		++samples;
    	}
    }
    return adc;
}
uint16_t getRSSI2_ADC() {
    return adc2;
}
void tunnerReloadSettings() {
	tun0_m = 100.0f / ((int32_t)systemSettings.rssi_max - (int32_t)systemSettings.rssi_min);
	tun0_b = (-1) * tun0_m * systemSettings.rssi_min;
	tun1_m = 100.0f / ((int32_t)systemSettings.rssi_max2 - (int32_t)systemSettings.rssi_min2);
	tun1_b = (-1) * tun1_m * systemSettings.rssi_min2;

}
uint16_t getRSSI(){
	uint16_t rssi;
	uint32_t adc;

	if((HAL_GetTick() - lastSetFrequencyTime > 3000) && (currentFrequency != systemSettings.last_used_frequency)) {
		systemSettings.last_used_frequency = currentFrequency;
		saveSettings();
	}
	adc = getRSSI_ADC();

	rssi = tun0_m * adc + tun0_b;
	return rssi;
}
uint16_t getRSSI2(){
	uint16_t rssi;
	uint32_t adc;

	adc = getRSSI2_ADC();

	rssi = tun1_m * adc + tun1_b;
	return rssi;
}
void handle_scan(){
	if(!isScanning)
		return;
	static uint8_t buf;
	static uint16_t adc;

	HAL_StatusTypeDef status = HAL_ERROR;
	status = HAL_I2C_Master_Receive(m_i2c, m_tunner_adr, &buf, 1, 100);
	if(status == HAL_OK)
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	if(buf & 0b01000000) {
		uint32_t accumulator = 0;
		for(uint8_t x = 0; x < 5; ++x) {
			adc = getRSSI_ADC();
			accumulator += adc;
		}
		adc = accumulator / 5;
		if(systemSettings.rssi_max > systemSettings.rssi_min) {
			if(adc > scan_higher_rssi) {
				scan_higher_rssi = adc;
				scan_higher_rssi_frequency = scan_current_frequency;
			}
		}
		else {
			if(adc < scan_higher_rssi) {
				scan_higher_rssi = adc;
				scan_higher_rssi_frequency = scan_current_frequency;
			}
		}
		if(current_scan_type == SCAN_COURSE)
			scan_current_frequency = scan_current_frequency + 1000;
		else
			scan_current_frequency += 125;
		if((scan_current_frequency > systemSettings.end_scan_freq) && (current_scan_type == SCAN_COURSE)) {
			current_scan_type = SCAN_FINE;
			scan_current_frequency = scan_higher_rssi_frequency - 5000;
			scan_higher_rssi_frequency_course = scan_higher_rssi_frequency;
			if(systemSettings.rssi_max > systemSettings.rssi_min)
				scan_higher_rssi = 0;
			else
				scan_higher_rssi = 0xFFFF;
		}
		else if(scan_current_frequency > (scan_higher_rssi_frequency_course + 5000) && (current_scan_type == SCAN_FINE)){
			isScanning = 0;
			scan_current_frequency = scan_higher_rssi_frequency;
		}
		setCurrentFrequency(scan_current_frequency);
	}
}
int8_t getCurrentPreset(){return currentPreset;}
void setCurrentPreset(uint8_t preset) {
	if(preset >= systemSettings.used_channels)
		return;
	currentFrequency = systemSettings.channels[preset].frequency;
	currentPreset = preset;
	setTunnerFrequency(currentFrequency);
}

void setTunnerFrequency(uint32_t freq) {
	lastSetFrequencyTime = HAL_GetTick();
	uint16_t val;
	val = (freq + 480000)/(2*62.5);
	uint8_t buf[4];
	buf[0] = val >> 8;
	buf[1] = val & 0x00FF;
	buf[2] = 0xc2;
	buf[3] = 0x01;
	HAL_StatusTypeDef status;
	status = HAL_I2C_Master_Transmit(m_i2c, m_tunner_adr, buf, 4, 100);
	if(systemSettings.diversity)
		status = HAL_I2C_Master_Transmit(m_i2c, m_tunner_adr2, buf, 4, 100);
	if(status == HAL_OK) {
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		}
		else
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

uint8_t getActiveTunner() {
	return current_tunner;
}
