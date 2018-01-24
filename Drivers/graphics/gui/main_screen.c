/*
 * main_screen.c
 *
 *  Created on: Aug 2, 2017
 *      Author: jose
 */

/*
 * mode(freq, presets)            name/freq (press edit preset)
 *                     freq/name (editable- long press start scan
 *  vu -----------
 */
#include "main_screen.h"

#include "../../../Src/settings.h"
#include "../../../generalIO/rotary_encoder.h"
#include "../../../generalIO/video_tunner.h"

typedef enum channel_mode_t {
	FREQUENCY, PRESET
} channel_mode_t;
static channel_mode_t currentChannelMode = FREQUENCY;

static char *modestr[] = {"FREQ", "PRESET"};
static widget_t *frequencyWidget = NULL;
static widget_t *presetWidget = NULL;
static widget_t *frequencyWidgetSec = NULL;
static widget_t *presetWidgetSec = NULL;

static char *presets[21];
static uint16_t m_temp;
static uint32_t m_ltemp;
static void set_frequency(uint32_t *value) {
	m_ltemp = *value;
	setCurrentFrequencyX10(m_ltemp);
}

static void * get_frequency() {
	m_ltemp = getCurrentFrequencyX10();
	return &m_ltemp;
}
static void setPreset(uint16_t *value) {
	m_temp = *value;
	setCurrentPreset(m_temp - 1);
}

static void * getPreset() {
	m_temp = getCurrentPreset();
	++m_temp;
	return &m_temp;
}
static void setMode(void *value) {
	UG_FillScreen(C_BLACK);
	m_temp = *(uint16_t*)value;
	if(m_temp == (uint8_t)FREQUENCY) {
		currentChannelMode = FREQUENCY;
		frequencyWidget->enabled = 1;
		presetWidgetSec->enabled = 1;
		presetWidget->enabled = 0;
		frequencyWidgetSec->enabled = 0;
	}
	else {
		currentChannelMode = PRESET;
		frequencyWidget->enabled = 0;
		presetWidgetSec->enabled = 0;
		presetWidget->enabled = 1;
		frequencyWidgetSec->enabled = 1;
	}
}

static void * getMode() {
	m_temp = (uint8_t)currentChannelMode;
	return &m_temp;
}

static void main_screen_init(screen_t *scr) {
	UG_FontSetHSpace(0);
	UG_FontSetVSpace(0);
	default_init(scr);
}

int processInput_frequency(widget_t*w, RE_Rotation_t r, RE_State_t *s) {
	if(r == LongClick && w->editable.selectable.state == widget_edit) {
		startScan();
		return -1;
	}
	else
		return default_widgetProcessInput(w,r,s);

}
int processInput_preset(widget_t*w, RE_Rotation_t r, RE_State_t *s) {
	if(r == LongClick && w->editable.selectable.state == widget_edit) {
		startScan();
		return -1;
	}
	else
		return default_widgetProcessInput(w,r,s);

}
int processInput_NameFreq(widget_t*w, RE_Rotation_t r, RE_State_t *s) {
	if(r == Click) {
		//TODO EDIT CURRENT FREQUENCY OR PRESET ie GOTO PRESETS SCREEN
		return -1;
	}
	else
		return default_widgetProcessInput(w, r, s);
}
void main_screenDraw(screen_t *scr) {
	UG_FillFrame(0,45, 127, 60,C_BLACK);
	UG_DrawFrame(0,45, 127, 60,C_WHITE);
	uint8_t val = getRSSI() * 127 / 100;
	if(val > 127)
		val = 127;
	UG_FillFrame(0,45, val, 60, C_WHITE);
	default_screenDraw(scr);
}
void main_screen_setup(screen_t *scr) {
	scr->draw = &main_screenDraw;
	scr->init = &main_screen_init;
	scr->update = &default_screenUpdate;
	scr->processInput = &default_screenProcessInput;
	//frequency display center of the display
	//only one display this or the one below
	widget_t *widget = screen_addWidget(scr);
	widgetDefaultsInit(widget, widget_editable);
	widget->editable.selectable.processInput = &processInput_frequency;
	widget->posX = 10;
	widget->posY = 20;
	widget->font_size = &FONT_12X20;
	widget->editable.inputData.getData = &get_frequency;
	widget->editable.inputData.number_of_dec = 3;
	widget->editable.inputData.type = field_uinteger32;
	widget->editable.big_step = 1000;
	widget->editable.step = 125;
	widget->editable.max_value = 2200000;//TODO
	widget->editable.min_value = 850000;
	widget->editable.selectable.tab = 0;
	widget->editable.setData = (void (*)(void *))&set_frequency;
	widget->reservedChars = 9;
	widget->editable.selectable.state = widget_edit;
	scr->current_widget = widget;
	frequencyWidget = widget;

	//preset display center of screen
	//only one display this or the one above
	widget = screen_addWidget(scr);
	widgetDefaultsInit(widget, widget_multi_option);
	widget->editable.selectable.processInput = &processInput_preset;
	widget->posX = 35;
	widget->posY = 20;
	widget->font_size = &FONT_12X20;
	widget->multiOptionWidget.editable.inputData.getData = &getPreset;
	widget->multiOptionWidget.editable.inputData.number_of_dec = 0;
	widget->multiOptionWidget.editable.inputData.type = field_uinteger16;
	widget->multiOptionWidget.editable.big_step = 0;
	widget->multiOptionWidget.editable.step = 0;
	widget->multiOptionWidget.editable.selectable.tab = 0;
	widget->multiOptionWidget.editable.setData = (void (*)(void *))&setPreset;
	widget->reservedChars = 5;
	presets[0] = "NOPRE";
	for(uint8_t x = 0; x < 20; ++x) {
		presets[x + 1] = systemSettings.channels[x].name;
	}
	widget->multiOptionWidget.options = presets;
	widget->multiOptionWidget.numberOfOptions = systemSettings.used_channels;;
	widget->multiOptionWidget.currentOption = 0;
	widget->multiOptionWidget.defaultOption = 0;
	widget->enabled = 0;
	presetWidget = widget;

	//secondary frequency display
	//only one display this or the one below
	//click goes to the preset edit screen
	widget = screen_addWidget(scr);
	widgetDefaultsInit(widget, widget_editable);
	widget->editable.selectable.processInput = &processInput_NameFreq;
	widget->posX = 53;
	widget->posY = 1;
	widget->font_size = &FONT_8X14;
	widget->editable.inputData.getData = &get_frequency;
	widget->editable.inputData.type = field_uinteger32;
	widget->editable.selectable.tab = 2;
	widget->reservedChars = 9;
	widget->editable.inputData.number_of_dec = 3;
	frequencyWidgetSec = widget;
	widget->enabled = 0;


	//secondary preset display
	//only one display this or the one above
	//click goes to the preset edit screen
	widget = screen_addWidget(scr);
	widgetDefaultsInit(widget, widget_multi_option);
	widget->editable.selectable.processInput = &processInput_NameFreq;
	widget->posX = 83;
	widget->posY = 1;
	widget->font_size = &FONT_8X14;
	widget->multiOptionWidget.editable.inputData.getData = &getPreset;
	widget->multiOptionWidget.editable.inputData.number_of_dec = 0;
	widget->multiOptionWidget.editable.inputData.type = field_uinteger16;		widget->multiOptionWidget.editable.big_step = 0;
	widget->multiOptionWidget.editable.selectable.tab = 2;
	widget->reservedChars = 5;
	widget->multiOptionWidget.options = presets;
	widget->multiOptionWidget.numberOfOptions = systemSettings.used_channels;;
	widget->multiOptionWidget.currentOption = 0;
	widget->multiOptionWidget.defaultOption = 0;
	widget->enabled = 1;
	presetWidgetSec = widget;

	//mode select
	widget = screen_addWidget(scr);
	widgetDefaultsInit(widget, widget_multi_option);
	widget->posX = 1;
	widget->posY = 1;
	widget->font_size = &FONT_8X14;
	widget->multiOptionWidget.editable.inputData.getData = &getMode;
	widget->multiOptionWidget.editable.setData = &setMode;
	widget->multiOptionWidget.editable.inputData.number_of_dec = 0;
	widget->multiOptionWidget.editable.inputData.type = field_uinteger16;		widget->multiOptionWidget.editable.big_step = 0;
	widget->multiOptionWidget.editable.selectable.tab = 1;
	widget->reservedChars = 6;
	widget->multiOptionWidget.options = modestr;
	widget->multiOptionWidget.numberOfOptions = 2;
	widget->multiOptionWidget.currentOption = 0;
	widget->multiOptionWidget.defaultOption = 0;
}
