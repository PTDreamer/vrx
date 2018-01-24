/*
 * debug_screen.c
 *
 *  Created on: Aug 2, 2017
 *      Author: jose
 */

#include "settings_screen.h"
#include "../../../../Src/settings.h"
#include "../ssd1306.h"
#include "oled.h"
#include "../../../generalIO/video_tunner.h"

static widget_t *combo = NULL;

static uint16_t CONTRAST = 0;
static char str[20]="aaa";
static widget_t *tipCombo = NULL;
static widget_t *delTipButton = NULL;
static comboBox_item_t *addNewPresetComboItem = NULL;
static uint32_t frequency = 1280000;

static void edit_preset_screen_init(screen_t *scr) {
	if(strcmp(tipCombo->comboBoxWidget.currentItem->text, "ADD NEW") == 0) {
		strcpy(str, "   ");
		frequency = 1280000;
		delTipButton->enabled = 0;
	}
	else {
		strcpy(str, tipCombo->comboBoxWidget.currentItem->text);
		for(int x = 0; x < sizeof(systemSettings.channels) / sizeof(systemSettings.channels[0]); ++ x) {
			if(strcmp(tipCombo->comboBoxWidget.currentItem->text, systemSettings.channels[x].name) == 0) {
				frequency = systemSettings.channels[x].frequency;
				break;
			}
		}
		delTipButton->enabled = 1;
	}
	default_init(scr);
}
static void edit_presets_list_screen_init(screen_t *scr) {
	comboBox_item_t *i = tipCombo->comboBoxWidget.items;
	for(int x = 0; x < sizeof(systemSettings.channels) / sizeof(systemSettings.channels[0]); ++x) {
		if(x < systemSettings.used_channels) {
			strcpy(i->text, systemSettings.channels[x].name);
			i->enabled = 1;
		}
		else
			i->enabled = 0;
		i = i->next_item;
	}
	tipCombo->comboBoxWidget.currentItem = tipCombo->comboBoxWidget.items;
	tipCombo->comboBoxWidget.currentScroll = 0;
	if(systemSettings.used_channels > sizeof(systemSettings.channels) / sizeof(systemSettings.channels[0])) {
		addNewPresetComboItem->enabled = 0;
	}
}
static void *getPresetNameStr() {
	return str;
}

static void setPresetNameStr(char *s) {
	strcpy(str, s);
}
static int savePreset(widget_t *w) {
	if(strcmp(tipCombo->comboBoxWidget.currentItem->text, "ADD NEW") == 0) {
		strcpy(systemSettings.channels[systemSettings.used_channels].name, str);
		systemSettings.channels[systemSettings.used_channels].frequency = frequency;
		++systemSettings.used_channels;
		saveSettings();
	}
	else {
		for(int x = 0; x < sizeof(systemSettings.channels) / sizeof(systemSettings.channels[0]); ++ x) {
			if(strcmp(tipCombo->comboBoxWidget.currentItem->text, systemSettings.channels[x].name) == 0) {
				strcpy(systemSettings.channels[x].name, str);
				systemSettings.channels[x].frequency = frequency;
				saveSettings();
				break;
			}
		}
	}
	return screen_edit_presets;
}
static int cancelPreset(widget_t *w) {
	return screen_edit_presets;
}
static int delPreset(widget_t *w) {
	uint8_t itemIndex = 0;
	for(int x = 0; x < sizeof(systemSettings.channels) / sizeof(systemSettings.channels[0]); ++ x) {
		if(strcmp(tipCombo->comboBoxWidget.currentItem->text, systemSettings.channels[x].name) == 0) {
			itemIndex = x;
			break;
		}
	}
	for(int x = itemIndex; x < sizeof(systemSettings.channels) / sizeof(systemSettings.channels[0]) - 1; ++ x) {
		systemSettings.channels[x] = systemSettings.channels[x + 1];
	}
	--systemSettings.used_channels;
	saveSettings();
	return screen_edit_presets;
}
////
static void * getContrast_() {
	CONTRAST = getContrast();
	return &CONTRAST;
}
static void setContrast_(uint16_t *val) {
	CONTRAST = *val;
	setContrast(CONTRAST);
}
static int saveContrast(widget_t *w) {
	systemSettings.contrast = CONTRAST;
	saveSettings();
	return screen_main;
}
static int cancelContrast(widget_t *w) {
	setContrast(systemSettings.contrast);
	return screen_main;
}
////
static void settings_screen_init(screen_t *scr) {
	UG_FontSetHSpace(0);
	UG_FontSetVSpace(0);
	default_init(scr);
	scr->current_widget = combo;
	scr->current_widget->comboBoxWidget.selectable.state = widget_selected;
}
int processInput_PresetFrequency(widget_t*w, RE_Rotation_t r, RE_State_t *s) {
	if(r == LongClick) {
		frequency = getCurrentFrequencyX10();
		return -1;
	}
	else
		return default_widgetProcessInput(w,r,s);

}
static void set_PresetFrequency(void *value) {
	frequency = *(uint32_t*)value;
}

static void * get_PresetFrequency() {
	return &frequency;
}
void settings_screen_setup(screen_t *scr) {

	///settings combobox
	scr->draw = &default_screenDraw;
	scr->processInput = &default_screenProcessInput;
	scr->init = &settings_screen_init;
	scr->update = &default_screenUpdate;
	widget_t *widget = screen_addWidget(scr);
	widgetDefaultsInit(widget, widget_label);
	char *s = "Settings";
	strcpy(widget->displayString, s);
	widget->posX = 10;
	widget->posY = 0;
	widget->font_size = &FONT_8X14;
	widget->reservedChars = 8;
	widget->draw = &default_widgetDraw;
	widget = screen_addWidget(scr);
	widget->posX = 0;
	widgetDefaultsInit(widget, widget_combo);
	widget->posY = 17;
	widget->font_size = &FONT_6X8;
	comboAddItem(widget, "PRESETS", screen_edit_presets);
	comboAddItem(widget, "SCAN", screen_edit_scan);
	comboAddItem(widget, "SCREEN", screen_edit_contrast);
	comboAddItem(widget, "EXIT", screen_main);
	combo = widget;

	///Edit PID screen
	screen_t *sc = oled_addScreen(screen_edit_contrast);
	sc->draw = &default_screenDraw;
	sc->processInput = &default_screenProcessInput;
	sc->init = &default_init;
	sc->update = &default_screenUpdate;
	widget_t *w = screen_addWidget(sc);

	widgetDefaultsInit(w, widget_label);
	s = "CONTRAST";
	strcpy(w->displayString, s);
	w->posX = 50;
	w->posY = 0;
	w->font_size = &FONT_8X14;
	w->reservedChars = 8;

	w = screen_addWidget(sc);
	widgetDefaultsInit(w, widget_label);
	s = "Value:";
	strcpy(w->displayString, s);
	w->posX = 30;
	w->posY = 17;
	w->font_size = &FONT_6X8;
	w->reservedChars = 6;

	w = screen_addWidget(sc);
	widgetDefaultsInit(w, widget_editable);
	w->posX = 70;
	w->posY = 17;
	w->font_size = &FONT_6X8;
	w->editable.inputData.getData = &getContrast_;
	w->editable.inputData.number_of_dec = 0;
	w->editable.inputData.type = field_uinteger16;
	w->editable.big_step = 10;
	w->editable.step = 1;
	w->editable.selectable.tab = 0;
	w->editable.setData = (void (*)(void *))&setContrast_;
	w->editable.max_value = 255;
	w->reservedChars = 3;

	w = screen_addWidget(sc);
	widgetDefaultsInit(w, widget_button);
	w->font_size = &FONT_6X8;
	w->posX = 2;
	w->posY = 56;
	s = "SAVE";
	strcpy(w->displayString, s);
	w->reservedChars = 4;
	w->buttonWidget.selectable.tab = 1;
	w->buttonWidget.action = &saveContrast;
	w = screen_addWidget(sc);
	widgetDefaultsInit(w, widget_button);
	w->font_size = &FONT_6X8;
	w->posX = 90;
	w->posY = 56;
	s = "CANCEL";
	strcpy(w->displayString, s);
	w->reservedChars = 6;
	w->buttonWidget.selectable.tab = 2;
	w->buttonWidget.action = &cancelContrast;

	//presets edit iron tips
	sc = oled_addScreen(screen_edit_presets);
	sc->draw = &default_screenDraw;
	sc->processInput = &default_screenProcessInput;
	sc->init = &edit_presets_list_screen_init;
	sc->update = &default_screenUpdate;
	w = screen_addWidget(sc);

	widgetDefaultsInit(w, widget_label);
	s = "PRESETS";
	strcpy(w->displayString, s);
	w->posX = 0;
	w->posY = 0;
	w->font_size = &FONT_8X14;
	w->reservedChars = 7;
	//
	w = screen_addWidget(sc);
	tipCombo = w;
	widgetDefaultsInit(w, widget_combo);
	w->posY = 17;
	w->font_size = &FONT_6X8;
	for(int x = 0; x < sizeof(systemSettings.channels) / sizeof(systemSettings.channels[0]); ++x) {
		char *t = malloc(sizeof(systemSettings.channels[0].name)/sizeof(systemSettings.channels[0].name[0]));
		t[0] = '\0';
		if(!t)
		    _Error_Handler(__FILE__, __LINE__);
		comboAddItem(w, t, screen_edit_preset);
	}
	addNewPresetComboItem = comboAddItem(w, "ADD NEW", screen_edit_preset);
	comboAddItem(w, "EXIT", screen_main);
	sc->current_widget = tipCombo;

	//Screen edit preset
	sc = oled_addScreen(screen_edit_preset);
	sc->draw = &default_screenDraw;
	sc->processInput = &default_screenProcessInput;
	sc->init = &edit_preset_screen_init;
	sc->update = &default_screenUpdate;
	w = screen_addWidget(sc);

	widgetDefaultsInit(w, widget_label);
	s = "PRESET";
	strcpy(w->displayString, s);
	w->posX = 0;
	w->posY = 0;
	w->font_size = &FONT_8X14;
	w->reservedChars = 6;

	w = screen_addWidget(sc);
	widgetDefaultsInit(w, widget_editable);
	w->posX = 30;
	w->posY = 17;
	w->font_size = &FONT_8X14;
	w->editable.inputData.getData = &getPresetNameStr;
	w->editable.inputData.number_of_dec = 0;
	w->editable.inputData.type = field_string;
	w->editable.big_step = 10;
	w->editable.step = 1;
	w->editable.selectable.tab = 0;
	w->editable.setData = (void (*)(void *))&setPresetNameStr;
	w->editable.max_value = 9999;
	w->reservedChars = 4;

	widgetDefaultsInit(w, widget_editable);
	w->editable.selectable.processInput = &processInput_PresetFrequency;
	w->posX = 20;
	w->posY = 30;
	w->font_size = &FONT_8X14;
	w->editable.inputData.getData = &get_PresetFrequency;
	w->editable.inputData.number_of_dec = 3;
	w->editable.inputData.type = field_uinteger32;
	w->editable.big_step = 1000;
	w->editable.step = 125;
	w->editable.max_value = 2200000;//TODO
	w->editable.min_value = 850000;
	w->editable.selectable.tab = 0;
	w->editable.setData = &set_PresetFrequency;
	w->reservedChars = 9;


	w = screen_addWidget(sc);
	widgetDefaultsInit(w, widget_button);
	w->font_size = &FONT_6X8;
	w->posX = 2;
	w->posY = 56;
	s = "SAVE";
	strcpy(w->displayString, s);
	w->reservedChars = 4;
	w->buttonWidget.selectable.tab = 2;
	w->buttonWidget.action = &savePreset;
	w = screen_addWidget(sc);
	widgetDefaultsInit(w, widget_button);
	w->font_size = &FONT_6X8;
	w->posX = 50;
	w->posY = 56;
	s = "CANCEL";
	strcpy(w->displayString, s);
	w->reservedChars = 6;
	w->buttonWidget.selectable.tab = 3;
	w->buttonWidget.action = &cancelPreset;

	w = screen_addWidget(sc);
	delTipButton = w;
	widgetDefaultsInit(w, widget_button);
	w->font_size = &FONT_6X8;
	w->posX = 90;
	w->posY = 56;
	s = "DELETE";
	strcpy(w->displayString, s);
	w->reservedChars = 6;
	w->buttonWidget.selectable.tab = 4;
	w->buttonWidget.action = &delPreset;
}

