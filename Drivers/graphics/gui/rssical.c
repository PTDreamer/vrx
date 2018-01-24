/*
 * rssical.c
 *
 *  Created on: Jan 23, 2018
 *      Author: jose
 */

/*
 * calibration_screen.c
 *
 *  Created on: Sep 21, 2017
 *      Author: jose
 */

#include "rssical.h"
#include "../../generalIO/adc_global.h"
#include "../../graphics/gui/oled.h"
#include "../../Src/settings.h"
typedef enum {rssi_min, rssi_max, rssi_end}state_t;
static state_t current_state = rssi_min;
static char *infoStr;

static int cancelAction(widget_t* w) {
	//return screen_edit_calibration_input;
	return screen_main;
}

static int okAction(widget_t *w) {
	if(current_state == rssi_min) {
		systemSettings.rssi_min = rssi_adc_avg;
		current_state = rssi_max;
		saveSettings();
		strcpy(infoStr, "Set RSSI max");
		return -1;
	}
	systemSettings.rssi_max = rssi_adc_avg;
	saveSettings();
	return screen_main;
}

static void onEnter(screen_t *scr) {
	current_state = rssi_min;
	strcpy(infoStr, "Set RSSI min");
}
static void onExit(screen_t *scr) {

}
void calibration_screen_setup(screen_t *scr) {
	scr->draw = &default_screenDraw;
	scr->processInput = &default_screenProcessInput;
	scr->init = &default_init;
	scr->update = &default_screenUpdate;
	scr->onEnter = &onEnter;
	scr->onExit = &onExit;
	widget_t *widget = screen_addWidget(scr);
	widgetDefaultsInit(widget, widget_label);

	char *s = "Set RSSI min";
	strcpy(widget->displayString, s);
	infoStr = widget->displayString;
	widget->posX = 10;
	widget->posY = 16;
	widget->font_size = &FONT_8X14;

	widget = screen_addWidget(scr);
	widgetDefaultsInit(widget, widget_button);
	widget->font_size = &FONT_6X8;
	widget->posX = 90;
	widget->posY = 56;
	s = "CANCEL";
	strcpy(widget->displayString, s);
	widget->reservedChars = 6;
	widget->buttonWidget.selectable.tab = 2;
	widget->buttonWidget.action = &cancelAction;

	widget = screen_addWidget(scr);
	widgetDefaultsInit(widget, widget_button);
	widget->font_size = &FONT_6X8;
	widget->posX = 20;
	widget->posY = 56;
	s = "OK";
	strcpy(widget->displayString, s);
	widget->reservedChars = 6;
	widget->buttonWidget.selectable.tab = 1;
	widget->buttonWidget.action = &okAction;
}

