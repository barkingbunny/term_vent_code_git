/*
 * button.c
 *
 *  Created on: Jul 3, 2019
 *      Author: jakub
 *      last Update: 3-Jul-2019
 *
 *      Tento soubor bude obsahovat obsluhu klavesnice, zejmena tlacitek.
 *      tento soubor je mimo main soubor, aby bylo vse prehlednejsi.
 *      Bude zde pouzit debouncing pro zjisteni stisknuti tlacitka.
 *
 *      zajima me jen, zda bylo stisknute tlacitko, jake to bylo uz je vedlejsi
 */

#include "button.h"


Buttons button_pressed(void){
	Buttons actual_button = BUT_NONE;
	actual_button = checkButtons();
	uint8_t press_debounce = 0;
	if (actual_button==BUT_NONE) press_debounce = DebounceSwitch(0); // tlacitko nebylo stisknuto
	else press_debounce = DebounceSwitch(1); // tlacitko bylo stisknuto

	if (1==press_debounce) return actual_button;
	return BUT_NONE;
}
