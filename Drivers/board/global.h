/*
 * global.h
 *
 *  Created on: Dec 29, 2016
 *      Author: jakub
 */

#ifndef MODULE_GLOBAL_H_
#define MODULE_GLOBAL_H_

// tato promena urcuje, ze kod se bude prekladat se vsim co je v DEBUG_TERMOSTAT modu

#include "stm32l0xx_hal.h"

#define DEBUG_TERMOSTAT // debug of the code is turned ON!
#define SW_VERSION 6	// verze softwaru  (nuly pred cislem znaci ze jde o octal number a to pak je pouze 0-7)

/**
 * text: is the size of all code in your application.
    data: is the size of initialized global variables. It counts both flash memory and RAM, as it's copied from flash to RAM during startup.
    bss: is the size of global variables which are initialized to zero (or are uninitialized, and hence default to zero). They're stored in RAM only.
    dec: and hex are the sum of text + data + bss in decimal and hexadecimal. This value doesn't really mean much on a microcontroller, so it should be ignored. (In environments where a program must be loaded into memory before running, it would be the total memory footprint of the program.)

	RAM usage of program, = data + bss 

	FLASH usage of program, = text + data.
 * 
 */

#define BUT_DELAY 10		 // in milisecond - I want to read it quckly
#define MAIN_LOOP 20		 // in milisecond
#define MEASURE_PERIODE 5000 // every 5 secondn
#define LED_PERIODE 500		 //
#define TIME_PERIODE 400	 // ms definition of periode for checking time change (RTC change )
#define VENT_PERIODE 1000 // every 5 minute check for change - turn on / off heater

#define LOG_PERIODE 600		 // in seconds - every 10 minute check for change - turn on / off heater

#define HEATING_INSTANT 900 // in seconds for 15 minutes is turned on the instant heating

#define HEATING_HYSTERESIS 50 // hysteresis is 0.5 deg C ( X/50)

#define VENTILATION_PWM_MAX 255  // 255 maximum PWM pro ventilator
#define VENTILATION_PWM_MIN 50 // minimalni hodnota, kdy se jeste ventilator toci

// INTITIAL STATE values for PWM
#define LCD_BACKLITE_DUTY 50

typedef enum
{
	FALSE = 0u,
	TRUE = 1u
} Bool;
// the priority is selected by place in the list.
typedef enum
{
	MEASURING,
	VENT,
	TEST,
	MENUn,
	TE_ERROR,
	SLEEP,
	TIME,
	IDLE,
	VOLTAGE,
	LOG
} States_loop;

typedef enum
{
	menu,
	blind,
	desktop,
	debug,
	idle,
	temp_show
} Screen;

typedef struct {
	uint8_t new_data_to_show:1;
	uint8_t new_data:1;
	uint8_t measure_activate:1;
	uint8_t measure_running:1;
	uint8_t menu_activate:1;
	uint8_t menu_running:1;
	uint8_t temp_new_set:1;
	uint8_t temp_running:1;
	uint8_t enc_changed:1;
// regulation of the temperature
	uint8_t regulation_temp:1; // Signal when regulation of temp is enabled.
	uint8_t heating_up:1; // record that heating is UP
	uint8_t vent_instant_req:1; // heating instant request for turning ON
	uint8_t vent_instant:1; // heating instant - heating up for defined time period.
	uint8_t regulation_disabled:1;
	uint8_t log_enable:1;
	uint8_t log_requsition:1;

}Flags_main;

/**
 * @brief definice stavu pro mody kdy v kterych muze byt termostat
 * 
 */
typedef enum  	{
	OFF,  	// vypnuto, topeni nezmi bezet!
	MANUAL,		// zapnuto topeni, bez casovace
	AUTO	// zapnut termostat, jede se dle casoveho planu
} Device_mode;


extern Flags_main flags;

#endif /* MODULE_GLOBAL_H_ */
