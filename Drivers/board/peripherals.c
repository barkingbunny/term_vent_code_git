/*
 * peripherals.c
 *
 *  Created on: Dec 29, 2016
 *      Author: jakub
 *  This file is contains the work with Input and Outputs of the board/micro.
 *  this should take care about switches, LEDs and rest.
 *
 */

#include "pinmap.h"
#include "peripherals.h"
#include "pwm.h"
#include "global.h"

static uint16_t pwmka_up = 1;

Buttons checkButtons(void)
{

	if (!SW1)
	{
		return BUT_1;
	}
	if (!SW2)
	{
		return BUT_2;
	}
	if (!SW_ENC)
	{
		return BUT_ENC;
	}
	return BUT_NONE;
}

void togleLED(uint8_t led)
{
	PWM_togle(led);
}

uint8_t turnOnVentilator(uint16_t ventPWM)
{
	if (VENTILATION_PWM_MIN <= ventPWM)
	{
		PWM_start(OUT1);
		PWM_duty_change(OUT1, ventPWM);
		PWM_start(OUT2);
		PWM_duty_change(OUT2, ventPWM);
	}

	else
	{
		turnOffVentilator();
	}
	return 0;
}

void turnOffVentilator(void)
{
	PWM_stop(OUT1);
	PWM_stop(OUT2);
}

void backlite_init(void)
{
	PWM_start(LCD_LIGHT);
	PWM_duty_change(LCD_LIGHT, LCD_BACKLITE_DUTY);
}

void backliteOn(void)
{
	PWM_start(LCD_LIGHT);
}

void backliteOff(void)
{
	PWM_stop(LCD_LIGHT);
}

void debug_led_heartbeat(void)
{
	PWM_stop(LED1);
	uint16_t duty = PWM_duty_read(LED1);

	if (pwmka_up)
	{
		duty++;
		if (duty >= 38)
			pwmka_up = 0;
	}
	else
	{
		duty--;
		if (duty <= 7)
			pwmka_up = 1;
	}
	PWM_duty_change(LED1, duty);
	PWM_start(LED1);
}