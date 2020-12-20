/*
 * pwm.h
 *
 *  Created on: Jan 27, 2019
 *      Author: jakub
 */

#ifndef MODULE_INC_PWM_H_
#define MODULE_INC_PWM_H_

#include "tim.h"
#include "global.h"


typedef struct {
	uint8_t TIM2CH1:1;  // 	LED1
	uint8_t TIM2CH2:1;	//	BUZZER
	uint8_t TIM2CH3:1;	//	LCD LIGHT
	uint8_t TIM2CH4:1;	//	TFT LCD LIGHT
	uint8_t TIM3CH1:1;	// 	OUT1
	uint8_t TIM3CH2:1;	//	LED2
	uint8_t TIM3CH3:1;	//
	uint8_t TIM3CH4:1;	//	OUT2

}Flags_PWM;


void PWM_start(uint8_t pwm_channel);
void PWM_stop(uint8_t pwm_channel);
void PWM_togle(uint8_t pwm_channel);


#endif /* MODULE_INC_PWM_H_ */
