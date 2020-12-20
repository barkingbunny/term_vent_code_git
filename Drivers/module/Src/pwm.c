/*
 * pwm.c
 *
 *  Created on: Jan 27, 2019
 *      Author: jakub
 */
/**
 * tento kod pisu pro moznost obslhovat PWM LED pomoci par prikazu
 */
#include "pwm.h"


/**
 * start the PWM channel with coding:
 *  first number - timer; second number is channel (21 - tim2, channel 1)
 *  // vent -board
 *
 *  21 Diganosticka LED -1 D_LED1
 *  22 pipak D_BUZZER
 *  23 LCD podsvetleni a klavesnice D_LCD_LIGHT
 *  24 LCD_TFT podsvetleni
 *  31 vystup 1 - D_OUT1
 *  32 diagnosticka LED D_LED2
 *  33 - DISABLED
 *  34 Vystup 2 - D_OUT2
 */
static Flags_PWM running;

void PWM_start(uint8_t pwm_channel){

	switch(pwm_channel)
	{
	case (21):
			{
		/*##-3- Start PWM signals generation #######################################*/
		/* Start channel 1 */
		if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
		{
			/* PWM Generation Error */
			Error_Handler();
		}
		running.TIM2CH1=TRUE;
		break;
			}
	case (22):	{
		if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)  Error_Handler();
		running.TIM2CH2=TRUE;
		break;
	}
	case (23):	{
			if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)  Error_Handler();
			running.TIM2CH3=TRUE;
			break;
		}
	case (24):	{
			if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)  Error_Handler();
			running.TIM2CH4=TRUE;
			break;
		}
	case (31):	{
		if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) 	Error_Handler();
		running.TIM3CH1=TRUE;
		break;
			}
	case (32):
				{
		if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)	Error_Handler();
		running.TIM3CH2=TRUE;
		break;
				}
	case (33):	{
		if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK)  Error_Handler();
		running.TIM3CH3=TRUE;
		break;
	}
	case (34):	{
		if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK)  Error_Handler();
		running.TIM3CH4=TRUE;
		break;
	}
		}


}

void PWM_stop(uint8_t pwm_channel){

	switch(pwm_channel)
	{
	case (21):
			{
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		running.TIM2CH1=FALSE;
		break;
			}
	case (22):
			{
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
		running.TIM2CH2=FALSE;
		break;
			}
	case (23):
			{
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
		running.TIM2CH3=FALSE;
		break;
			}
	case (24):
			{
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
		running.TIM2CH4=FALSE;
		break;
			}
	case (31):
					{
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		running.TIM3CH1=FALSE;
		break;
				}
	case (32):
				{
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
		running.TIM3CH2=FALSE;
		break;
				}
	case (33):
				{
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
		running.TIM3CH3=FALSE;
		break;
				}
	case (34):
				{
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
		running.TIM3CH4=FALSE;
		break;
				}

	}// SWITCH end
}


void PWM_togle(uint8_t pwm_channel){
	uint8_t running_bool;
	switch(pwm_channel){
	case (21): {
		running_bool = running.TIM2CH1;
		break; }
	case (22): {
		running_bool = running.TIM2CH2;
		break; }
	case (23): {
		running_bool = running.TIM2CH3;
		break;}
	case (24): {
		running_bool = running.TIM2CH4;
		break;}
	case (31):{
		running_bool = running.TIM3CH1;
		break; }
	case (32):{
		running_bool = running.TIM3CH2;
		break; }
	case (33):{
		running_bool = running.TIM3CH3;
		break; }
	case (34):{
		running_bool = running.TIM3CH4;
		break; }

	}


	if(TRUE==running_bool){
		PWM_stop(pwm_channel);
	}
	else
		PWM_start(pwm_channel);

}

