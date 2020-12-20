/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define D_SW1_Pin GPIO_PIN_13
#define D_SW1_GPIO_Port GPIOC
#define A_TEMP5_Pin GPIO_PIN_0
#define A_TEMP5_GPIO_Port GPIOC
#define A_TEMP4_Pin GPIO_PIN_1
#define A_TEMP4_GPIO_Port GPIOC
#define A_TEMP3_Pin GPIO_PIN_2
#define A_TEMP3_GPIO_Port GPIOC
#define A_V_IN_MEAS_Pin GPIO_PIN_3
#define A_V_IN_MEAS_GPIO_Port GPIOC
#define D_IO1_Pin GPIO_PIN_0
#define D_IO1_GPIO_Port GPIOA
#define A_TEMP2_Pin GPIO_PIN_1
#define A_TEMP2_GPIO_Port GPIOA
#define A_TEMP1_Pin GPIO_PIN_2
#define A_TEMP1_GPIO_Port GPIOA
#define D_LCD_TFT_LED_Pin GPIO_PIN_3
#define D_LCD_TFT_LED_GPIO_Port GPIOA
#define A_IN_50HZ_Pin GPIO_PIN_4
#define A_IN_50HZ_GPIO_Port GPIOA
#define D_LCD_D_C_Pin GPIO_PIN_4
#define D_LCD_D_C_GPIO_Port GPIOC
#define D_LCD_RST_Pin GPIO_PIN_5
#define D_LCD_RST_GPIO_Port GPIOC
#define _D_LCD_CS_Pin GPIO_PIN_0
#define _D_LCD_CS_GPIO_Port GPIOB
#define D_OUT2_Pin GPIO_PIN_1
#define D_OUT2_GPIO_Port GPIOB
#define _D_MEM_CS_Pin GPIO_PIN_2
#define _D_MEM_CS_GPIO_Port GPIOB
#define D_LCD_LIGHT_Pin GPIO_PIN_10
#define D_LCD_LIGHT_GPIO_Port GPIOB
#define D_OUT3_Pin GPIO_PIN_11
#define D_OUT3_GPIO_Port GPIOB
#define D_RLY2_Pin GPIO_PIN_12
#define D_RLY2_GPIO_Port GPIOB
#define D_CHAN_A_Pin GPIO_PIN_6
#define D_CHAN_A_GPIO_Port GPIOC
#define D_CHAN_B_Pin GPIO_PIN_7
#define D_CHAN_B_GPIO_Port GPIOC
#define _D_EXTRA_CS_Pin GPIO_PIN_8
#define _D_EXTRA_CS_GPIO_Port GPIOC
#define D_EXTRA_PSU_EN_Pin GPIO_PIN_9
#define D_EXTRA_PSU_EN_GPIO_Port GPIOC
#define D_EXTRA_IO_Pin GPIO_PIN_8
#define D_EXTRA_IO_GPIO_Port GPIOA
#define D_LED1_Pin GPIO_PIN_15
#define D_LED1_GPIO_Port GPIOA
#define D_WIRE1_Pin GPIO_PIN_12
#define D_WIRE1_GPIO_Port GPIOC
#define D_WIRE2_Pin GPIO_PIN_2
#define D_WIRE2_GPIO_Port GPIOD
#define D_BUZER_Pin GPIO_PIN_3
#define D_BUZER_GPIO_Port GPIOB
#define D_OUT1_Pin GPIO_PIN_4
#define D_OUT1_GPIO_Port GPIOB
#define D_LED2_Pin GPIO_PIN_5
#define D_LED2_GPIO_Port GPIOB
#define D_RLY1_Pin GPIO_PIN_6
#define D_RLY1_GPIO_Port GPIOB
#define D_IO2_Pin GPIO_PIN_7
#define D_IO2_GPIO_Port GPIOB
#define D_SW_ENC_Pin GPIO_PIN_8
#define D_SW_ENC_GPIO_Port GPIOB
#define D_SW2_Pin GPIO_PIN_9
#define D_SW2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
