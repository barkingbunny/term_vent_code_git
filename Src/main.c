/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/**
  * poznamky k debugu - oteviram tento zdrojak pomoci VS Code
  * - build: CTRL+SHIFT+B
  *  - download and run (nahrat do desky a spustit) - Termina - Run task - CPU: build, download and run
  * 
  *   
  */
// !!!soubor main.c v rootu je z funkcniho zdrojaku - neni uplne zkopirovan,

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/**
 * Program pro ventilacni system
 */
#include "pinmap.h"
#include "peripherals.h"
#include "global.h"
#include "lcd_12864.h"
#include "BME280.h"
#include "ds18b20.h"
#include <stdio.h>
#include "rtc_api.h"
#include "Time.h"
#include "log.h"
#include "sleep.h"
#include "pwm.h"
#include "mode-auto.h"
#include "menu.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static States_loop current_state;
static Screen show;
Device_mode vent_mode;
Flags_main flags;
Mode_auto_s vent_auto;

uint32_t citac = 0;
int8_t en_count = 0;
uint16_t ventilation_manual;
uint16_t ventilation_set;

Buttons pushed_button; //cleared each main cycle

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  char buffer_s[32];
  uint16_t sirka = 0;
  uint32_t InputVoltage = 0;
  uint8_t en_count_last = 0;
  Bool show_time = TRUE;
  uint8_t ventilation_index;
  uint8_t vent_instant = 0;
  char aShowTime[50] = {0};

  //timeouts
  Compare_t backlite_compare, measure_compare, led_compare, time_compare, button_compare, vent_compare, logging_compare, show_timeout, vent_instant_timeout;
  backlite_compare.overflow = FALSE, measure_compare.overflow = FALSE, led_compare.overflow = FALSE, time_compare.overflow = FALSE, button_compare.overflow = FALSE, vent_compare.overflow = FALSE, logging_compare.overflow = FALSE, show_timeout.overflow = FALSE, vent_instant_timeout.overflow = FALSE;
  actual_HALtick.overflow = FALSE;
  past_HALtick.overflow = FALSE;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM21_Init();
  MX_TIM22_Init();
  MX_USART1_UART_Init();
  MX_USART4_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  backlite_init();
  fill_comparer(BACKLITE_TIMEOUT, &backlite_compare);

  lcd12864_init(&hspi1);
  line(0, 60, 110, 60, 1);
  lcd_setCharPos(0, 0);
  lcd_printString("Initialization unit\r");
  lcd_printString("term_vent_git\r");
  snprintf(buffer_s, 11, "SW v 0.%03d", SW_VERSION);
  lcd_printString(buffer_s);
  //ENCODER initialization
  HAL_TIM_Encoder_Start(&htim22, TIM_CHANNEL_1);
  htim22.Instance->EGR = 1; // Generate an update event
  htim22.Instance->CR1 = 1; // Enable the counter

  HAL_Delay(1700);
  lcd_clear();

  //Init of timers
  fill_comparer(TIME_PERIODE, &time_compare);
  fill_comparer(BUT_DELAY, &button_compare);
  fill_comparer(10, &vent_compare);
  show = desktop;

  mode_auto_init(&vent_auto); // inicializace automatickeho temperovani - nulovani

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  for (;;)
  {

    switch (current_state)
    {
    case SLEEP:
    { //			neni potreba spanku
      Go_To_Sleep(&hrtc);
      current_state = MEASURING;
      show_time = TRUE;
      break;
    }
    case IDLE:
    {
      break;
    }
    case VENT:
    {
      /*   AUTO   */
      if (AUTO == vent_mode)
      {
        ventilation_index = mode_auto_index(&vent_auto, &hrtc); // aktualni index dle hodin

        if (vent_auto.PWM_OUT[ventilation_index] == 0) // vypni ventilator
        {
          flags.heating_up = FALSE;
          flags.regulation_temp = FALSE;
        }
        else if (vent_auto.PWM_OUT[ventilation_index] == 1)
        {
          flags.heating_up = FALSE;
          flags.regulation_temp = FALSE;
        }
        else
        {
          flags.regulation_temp = TRUE;
        }
      }
      /*   INSTANT   */
      if (flags.vent_instant) // INSTANTni vyhrivani - vzdy jen max 2 hodiny
      {
        if (!comparer_timeout(&vent_instant_timeout))
          turnOnVentilator(255);
        else
        {
          flags.vent_instant = FALSE;
          flags.vent_instant_req = TRUE;
          vent_instant = 0;
        }
      }
      else // bezny provoz
      {
        // vypni topeni
        {
          turnOffVentilator();
          flags.heating_up = FALSE;
        }
      }
      fill_comparer(VENT_PERIODE, &vent_compare);
      current_state = IDLE;
      break;
    }
#ifdef LOG_ENABLE
    case LOG:
    {
      if (flags.log_enable)
      {
        Log_Temperature(&hrtc, temperature, humid);
      }
      fill_comparer_seconds(LOG_PERIODE, &logging_compare);
      sirka = Log_memory_fullness() * LCD_WIDTH / LOG_DATA_LENGTH;
      current_state = IDLE;
      break;
    }
#endif
    default:
    {
      break;
    }

    } // end of switch CURRENT STATEF

    /* **** SCREEN **** */
    switch (show)
    {

    case blind:
    {
      lcd_clear();
      show = idle;
      break;
    }
    case desktop:
    { // showing main screen - temperatures and Hum
      if (flags.new_data_to_show == TRUE)
      {

        // Marking - heating is active/not active
        lcd_setCharPos(1, 19);
        switch (vent_mode)
        {
        case OFF:
          lcd_printString("-");
          break;
        case MANUAL:
          _putc(0x07f);
          break;
        case AUTO:
          lcd_printString("A");
          break;
        default:
          lcd_printString("E"); //ERROR dostal jsem se mimo definovanou skupinu
          break;
        }

        if (flags.vent_instant_req)
        {
          flags.vent_instant_req = FALSE;
          if (!flags.vent_instant)
          {
            char_magnitude(2);
            lcd_setCharPos(4, 19);
            lcd_printString(" ");
            char_magnitude(1);
            lcd_setCharPos(6, 15);
            snprintf(buffer_s, 8, "        ");
            lcd_printString(buffer_s);

            PWM_duty_change(LED2, 0);
          }
        }

        // END Marking - heating is active/not active

        char_magnitude(1);
        lcd_setCharPos(3, 2);
        if (0 == ventilation_set)
          snprintf(buffer_s, 12, "set  OFF");
        else if (1 == ventilation_set)
        {
          snprintf(buffer_s, 15, "set  NO PRESET");
        }
        else
          snprintf(buffer_s, 12, "set %3d", ventilation_set);
        lcd_printString(buffer_s);

        /*	lcd_setCharPos(6,4);
  					snprintf(buffer_s, 18, "Pres %d.%02d hp",presure/100,presure%100);
  					lcd_printString(buffer_s);
				 */

        flags.new_data_to_show = FALSE; // the data was showed.

        /* Ukazka plnosti pameti na logovani */
        // cara bude na spodni strane a bude tri tecky siroka
        {
          line(0, 62, sirka, 62, 1);
        }

        if ((vent_mode == AUTO) & (!flags.vent_instant))
          mode_auto_graph(&vent_auto, &hrtc); // zakresleni casoveho diagramu zapnuti/ vypnuti topeni

      } // end if - new data to show
      if (show_time)
      {
        RTC_DateShow(&hrtc, aShowTime);
        lcd_setCharPos(0, 11);
        lcd_printString(aShowTime);
        RTC_TimeShow(&hrtc, aShowTime);
        lcd_setCharPos(0, 1);
        lcd_printString(aShowTime);

        if (flags.vent_instant) // odpocet casu pro instant heating
        {
          char_magnitude(2);
          lcd_setCharPos(4, 19);
          _putc(0x07f);
          char_magnitude(1);
          lcd_setCharPos(6, 15);
          uint16_t instantEndTime = end_of_timeout(&vent_instant_timeout);
          snprintf(buffer_s, 8, "%3u:%02u", instantEndTime / 60, instantEndTime % 60);
          lcd_printString(buffer_s);
		      PWM_duty_change(LED2, instantEndTime/60);

        }
        fill_comparer(TIME_PERIODE, &time_compare);
        show_time = FALSE;
      }
      break;
    }
    case menu:
    {
      display_menu(ActualMenu);
      break;
    }
    default:
    {
      lcd_setCharPos(1, 1);
      lcd_printString("ERROR display -default");
    }
    } // switch show

    /* *------ TIME ELAPSING CHECK -------* */
    get_actual_HAL_tick(); // put the actual number of ms counter to variable actual_HALtic.tick

    if (comparer_timeout(&logging_compare)) //log data after defined period.
    {
      current_state = LOG;
    }
    if (comparer_timeout(&vent_compare)) //measure after defined period.
    {
      current_state = VENT;
    }
    if (comparer_timeout(&time_compare)) //change time after defined period.
    {
      show_time = TRUE;
    }

    if (comparer_timeout(&show_timeout)) //change time after defined period.
    {
      show = desktop;
    }
    // MENU TIMEOUT
    if (TRUE == flags.menu_running)
    { // je to takhle slozite , protoze jsem neprisel na jiny efektivni zpusob, jak smazat displej, po zkonceni menu
      if (!menu_timout())
      {
        uint8_t menu_exit_code = menu_action();
        if (1 != menu_exit_code)
        { // exit from menu condition

          switch (menu_exit_code)
          {
          case 20: // exit after log erasse
            sirka = Log_memory_fullness() * LCD_WIDTH / LOG_DATA_LENGTH;
            break;
          }
          flags.menu_running = 0;
          lcd_clear();
          show = desktop;
        }
        else
          show = menu;
      } // if menu - TIMEOUT
      else
      {
        flags.menu_running = 0;
        lcd_clear();
        show = desktop;
      }
      fill_comparer(BACKLITE_TIMEOUT, &backlite_compare); // Pro udrzeni rozsviceneho displeje v menu! NETESTOVANO.
    }
    if (flags.temp_new_set)
    {
      flags.temp_new_set = FALSE;
      ventilation_set = ventilation_manual;
      show = desktop;
    }
    if (comparer_timeout(&backlite_compare))
    {
      backliteOff();
    }

    /* *---- READ KEYBOARD -----* */

    pushed_button = BUT_NONE;
    if (comparer_timeout(&button_compare)) //every delay time
    {
      pushed_button = checkButtons();
      fill_comparer(BUT_DELAY, &button_compare);

      // reading the actual number from encoder
      uint32_t actual_counter_CNT = htim22.Instance->CNT;
      if (en_count_last != actual_counter_CNT)
      {
        en_count += actual_counter_CNT - en_count_last;
        en_count_last = actual_counter_CNT;
        flags.enc_changed = TRUE;
        if (pushed_button == BUT_NONE) // enabling light/ increase time constants.
          pushed_button = ENCODER;
      }
    }
    if (pushed_button != BUT_NONE) // any button pushed?
    {
      backliteOn();
      fill_comparer(BUT_DELAY * 100, &button_compare); // 200x - zpozdeni cteni pri stisknuti
      fill_comparer(BACKLITE_TIMEOUT, &backlite_compare);
      fill_comparer(SHOW_TIMEOUT, &show_timeout);
      // neco se zmenilo uzivatel - prekreslit cely displej
      flags.new_data_to_show = TRUE;
    }

    // -- BUTTON PROCCESS
    switch (pushed_button)
    {
    case BUT_1:
    { // activate heater
      /*
//      vent_mode++; // cyklovani modu pro termostat

      switch (vent_mode)
      {
      case OFF:
      {
        flags.regulation_temp = FALSE;
#ifndef DEBUG_TERMOSTAT                       ///// NOT - NEGOVANE!
        ventilation_set = ventilation_manual; // kvuli zobrazovani, jinak tam muze byt -210 apodobne
#endif
        mode_auto_graph_delete();
        break;
      }
      case MANUAL:
      {
        flags.regulation_temp = TRUE;
        flags.heating_up = TRUE;
        ventilation_set = ventilation_manual;
        mode_auto_graph_delete();
        break;
      }
      case AUTO:
      {
        flags.regulation_temp = TRUE;
        flags.heating_up = TRUE;
        break;
      }
      default:
      {
        flags.regulation_temp = FALSE;
        vent_mode = OFF;
        break;
      }
      }
*/
      break;
    }
    case BUT_2:
    { // Immediattely heating for 15 minutes

      flags.vent_instant = TRUE;
      flags.vent_instant_req = TRUE;
      vent_instant++;
      if (6 < vent_instant)
      {
        flags.vent_instant = FALSE;
        vent_instant = 0;
      }
      fill_comparer_seconds(HEATING_INSTANT * vent_instant, &vent_instant_timeout);
      break;
    }
    case BUT_ENC:
    {
      flags.enc_changed = TRUE; // jenom pomoc aby se zobrazila sipka vzdy...
      if (0 == flags.menu_running)
      {
        flags.menu_activate = 1;
        en_count = 0;
        en_count_last = 0;
        htim22.Instance->CNT = 0;
        activation_memu();
        lcd_clear();
        show = menu;

        // if this command means go to menu, That there shouldn't be no more pressed.
        pushed_button = BUT_NONE;
      }
      break;
    }
    case ENCODER:
    {
      break;
    }
    default:
    {

      break;
    }

    } // switch pushed button

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    HAL_Delay(MAIN_LOOP);
    debug_led_heartbeat();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_12;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
