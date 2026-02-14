/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  ******************************************************************************
  ******************************************************************************
  ******************OK! работает************************************************
  ******************************************************************************
  ******************************************************************************
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DWT_CONTROL *(volatile unsigned long *)0xE0001000
#define SCB_DEMCR   *(volatile unsigned long *)0xE000EDFC
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
// кнопки
int8_t Select = 0;
int8_t position = 4;
int8_t value_up = 0;
int8_t value_down = 0;
int8_t letsWelding = 0;
uint32_t time_key_press = 0;// time_key_press = HAL_GetTick(); (HAL_GetTick() - time_key1_press) > 300
int8_t correction = 14; // поправка, причина в стартовой задержке в 14 микросекунд в delay_microseconds(), я не знаб почему

int voltage = 12;
char str[5];
int firstPulse = 400;
int pause = 3200;
int secondPulse = 1400;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void usDelay(uint16_t useconds)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while(__HAL_TIM_GET_COUNTER(&htim2) < useconds);
}

void DWT_Init(void) // OK
{
    SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // разрешаем использовать счётчик
	DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk;   // запускаем счётчик
}

void blinkLED(int count)
{
	for(int i=0; i<count; i++)
	{
		HAL_GPIO_WritePin(led13_GPIO_Port, led13_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(led13_GPIO_Port, led13_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
	}
	HAL_Delay(300);
}

void delay_microseconds(uint32_t usec)
{
    uint32_t us_count_tic =  usec * (SystemCoreClock / 1000000);
    DWT->CYCCNT = 0U; // обнуляем счётчик
    while(DWT->CYCCNT < us_count_tic); //{blinkLED(1);}
}

void test_uSecondDelayFunc(void)
{
	int i=0;
	  while(1)
	  {
		HAL_GPIO_TogglePin(WELD_OUT_GPIO_Port, WELD_OUT_Pin);
		//usDelay(40); // 10->100 uSec 40->370 uSec
		delay_microseconds(100); // 1->14uSec 10->24uSec 100->114uSec
		i+=1;
		if(i>100000) break;
	  }
}

void updateDelays(int inc)
{
	if(position==1) firstPulse = firstPulse + inc;
	if(position==2) pause = pause + inc;
	if(position==3) secondPulse = secondPulse + inc;
}

void keyboard(void)
{
	if(Select==0 && HAL_GPIO_ReadPin(GPIOB, BTN_SELECT_Pin)==GPIO_PIN_RESET)
	{
		time_key_press = HAL_GetTick();
		Select=1;
		if(position==4) position=1;
		else position++;
	}
	if(Select>0 && (HAL_GetTick() - time_key_press) > 300) Select=0;

	if(value_up==0 && HAL_GPIO_ReadPin(GPIOB, BTN_UP_Pin)==GPIO_PIN_RESET)
	{
		time_key_press = HAL_GetTick();
		value_up=1;
		updateDelays(100);
	}
	if(value_up>0 && (HAL_GetTick() - time_key_press) > 300) value_up=0;

	if(value_down==0 && HAL_GPIO_ReadPin(GPIOB, BTN_DOWN_Pin)==GPIO_PIN_RESET)
	{
		time_key_press = HAL_GetTick();
		value_down=1;
		updateDelays(-100);
	}
	if(value_down>0 && (HAL_GetTick() - time_key_press) > 300) value_down=0;

	if(letsWelding==0 && HAL_GPIO_ReadPin(LETS_WELD_GPIO_Port, LETS_WELD_Pin)==GPIO_PIN_RESET)
	{
		time_key_press = HAL_GetTick();
		letsWelding=1;
	}
	if(letsWelding>0 && (HAL_GetTick() - time_key_press) > 300) letsWelding=0;
}


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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_Init();


  SSD1306_GotoXY(0,0);
  SSD1306_Puts ("Welding", &Font_11x18, 1);
  SSD1306_GotoXY(10,30);
  SSD1306_Puts ("Machine", &Font_11x18, 1);
  SSD1306_UpdateScreen();
  HAL_Delay(1500);
  blinkLED(1);

  //test_uSecondDelayFunc();



  SSD1306_Clear();

  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  keyboard();

	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 100);
	  voltage = HAL_ADC_GetValue(&hadc1); // "T: %d.%d ->%d %d", temperatura/10, temperatura-(temperatura/10)*10,
	  int volH = voltage*0.0041;
	  int volL = (voltage*0.0041-volH)*10;
	  HAL_ADC_Stop(&hadc1);

	  if(Select==1 || value_up==1 || value_down==1) SSD1306_Clear();

	  SSD1306_GotoXY(1,0);
	  if(position==1) SSD1306_Puts (".", &Font_7x10, 1);
	  SSD1306_Puts ("first ", &Font_7x10, 1);
	  sprintf((char *)str, "%d", firstPulse);
      SSD1306_Puts (str, &Font_11x18, 1);
      SSD1306_Puts (" us", &Font_7x10, 1);

      SSD1306_GotoXY(1,15);
      if(position==2) SSD1306_Puts (".", &Font_7x10, 1);
      SSD1306_Puts ("pause ", &Font_7x10, 1);
      sprintf((char *)str, "%d", pause);
      SSD1306_Puts (str, &Font_11x18, 1);
      SSD1306_Puts (" us", &Font_7x10, 1);

      SSD1306_GotoXY(1,30);
      if(position==3) SSD1306_Puts (".", &Font_7x10, 1);
      SSD1306_Puts ("second ", &Font_7x10, 1);
      sprintf((char *)str, "%d", secondPulse);
      SSD1306_Puts (str, &Font_11x18, 1);
      SSD1306_Puts (" us", &Font_7x10, 1);

      SSD1306_GotoXY(1,45);
      if(position==4) SSD1306_Puts (".", &Font_7x10, 1);
      SSD1306_Puts ("voltage ", &Font_7x10, 1);
      sprintf((char *)str, "%d.%d", volH, volL);
      SSD1306_Puts (str, &Font_11x18, 1);
      SSD1306_Puts ("V ", &Font_11x18, 1);

	  SSD1306_UpdateScreen();
	    //HAL_Delay(200);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(letsWelding==1)
	{
		blinkLED(1);
		HAL_GPIO_WritePin(WELD_OUT_GPIO_Port, WELD_OUT_Pin, GPIO_PIN_SET);
		blinkLED(2);
	    delay_microseconds(firstPulse-correction); // 	1->14uSec 10->24uSec 100->114uSec
	    blinkLED(3);
	    HAL_GPIO_WritePin(WELD_OUT_GPIO_Port, WELD_OUT_Pin, GPIO_PIN_RESET);
	    delay_microseconds(pause-correction);
	    HAL_GPIO_WritePin(WELD_OUT_GPIO_Port, WELD_OUT_Pin, GPIO_PIN_SET);
	    delay_microseconds(secondPulse-correction);
	    HAL_GPIO_WritePin(WELD_OUT_GPIO_Port, WELD_OUT_Pin, GPIO_PIN_RESET);
	    HAL_Delay(500);
	    letsWelding=0;
	}
	    //HAL_GPIO_WritePin(WELD_OUT_GPIO_Port, WELD_OUT_Pin, GPIO_PIN_SET); // WELD_OUT_GPIO_Port, WELD_OUT_Pin, GPIO_PIN_RESET

	    //HAL_TIM_PeriodElapsedCallback(&htim2);
	  //test_uSecondDelayFunc();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led13_GPIO_Port, led13_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WELD_OUT_GPIO_Port, WELD_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : led13_Pin */
  GPIO_InitStruct.Pin = led13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LETS_WELD_Pin */
  GPIO_InitStruct.Pin = LETS_WELD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LETS_WELD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WELD_OUT_Pin */
  GPIO_InitStruct.Pin = WELD_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WELD_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_DOWN_Pin BTN_UP_Pin BTN_SELECT_Pin */
  GPIO_InitStruct.Pin = BTN_DOWN_Pin|BTN_UP_Pin|BTN_SELECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
        if(htim->Instance == TIM2) //check if the interrupt comes from TIM1
        {
                //HAL_GPIO_TogglePin(led13_GPIO_Port, led13_Pin);
        }
}
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

#ifdef  USE_FULL_ASSERT
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
