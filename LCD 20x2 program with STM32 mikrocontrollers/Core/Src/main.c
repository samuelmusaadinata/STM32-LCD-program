/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_RS_Pin GPIO_PIN_9
#define LCD_RS_Port GPIOB
#define LCD_RW_Pin GPIO_PIN_8
#define LCD_RW_Port GPIOB
#define LCD_E_Pin GPIO_PIN_7
#define LCD_E_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_6
#define LCD_D4_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_5
#define LCD_D5_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_4
#define LCD_D6_Port GPIOB
#define LCD_D7_Pin GPIO_PIN_3
#define LCD_D7_Port GPIOB
#define LCD_BL_Pin GPIO_PIN_15
#define LCD_BL_Port GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

/* USER CODE BEGIN PV */
uint8_t maju = 0;
_Bool mA, mB, mC, mw4, mw5, pb1, pb2, pb3, buzzer;
uint32_t pot;
uint32_t pov;
char  lcd_buffer[15];
char lcd_bu[15];
uint32_t delay;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
uint32_t read_ADC(){
	uint16_t adc;
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 100);
	adc=HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);

	return adc;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void lcd_bit(uint8_t data, uint8_t rs_state){
  	HAL_GPIO_WritePin(LCD_RS_Port, LCD_RS_Pin, rs_state);
  	HAL_GPIO_WritePin(LCD_D7_Port, LCD_D7_Pin, (data >> 3) & 0x01);
  	HAL_GPIO_WritePin(LCD_D6_Port, LCD_D6_Pin, (data >> 2) & 0x01);
  	HAL_GPIO_WritePin(LCD_D5_Port, LCD_D5_Pin, (data >> 1) & 0x01);
  	HAL_GPIO_WritePin(LCD_D4_Port, LCD_D4_Pin, (data >> 0) & 0x01);
  	HAL_GPIO_WritePin(LCD_E_Port, LCD_E_Pin, GPIO_PIN_SET);
  	for(volatile uint16_t i=0; i<7200; i++);
  	HAL_GPIO_WritePin(LCD_E_Port, LCD_E_Pin, GPIO_PIN_RESET);
  }

  void lcd_cmd(uint8_t cmd) {
      lcd_bit((cmd >> 4) & 0x0F, 0);
      lcd_bit(cmd & 0x0F, 0);
  }

  void lcd_data(uint8_t data) {
      lcd_bit((data >> 4) & 0x0F, 1);
      lcd_bit(data & 0x0F, 1);
  }
  void LCD_Init(void) {
      HAL_Delay(100);
      lcd_cmd(0x33);
      lcd_cmd(0x32);
      lcd_cmd(0x28);
      lcd_cmd(0x08);
      lcd_cmd(0x06);
      lcd_cmd(0x0C);
      lcd_cmd(0x01);
      HAL_Delay(2);
  }

  void LCD_SetCursor(uint8_t x, uint8_t y) {
      uint8_t col = 0x80;
      switch (y) {
          case 0: col |= 0x00; break;
          case 1: col |= 0x40; break;
          case 2: col |= 0x14; break;
          case 3: col |= 0x54; break;
      }
      col += x;
      lcd_cmd(col);
  }

  void LCD_Clear(void) {
      lcd_cmd(0x01);
      HAL_Delay(2);
  }

  void LCD_SetBacklight(_Bool state) {
      HAL_GPIO_WritePin(LCD_BL_Port, LCD_BL_Pin, state);
  }

  void LCD_Print(char *str) {
      while (*str) lcd_data(*str++);
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  //	insert your setup code here.....
  LCD_Init();
  LCD_SetBacklight(1);





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t waktu = HAL_GetTick();
	  //	insert your loop code here.....
	  if(HAL_GPIO_ReadPin(TOGGLE_GPIO_Port, TOGGLE_Pin)==0){
		  mw4=1;
		  if(mw4==1){
		  LCD_Clear();
		  }
	  }
	  if(HAL_GPIO_ReadPin(SLIDE_GPIO_Port, SLIDE_Pin)==0){
	  		  mw5=1;
	  		if(mw5==1){
	  				  LCD_Clear();
	  				  }
	  	  }
	  else{
		  mw4=0;
		  mw5=0;

	  }
// suara saat tombol ditekan
	if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin)==0 && pb1 == 0){
		pb1=1;
	}
		if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin)==1 && pb1==1){
			pb1=0;
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
			delay=waktu;
			buzzer=1;
		}
	if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin)==0 && pb2 == 0){
		pb2=1;
	}
		if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin)==1 && pb2==1){
			pb2=0;
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
			delay=waktu;
			buzzer=1;
		}
	if(HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin)==0 && pb3 == 0){
		pb3=1;
	}
		if(HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin)==1 && pb3==1){
			pb3=0;
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
			delay=waktu;
			buzzer=1;
		}
if(buzzer == 1){
	if(waktu - delay >= 100){
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
		buzzer = 0;
	}
}
else {
HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
}

// tampilan LCD --------------------
LCD_SetCursor(0,0);
 LCD_Print("Peserta No. 06");
 LCD_SetCursor(1,1);
 LCD_Print("SW4:     SW5:");
 LCD_SetCursor(6,1);
 sprintf(lcd_buffer, "%01d", mw4);
 LCD_Print(lcd_buffer);
 LCD_SetCursor(15,1);
 sprintf(lcd_bu, "%01d", mw5);
 LCD_Print(lcd_bu);

//if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin)==0 && mA == 0){
//	mA=1;
//}
//if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin)==1 && mA == 1){
//	maju++;
//	mA=0;
//}
//if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin)==0 && mB == 0){
//	mB=1;
//}
//if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin)==1 && mB == 1){
//	maju--;
//	mB=0;
//}
//if(HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin)==0 && mC == 0){
//	mC=1;
//}
//if(HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin)==1 && mC == 1){
//	maju=0;
//	mC=0;
//}
pot = read_ADC();
 pov = (4020*4)/pot;

if(maju>5){
	maju=0;
}
if(maju<0){
	maju=0;
}

if(pot < 1100 ){
HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 1);
}

if(pot > 1800 ){
HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 0);
}
//if(maju == 1){
//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
//HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
//HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
//HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 1);
//HAL_Delay(250);
//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
//HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
//HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
//HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 0);
//HAL_Delay(250);
//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
//HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
//HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
//HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 1);
//HAL_Delay(250);
//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
//HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
//HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
//HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 1);
//HAL_Delay(250);
//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
//HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
//HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
//HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 1);
//HAL_Delay(250);
//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
//HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
//HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
//HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 1);
//HAL_Delay(250);
//maju = 0;
//
//}



  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|LED5_Pin|LCD_D7_Pin|LCD_D6_Pin
                          |LCD_D5_Pin|LCD_D4_Pin|LCD_E_Pin|LCD_RW_Pin
                          |LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RING_Pin|LED4_Pin|LED3_Pin|LED2_Pin
                          |LED1_Pin|LCD_BL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin SLIDE_Pin
                           TOGGLE_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin|SW3_Pin|SLIDE_Pin
                          |TOGGLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin LED5_Pin LCD_D7_Pin LCD_D6_Pin
                           LCD_D5_Pin LCD_D4_Pin LCD_E_Pin LCD_RW_Pin
                           LCD_RS_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|LED5_Pin|LCD_D7_Pin|LCD_D6_Pin
                          |LCD_D5_Pin|LCD_D4_Pin|LCD_E_Pin|LCD_RW_Pin
                          |LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RING_Pin LED4_Pin LED3_Pin LED2_Pin
                           LED1_Pin LCD_BL_Pin */
  GPIO_InitStruct.Pin = LED_RING_Pin|LED4_Pin|LED3_Pin|LED2_Pin
                          |LED1_Pin|LCD_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
