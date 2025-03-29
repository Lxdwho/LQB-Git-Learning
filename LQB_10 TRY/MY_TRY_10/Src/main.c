/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "led.h"
#include "key.h"
#include "M_adc.h"
#include "stdio.h"

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
// 变量定义
extern struct keys key[4];
unsigned int page=0,line=0,LLED=0,ULED=1,Volt=0,time=0;
char text[30];
double MAX_V = 2.3,MIN_V = 1.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// 函数定义
void KEY_Proc(void);
void LCD_Proc(void);

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	HAL_TIM_Base_Start_IT(&htim3);
	
	
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Blue2);
	
	LED_Proc(0x00);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		KEY_Proc();
		LCD_Proc();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void KEY_Proc(void)
{
/*******************************************页面切换*******************************************/
	if(key[0].single_sta == 1 && key[0].key_sta == 1) 
	{
		page = 1-page;
		key[0].single_sta = 0;
	}
/*******************************************选定行切换*******************************************/
	if(key[1].single_sta == 1 && key[1].key_sta == 1)
	{
		if(page == 1) line = (line+1)>3?0:(line+1);
		key[1].single_sta = 0;
	}
/*******************************************增加*******************************************/
	if(key[2].single_sta == 1 && key[2].key_sta == 1)
	{
		if(page == 1) 
		{
			if(line == 0)
			{
				MAX_V = (MAX_V+0.3)>3.3?3.3:(MAX_V+0.3);
			}
			else if(line == 1)
			{
				MIN_V = ((MIN_V+0.3)>3.3 || (MIN_V+0.3)>MAX_V)?MIN_V:(MIN_V+0.3);
			}
			else if(line == 2)
			{
				ULED = (ULED+1)>7?0:(ULED+1);
				ULED = (ULED == LLED)?ULED+1:ULED;
				ULED = ULED>7?0:ULED;
			}
			else
			{
				LLED = (LLED+1)>7?0:(LLED+1);
				LLED = (LLED == ULED)?LLED+1:LLED;
				LLED = LLED>7?0:LLED;
			}
		}
		key[2].single_sta = 0;
	}
/*******************************************减小*******************************************/
	if(key[3].single_sta == 1 && key[3].key_sta == 1)
	{
		if(page == 1) 
		{
			if(line == 0)
			{
				MAX_V = ((MAX_V-0.3)<0 || (MAX_V-0.3)<MIN_V)?MAX_V:(MAX_V-0.3);
			}
			else if(line == 1)
			{
				MIN_V = (MIN_V-0.3)<0?0:(MIN_V-0.3);
			}
			else if(line == 2)
			{
				ULED = (ULED == 0)?7:(ULED-1);
				ULED = (ULED == LLED)?(ULED == 0?7:(ULED-1)):ULED;
			}
			else
			{
				LLED = (LLED == 0)?7:(LLED-1);
				LLED = (LLED == ULED)?(LLED == 0?7:(LLED-1)):LLED;
			}
		}
		key[3].single_sta = 0;
	}
}

void LCD_Proc(void)
{
	double adc = GetADC(&hadc2);
	if(page ==0)
	{
		LCD_DisplayStringLine(Line2,(unsigned char *)"        Main          ");
		
		sprintf(text,"    Volt:%.2f         ",adc);
		LCD_DisplayStringLine(Line4,(unsigned char *)text);
		
		if(MAX_V < adc) LCD_DisplayStringLine(Line5,(unsigned char *)"    Status:Upper      ");
		else if(MIN_V > adc) LCD_DisplayStringLine(Line5,(unsigned char *)"    Status:Lower      ");
		else LCD_DisplayStringLine(Line5,(unsigned char *)"    Status:Normal      ");
		
		LCD_DisplayStringLine(Line6,(unsigned char *)"                      ");
		LCD_DisplayStringLine(Line7,(unsigned char *)"                      ");
	}
	else
	{
		LCD_DisplayStringLine(Line2,(unsigned char *)"       Setting         ");
		
		if(line == 0) LCD_SetBackColor(Green);
		sprintf(text,"    Max Volt:%.1fV      ",MAX_V);
		LCD_DisplayStringLine(Line4,(unsigned char *)text);
		LCD_SetBackColor(White);
		
		if(line == 1) LCD_SetBackColor(Green);
		sprintf(text,"    Min Volt:%.1fV      ",MIN_V);
		LCD_DisplayStringLine(Line5,(unsigned char *)text);
		LCD_SetBackColor(White);
		
		if(line == 2) LCD_SetBackColor(Green);
		sprintf(text,"    Upper:LD%d          ",ULED+1);
		LCD_DisplayStringLine(Line6,(unsigned char *)text);
		LCD_SetBackColor(White);
		
		if(line == 3) LCD_SetBackColor(Green);
		sprintf(text,"    Lower:LD%d          ",LLED+1);
		LCD_DisplayStringLine(Line7,(unsigned char *)text);
		LCD_SetBackColor(White);
	}
	if(MAX_V < adc) Volt = 0;
	else if(MIN_V > adc) Volt = 1;
	else  Volt = 2;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
