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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "mint.h"
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
uint8_t rxdat,flag=2,time_flag=0;

bool page=0,mode=0;
int frq1=1000,frq7=1000;
double duty1=10,duty7=10;

char text[30];

extern keys key[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LCD_Proc(void);
void KEY_Proc(void);

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
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim6);
	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,100);
	
	HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);
	__HAL_TIM_SetCompare(&htim17,TIM_CHANNEL_1,100);
	
	HAL_UART_Receive_IT(&huart1,&rxdat,1);

	LCD_Init();
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		LCD_Proc();
		KEY_Proc();
		if(page == 0 && flag == 0) 
		{
			LED_Proc(2,0);
			LED_Proc(1,time_flag);
		}
		else if(page == 0 && flag == 1) 
		{
			LED_Proc(2,time_flag);
			LED_Proc(1,1);
		}
		else if(page == 1 && flag == 0) 
		{
			LED_Proc(2,1);
			LED_Proc(1,time_flag);
		}
		else if(page == 1 && flag == 1) 
		{
			LED_Proc(2,time_flag);
			LED_Proc(1,0);
		}
		else if(page == 1 && flag == 2) 
		{
			LED_Proc(1,0);
			LED_Proc(2,1);
		}
		else if(page == 0 && flag == 2) 
		{
			LED_Proc(2,0);
			LED_Proc(1,1);
		}
		if(mode == 0) LED_Proc(3,1);
		else LED_Proc(3,0);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void LCD_Proc(void)
{
	if(page == 0)
	{
		LCD_DisplayStringLine(Line1,(u8*)"        PA1         ");
		sprintf(text,"      F:%dHz      ",frq1);
		LCD_DisplayStringLine(Line3,(u8*)text);
		sprintf(text,"      D:%.0f%%      ",duty1);
		LCD_DisplayStringLine(Line4,(u8*)text);
	}
	else if(page == 1)
	{
		LCD_DisplayStringLine(Line1,(u8*)"        PA7         ");
		sprintf(text,"      F:%dHz      ",frq7);
		LCD_DisplayStringLine(Line3,(u8*)text);
		sprintf(text,"      D:%.0f%%      ",duty7);
		LCD_DisplayStringLine(Line4,(u8*)text);
	}
	else page = 0;
}

void KEY_Proc(void)
{
	/******************************B0******************************/
	if(key[0].single_sta == 1)
	{
		if(page == 0) frq1 = (frq1+1000)>10000?1000:(frq1+1000);
		else frq7 = (frq7+1000)>10000?1000:(frq7+1000);
		
		if(frq1 > frq7) flag = 0;
		else if(frq1 < frq7)flag = 1;
		else if(frq1 == frq7)flag = 2;
		
		htim2.Init.Period = (1000000/frq1)-1;
		HAL_TIM_Base_Init(&htim2);
		
		htim17.Init.Period = (1000000/frq7)-1;
		HAL_TIM_Base_Init(&htim17);
		
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,(int)(duty1/100*(double)(1000000/frq1)));
		__HAL_TIM_SetCompare(&htim17,TIM_CHANNEL_1,(int)(duty7/100*(double)(1000000/frq7)));
		
//		htim2.Init.Prescaler = (800000/frq1)-1;
//		HAL_TIM_Base_Init(&htim2);
		
		key[0].single_sta = 0;
	}
	/******************************B1******************************/
	if(key[1].single_sta == 1)
	{
		if(page == 0) duty1 = (duty1+10)>90?10:(duty1+10);
		else duty7 = (duty7+10)>90?10:(duty7+10);
		
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,(int)(duty1/100*(double)(1000000/frq1)));
		__HAL_TIM_SetCompare(&htim17,TIM_CHANNEL_1,(int)(duty7/100*(double)(1000000/frq7)));
		
		key[1].single_sta = 0;
	}
	/******************************B2******************************/
	if(key[2].single_sta == 1)
	{
		if(mode == 0) page = 1-page;
		key[2].single_sta = 0;
	}
	/******************************B3******************************/
	if(key[3].single_sta == 1)
	{
		mode = 1 - mode;
		if(mode == 1) LED_Proc(3,0);
		else LED_Proc(3,1);
		key[3].single_sta = 0;
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
