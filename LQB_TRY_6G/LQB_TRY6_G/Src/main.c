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
#include "mint.h"
#include "i2c_hal.h"
#include "stdio.h"
#include "string.h"

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
extern keys key[4];
char text[30];
bool page=0,out_sta=1;
unsigned char duty=0,frq=1;
double adc_val=0;
extern double DUTY,FRQ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void KEY_Proc(void);
void LCD_Proc(void);
double ADC_Proc(void);
void LED_Proc(unsigned char led,unsigned char sta);
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
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,50);
	
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
	
	LCD_Init();
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Black);
	frq = eeprom_read(0);
	if(frq ==0 || frq > 10) frq = 1;
	htim1.Init.Prescaler = (800000/frq/1000)-1;
	HAL_TIM_Base_Init(&htim1);
	LED_Proc(1,1);
	
	HAL_TIM_IC_Start_IT(&htim15,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim15,TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		KEY_Proc();
		LCD_Proc();
		adc_val = ADC_Proc();
		duty = (unsigned char)(adc_val/3.3*100);
		if(out_sta == 1) __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,duty);
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
	if(key[0].single_sta == 1)
	{
		if(out_sta == 0) 
		{
			out_sta = 1;
			LED_Proc(1,1);
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
		}
		else
		{
			out_sta = 0;
			LED_Proc(1,0);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_2);
		}
		key[0].single_sta = 0;
	}
	if(key[1].single_sta == 1)
	{
		page = page==0?1:0;
		if(page == 0) eeprom_write(0,frq);
		key[1].single_sta = 0;
	}
	if(key[2].single_sta == 1)
	{
		if(page == 1)
		{
			frq = frq==10?1:frq+1;
			htim1.Init.Prescaler = (800000/frq/1000)-1;
			HAL_TIM_Base_Init(&htim1);
		}
		key[2].single_sta = 0;
	}
	if(key[3].single_sta == 1)
	{
		
		key[3].single_sta = 0;
	}
}
void LCD_Proc(void)
{
	if(page == 0)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        Para        ");
		sprintf(text,"    Vol:%4.2fV       ",adc_val);
		LCD_DisplayStringLine(Line2,(u8 *)text);
		if(out_sta == 1) sprintf(text,"    Out:Start        ");
		else sprintf(text,"    Out:Stop         ");
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"    Par:PA9=%d%%     ",duty);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		sprintf(text,"        PB14=%d%%    ",(100-duty));
		LCD_DisplayStringLine(Line5,(u8 *)text);
		sprintf(text,"        %dKHZ        ",frq);
		LCD_DisplayStringLine(Line6,(u8 *)text);
		
		sprintf(text,"   %4.0fHZ   %2.0f        ",FRQ,DUTY);
		LCD_DisplayStringLine(Line9,(u8 *)text);
		
	}
	else if(page == 1)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"      Setting       ");
		LCD_DisplayStringLine(Line2,(u8 *)"                    ");
		
		sprintf(text,"      FRQ=%dKHZ      ",frq);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		
		LCD_DisplayStringLine(Line4,(u8 *)"                    ");
		LCD_DisplayStringLine(Line5,(u8 *)"                    ");
		LCD_DisplayStringLine(Line6,(u8 *)"                    ");
		
		sprintf(text,"   %4.0fHZ   %2.0f        ",FRQ,DUTY);
		LCD_DisplayStringLine(Line9,(u8 *)text);
	}
	else page = 0;
}
double ADC_Proc(void)
{
	double adc;
	HAL_ADC_Start(&hadc2);
	adc = HAL_ADC_GetValue(&hadc2)*3.3/4095;
	return adc;
}
void LED_Proc(unsigned char led,unsigned char sta)
{
	static unsigned char LED_sta = 0xff;
	if(sta == 0) LED_sta |= (0x01 << (led-1));
	else if(sta == 1) LED_sta &= ~(0x01 << (led-1));
	
	GPIOC->ODR = LED_sta << 8;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
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
