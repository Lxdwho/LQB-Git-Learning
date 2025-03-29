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
#include "dma.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
bool page=0;
char text[30],data[60][20];
double T=0,H=0,T_U=40,H_U=80,TS=1,Test=1.5;
unsigned char line=0;
extern unsigned char p;
extern keys key[4];

extern double frq;

uint8_t rxdata[100];

extern RTC_AlarmTypeDef sAlarm;
RTC_TimeTypeDef GetTime;
RTC_DateTypeDef GetDate;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
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
  MX_DMA_Init();
  MX_ADC2_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim6);
	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,5);
	
	HAL_TIM_IC_Start_IT(&htim17,TIM_CHANNEL_1);
	
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
	
	LCD_Init();
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Black);
	
	I2CInit();
	
	LED_Proc(1,0);
	
	HAL_RTC_GetTime(&hrtc,&GetTime,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&GetDate,RTC_FORMAT_BIN);
	sAlarm.AlarmTime.Seconds = GetTime.Seconds + 1;
	HAL_RTC_SetAlarm_IT(&hrtc,&sAlarm,RTC_FORMAT_BIN);
	
	HAL_UART_Receive_DMA(&huart1,rxdata,100);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	
	T_U = eeprom_read(0);
	if(T_U > 60) T_U = 40;
	H_U = eeprom_read(1);
	if(H_U > 100) H_U = 80;
	TS = eeprom_read(2);
	if(TS > 5) TS = 1;
	Test = (double)(eeprom_read(3))/10;
	if(Test > 10.1) Test = 1.5;
	
	htim2.Init.Prescaler = (8000/Test)-1;
	HAL_TIM_Base_Init(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		KEY_Proc();
		LCD_Proc();
		if(H > H_U) LED_Proc(2,1);
		else LED_Proc(2,0);
		if(T >T_U) LED_Proc(1,1);
		else LED_Proc(1,0);
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV32;

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
		if(page == 1)
		{
			eeprom_write(0,(unsigned char)T_U);
			HAL_Delay(10);
			eeprom_write(1,(unsigned char)H_U);
			HAL_Delay(10);
			eeprom_write(2,(unsigned char)TS);
			HAL_Delay(10);
			eeprom_write(3,(unsigned char)(Test*10));
			sprintf(text,"    Time:%02d-%02d-%02d      ",GetTime.Hours,GetTime.Minutes,GetTime.Seconds);
			LCD_DisplayStringLine(Line5,(u8 *)text);
		}
		page = page == 1?0:1;
		key[0].single_sta = 0;
	}
	if(key[1].single_sta == 1)
	{
		if(page == 1) line = line==3?0:line+1;
		key[1].single_sta = 0;
	}
	if(key[2].single_sta == 1)
	{
		if(page == 1)
		{
			if(line == 0) T_U = T_U == 60?60:T_U+1;
			else if(line == 1) H_U = H_U >= 99?100:H_U+5;
			else if(line == 2) TS = TS == 5?5:TS+1;
			else if(line == 3) 
			{
				Test = Test >= 9.9?10:Test+0.5;
				htim2.Init.Prescaler = (8000/Test)-1;
				HAL_TIM_Base_Init(&htim2);
			}
		}
		key[2].single_sta = 0;
	}
	if(key[3].single_sta == 1)
	{
		if(page == 1)
		{
			if(line == 0) T_U = T_U == -20?-20:T_U-1;
			else if(line == 1) H_U = H_U <= 1?0:H_U-5;
			else if(line == 2) TS = TS == 1?1:TS-1;
			else if(line == 3) 
			{
				Test = Test == 1?1:Test-0.5;
				htim2.Init.Prescaler = (8000/Test)-1;
				HAL_TIM_Base_Init(&htim2);
			}
		}
		key[3].single_sta = 0;
	}
}

void LCD_Proc(void)
{
	if(page == 0)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        Data        ");
		sprintf(text,"    Tem:%02dC        ",(int)(T));
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"    Hmd:%02d%%        ",(int)(H+0.1));
		LCD_DisplayStringLine(Line4,(u8 *)text);
		sprintf(text,"          Times:%d        ",p);
		LCD_DisplayStringLine(Line6,(u8 *)text);
	}
	else if(page == 1)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"      Setting        ");
		
		sprintf(text,"    Tem_U:%02dC        ",(int)T_U);
		if(line == 0) LCD_SetTextColor(Green);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		LCD_SetTextColor(Black);
		
		sprintf(text,"    Hmd_U:%02d%%      ",(int)H_U);
		if(line == 1) LCD_SetTextColor(Green);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		LCD_SetTextColor(Black);
		
		sprintf(text,"    TS:%01dS        ",(int)TS);
		if(line == 2) LCD_SetTextColor(Green);
		LCD_DisplayStringLine(Line5,(u8 *)text);
		LCD_SetTextColor(Black);
		
		sprintf(text,"    Test_s:%.1fHZ        ",Test);
		if(line == 3) LCD_SetTextColor(Green);
		LCD_DisplayStringLine(Line6,(u8 *)text);
		LCD_SetTextColor(Black);
	}
	else page = 0;
}

double GetADC(void)
{
	double adc;
	HAL_ADC_Start(&hadc2);
	adc = HAL_ADC_GetValue(&hadc2);
	return adc/4095*3.3;
}

void LED_Proc(unsigned char led,unsigned char sta)
{
	static unsigned char LED_STA = 0xff;
	if(sta == 0) LED_STA |= (0x01 << (led - 1));
	else LED_STA &= ~(0x01 << (led - 1));
	GPIOC->ODR = LED_STA << 8;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(rxdata == 'C') 
//	{
//		sprintf(tem,"T_U=%d\r\nH_U=%d\r\n%02d-%02d-%02d\r\n",(int)T_U,(int)H_U,GetTime.Hours,GetTime.Minutes,GetTime.Seconds);
//		HAL_UART_Transmit(huart,(u8 *)tem,strlen(tem),50);
//	}
//	else if(rxdata == 'T')
//	{
//			HAL_UART_Transmit(huart,(u8 *)data[1],strlen(data[1]),50);
//	}
//	HAL_UART_Receive_IT(huart,&rxdata,1);
//}
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
