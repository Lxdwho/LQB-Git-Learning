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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_hal.h"
#include "lcd.h"
#include "mint.h"
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

bool page=0,flag1=0,flag2=0,flag3=0;

char text[30],rxdat,text1[20];
extern int dtime,dtime1,dtime2,dtime3;

uint8_t Height=0,Level=4,Level_l=4,TH[3] = {10,20,30},Line=0;
double adc_val=0;

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
  MX_ADC2_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);//ADC校准
	HAL_UART_Receive_IT(&huart1,(unsigned char *)&rxdat,1);
	LED_Proc(1,0);
//	HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);
	//LCD初始化
	LCD_Init();
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Black);
	//初始化I2C写入
	if(eeprom_read(0)<eeprom_read(1) && eeprom_read(1)<eeprom_read(2) && eeprom_read(2)<100)
	{
		TH[0] = eeprom_read(0);
		TH[1] = eeprom_read(1);
		TH[2] = eeprom_read(2);
	}
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		LCD_Proc();
		KEY_Proc();
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**************************************显示模块**************************************/
void LCD_Proc(void)
{
	if(page == 0)
	{
		flag1 = 1;
		LCD_DisplayStringLine(Line1,(u8 *)"    Liquid Level    ");
		sprintf(text,"  Threshold1:%dcm   ",Height);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		sprintf(text,"     ADC:%.2fV      ",adc_val);
		LCD_DisplayStringLine(Line6,(u8 *)text);
		sprintf(text,"       Level:%d      ",Level);
		LCD_DisplayStringLine(Line8,(u8 *)text);
	}
	else if(page == 1)
	{
		//此页面为设置页面，认为不工作，进行相应操作
		flag1 = 0;
		dtime1 = 1;
		LED_Proc(1,0);
		//page1显示部分
		LCD_DisplayStringLine(Line1,(u8 *)"   Paramter Setup   ");
		sprintf(text,"  Threshold1:%dcm   ",TH[0]);
		if(Line == 0) LCD_SetTextColor(Green);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		LCD_SetTextColor(Black);
		
		sprintf(text,"  Threshold1:%dcm   ",TH[1]);
		if(Line == 1) LCD_SetTextColor(Green);
		LCD_DisplayStringLine(Line6,(u8 *)text);
		LCD_SetTextColor(Black);
		
		sprintf(text,"  Threshold1:%dcm   ",TH[2]);
		if(Line == 2) LCD_SetTextColor(Green);
		LCD_DisplayStringLine(Line8,(u8 *)text);
		LCD_SetTextColor(Black);
	}
	else page = 0;
	//等级判断、等级变化判断部分
	if(Level !=4 && Level_l != 4)
	{
		if(Level_l>Level) 
		{
			flag2 = 1;
			dtime2 = 0;
			sprintf(text1,"A:H%d+L%d+D\r\n",Height,Level);
			HAL_UART_Transmit(&huart1,(unsigned char *)text1,strlen(text1),50);
		}
		else if(Level_l<Level) 
		{
			flag2 = 1;
			dtime2 = 0;
			sprintf(text1,"A:H%d+L%d+U\r\n",Height,Level);
			HAL_UART_Transmit(&huart1,(unsigned char *)text1,strlen(text1),50);
		}
	}
	Level_l = Level;
	if(Height<=TH[0]) Level = 0;
	else if(Height>TH[0] && Height<=TH[1]) Level = 1;
	else if(Height>TH[1] && Height<=TH[2]) Level = 2;
	else if(Height>TH[2]) Level = 3;
	
}
/**************************************按键模块**************************************/
void KEY_Proc(void)
{
	if(key[0].single_sta == 1)
	{
		page = (page == 0)?1:0;
		if(page == 0) for(int j=0;j<3;j++) 
		{
			eeprom_write(j,TH[j]);
			HAL_Delay(10);
		}
		key[0].single_sta = 0;
	}
	if(key[1].single_sta == 1)
	{
		if(page == 1) Line = Line == 2?0:Line+1;
		key[1].single_sta = 0;
	}
	if(key[2].single_sta == 1)
	{
		if(page == 1) TH[Line] = TH[Line]>=95?TH[Line]:TH[Line]+5;
		key[2].single_sta = 0;
	}
	if(key[3].single_sta == 1)
	{
		if(page == 1) TH[Line] = TH[Line]<=5?TH[Line]:TH[Line]-5;
		key[3].single_sta = 0;
	}
}
/**************************************ADC读值部分**************************************/
double GetADCValue(void)
{
	double adc;
	HAL_ADC_Start(&hadc2);
	adc = HAL_ADC_GetValue(&hadc2);
	adc = adc*3.3/4095;
	return adc;
}
/**************************************LED模块**************************************/
void LED_Proc(unsigned char led,unsigned char sta)
{
	static unsigned char LED_STATE = 0xff;
	if(sta == 0) LED_STATE |= (GPIO_PIN_0 << (led - 1));
	else LED_STATE &= ~(GPIO_PIN_0 << (led - 1));
	
	GPIOC->ODR = LED_STATE << 8;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}
/**************************************串口回调函数**************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(rxdat == 'C')
	{
		flag3 = 1;
		dtime3 = 0;
		sprintf(text1,"C:H%d+L%d\r\n",(int)(adc_val*100/3.3),Level);
		HAL_UART_Transmit(huart,(unsigned char *)text1,strlen(text1),50);
	}
	else if(rxdat == 'S')
	{
		flag3 = 1;
		dtime3 = 0;
		sprintf(text1,"S:TL%d+TM%d+TH%d\r\n",TH[0],TH[1],TH[2]);
		HAL_UART_Transmit(huart,(unsigned char *)text1,strlen(text1),50);
	}
	else;
	HAL_UART_Receive_IT(huart,(unsigned char *)&rxdat,1);
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
