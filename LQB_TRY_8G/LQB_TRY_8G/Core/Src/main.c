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
#include "mint.h"
#include "lcd.h"
#include "i2c_hal.h"

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
bool page=0,line=0;
unsigned char puls1,puls2,f_math,p_math;
double adc[2];
char text[30];

extern keys key[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LCD_Proc(void);
void KEY_Proc(void);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM15_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	
	//LCD初始化
	LCD_Init();
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	//LED初始化
	LED_Proc(0,0);
	//KEY初始化
	HAL_TIM_Base_Start_IT(&htim6);
	//ADC初始化
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
	//输入捕获初始化
	HAL_TIM_IC_Start_IT(&htim15,TIM_CHANNEL_1); //PA1
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);	//PA2
	//PWM输出初始化
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);		//PA6
	HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);		//PA7
	//I2C初始化
	I2CInit();
	f_math = eeprom_read(0);
	p_math = eeprom_read(1);
	if(f_math > 4 || f_math <1) f_math = 1;
	if(p_math > 4 || p_math <1) p_math = 1;
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		LCD_Proc();
		KEY_Proc();
		if(adc[0] > adc[1]) LED_Proc(8,1);
		else LED_Proc(8,0);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/********************************************************
*LCD显示
********************************************************/
void LCD_Proc(void)
{
	if(page == 0)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        DATA        ");
		sprintf(text,"    PULS1:%dKHZ         ",puls1);
		LCD_DisplayStringLine(Line2,(u8 *)text);
		sprintf(text,"    PULS2:%dKHZ         ",puls2);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"    AO1:%.2fV         ",adc[0]/4095*3.3);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		sprintf(text,"    AO2:%.2fV         ",adc[1]/4095*3.3);
		LCD_DisplayStringLine(Line5,(u8 *)text);
		LCD_DisplayStringLine(Line7,(u8 *)"                 1 ");
	}
	else if(page == 1)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"       Setting      ");
		if(line == 0) LCD_SetTextColor(Green);
		sprintf(text,"    F_MATH:%d             ",f_math);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		LCD_SetTextColor(White);
		
		if(line == 1) LCD_SetTextColor(Green);
		sprintf(text,"    P_MATH:%d          ",p_math);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		LCD_SetTextColor(White);
		
		LCD_DisplayStringLine(Line2,(u8 *)"                    ");
		LCD_DisplayStringLine(Line5,(u8 *)"                    ");
		LCD_DisplayStringLine(Line7,(u8 *)"                 2  ");
	}
	else page = 0;
}
/********************************************************
*按键操作
********************************************************/
void KEY_Proc(void)
{
	if(key[0].single_sta == 1)
	{
		page = page == 0?1:0;
		if(page == 1) LED_Proc(1,1);
		else LED_Proc(1,0);
		key[0].single_sta = 0;
	}
	if(key[1].single_sta == 1)
	{
		if(page == 1) line = line==0?1:0;
		key[1].single_sta = 0;
	}
	if(key[2].single_sta == 1)
	{
		if(page == 1)
		{
			if(line == 0) f_math = f_math==4?1:f_math+1;
			else if(line == 1) p_math = p_math==4?1:p_math+1;
			eeprom_write(0,f_math);
			HAL_Delay(10);
			eeprom_write(1,p_math);
			HAL_Delay(10);
		}
		key[2].single_sta = 0;
	}
	if(key[3].single_sta == 1)
	{
		if(page == 1)
		{
			if(line == 0) f_math = f_math==1?4:f_math-1;
			else if(line == 1) p_math = p_math==1?4:p_math-1;
			eeprom_write(0,f_math);
			HAL_Delay(10);
			eeprom_write(1,p_math);
			HAL_Delay(10);
		}
		key[3].single_sta = 0;
	}
}
/********************************************************
*LED灯控制
********************************************************/
void LED_Proc(unsigned char led,unsigned char sta)
{
	static unsigned char LED_STA = 0xff;
	if(sta == 0) LED_STA |= (0x01 << (led-1));
	else if(sta == 1) LED_STA &= ~(0x01 << (led-1));
	
	GPIOC->ODR = LED_STA << 8;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}
/********************************************************
*获取ADC值
********************************************************/
void ADCvalue(void)
{
	HAL_ADC_Start(&hadc2);
	adc[0] = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Start(&hadc2);
	adc[1] = HAL_ADC_GetValue(&hadc2);
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
