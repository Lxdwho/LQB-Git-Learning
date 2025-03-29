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
#include "i2c_hal.h"
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
extern struct keys key[4];
extern unsigned char rxdat;


uint8_t page=0;
uint8_t Buy_X=0,Buy_Y=0,Num_X=10,Num_Y=10;
double Money_X=1.0,Money_Y=1.0;

char text[30],text_uart[20];

int time_led1=0,time_led2=0;

uint8_t flag=0,flag2 = 0;

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
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim6);
	//PWM初始化
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	__HAL_TIM_SetCompare(&htim6,TIM_CHANNEL_2,25);
	//LCD初始化
	LCD_Init();
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	//初始化灯为灭
	LED_Proc(0x00);
	//开启I2C
	I2CInit();
	//开启串口中断
	HAL_UART_Receive_IT(&huart1,&rxdat,1);
	//存储值不合法，则认为是第一次上电
	Num_X = eeprom_read(0) == 255?10:eeprom_read(0);
	Num_Y = eeprom_read(1) == 255?10:eeprom_read(1);
	Money_X = eeprom_read(2) >20?1:((double)eeprom_read(2))/10;
	Money_Y = eeprom_read(3) >20?1:((double)eeprom_read(3))/10;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		LCD_Proc();
		KEY_Proc();
		
		if(flag == 1 && flag2 == 1) LED_Proc(0x03);
		else if(flag == 0 && flag2 == 1) LED_Proc(0x02);
		else if(flag == 1 && flag2 == 0) LED_Proc(0x01);
		else if(flag == 0 && flag2 == 0) LED_Proc(0x00);
		
		if(flag == 1) __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,25*6);
		else __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,25);
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
		LCD_DisplayStringLine(Line1,(u8 *)"        SHOP        ");
		sprintf(text,"     X:%d            ",Buy_X);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"     Y:%d            ",Buy_Y);
		LCD_DisplayStringLine(Line4,(u8 *)text);
	}
	else if(page == 1)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        PRICE       ");
		sprintf(text,"     X:%.1f            ",Money_X);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"     Y:%.1f            ",Money_Y);
		LCD_DisplayStringLine(Line4,(u8 *)text);
	}
	else if(page == 2)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        REP         ");
		sprintf(text,"     X:%d            ",Num_X);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"     Y:%d            ",Num_Y);
		LCD_DisplayStringLine(Line4,(u8 *)text);
	}
	else page = 0;
}

void KEY_Proc(void)
{
	if(key[0].single_sta == 1)
	{
		page = (page+1)>2?0:(page+1);
		key[0].single_sta = 0;
	}
	
	if(key[1].single_sta == 1)
	{
		if(page == 0) Buy_X = (Buy_X+1)>Num_X?0:(Buy_X+1);
		else if(page == 1) 
		{
			Money_X = (Money_X+0.1)>2.04?1:(Money_X+0.1);
			eeprom_write(2,(unsigned char)(Money_X*10));
		}
		else 
		{
			Num_X++;
			eeprom_write(0,(unsigned char)Num_X);
		}
		key[1].single_sta = 0;
	}
	
	if(key[2].single_sta == 1)
	{
		if(page == 0) Buy_Y = (Buy_Y+1)>Num_Y?0:(Buy_Y+1);
		else if(page == 1) 
		{
			Money_Y = (Money_Y+0.1)>2.04?1:(Money_Y+0.1);
			eeprom_write(3,(unsigned char)(Money_Y*10));
		}
		else 
		{
			Num_Y++;
			eeprom_write(1,(unsigned char)Num_Y);
		}
		key[2].single_sta = 0;
	}
	
	if(key[3].single_sta == 1)
	{
		if(page == 0) 
		{
			Num_X -= Buy_X;
			Num_Y -= Buy_Y;
			//存储
			if(Buy_X != 0) eeprom_write(0,(unsigned char)Num_X);
			HAL_Delay(10);
			if(Buy_Y != 0) eeprom_write(1,(unsigned char)Num_Y);
			//打印输出
			sprintf(text_uart,"X:%d,Y:%d,Z=%.1f\r\n",Buy_X,Buy_Y,(Buy_X*Money_X+Buy_Y*Money_Y));
			HAL_UART_Transmit(&huart1,(unsigned char *)text_uart,strlen(text_uart),50);
			//购买数量清零
			Buy_X = 0;
			Buy_Y = 0;
			//亮灯、PWM输出标志位置一
			flag = 1;
		}
		key[3].single_sta = 0;
	}
}

void LED_Proc(unsigned char pin)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_All,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,pin << 8,GPIO_PIN_RESET);
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
