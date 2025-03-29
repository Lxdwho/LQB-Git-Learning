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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "i2c_hal.h"
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
extern struct keys key[4];
uint8_t page=0,setting=0,time_sta=0;
char text[30];
char text1;

struct utimes utime[5] = { 0 };
struct utimes stime = { 0 };

extern int dtime;

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
  MX_TIM6_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim6);
	
//	HAL_TIM_PWM_Start(&htim16,TIM_CHANNEL_1);
//	__HAL_TIM_SetCompare(&htim16,TIM_CHANNEL_1,80);
	
	LCD_Init();
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Blue);
	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_All,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
	
	for(int i=0;i<5;i++)
	{
		utime[i].hour = eeprom_read(3*i)<24?eeprom_read(3*i):0;
		utime[i].min = eeprom_read(3*i+1)<60?eeprom_read(3*i+1):0;
		utime[i].sec = eeprom_read(3*i+2)<60?eeprom_read(3*i+2):0;
	}
	

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
}

/* USER CODE BEGIN 4 */
int i=0;
void KEY_Proc(void)
{
	/**************************************************短按键*************************************************/
	if(key[0].short_sta == 1)
	{
		page = page==4?0:page+1;
		key[0].short_sta = 0;
	}
	if(key[1].short_sta == 1)
	{
		setting = setting==3?1:setting+1;
		key[1].short_sta = 0;
	}
	if(key[2].short_sta == 1)
	{
		if(setting == 1) utime[page].hour = utime[page].hour==23?0:utime[page].hour+1;
		else if(setting == 2) utime[page].min = utime[page].min==59?0:utime[page].min+1;
		else if(setting == 3) utime[page].sec = utime[page].sec==59?0:utime[page].sec+1;
		key[2].short_sta = 0;
	}
	if(key[3].short_sta == 1)
	{
		if(time_sta == 0)
		{
			//dtime清零，开始计时
			dtime = 0;
			time_sta = 1;
			//定时时不再运行设置
			setting = 0;
			//存储原本定时值
			stime.hour = utime[page].hour;
			stime.min = utime[page].min;
			stime.sec = utime[page].sec;
		}
		else if(time_sta == 1) time_sta = 2;//暂停计时
		else time_sta = 1;//开始计时
		key[3].short_sta = 0;
	}
	/**************************************************长按键*************************************************/
	if(key[0].long_sta == 1)
	{
		
		key[0].long_sta = 0;
	}
	if(key[1].long_sta == 1)
	{
		setting = 0;
		eeprom_write(3*page,utime[page].hour);
		HAL_Delay(10);
		eeprom_write(3*page+1,utime[page].min);
		HAL_Delay(10);
		eeprom_write(3*page+2,utime[page].sec);
		key[1].long_sta = 0;
	}
	if(key[2].long_sta == 1)
	{
		if(setting == 1) utime[page].hour = utime[page].hour==23?0:utime[page].hour+1;
		else if(setting == 2) utime[page].min = utime[page].min==59?0:utime[page].min+1;
		else if(setting == 3) utime[page].sec = utime[page].sec==59?0:utime[page].sec+1;
		key[2].long_sta = 0;
	}
	if(key[3].long_sta == 1)
	{
		//恢复原本定时值
		utime[page].hour = stime.hour;
		utime[page].min = stime.min;
		utime[page].sec = stime.sec;
		time_sta = 0;//停止计时
		key[3].long_sta = 0;
	}
}

void LCD_Proc(void)
{
	sprintf(text,"    NO %d",page+1);
	LCD_DisplayStringLine(Line1,(unsigned char *)text);
	//时
	if(setting == 1) LCD_SetBackColor(Green);
	sprintf(&text1,"%d",utime[page].hour/10);
	LCD_DisplayChar(Line3,223,text1);
	sprintf(&text1,"%d",utime[page].hour%10);
	LCD_DisplayChar(Line3,207,text1);
	LCD_SetBackColor(White);
	
	LCD_DisplayChar(Line3,191,':');
	//分
	if(setting == 2) LCD_SetBackColor(Green);
	sprintf(&text1,"%d",utime[page].min/10);
	LCD_DisplayChar(Line3,175,text1);
	sprintf(&text1,"%d",utime[page].min%10);
	LCD_DisplayChar(Line3,159,text1);
	LCD_SetBackColor(White);
	
	LCD_DisplayChar(Line3,143,':');
	//秒
	if(setting == 3) LCD_SetBackColor(Green);
	sprintf(&text1,"%d",utime[page].sec/10);
	LCD_DisplayChar(Line3,127,text1);
	sprintf(&text1,"%d",utime[page].sec%10);
	LCD_DisplayChar(Line3,111,text1);
	LCD_SetBackColor(White);
	
	if(setting != 0) sprintf(text,"      Setting      ");
	else if(time_sta == 1) 
	{
		if(dtime <= 50) LED_Proc(1,1);
		else if(dtime > 50) LED_Proc(1,0);
		HAL_TIM_PWM_Start(&htim16,TIM_CHANNEL_1);
		sprintf(text,"      Running      ");
	}
	else if(time_sta == 2) 
	{
		LED_Proc(1,0);
		HAL_TIM_PWM_Stop(&htim16,TIM_CHANNEL_1);
		sprintf(text,"       Pause       ");
	}
	else 
	{
		LED_Proc(1,0);
		HAL_TIM_PWM_Stop(&htim16,TIM_CHANNEL_1);
		sprintf(text,"      Standby      ");
	}
	LCD_DisplayStringLine(Line5,(unsigned char *)text);
	
}

void LED_Proc(unsigned char led,unsigned char sta)
{
	static unsigned char LED_State = 0xff;
	if(led>8 || led <1) return;
	if(sta == 0) LED_State |= (GPIO_PIN_0 << (led-1)); 
	else LED_State &= ~(GPIO_PIN_0 << (led-1));
	GPIOC->ODR = LED_State << 8;	
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_All,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOC,~(LED_State << 8),GPIO_PIN_RESET);
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
