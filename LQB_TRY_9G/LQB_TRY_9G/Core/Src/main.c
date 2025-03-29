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
bool page=0,flag=0,lflag=0;
double adc[2] = { 0 };
unsigned char line=0,goods=0,key_value=0,key_value0=0,key_value1=0,judge=0;
char text[30],text1[40];
int st=0,time=0,ktime=0,price[3] = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LCD_Proc(void);
void LED_Proc(unsigned char led,unsigned char sta);
void ADC_Proc(void);
void ADC_KEY(void);
uint16_t ADC_Key_Scan(void);
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
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	//LCD_INIT
	LCD_Init();
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Black);
	//ADC_INIT
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
	//I2C_INIT
	I2CInit();
	price[0] = (eeprom_read(0)<<8) + eeprom_read(1);
	if(price[0] > 1000 || price[0] < 0) price[0] = 20;
	price[1] = (eeprom_read(2)<<8) + eeprom_read(3);
	if(price[1] > 1000 || price[1] < 0) price[1] = 30;
	price[2] = (eeprom_read(4)<<8) + eeprom_read(5);
	if(price[2] > 1000 || price[2] < 0) price[2] = 30;
	st = (eeprom_read(6) << 8) + eeprom_read(7);
	if(st < 0) st = 0;
	//TIM_INIT
	HAL_TIM_Base_Start_IT(&htim6);
	//LED_INIT
	LED_Proc(0,0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		LCD_Proc();
		ADC_KEY();
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
/************************************************************************************************
*屏幕显示部分
*LCD
************************************************************************************************/
void LCD_Proc(void)
{
	if(page == 0)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        USER        ");
		sprintf(text,"    Num:%d           ",goods+1);
		LCD_DisplayStringLine(Line2,(u8 *)text);
		sprintf(text,"    Price:%.2f           ",((double)price[goods])/100);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"    weight:%.2f           ",adc[0]/409.5);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		sprintf(text,"    Money:%.1f           ",adc[0]/409.5*((double)price[goods])/100);
		LCD_DisplayStringLine(Line5,(u8 *)text);
		
		LCD_DisplayStringLine(Line6,(u8 *)"                      ");
	}
	else if(page == 1)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"      Setting       ");
		
		if(line==0) LCD_SetTextColor(Green);
		sprintf(text,"    Price1:%.2f/Kg        ",((double)price[0])/100);
		LCD_DisplayStringLine(Line2,(u8 *)text);
		LCD_SetTextColor(Black);
		
		if(line==1) LCD_SetTextColor(Green);
		sprintf(text,"    Price2:%.2f/Kg        ",((double)price[1])/100);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		LCD_SetTextColor(Black);
		
		if(line==2) LCD_SetTextColor(Green);
		sprintf(text,"    Price3:%.2f/Kg        ",((double)price[2])/100);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		LCD_SetTextColor(Black);
		
		LCD_DisplayStringLine(Line5,(u8 *)"                      ");
		
		sprintf(text,"          STimes:%d",st);
		LCD_DisplayStringLine(Line6,(u8 *)text);
	}
	else page = 0;
}
/************************************************************************************************
*LED控制部分
*LED
************************************************************************************************/
void LED_Proc(unsigned char led,unsigned char sta)
{
	static unsigned char LED_STA = 0xff;
	if(sta == 0) LED_STA |= (0x01 << (led - 1));
	else LED_STA &= ~(0x01 << (led - 1));
	
	GPIOC->ODR = LED_STA << 8;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}
/************************************************************************************************
*ADC值获取部分
*ADC
************************************************************************************************/
void ADC_Proc(void)
{
	HAL_ADC_Start(&hadc2);
	adc[0] = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Start(&hadc2);
	adc[1] = HAL_ADC_GetValue(&hadc2);
}
/************************************************************************************************
*按键操作（功能）部分
*KEY
************************************************************************************************/
void ADC_KEY(void)
{
	if(key_value1 == 1)
	{
		page = page==0?1:0;
		if(page == 0)
		{
			unsigned char h,l;
			st++;
			flag = 0;
			time = 0;
			for(int i=0;i<3;i++)
			{
				h = price[i] >> 8;
				l = price[i] & 0x00ff;
				eeprom_write(2*i+0,h);
				HAL_Delay(10);
				eeprom_write(2*i+1,l);
				HAL_Delay(10);
			}
			h = st >> 8;
			l = st & 0x00ff;
			eeprom_write(6,h);
			HAL_Delay(10);
			eeprom_write(7,l);
			HAL_Delay(10);
			sprintf(text1,"U.W.1:%.2f\r\nU.W.2:%.2f\r\nU.W.3:%.2f\r\n\r\n",
			((double)price[0])/100,((double)price[1])/100,((double)price[2])/100);
			HAL_UART_Transmit(&huart1,(u8 *)text1,strlen(text1),50);
		}
		else 
		{
			flag = 1;
			time = 0;
		}
		key_value1 = 0;
	}
	if(key_value1 == 2)
	{
		if(page == 1) 
		{
			price[line] = price[line]>=1000?1000:price[line]+1;
		}
		key_value1 = 0;
	}
	if(key_value1 == 3)
	{
		if(page == 1) 
		{
			price[line] = price[line]<=0?0:price[line]-1;
		}
		key_value1 = 0;
	}
	if(key_value1 == 4)
	{
		line = line==2?0:line+1;
		key_value1 = 0;
	}
	if(key_value1 == 5)
	{
		if(page == 0) goods = 0;
		key_value1 = 0;
	}
	if(key_value1 == 6)
	{
		if(page == 0) goods = 1;
		key_value1 = 0;
	}
	if(key_value1 == 7)
	{
		if(page == 0) goods = 2;
		key_value1 = 0;
	}
	if(key_value1 == 8)
	{
		if(page == 0) 
		{
			sprintf(text1,"U.W.%d:%.2f\r\nG.W:%.2f\r\nTotal:%.2f\r\n\r\n",
			goods+1,((double)price[goods])/100,adc[0]/409.5,adc[0]/409.5*((double)price[goods])/100);
			HAL_UART_Transmit(&huart1,(u8 *)text1,strlen(text1),50);
		}
		key_value1 = 0;
	}
}
/************************************************************************************************
*按键判断部分
*KEY
************************************************************************************************/
uint16_t ADC_Key_Scan(void)
{
	if((int)adc[1] < 150) return 1;
	if((int)adc[1] < 800) return 2;
	if((int)adc[1] < 1600) return 3;
	if((int)adc[1] < 2000) return 4;
	if((int)adc[1] < 2700) return 5;
	if((int)adc[1] < 3300) return 6;
	if((int)adc[1] < 3700) return 7;
	if((int)adc[1] < 4000) return 8;
	return 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		//LED闪烁部分
		time++;
		if(flag == 0) 
		{
			if(time >= 40) 
			{
				lflag = 1 - lflag;
				LED_Proc(1,lflag);
				time = 0;
			}
		}
		else if(flag == 1)
		{
			if(time >= 80) 
			{
				lflag = 1 - lflag;
				LED_Proc(1,lflag);
				time = 0;
			}
		}
		//ADC读值部分
		ADC_Proc();
		/**************************************
		*按键判断部分
		**************************************/
		key_value0 = ADC_Key_Scan();
		switch(judge)
		{
			case 0:
			{
				if(key_value0 != 0) judge = 1;
			}
			break;
			case 1:
			{
				if(key_value0 != 0) 
				{
					judge = 2;
					key_value = key_value0;
				}
				else judge = 0;
			}
			break;
			case 2:
			{
				ktime++;
				if(ktime < 80 && key_value0 == 0)
				{
					key_value1 = key_value;
					ktime = 0;
					judge = 0;
					key_value = 0;
				}
				else if(ktime > 80 && key_value !=2 && key_value != 3)
				{
					key_value1 = key_value;
					ktime = 0;
					key_value = 0;
				}
				else if(key_value0 != 0 && ktime > 80)
				{
					judge = 3;
					key_value1 = key_value;
					ktime = 0;
				}
				else if(key_value0 == 0)
				{
					judge = 0;
					key_value = 0;
					ktime = 0;
				}
			}
			break;
			case 3:
			{
				ktime++;
				if(key_value0 != 0 && ktime > 10)
				{
					ktime = 0;
					key_value1 = key_value;
				}
				else if(key_value0 == 0)
				{
					ktime = 0;
					judge = 0;
					key_value = 0;
				}
			}
			break;
		}	
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
