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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	bool key_sta,single_sta;
	unsigned char judge_sta;
}keys;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
keys key[4] = { 0 };
bool page = 0,mode=0,adc_flag=0;
double a[6]={0},ax=0,bx=0,b[6]={0},f=0,val3[2]={0},val17[3]={0},adc[2]={0};
unsigned char pax=20,pbx=20,duty3=0,duty17=0,tim17_flag=0;
uint16_t pf = 1000;
char text[30],text1[30],rxdata[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void KEY_Proc(void);
void LCD_Proc(void);
void LED_Proc(unsigned char led,unsigned char sta);
void ADCValue(void);
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  MX_TIM6_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	//TIM6_INIT
	HAL_TIM_Base_Start_IT(&htim6);
	//LCD_INIT
	LCD_Init();
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	//TIM2\TIM3\TIM17_INIT
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim17,TIM_CHANNEL_1);
	//LED_INIT
	LED_Proc(1,0);
	//UART_INIT
	HAL_UART_Receive_DMA(&huart1,(uint8_t *)rxdata,100);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		KEY_Proc();
		LCD_Proc();
		//LED灯控制
		if(ax > pax) LED_Proc(1,1);
		else LED_Proc(1,0);
		if(bx > pbx) LED_Proc(2,1);
		else LED_Proc(2,0);
		if(f > pf) LED_Proc(3,1);
		else LED_Proc(3,0);
		if(mode == 0) LED_Proc(4,1);
		else LED_Proc(4,0);
		if(90+b[1]-a[1] < 10) LED_Proc(5,1);
		else LED_Proc(5,0);
		//光敏电阻状态
		if(mode == 1)
		{
			if(adc_flag == 1)
			{
				for(int j=5;j>0;j--) 
				{
					a[j] = a[j-1];
					b[j] = b[j-1];
				}
				ax = a[2]>=a[1]?a[2]-a[1]:a[1]-a[2];
				bx = b[2]>b[1]?b[2]-b[1]:b[1]-b[2];
				adc_flag = 0;
			}
		}
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
/************************************************************************************
*0.01s进入一次
*TIM6
************************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		if(tim17_flag == 3)
		{
			b[0] = ((val17[1]-val17[0])/(val17[2]-val17[0])*100 - 10) * 9 / 8;
			b[0] = b[0] > 90?90:b[0];
			b[0] = b[0] < 0?0:b[0];
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim17,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
			HAL_TIM_IC_Start_IT(&htim17,TIM_CHANNEL_1);
			tim17_flag = 0;
		}
		
		ADCValue();
		if(adc[0] > 2000 && adc[0] > adc[1] && adc[1] < 2000) adc_flag = 1;
		adc[1] = adc[0];
		
		key[0].key_sta = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
		key[1].key_sta = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
		key[2].key_sta = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
		key[3].key_sta = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
		
		for(int i=0;i<4;i++)
		{
			switch(key[i].judge_sta)
			{
				case 0:
				{
					if(key[i].key_sta == 0) key[i].judge_sta = 1;
				}
				break;
				case 1:
				{
					if(key[i].key_sta == 0)
					{
						key[i].judge_sta = 2;
						key[i].single_sta = 1;
					}
					else key[i].judge_sta = 0;
				}
				break;
				case 2:
				{
					if(key[i].key_sta == 1) key[i].judge_sta = 0;
				}
				break;
			}
		}
	}
}
/************************************************************************************
*按键操作函数
*KEY_Proc
************************************************************************************/
void KEY_Proc(void)
{
	if(key[0].single_sta == 1)
	{
		page = page == 0?1:0;
		key[0].single_sta = 0;
	}
	if(key[1].single_sta == 1)
	{
		if(page == 1)
		{
			pax = pax >= 60?10:pax+10;
			pbx = pbx >= 60?10:pbx+10;
		}
		key[1].single_sta = 0;
	}
	if(key[2].single_sta == 1)
	{
		if(page == 1) pf = pf >= 10000?1000:pf+1000;
		else mode = mode == 0?1:0;
		key[2].single_sta = 0;
	}
	if(key[3].single_sta == 1)
	{
		if(mode == 0) 
		{
			for(int j=5;j>0;j--) 
			{
				a[j] = a[j-1];
				b[j] = b[j-1];
			}
			ax = a[2]>=a[1]?a[2]-a[1]:a[1]-a[2];
			bx = b[2]>b[1]?b[2]-b[1]:b[1]-b[2];
		}
		key[3].single_sta = 0;
	}
}
/************************************************************************************
*屏幕显示函数
*LCD_Proc
************************************************************************************/
void LCD_Proc(void)
{
	if(page == 0)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        DATA ");
		
		sprintf(text,"   a:%.1f       ",a[1]);
		LCD_DisplayStringLine(Line2,(u8 *)text);
		sprintf(text,"   b:%.1f       ",b[1]);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"   f:%.0fHz     ",f);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		sprintf(text,"   ax:%.0f     ",ax);
		LCD_DisplayStringLine(Line6,(u8 *)text);
		sprintf(text,"   bx:%.0f     ",bx);
		LCD_DisplayStringLine(Line7,(u8 *)text);
		if(mode == 0) LCD_DisplayStringLine(Line8,(u8 *)"   mode:A     ");
		else LCD_DisplayStringLine(Line8,(u8 *)"   mode:B     ");
		
	}
	else if(page == 1)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        PARA ");
		
		sprintf(text,"   pax:%d       ",pax);
		LCD_DisplayStringLine(Line2,(u8 *)text);
		sprintf(text,"   pbx:%d       ",pbx);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"   pf:%d        ",pf);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		
		LCD_DisplayStringLine(Line6,(u8 *)"                  ");
		LCD_DisplayStringLine(Line7,(u8 *)"                  ");
		LCD_DisplayStringLine(Line8,(u8 *)"                  ");
	}
	else page = 0;
}
/************************************************************************************
*LED控制函数
*LED_Proc
************************************************************************************/
void LED_Proc(unsigned char led,unsigned char sta)
{
	static unsigned char LED_STA = 0xff;
	if(sta == 0) LED_STA |= (0x01 << (led - 1));
	else if(sta == 1) LED_STA &= ~(0x01 << (led - 1));
	
	GPIOC->ODR = LED_STA << 8;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}
/************************************************************************************
*输入捕获部分
*TIM3、TIM17
************************************************************************************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		f = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
		__HAL_TIM_SetCounter(htim,0);
		f = 1000000 / f;
		HAL_TIM_IC_Start(htim,TIM_CHANNEL_2);
	}
	else if(htim->Instance == TIM3)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			val3[0] = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
			val3[1] = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
			__HAL_TIM_SetCounter(htim,0);
			a[0] = (val3[1]/val3[0]*100-10) *18/8;
			a[0] = a[0] > 180?180:a[0];
			a[0] = a[0] < 0?0:a[0];
			HAL_TIM_IC_Start(htim,TIM_CHANNEL_1);
			HAL_TIM_IC_Start(htim,TIM_CHANNEL_2);
		}
	}
	else if(htim->Instance == TIM17)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			switch(tim17_flag)
			{
				case 0:
				{
					val17[0] = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
					tim17_flag=1;
					__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
				}
				break;
				case 1:
				{
					val17[1] = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
					tim17_flag=2;
					__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
				}
				break;
				case 2:
				{
					val17[2] = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
					tim17_flag=3;
					__HAL_TIM_SetCounter(htim,0);
					HAL_TIM_IC_Stop(htim,TIM_CHANNEL_1);
				}
				break;
			}
		}
	}
}
/************************************************************************************
*ADC读值部分
*ADC2_IN17
************************************************************************************/
void ADCValue(void)
{
	HAL_ADC_Start(&hadc2);
	adc[0] = HAL_ADC_GetValue(&hadc2);
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
