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
bool page=0,pa7=0;
double frq_2=0,frq_15=0,adc[2]={0};
keys key[4] = { 0 };
char text[30];
unsigned char v_led[2]={1,1},f_led[2]={2,2},adc_time=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LCD_Proc(void);
void KEY_Proc(void);
void ADCValue(void);
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
  MX_TIM6_Init();
  MX_TIM15_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	//TIM15、TIM2_INIT
	HAL_TIM_IC_Start_IT(&htim15,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
	//TIM6_INIT
	HAL_TIM_Base_Start_IT(&htim6);
	//TIM17_INIT
	HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);
	//LED_INIT
	LED_Proc(1,0);
	LCD_Init();
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		KEY_Proc();
		LCD_Proc();
		if(frq_2 > frq_15) LED_Proc(f_led[0],1);
		else LED_Proc(f_led[0],0);
		if(adc[0] > adc[1]) LED_Proc(v_led[0],1);
		else LED_Proc(v_led[0],0);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/************************************************************************************
*定时器定时
*TIM6 0.01s
************************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		//ADC
		adc_time++;
		if(adc_time >= 20)
		{
			ADCValue();
			
			if(pa7 == 0)
			{
				htim17.Init.Prescaler = (800000/frq_2)-1;
				HAL_TIM_Base_Init(&htim17);
			}
			else if(pa7 == 1)
			{
				htim17.Init.Prescaler = (800000/frq_15)-1;
				HAL_TIM_Base_Init(&htim17);
			}
			else pa7 = 0;
			adc_time = 0;
		}
		
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
						key[i].single_sta = 1;
						key[i].judge_sta = 2;
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
*按键操作
*KEY_Proc
************************************************************************************/
void KEY_Proc(void)
{
	if(key[0].single_sta == 1)
	{
		page = page == 0?1:0;
		if(page == 0)
		{
			LED_Proc(v_led[0],0);
			LED_Proc(f_led[0],0);
			v_led[0] = v_led[1];
			f_led[0] = f_led[1];
		}
		key[0].single_sta = 0;
	}
	if(key[1].single_sta == 1)
	{
		if(page == 1)
		{
			v_led[1] = v_led[1] >= 8?1:v_led[1]+1;
			v_led[1] = v_led[1] == f_led[1]?v_led[1]+1:v_led[1];
			v_led[1] = v_led[1] >= 9?1:v_led[1];
		}
		key[1].single_sta = 0;
	}
	if(key[2].single_sta == 1)
	{
		if(page == 1)
		{
			f_led[1] = f_led[1] >= 8?1:f_led[1]+1;
			f_led[1] = f_led[1] == v_led[1]?f_led[1]+1:f_led[1];
			f_led[1] = f_led[1] >= 9?1:f_led[1];
		}
		key[2].single_sta = 0;
	}
	if(key[3].single_sta == 1)
	{
		pa7 = pa7 == 0?1:0;
		key[3].single_sta = 0;
	}
}
/************************************************************************************
*屏幕显示
*LCD_Proc
************************************************************************************/
void LCD_Proc(void)
{
	if(page == 0)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"    DATA  ");
		
		sprintf(text,"    V1:%.2fV    ",adc[0]/4095*3.3);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"    V2:%.2fV    ",adc[1]/4095*3.3);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		sprintf(text,"    F1:%.0fHz    ",frq_2);
		LCD_DisplayStringLine(Line5,(u8 *)text);
		sprintf(text,"    F2:%.0fHz    ",frq_15);
		LCD_DisplayStringLine(Line6,(u8 *)text);
	}
	else if(page == 1)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"    PARA  ");
		
		sprintf(text,"    VD:LD%d    ",v_led[1]);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"    FD:LD%d    ",f_led[1]);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		LCD_DisplayStringLine(Line5,(u8 *)"                    ");
		LCD_DisplayStringLine(Line6,(u8 *)"                    ");
	}
	else page = 0;
}
/************************************************************************************
*ADC读值
*ADCValue
************************************************************************************/
void ADCValue(void)
{
	HAL_ADC_Start(&hadc2);
	adc[0] = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Start(&hadc2);
	adc[1] = HAL_ADC_GetValue(&hadc2);
}
/************************************************************************************
*LED控制
*LED_Proc
************************************************************************************/
void LED_Proc(unsigned char led,unsigned char sta)
{
	static unsigned char LED_STA = 0xff;
	if(sta == 0) LED_STA |= (0x01 << (led - 1));
	else LED_STA &= ~(0x01 << (led - 1));
	
	GPIOC->ODR = LED_STA << 8;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}
/************************************************************************************
*输入捕获
*TIM2\CH2 || TIM15\CH1
************************************************************************************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		frq_2 = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
		__HAL_TIM_SetCounter(htim,0);
		frq_2 = 1000000 / frq_2;
		HAL_TIM_IC_Start(htim,TIM_CHANNEL_2);
	}
	else if(htim->Instance == TIM15)
	{
		frq_15 = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
		__HAL_TIM_SetCounter(htim,0);
		frq_15 = 1000000 / frq_15;
		HAL_TIM_IC_Start(htim,TIM_CHANNEL_1);
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
