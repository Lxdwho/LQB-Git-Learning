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
	bool key_sta,long_sta,single_sta;
	unsigned char time_sta,judge_sta;
}keys;
typedef struct
{
	bool flag1,flag2;
	int time;
}flags;
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
unsigned char page=0,R[2]={1,1},K[2]={1,1},N=0;
bool line=0,mode=0,lock=0;
char text[30];
double duty=0,V[2]={0},MH=0,ML=0,frq = 0,frq1;
flags adcflag = {0},frqflag = {0},vflag={0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LED_Proc(unsigned char led,unsigned char sta);
void KEY_Proc(void);
void LCD_Proc(void);
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
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	//TIM6_INIT
	HAL_TIM_Base_Start_IT(&htim6);
	//LCD_INIT
	LCD_Init();
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	//TIM17_INIT
	HAL_TIM_IC_Start_IT(&htim17,TIM_CHANNEL_1);
	//TIM2_INIT
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	//LED_INIT
	LED_Proc(1,1);
	//ADC_INIT
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
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
/**********************************************************************************
*TIM6:0.01s进入一次
*TIM6
**********************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		adcflag.time++;
		if(adcflag.time>10)
		{
			adcflag.time = 0;
			if(lock == 0) ADCValue();
		}
		
		if(frqflag.flag1 == 1)
		{
			frqflag.time++;
			if(frqflag.time %10 == 0 && frqflag.time<=500)
			{
				if(mode == 0) frq1 += 80;
				else frq1 -= 80;
				htim2.Init.Prescaler = 800000/frq1 - 1;
				HAL_TIM_Base_Init(&htim2);
				LED_Proc(2,frqflag.time%20==0);
			}
			else if(frqflag.time > 500)
			{
				frqflag.time = 0;
				mode = mode == 0?1:0;
				frqflag.flag1 = 0;
				frqflag.flag2 = 0;
				vflag.time = 0;
				LED_Proc(2,0);
			}
		}
		vflag.time++;
		if(vflag.time%10 == 0)
		{
			V[0] = frq * 6.28 * R[1] / 100 / K[1];
			if((V[0] > V[1] && V[0] - V[1] > 0.5) || (V[0] < V[1] && V[1] - V[0] > 0.5))
			{
				vflag.time = 0;
				V[1] = V[0];
			}
		}
		if(vflag.time >= 200)
		{
			vflag.time = 0;
			V[1] = V[0];
			if(mode == 0 && V[1] > ML) ML = V[1];
			else if(mode == 1 && V[1] > MH) MH = V[1];
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
					if(key[i].key_sta == 0) key[i].judge_sta = 2;
				}
				break;
				case 2:
				{
					key[i].time_sta++;
					if(key[i].time_sta < 200 && key[i].key_sta == 1)
					{
						key[i].single_sta = 1;
						key[i].time_sta = 0;
						key[i].judge_sta = 0;
					}
					else if(key[i].time_sta >= 200 && key[i].key_sta == 0)
					{
						key[i].long_sta = 1;
						key[i].judge_sta = 3;
						key[i].time_sta = 0;
					}
				}
				break;
				case 3:
				{
					if(key[i].key_sta == 1) key[i].judge_sta = 0;
				}
				break;
			}
		}
	}
}
/**********************************************************************************
*LCD显示函数
*LCD_Proc
**********************************************************************************/
void LCD_Proc(void)
{
	if(page == 0)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        DATA        ");
		
		if(mode == 0) LCD_DisplayStringLine(Line3,(u8 *)"    M=L             ");
		else if(mode == 1) LCD_DisplayStringLine(Line3,(u8 *)"    M=H             ");
		sprintf(text,"    P=%.0f%%             ",duty);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		sprintf(text,"    V=%.1f             ",V[0]);
		LCD_DisplayStringLine(Line5,(u8 *)text);
	}
	else if(page == 1)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        PARA        ");
		sprintf(text,"    R=%d             ",R[0]);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"    K=%d             ",K[0]);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		
		LCD_DisplayStringLine(Line5,(u8 *)"                    ");
	}
	else if(page == 2)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        RECD        ");
		
		sprintf(text,"    N=%d             ",N);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"    MH=%.1f             ",MH);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		sprintf(text,"    ML=%.1f             ",ML);
		LCD_DisplayStringLine(Line5,(u8 *)text);
	}
	else page = 0;
}
/**********************************************************************************
*按键操作函数
*KEY_Proc
**********************************************************************************/
void KEY_Proc(void)
{
	if(key[0].single_sta == 1)
	{
		page = page == 2?0:page+1;
		if(page == 0) LED_Proc(1,1);
		else LED_Proc(1,0);
		if(page == 2) 
		{
			R[1] = R[0];
			K[1] = K[0];
		}
		key[0].single_sta = 0;
	}
	if(key[1].single_sta == 1)
	{
		if(page == 0 && frqflag.flag2 == 0) 
		{
			N++;
			frqflag.flag1 = 1;
			frqflag.flag2 = 1;
			if(mode == 0) frq1 = 4000;
			else frq1 = 8000;
		}
		else if(page == 1) line = line == 0?1:0;
		key[1].single_sta = 0;
	}
	if(key[2].single_sta == 1)
	{
		if(page == 1)
		{
			if(line == 0) R[0] = R[0] == 10?1:R[0]+1;
			else K[0] = K[0] == 10?1:K[0]+1;
		}
		key[2].single_sta = 0;
	}
	if(key[3].single_sta == 1)
	{
		if(page == 1)
		{
			if(line == 0) R[0] = R[0] == 1?10:R[0]-1;
			else K[0] = K[0] == 1?10:K[0]-1;
		}
		if(page == 0) 
		{
			lock = lock == 1?0:0;
			LED_Proc(3,0);
		}
		key[3].single_sta = 0;
	}
	if(key[3].long_sta == 1)
	{
		if(page == 0) 
		{
			lock = 1;
			LED_Proc(3,1);
		}
		key[3].long_sta = 0;
	}
}
/**********************************************************************************
*LED操作函数
*LED_Proc
**********************************************************************************/
void LED_Proc(unsigned char led,unsigned char sta)
{
	static unsigned char LED_STA = 0xff;
	if(sta == 0) LED_STA |= (0x01 << (led-1));
	else if(sta == 1) LED_STA &= ~(0x01 << (led-1));
	
	GPIOC->ODR = LED_STA << 8;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}
/**********************************************************************************
*ADC读值
*ADCValue
**********************************************************************************/
void ADCValue(void)
{
	HAL_ADC_Start(&hadc2);
	duty = HAL_ADC_GetValue(&hadc2);
	duty = (duty / 4096 * 3.3 - 1) * 37.5 +10;
	duty = duty<10?10:duty;
	duty = duty>85?85:duty;
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,duty);
}
/**********************************************************************************
*输入捕获
*TIM17
**********************************************************************************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM17)
	{
		frq = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
		__HAL_TIM_SetCounter(htim,0);
		frq = 1000000/frq;
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
