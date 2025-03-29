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
#include "i2c_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	bool long_sta,key_sta,single_sta;
	unsigned char time_sta,judge_sta;
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
unsigned char page=0,X=1,Y=1,N=0,ADCTIME=0,lcd_sta=0,v_time=0;
char text[30],rxdata[100];
double PA4[101]={0},PA5[101]={0},A[2]={0},T[2]={10,10},H[2]={0},PA1=0;
bool PA=0,mode=0,v_flag=0,vl_flag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void KEY_Proc(void);
void LCD_Proc(void);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
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
	//ADC_INIT
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
	//TIM2_CH2、TIM17_CH1_INIT
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);
	//LED_INIT
	LED_Proc(1,1);
	//I2C_INIT
	I2CInit();
	X = eeprom_read(1);
	if(X < 1 || X > 4) X = 1;
	Y = eeprom_read(0);
	if(Y < 1 || Y > 4) Y = 1;
	if(mode == 0) htim17.Init.Prescaler = 8000000/PA1/X;
	else htim17.Init.Prescaler = 8000000/PA1*X;
	HAL_TIM_Base_Init(&htim17);
	//UART_INIT
	HAL_UART_Receive_DMA(&huart1,(uint8_t *)rxdata,100);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		LCD_Proc();
		KEY_Proc();
		if(lcd_sta == 0) LED_Proc(4,1);
		else LED_Proc(4,0);
		if(PA4[1] > PA5[1] * Y) v_flag = 1;
		else 
		{
			LED_Proc(3,0);
			v_flag = 0;
			v_time = 0;
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
/**********************************************************************************
*TIM6:0.01s
*TIM
**********************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		ADCTIME++;
		if(ADCTIME >= 10)
		{
			ADCTIME = 0;
			ADCValue();
		}
		
		if(v_flag == 1)
		{
			v_time++;
			if(v_time >= 10)
			{
				vl_flag = vl_flag >=1?0:1;
				LED_Proc(3,vl_flag);
				v_time = 0;
			}
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
					else key[i].judge_sta = 0;
				}
				break;
				case 2:
				{
					key[i].time_sta++;
					if(key[i].key_sta == 1 && key[i].time_sta<100)
					{
						key[i].single_sta = 1;
						key[i].judge_sta = 0;
						key[i].time_sta = 0;
					}
					else if(key[i].time_sta >= 100 && key[i].key_sta == 0) 
					{
						key[i].judge_sta = 3;
						key[i].long_sta = 1;
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
*按键操作函数
*KEY_Proc
**********************************************************************************/
void KEY_Proc(void)
{
	if(key[0].single_sta == 1)
	{
		page = page >= 2?0:page+1;
		if(page == 2) PA = 0;
		key[0].single_sta = 0;
	}
	if(key[1].single_sta == 1)
	{
		if(page == 1) X = X >= 4?1:X+1;
		eeprom_write(1,X);
		HAL_Delay(10);
		if(mode == 0) htim17.Init.Prescaler = 8000000/PA1/X;
		else htim17.Init.Prescaler = 8000000/PA1*X;
		HAL_TIM_Base_Init(&htim17);
		key[1].single_sta = 0;
	}
	if(key[2].single_sta == 1)
	{
		if(page == 1) Y = Y >= 4?1:Y+1;
		eeprom_write(0,Y);
		HAL_Delay(10);
		key[2].single_sta = 0;
	}
	if(key[3].single_sta == 1)
	{
		if(page == 0)
		{
			for(int j=100;j>0;j--)
			{
				PA4[j] = PA4[j-1];
				PA5[j] = PA5[j-1];
			}
			if(PA4[1] > A[0]) A[0] = PA4[1];
			if(PA4[1] < T[0]) T[0] = PA4[1];
			if(PA5[1] > A[1]) A[1] = PA5[1];
			if(PA5[1] < T[1]) T[1] = PA5[1];
			N = N>=100?100:N+1;
			H[0] = 0;
			H[1] = 0;
			for(int j=0;j<N;j++)
			{
				H[0] = H[0] + PA4[j+1];
				H[1] = H[1] + PA5[j+1];
			}
			H[0] /= N;
			H[1] /= N;
		}
		else if(page == 1) 
		{
			mode = mode ==0?1:0;
			if(mode == 0) 
			{
				htim17.Init.Prescaler = 8000000/PA1/X;
				LED_Proc(1,1);
				LED_Proc(2,0);
			}
			else 
			{
				LED_Proc(1,0);
				LED_Proc(2,1);
				htim17.Init.Prescaler = 8000000/PA1*X;
			}
			HAL_TIM_Base_Init(&htim17);
		}
		else if(page == 2) PA = PA == 0?1:0;
		key[3].single_sta = 0;
	}
	if(key[3].long_sta == 1)
	{
		if(page == 2)
		{
			for(int j=1;j<101;j++)
			{
				PA4[j] = 0;
				PA5[j] = 0;
			}
			N = 0;
			A[0] = 0;
			A[1] = 0;
			T[0] = 10;
			T[1] = 10;
			H[0] = 0;
			H[1] = 0;
		}
		key[3].long_sta = 0;
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
		
		sprintf(text,"     PA4:%1.2f     ",PA4[1]);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"     PA5:%1.2f     ",PA5[1]);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		sprintf(text,"     PA1:%.0f     ",PA1);
		LCD_DisplayStringLine(Line5,(u8 *)text);
		
		LCD_DisplayStringLine(Line6,(u8 *)"                    ");

	}
	else if(page == 1)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        PARA        ");
		
		sprintf(text,"     X:%d         ",X);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"     Y:%d         ",Y);
		LCD_DisplayStringLine(Line4,(u8 *)text);

		LCD_DisplayStringLine(Line5,(u8 *)"                    ");
		LCD_DisplayStringLine(Line6,(u8 *)"                    ");
	}
	else if(page == 2)
	{
		if(PA == 0) LCD_DisplayStringLine(Line1,(u8 *)"        REC-PA4     ");
		else LCD_DisplayStringLine(Line1,(u8 *)"        REC-PA5     ");
		sprintf(text,"     N:%d     ",N);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		if(N == 0) LCD_DisplayStringLine(Line4,(u8 *)"     A:0     ");
		else
		{
			sprintf(text,"     A:%1.2f     ",(double)A[PA]);
			LCD_DisplayStringLine(Line4,(u8 *)text);
		}
		if(N == 0) LCD_DisplayStringLine(Line5,(u8 *)"     T:0     ");
		else 
		{
			sprintf(text,"     T:%1.2f     ",(double)T[PA]);
			LCD_DisplayStringLine(Line5,(u8 *)text);
		}
		sprintf(text,"     H:%1.2f     ",(double)H[PA]);
		LCD_DisplayStringLine(Line6,(u8 *)text);
					
	}
	else page = 0;
}
/**********************************************************************************
*ADC读值
*ADCValue
**********************************************************************************/
void ADCValue(void)
{
	HAL_ADC_Start(&hadc2);
	PA4[0] = HAL_ADC_GetValue(&hadc2);
	PA4[0] = PA4[0]/4095*3.3;
	HAL_ADC_Start(&hadc2);
	PA5[0] = HAL_ADC_GetValue(&hadc2);
	PA5[0] = PA5[0]/4095*3.3;
}
/**********************************************************************************
*输入捕获函数
*TIM2_CH2
**********************************************************************************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		PA1 = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
		PA1 = 1000000/PA1;
		__HAL_TIM_SetCounter(htim,0);
		HAL_TIM_IC_Start(htim,TIM_CHANNEL_2);
	}
}
/**********************************************************************************
*LED控制函数
*LED_Proc
**********************************************************************************/
void LED_Proc(unsigned char led,unsigned char sta)
{
	static unsigned char LED_STA = 0xff;
	if(sta == 0) LED_STA |= (0x01 << (led -1));
	else LED_STA &= ~(0x01 << (led -1));
	
	GPIOC->ODR = LED_STA << 8;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}

void YY(void)
{
		
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
