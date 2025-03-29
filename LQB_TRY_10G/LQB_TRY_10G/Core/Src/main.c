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
#include "i2c_hal.h"
#include "lcd.h"
#include "ds18b20.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	bool key_sta,single_sta,long_sta;
	unsigned char judge_sta,time_sta;
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
double val_a=0,val_b=0,adc[2]={0},temp=0;
bool page=0,line=0,leds_flag=0,temp_flag=0,tl_flag=0,uart_flag=0;
unsigned char duty=0,T=30,AO=0,stime=0,leds_time=0,judge[2]={0},t_time=0,uart_time=0;
char text[30],text1[20],text2[100];
uint16_t times=0;
unsigned char num[11] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x00};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LCD_Proc(void);
void KEY_Proc(void);
void ADCValue(void);
void LED_Proc(unsigned char led,unsigned char sta);
void LEDS_Proc(void);
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
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	//TIM6_INIT
	HAL_TIM_Base_Start_IT(&htim6);
	//TIM3_INIT
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
	//ADC_INIT
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
	//LCD_INIT
	LCD_Init();
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Black);
	//ds18b20_init
	ds18b20_init_x();
	//LED_INIT
	LED_Proc(1,0);
	//DATA_Init
	I2CInit();
	judge[0] = T ;
	judge[1] = AO;
	times = (eeprom_read(0) << 8) | eeprom_read(1);
	if(times >= 65535) times = 0;
	//UART_DMA_INIT
	HAL_UART_Receive_DMA(&huart1,(u8 *)text2,100);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		LCD_Proc();
		KEY_Proc();
		//达到上报条件标记
		if(adc[AO]/4095 > (double)duty/100)
		{
			LED_Proc(1,1);
			uart_flag = 1;
		}
		else
		{
			LED_Proc(1,0);
			uart_flag = 0;
			uart_time = 0;
		}
		//达到温度上限标记
		if(temp > T) temp_flag = 1;
		else 
		{
			temp_flag = 0;
			t_time = 0;
			LED_Proc(8,0);
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
/*****************************************************************************
*定时器中断，0.01s进入一次：按键部分
*KEY
*****************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		//采样部分
		stime++;
		if(stime >= 25)
		{
			stime = 0;
			ADCValue();
			temp = Read_temperature();
		}
		//数码管交替显示部分
		leds_time++;
		if(leds_time >= 200)
		{
			leds_flag = 1 - leds_flag;
			LEDS_Proc();
			leds_time = 0;
		}
		//LED闪烁部分
		if(temp_flag == 1)
		{
			t_time++;
			if(t_time >= 20)
			{
				tl_flag = 1 - tl_flag;
				LED_Proc(8,tl_flag);
				t_time = 0;
			}
		}
		//串口数据发送部分
		if(uart_flag == 1)
		{
			uart_time++;
			if(uart_time >= 100)
			{
				uart_time = 0;
				sprintf(text1,"$%.2f\r\n",temp);
				HAL_UART_Transmit(&huart1,(u8 *)text1,strlen(text1),50);
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
					if(key[i].key_sta == 1 && key[i].time_sta <= 80) 
					{
						key[i].judge_sta = 0;
						key[i].time_sta = 0;
						key[i].single_sta = 1;
					}
					else if(key[i].time_sta > 80) key[i].judge_sta = 3;
				}
				break;
				case 3:
				{
					key[i].time_sta++;
					if(key[i].key_sta == 0 && key[i].time_sta >= 10) 
					{
						key[i].time_sta = 0;
						key[i].long_sta = 1;
					}
					else if(key[i].key_sta == 1) 
					{
						key[i].judge_sta = 0;
						key[i].time_sta = 0;
					}
				}
				break;
			}
		}
	}
}
/*****************************************************************************
*脉冲捕捉
*puls
*****************************************************************************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM3)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			val_a = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);
			val_b = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_2);
			__HAL_TIM_SetCounter(&htim3,0);
			duty = (unsigned char)(val_a/val_b*100)>100?duty:(unsigned char)(val_a/val_b*100);
			HAL_TIM_IC_Start(&htim3,TIM_CHANNEL_1);
			HAL_TIM_IC_Start(&htim3,TIM_CHANNEL_2);
		}
	}
}
/*****************************************************************************
*ADC读值
*ADC
*****************************************************************************/
void ADCValue(void)
{
	HAL_ADC_Start(&hadc2);
	adc[0] = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Start(&hadc2);
	adc[1] = HAL_ADC_GetValue(&hadc2);
}
/*****************************************************************************
*按键操作函数
*KEY
*****************************************************************************/
void KEY_Proc(void)
{
	if(key[0].single_sta == 1)
	{
		page = page == 0?1:0;
		if(page == 0)
		{
			if(judge[0] != T || judge[1] != AO) 
			{
				times++;
				eeprom_write(0,times>>8);
				HAL_Delay(10);
				eeprom_write(1,times&0x00ff);
				HAL_Delay(10);
				judge[0] = T ;
				judge[1] = AO;
			}
		}
		key[0].single_sta = 0;
	}
	if(key[1].single_sta == 1)
	{
		if(page == 1) line = line == 0?1:0;
		key[1].single_sta = 0;
	}
	if(key[2].single_sta == 1)
	{
		if(page == 1)
		{
			if(line == 0) T = T >= 40?40:T+1;
			else AO = AO == 0?1:0;
		}
		key[2].single_sta = 0;
	}
	if(key[3].single_sta == 1)
	{
		if(page == 1)
		{
			if(line == 0) T = T <= 20?20:T-1;
			else AO = AO == 0?1:0;
		}
		key[3].single_sta = 0;
	}
	if(key[2].long_sta == 1)
	{
		if(page == 1) if(line == 0) T = T >= 40?40:T+1;
		key[2].long_sta = 0;
	}
	if(key[3].long_sta == 1)
	{
		if(page == 1) if(line == 0) T = T <= 20?20:T-1;
		key[3].long_sta = 0;
	}
}
/*****************************************************************************
*LCD显示功能
*LCD
*****************************************************************************/
void LCD_Proc(void)
{
	if(page == 0)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        Main        ");
		
		sprintf(text,"    AO1:%1.2fV            ",adc[0]/4095*3.3);
		LCD_DisplayStringLine(Line2,(u8 *)text);
		sprintf(text,"    AO2:%1.2fV            ",adc[1]/4095*3.3);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"    PWM2:%d%%   ",duty);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		sprintf(text,"    Temp:%.2f   ",temp);
		LCD_DisplayStringLine(Line5,(u8 *)text);
		sprintf(text,"    N:%d   ",times);
		LCD_DisplayStringLine(Line6,(u8 *)text);
	}
	else if(page == 1)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        Para        ");
		
		if(line == 0) LCD_SetBackColor(Yellow);
		sprintf(text,"    T:%d              ",T);
		LCD_DisplayStringLine(Line2,(u8 *)text);
		LCD_SetBackColor(White);
		
		if(line == 1) LCD_SetBackColor(Yellow);
		sprintf(text,"    X:AO%d            ",AO+1);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		LCD_SetBackColor(White);
		
		LCD_DisplayStringLine(Line4,(u8 *)"              ");
		LCD_DisplayStringLine(Line5,(u8 *)"              ");
		LCD_DisplayStringLine(Line6,(u8 *)"              ");
	}
	else page = 0;
}
/*****************************************************************************
*LED显示功能
*LED
*****************************************************************************/
void LED_Proc(unsigned char led,unsigned char sta)
{
	static unsigned char LED_STA = 0xff;
	if(sta == 0) LED_STA |= (0x01 << (led-1));
	else if(sta == 1) LED_STA &= ~(0x01 << (led-1));
	
	GPIOC->ODR = LED_STA << 8;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}
/*****************************************************************************
*数码管显示功能
*LEDS
*num[11] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x00};
*****************************************************************************/
void LEDS_Proc(void)
{
	int data=T,tep=0;
	
	if(leds_flag == 0)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
		for(int k=0;k<2;k++)
		{
			tep = num[data%10];
			data /= 10;
			for(int j=0;j<8;j++)
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
				if(tep & 0x80) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
				else HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
				tep = tep << 1;
			}
		}
		tep = 0x39;
		for(int j=0;j<8;j++)
		{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
			if(tep & 0x80) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
			else HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
			tep = tep << 1;
		}
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
	}
	else if(leds_flag == 1)
	{
		data = AO+1;
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
		for(int k=0;k<2;k++)
		{
			tep = num[data];
			data /= 10;
			for(int j=0;j<8;j++)
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
				if(tep & 0x80) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
				else HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
				tep = tep << 1;
			}
		}
		tep = 0x77;
		for(int j=0;j<8;j++)
		{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
			if(tep & 0x80) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
			else HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
			tep = tep << 1;
		}
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
	}
	else leds_flag = 0;
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
