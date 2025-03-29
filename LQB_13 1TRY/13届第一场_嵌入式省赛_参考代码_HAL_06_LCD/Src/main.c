/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *作者：鹏老师
	*时间：2023.3.29
	*CSDN：我是鹏老师
	*Bilibili：我是鹏老师
	*新版(G431)蓝桥杯嵌入式开发板，只卖199
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int PSD_B1 = 1; //密码第一位
int PSD_B2 = 2; //密码第二位
int PSD_B3 = 3; //密码第三位

int LD1 = 0; //记录LD1亮灭状态(在stm32g4xx_it.c 文件 SysTick_Handler 函数中使用)
int LD2_Flag = 0; //用来便捷LD2是否要闪烁(在stm32g4xx_it.c 文件 SysTick_Handler 函数中使用)

uint8_t Key_Scan(void) //按键扫描函数
{   
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0) //PB0 （按键B1）被按下
	{
		HAL_Delay(10);//延时消抖
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0)
		{
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0); //等待按键抬起
			return 1;//返回1
		}
	}
	
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0) //PB1 （按键B2）被按下
	{
		HAL_Delay(10);//延时消抖
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0)
		{
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0); //等待按键抬起
			return 2;//返回2
		}
	}
	
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0) //PB2 （按键B3）被按下
	{
		HAL_Delay(10);//延时消抖
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0)
		{
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0); //等待按键抬起
			return 3; //返回3
		}
	}
	
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0) //PA0 （按键B4）被按下
	{
		HAL_Delay(10);//延时消抖
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0)
		{
			while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0); //等待按键抬起
			return 4; //返回4
		}
	}
	
	return 0; //没有按键按下返回0
}

uint8_t uart_rx_buf[128]; //串口接收数据缓冲区
//串口中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	if((uart_rx_buf[0] - 0x30) == PSD_B1 && (uart_rx_buf[1] - 0x30) == PSD_B2 && (uart_rx_buf[2] - 0x30) == PSD_B3) //如果旧密码正确
	{
		//则修改密码
		PSD_B1 = uart_rx_buf[4] - 0x30;
		PSD_B2 = uart_rx_buf[5] - 0x30;
		PSD_B3 = uart_rx_buf[6] - 0x30;
	}
	
	HAL_UART_Receive_IT(&huart1, uart_rx_buf, 7); //设置串口中断缓冲区及中断阈值(当前为7)
}
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

    LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    LCD_Clear(Black);
    LCD_SetBackColor(Black);
    LCD_SetTextColor(White);
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All , 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2 , 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2 , 0);
		
		HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);//启动定时器2通道2的PWM输出

		HAL_UART_Receive_IT(&huart1, uart_rx_buf, 7); //设置串口中断缓冲区及中断阈值(当前为7)
		
		int Key_Value = 0;
		
		int B1 = -1;
		int B2 = -1;
		int B3 = -1;
		
		int Error_Time = 0;
		
		char buf[32] = { 0 };
		
		
    while (1)
    {
				Key_Value = Key_Scan();

				LCD_DisplayStringLine(Line1, (unsigned char *)"       PSD        ");
				
				if(B1 == -1) 
				{
					LCD_DisplayStringLine(Line3, (unsigned char *)"    B1:@          ");
				}
				else
				{
					sprintf(buf, "    B1:%d          ", B1);
					LCD_DisplayStringLine(Line3, (unsigned char *)buf);
				}
				
				if(B2 == -1) 
				{
					LCD_DisplayStringLine(Line4, (unsigned char *)"    B2:@          ");
				}
				else
				{
					sprintf(buf, "    B2:%d          ", B2);
					LCD_DisplayStringLine(Line4, (unsigned char *)buf);
				}
				
				if(B3 == -1) 
				{
					LCD_DisplayStringLine(Line5, (unsigned char *)"    B3:@          ");
				}
				else
				{
					sprintf(buf, "    B3:%d          ", B3);
					LCD_DisplayStringLine(Line5, (unsigned char *)buf);
				}

				
				if(Key_Value == 1) //按键B1被按下
				{
					B1++;
					if(B1 == 10) B1 = 0;
				}
				
				if(Key_Value == 2) //按键B2被按下
				{
					B2++;
					if(B2 == 10) B2 = 0;
				}
				
				if(Key_Value == 3) //按键B3被按下
				{
					B3++;
					if(B3 == 10) B3 = 0;
				}
				
				if(Key_Value == 4) //按键B4被按下
				{
					//验证密码， 如果密码正取则切换页面
					if(B1 == PSD_B1 && B2 == PSD_B2 && B3 == PSD_B3) //密码正确
					{
						Error_Time = 0; // 输错次数清零
						
						//显示状态信息
						LCD_DisplayStringLine(Line1, (unsigned char *)"       STA        ");
				
					  LCD_DisplayStringLine(Line3, (unsigned char *)"    F:2000Hz      ");
					  LCD_DisplayStringLine(Line4, (unsigned char *)"    D:10%         ");
					  LCD_DisplayStringLine(Line5, (unsigned char *)"                  ");
						
						htim2.Init.Period = 500; 
						HAL_TIM_Base_Init(&htim2);//频率设为2000Hz
						__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 50);  //  50/500 = 10%
						
						//LD1 亮
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All , 1);
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 , 0);
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2 , 1);
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2 , 0);
						LD1 = 1;	//防止LD2闪烁时哪LD1关掉
						
						HAL_Delay(5000); //演示5S

						htim2.Init.Period = 1000; 
						HAL_TIM_Base_Init(&htim2);//频率设为1000Hz
						__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 500);  //  500/1000 = 50%
						
						//LD1 灭
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_All , 1);
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2 , 1);
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2 , 0);
						LD1 = 0;		
						
						B1 = -1;
						B2 = -1;
						B3 = -1;
					}
					else //密码错误
					{
						B1 = -1; //将密码重置为@
						B2 = -1;
						B3 = -1;
						
						Error_Time ++;
						
						if(Error_Time > 2) //密码错误三次及以上
						{
							//LED2 0.1秒闪烁
							LD2_Flag = 1;//在stm32g4xx_it.c 文件 SysTick_Handler 函数中使用执行闪烁
						}
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
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
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

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0
                           PC1 PC2 PC3 PC4
                           PC5 PC6 PC7 PC8
                           PC9 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
