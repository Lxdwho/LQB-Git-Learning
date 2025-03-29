/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mint.h"
#include "stdio.h"
#include "string.h"
#include "time.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
struct cars
{
	char car_num[5];
	char car_type[5];
	int year;
	int month;
	int day;
	int hours;
	int minter;
	int second;
	bool exit;
};
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
char text1[30];
struct cars car[9];

unsigned char rxdata[100];

int d_time=0;											//记录接收时间，20ms内未接受完成，视为发送字符数量不对
long TIME,time_0,time_1;					//TIME：记录停车时间，time_0：记录停车结束时间，time_1：记录停车开始时间
uint8_t flag0,num,num1;						//flag0：用于判断车辆是否合法，num：用于记录合法车辆位置，num1：记录找到的第一个空闲车位
double MONEY;											//MONEY：记录停车花费

extern double money_c,money_v;		//
extern uint8_t num_C,num_V;				//

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET)
	{
		HAL_UART_DMAStop(&huart1);
		if((100-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx)) != 22 && (100-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx)) != 0)
		{
			HAL_UART_Transmit_DMA(&huart1,(uint8_t *)"Error1\r\n",8);
		}
		else if((100-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx)) == 22)
		{
			sscanf((char *)rxdata, "%4s:%4s:%2d%2d%2d%2d%2d%2d", 
			car[0].car_type, car[0].car_num, &car[0].year, &car[0].month, &car[0].day, &car[0].hours, &car[0].minter, &car[0].second);
			//车类型判断
			if(car[0].car_type[0] != 'C' && car[0].car_type[0] != 'V') 
			{
				HAL_UART_Transmit_DMA(&huart1,(uint8_t *)"Error2\r\n",8);
				goto Error;
			}
			if(car[0].car_type[1] != 'N' || car[0].car_type[2] != 'B' || car[0].car_type[3] != 'R') 
			{
				HAL_UART_Transmit_DMA(&huart1,(uint8_t *)"Error3\r\n",8);
				goto Error; 
			}
			//时间格式判断
			if(car[0].month > 12 || car[0].month < 1 || car[0].day > 31 || car[0].day < 1 || car[0].hours > 23 || 
				 car[0].hours < 0 || car[0].minter > 59 || car[0].minter < 0 || car[0].second > 59 || car[0].second < 0) 
			{
				HAL_UART_Transmit_DMA(&huart1,(uint8_t *)"Error4\r\n",8);
				goto Error; 
			}
			
			struct tm tm_info;
			tm_info.tm_year = car[0].year - 1900;
			tm_info.tm_mon = car[0].month - 1;
			tm_info.tm_mday = car[0].day;
			tm_info.tm_hour = car[0].hours;
			tm_info.tm_min = car[0].minter;
			tm_info.tm_sec = car[0].second;
			tm_info.tm_isdst = -1; //夏令时
			
			time_0 = mktime(&tm_info);
			if(time_0 == -1) 
			{
				HAL_UART_Transmit_DMA(&huart1,(uint8_t *)"Error5\r\n",8);
				goto Error; 
			}
			
			//查看这辆车是否存在、合法
			num = 0;
			for (int i = 1; i < 9; i++)
			{
					flag0 = 0;
					for (int j = 0; j < 4; j++)
					{
						if(car[0].car_num[j] != car[i].car_num[j]) flag0 = 1;
					}
					if(flag0 == 0) 
					{
						num = i;
						break;
					}
			}
			if(num != 0) //存在该车
			{
				if(car[0].car_type[0] != car[num].car_type[0]) 
				{
					HAL_UART_Transmit_DMA(&huart1,(uint8_t *)"Error6\r\n",8);
					goto Error; 
				}
				
				car[num].exit = 0; //设为不存在
				//信息合法
				tm_info.tm_year = car[num].year - 1900;
				tm_info.tm_mon = car[num].month - 1;
				tm_info.tm_mday = car[num].day;
				tm_info.tm_hour = car[num].hours;
				tm_info.tm_min = car[num].minter;
				tm_info.tm_sec = car[num].second;
				tm_info.tm_isdst = -1; //夏令时
				
				time_1 = mktime(&tm_info);
				TIME = time_0 - time_1;
				if(TIME < 0) 
				{
					HAL_UART_Transmit_DMA(&huart1,(uint8_t *)"Error7\r\n",8);
					goto Error; 
				}
				
				TIME = TIME / 3600;
				if((time_0 - time_1) % 3600 > 0) TIME++;
				if(car[num].car_type[0] == 'C') 
				{
					MONEY = TIME * money_c;
					if(num_C == 0) 
					{
						HAL_UART_Transmit_DMA(&huart1,(uint8_t *)"Error8\r\n",8);
						goto Error;
					}
					num_C--;
				}
				else 
				{
					MONEY = TIME * money_v;
					if(num_V == 0) 
					{
						HAL_UART_Transmit_DMA(&huart1,(uint8_t *)"Error9\r\n",8);
						goto Error;
					}
					num_V--;
				}
				sprintf(text1,"%s:%s:%ld:%.2f\r\n",car[num].car_type,car[num].car_num,TIME,MONEY);
				HAL_UART_Transmit_DMA(&huart1,(unsigned char *)text1,strlen(text1));
				for(int i=0;i<4;i++) car[num].car_num[i] = 0;
				goto Yes;
			}
			else //不存在该车
			{
				if(num_C+num_V >= 8) 
				{
					HAL_UART_Transmit_DMA(&huart1,(uint8_t *)"Error10\r\n",8);
					goto Error; //车库已满
				}
				
				if(car[num].car_type[0] == 'C') num_C++;
				else num_V++;
				
				for (num1 = 1; num1 < 9; num1++)
				{
					if(car[num1].exit == 0) 
					{
						car[num1].exit = 1;
						break;
					}
				}
				sscanf((char *)rxdata, "%4s:%4s:%2d%2d%2d%2d%2d%2d", 
				car[num1].car_type, car[num1].car_num, &car[num1].year, &car[num1].month, &car[num1].day,
				&car[num1].hours, &car[num1].minter, &car[num1].second);
				goto Yes;
			}
			Error:
//			HAL_UART_Transmit_DMA(&huart1,(unsigned char *)"Error\r\n",strlen("Error\r\n"));
			Yes:
			;
		}
		HAL_UART_Receive_DMA(&huart1,rxdata,100);
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	}
  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
