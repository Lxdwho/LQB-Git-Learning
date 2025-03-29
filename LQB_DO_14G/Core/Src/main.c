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
#include "i2c_hal.h"
#include "ds18b20_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	bool key_sta,single_sta,long_sta;
	unsigned char judge_sta,time_sta;
}keys;
typedef struct
{
	double frq[110],v[110];
	unsigned char duty[110],meoflag,meotimes;
	bool test;
}FSET;
typedef struct
{
	unsigned char time1,time2;
	bool flag1,flag2;
}mflags;
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
unsigned char page=0,TH[2]={30,30},FN=0,AN=0,TN=0,FP=1,TT=6,line=0,line3=0,TTIME=0;
bool led1_sta=0;
char text[30];
double F[2]={0},D=0,A[2]={0},T[3]={0},AH[2]={3.0,3.0},VP=0.9,ftim2[3]={0};
int FH[2]={2000,2000};
FSET meo={0};
mflags mflagf={0},mflagv={0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LED_Proc(unsigned char led,unsigned char sta);
void LCD_Proc(void);
void KEY_Proc(void);
void ADCVlaue(void);
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
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	//LCD_Init
	LCD_Init();
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	//TIM6_INIT
	HAL_TIM_Base_Start_IT(&htim6);
	//TIM2_INIT
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
	//ADC_INIT
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
	//LED_INIT
	LED_Proc(1,0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		LCD_Proc();
		if(meo.meoflag == 0) KEY_Proc();
		else for(int k=0;k<4;k++) key[k].single_sta = 0,key[k].long_sta = 0;
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
/********************************************************************
*TIM6:0.01S
*TIM6
********************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		TTIME++;
		if(TTIME >= 10)
		{
			ADCVlaue();
			T[0] = TEMP_READ();
			if(T[0]<90 && T[0]>0) T[1] = T[0];  
			
			if(F[1] > FH[1] && F[0] <= FH[1]) LED_Proc(4,1),FN++;
			else if(F[1] <= FH[1] && F[0] > FH[1])LED_Proc(4,0);
			if(A[1] > AH[1] && A[0] <= AH[1]) LED_Proc(5,1),AN++;
			else if(A[1] <= AH[1] && A[0] > AH[1])LED_Proc(5,0);
			if(T[1] > TH[1] && T[2] <= TH[1]) LED_Proc(6,1),TN++;
			else if(T[1] <= TH[1] && T[2] > TH[1])LED_Proc(6,0);
			F[0] = F[1];
			A[0] = A[1];
			T[2] = T[1];
			TTIME = 0;
			if(meo.meoflag == 1)
			{
				led1_sta = 1 - led1_sta;
				LED_Proc(1,led1_sta);
				meo.duty[meo.meotimes] = D;
				meo.frq[meo.meotimes] = F[1];
				meo.v[meo.meotimes] = A[1];
				meo.meotimes++;
				mflagf.time1++;
				mflagv.time1++;
			}
			if((TT * 10) <= (meo.meotimes)) 
			{
				LED_Proc(1,0);
				meo.meotimes = 0;
				meo.meoflag = 0;
				meo.test = 1;
			}
			
			if(mflagf.flag1 == 1 && mflagv.flag1 != 1)
			{
				mflagf.flag2 = 1 - mflagf.flag2;
				LED_Proc(2,mflagf.flag2);
				__HAL_TIM_SetCompare(&htim17,TIM_CHANNEL_1,meo.duty[mflagf.time2-mflagf.time1]);
				htim17.Init.Prescaler = (int)(800000/meo.frq[mflagf.time2-mflagf.time1]*FP)-1;
				HAL_TIM_Base_Init(&htim17);
				HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);
				if(mflagf.time1 == 0) 
				{
					mflagf.flag1 = 0;
					mflagf.time1 = mflagf.time2+1;
					LED_Proc(2,0);
					HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);
				}
				if(mflagf.time1 > 0) mflagf.time1--;
			}
			else if(mflagf.flag1 == 1 && mflagv.flag1 == 1) 
			{
				if(mflagv.time1 > mflagf.time1) mflagv.flag1=0;
				else mflagf.flag1=0;
			}
			
			if(mflagv.flag1 == 1 && mflagf.flag1 != 1)
			{
				mflagv.flag2 = 1 - mflagv.flag2;
				LED_Proc(3,mflagv.flag2);
				unsigned char d = (int)((90/(3.3-VP)*(meo.v[mflagf.time2-mflagf.time1]-VP))+10);
				if(d < 10) d = 10;
				if(d > 100) d = 100;
				__HAL_TIM_SetCompare(&htim17,TIM_CHANNEL_1,d);
				if(mflagv.time1 == 0) 
				{
					mflagv.flag1 = 0;
					mflagv.time1 = mflagv.time2+1;
					LED_Proc(3,0);
					HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);
				}
				if(mflagv.time1 > 0) mflagv.time1--;
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
					if(key[i].key_sta == 0 && key[i].time_sta >= 200)
					{
						key[i].judge_sta = 3;
						key[i].time_sta = 0;
						key[i].long_sta = 1;
					}
					else if(key[i].key_sta == 1 && key[i].time_sta < 200)
					{
						key[i].judge_sta = 0;
						key[i].single_sta = 1;
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
/********************************************************************
*按键操作函数
*KEY_Proc
********************************************************************/
void KEY_Proc(void)
{
	if(key[0].single_sta == 1)
	{
		page = page >= 3?0:page+1;
		if(page == 1) line = 0;
		if(page == 2)
		{
			
			if(FH[0] < F[1] && FH[1] != FH[0]) LED_Proc(4,1),FN++;
			else if(FH[0] > F[1])LED_Proc(4,0);
			FH[1] = FH[0];
			
			if(AH[0] < A[1] && AH[1] != AH[0]) LED_Proc(5,1),AN++;
			else if(AH[0] > A[1])LED_Proc(5,0);
			AH[1] = AH[0];
			
			if(TH[0] < T[1] && TH[1] != TH[0]) LED_Proc(6,1),TN++;
			else if(TH[0] > T[1])LED_Proc(6,0);
			TH[1] = TH[0];
		}
		if(page == 3) line3 = 0;
		key[0].single_sta = 0;
	}
	if(key[1].single_sta == 1)
	{
		if(page == 0)
		{
			for(int o=0;o<110;o++)
			{
				meo.duty[o] = 0;
				meo.frq[o] = 0;
				meo.v[o] = 0;
			}
			mflagf.time1=0;
			mflagv.time1=0;
			meo.meoflag = 1;
		}
		else if(page == 1)
		{
			line = line == 2?0:line+1;
		}
		else if(page == 3)
		{
			line3 = line3 == 2?0:line3+1;
		}
		else if(page == 2)
		{
			FN = 0;
			AN = 0;
			TN = 0;
		}
		key[1].single_sta = 0;
	}
	if(key[2].single_sta == 1)
	{
		if(page == 1)
		{
			if(line == 0) FH[0] = FH[0] >= 10000?10000:FH[0]+1000;
			else if(line == 1) AH[0] = AH[0] >=3.2?3.3:AH[0]+0.3;
			else if(line == 2) TH[0] = TH[0] >=80?80:TH[0]+1;
		}
		else if(page == 3)
		{
			if(line3 == 0) FP = FP >= 10?10:FP+1;
			else if(line3 == 1) VP = VP >=3.2?3.3:VP+0.3;
			else if(line3 == 2) TT = TT >=10?10:TT+2;
		}
		else if(page == 0)
		{
			//VVVVVVV
			if(meo.test == 1)
			{
				mflagv.flag1 = 1;
				mflagv.time2 = mflagv.time1;
				htim17.Init.Prescaler = 799;
				HAL_TIM_Base_Init(&htim17);
				HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);
			}
		}
		key[2].single_sta = 0;
	}
	if(key[3].single_sta == 1)
	{
		if(page == 1)
		{
			if(line == 0) FH[0] = FH[0] <= 1000?1000:FH[0]-1000;
			else if(line == 1) AH[0] = AH[0] <=0.1?0:AH[0]-0.3;
			else if(line == 2) TH[0] = TH[0] <=0?0:TH[0]-1;
		}
		else if(page == 3)
		{
			if(line3 == 0) FP = FP <= 1?1:FP-1;
			else if(line3 == 1) VP = VP <=0.1?0:VP-0.3;
			else if(line3 == 2) TT = TT <=2?2:TT-2;
		}
		else if(page == 0)
		{
			//FFFFFFF
			if(meo.test == 1)
			{
				mflagf.flag1 = 1;
				mflagf.time2 = mflagf.time1;
			}
		}
		key[3].single_sta = 0;
	}
	if(key[2].long_sta == 1)
	{
		page=0,TH[0]=30,TH[1] = 30,FN=0,AN=0,TN=0,FP=1,TT=6,line=0,line3=0,TTIME=0,AH[0]=3.0,AH[1]=3.0,VP=0.9,FH[0]=2000,FH[1]=2000,meo.test = 0,mflagf.time1=0,mflagv.time1=0;
		key[2].long_sta = 0;
	}
	if(key[3].long_sta == 1)
	{
		page=0,TH[0]=30,TH[1] = 30,FN=0,AN=0,TN=0,FP=1,TT=6,line=0,line3=0,TTIME=0,AH[0]=3.0,AH[1]=3.0,VP=0.9,FH[0]=2000,FH[1]=2000,meo.test = 0,mflagf.time1=0,mflagv.time1=0;
		key[3].long_sta = 0;
	}
}
/********************************************************************
*LCD显示函数
*LCD_Proc
********************************************************************/
void LCD_Proc(void)
{
	if(page == 0)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        DATA        ");
		sprintf(text,"     F=%.0f       ",F[1]);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"     D=%.0f%%       ",D);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		sprintf(text,"     A=%.1f       ",A[1]);
		LCD_DisplayStringLine(Line5,(u8 *)text);
		sprintf(text,"     T=%.1f       ",T[1]);
		LCD_DisplayStringLine(Line6,(u8 *)text);
	}
	else if(page == 1)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        PARA        ");
		sprintf(text,"     FH=%d       ",FH[0]);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"     AH=%.1f       ",AH[0]);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		sprintf(text,"     TH=%d       ",TH[0]);
		LCD_DisplayStringLine(Line5,(u8 *)text);
		LCD_DisplayStringLine(Line6,(u8 *)"                    ");

	}
	else if(page == 2)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        RECD        ");
		sprintf(text,"     FN=%d       ",FN);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"     AN=%d       ",AN);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		sprintf(text,"     TN=%d       ",TN);
		LCD_DisplayStringLine(Line5,(u8 *)text);
		LCD_DisplayStringLine(Line6,(u8 *)"                    ");
	}
	else if(page == 3)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        FSET        ");
		sprintf(text,"     FP=%d       ",FP);
		LCD_DisplayStringLine(Line3,(u8 *)text);
		sprintf(text,"     VP=%.1f       ",VP);
		LCD_DisplayStringLine(Line4,(u8 *)text);
		sprintf(text,"     TT=%d       ",TT);
		LCD_DisplayStringLine(Line5,(u8 *)text);
		LCD_DisplayStringLine(Line6,(u8 *)"                    ");
	}
	else page = 0;
}
/********************************************************************
*LED控制函数
*LED_Proc
********************************************************************/
void LED_Proc(unsigned char led,unsigned char sta)
{
	static unsigned char LED_STA = 0xff;
	if(sta == 0) LED_STA |= (0x01 << (led - 1));
	else if(sta == 1) LED_STA &= ~(0x01 << (led - 1));
	
	GPIOC->ODR = LED_STA << 8;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}
/********************************************************************
*TIM2_Proc
********************************************************************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			ftim2[0] = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
			ftim2[1] = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
			__HAL_TIM_SetCounter(htim,0);
			D = ftim2[1] / ftim2[0] * 100;
			if(D>100 || D<0) D = 50;
			if(ftim2[0] != 0) F[1] = 1000000/ftim2[0];
			HAL_TIM_IC_Start(htim,TIM_CHANNEL_1);
			HAL_TIM_IC_Start(htim,TIM_CHANNEL_2);
		}
	}
}
/********************************************************************
*ADCValue
********************************************************************/
void ADCVlaue(void)
{
	HAL_ADC_Start(&hadc2);
	A[1] = HAL_ADC_GetValue(&hadc2);
	A[1] = A[1] / 4095 * 3.3;
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
