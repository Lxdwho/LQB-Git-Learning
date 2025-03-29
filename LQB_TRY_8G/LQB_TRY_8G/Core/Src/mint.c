#include "mint.h"
#include "tim.h"

keys key[4] = { 0 };
extern unsigned char puls1,puls2,f_math,p_math;
extern double adc[2];
int t=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	t++;
	if(t == 10)
	{
		ADCvalue();
		if(puls1 != 0) htim3.Init.Prescaler = (800/puls1/f_math)-1;
		HAL_TIM_Base_Init(&htim3);
		if(puls2 != 0) htim17.Init.Prescaler = (800/puls2*p_math)-1;
		HAL_TIM_Base_Init(&htim17);
		t=0;
	}

	if(htim->Instance == TIM6)
	{
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

double val1,val2;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM15)
	{
		val1 = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
		__HAL_TIM_SetCounter(htim,0);
		puls1 = (int)(1000/val1);
		HAL_TIM_IC_Start(htim,TIM_CHANNEL_1);
	}
	else if(htim->Instance == TIM2)
	{
		val2 = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
		__HAL_TIM_SetCounter(htim,0);
		puls2 = (int)(1000/val2);
		HAL_TIM_IC_Start(htim,TIM_CHANNEL_2);
	}
	else;
}	










