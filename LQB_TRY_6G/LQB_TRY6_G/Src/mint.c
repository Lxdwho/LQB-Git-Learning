#include "mint.h"

keys key[4] = { 0 };

double val1=0,val2=0,DUTY=0,FRQ=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
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


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM15)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			val1 = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
			val2 = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
			__HAL_TIM_SetCounter(htim,0);
			
			FRQ = 100000/val1;
			DUTY = val2/val1*100;
			HAL_TIM_IC_Start(htim,TIM_CHANNEL_1);
			HAL_TIM_IC_Start(htim,TIM_CHANNEL_2);
		}
	}
}






















