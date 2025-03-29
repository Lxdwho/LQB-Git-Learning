#include "key.h"

struct keys key[4] = {0,0,0};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim -> Instance == TIM3)
	{
		key[0].key_sta  = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
		key[1].key_sta  = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
		key[2].key_sta  = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
		key[3].key_sta  = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
		
		for(int i=0;i<4;i++)
		{
			if(key[i].key_sta == 0)
			{
				switch(key[i].judge_sta)
				{
					case 0 :
					{
						key[i].judge_sta++;
					}
					break;
					case 1 :
					{
						key[i].judge_sta = 0;
						key[i].single_sta = 1;
					}
					break;
				}
			}
			else key[i].judge_sta = 0;
		}
	}
}
