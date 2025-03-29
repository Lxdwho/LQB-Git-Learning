#include "m_int.h"

struct keys key[4] = {0,0,0};
extern double madc,VMAX,VMIN;
extern uint8_t time;
uint8_t flag = 0,time_flag = 0;
double last_adc = 3.3;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		key[0].key_sta = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
		key[1].key_sta = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
		key[2].key_sta = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
		key[3].key_sta = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
		
		for(int i=0;i<4;i++)
		{
			switch(key[i].judge_flag)
			{
				case 0:
				{
					if(key[i].key_sta == 0) key[i].judge_flag = 1;
				}
				break;
				case 1:
				{
					if(key[i].key_sta == 0) key[i].judge_flag = 2;
					else key[i].judge_flag = 0;
				}
				break;
				case 2:
				{
					if(key[i].key_sta == 1) 
					{
						key[i].single_sta = 1;
						key[i].judge_flag = 0;
					}
				}
				break;
			}
		}
		if(last_adc < madc && madc >= VMIN && last_adc < VMIN) 
		{
			flag = 1;
			time = 0;
		}
		if(last_adc < madc &&madc >= VMAX && last_adc < VMAX)
		{
			flag = 0;
		}
		if(flag == 1)
		{
			time_flag++;
			if(time_flag >= 200) 
			{
				time_flag = 0;
				time++;
			}
		}
		last_adc = madc;
	}
}
