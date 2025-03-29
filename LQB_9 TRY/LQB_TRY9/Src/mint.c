#include "mint.h"

struct keys key[4] = { 0 };
extern struct utimes utime[5],stime;
extern uint8_t page,time_sta;
int dtime=0;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		//定时部分，定时标志开启（time_sta==1），开始计数,定时结束，恢复原来状态
		if(time_sta == 1)
		{
			dtime++;
			if(utime[page].hour == 0 && utime[page].min == 0 && utime[page].sec == 0 && dtime == 100) //定时结束
			{
				//恢复定时值
				utime[page].hour = stime.hour;
				utime[page].min = stime.min;
				utime[page].sec = stime.sec;
				time_sta = 0;//置标志为0（定时未运行）
				dtime = 0;
			}
			else if(dtime == 100)
			{
				if(utime[page].sec == 0 && utime[page].min == 0)
				{
					utime[page].sec = 59;
					utime[page].min = 59;
					utime[page].hour--;
				}
				else if(utime[page].sec == 0 && utime[page].min != 0)
				{
					utime[page].sec = 59;
					utime[page].min--;
				}
				else
				{
					utime[page].sec--;
				}
				dtime = 0;
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
					key[i].time_sta ++;
					if(key[i].time_sta < 78 && key[i].key_sta == 1) 
					{
						key[i].short_sta = 1;
						key[i].judge_sta = 0;
						key[i].time_sta = 0;
					}
					else if(key[i].time_sta >= 78) key[i].judge_sta = 3;
				}
				break;
				case 3:
				{
					if(key[i].key_sta == 0)
					{
						key[i].time_sta++;
						if(key[i].time_sta >= 15) 
						{
							key[i].long_sta = 1;
							key[i].time_sta = 0;
						}
					}
					else
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
