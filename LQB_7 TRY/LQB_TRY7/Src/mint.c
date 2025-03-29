#include "mint.h"

keys key[4] = {0};
extern bool flag1,flag2,flag3;
extern double adc_val;
extern uint8_t Height;
int dtime=0,dtime1=0,dtime2=0,dtime3=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		//ADC值的读取
		if(dtime<10) dtime++;
		else 
		{
			dtime = 0;
			adc_val = GetADCValue();
			Height = (int)(adc_val*100/3.3);
		}
		//LED1闪烁控制部分
		if(flag1 == 1)
		{
			if(dtime1 <= 200) 
			{
				dtime1++;
				if(dtime1%100 == 0 && dtime1%200 != 0) LED_Proc(1,1);
				else if(dtime1%200 == 0) LED_Proc(1,0);
			}
			else dtime1 = 1;
		}
		//LED2闪烁控制部分
		if(flag2 == 1)
		{
			if(dtime2 < 200) 
			{
				dtime2++;
				if(dtime2 % 20 == 0 && dtime2 % 40 != 0) LED_Proc(2,1);
				else if(dtime2 % 40 == 0) LED_Proc(2,0);
			}
			else
			{
				LED_Proc(2,0);
				flag2 = 0;
				dtime2 = 0;
			}
		}
		//LED3闪烁控制部分
		if(flag3 == 1)
		{
			if(dtime3 < 200) 
			{
				dtime3++;
				if(dtime3 % 20 == 0 && dtime3 % 40 != 0) LED_Proc(3,1);
				else if(dtime3 % 40 == 0) LED_Proc(3,0);
			}
			else
			{
				LED_Proc(3,0);
				flag3 = 0;
				dtime3 = 0;
			}
		}
		//按键判断部分
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


