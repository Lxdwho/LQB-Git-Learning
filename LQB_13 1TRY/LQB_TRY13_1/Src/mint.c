#include "mint.h"
#include "main.h"
#include "usart.h"

struct keys key[4] = {0};
extern int s_time,e_time;
extern bool page;
extern uint8_t error_times,error_timesy;
unsigned char rxdat;
char rxdata[8];
int rxp;

uint8_t k=0;

int frq_r;
double carr1,carr2,duty_r;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		//计时部分
		if(page == 1) s_time++;
		if(error_times >= 3 && error_timesy < error_times) 
		{
			e_time++;
			if(e_time % 10 == 0 && e_time % 20 != 0) LED_Proc(0x02);
			else if(e_time % 20 == 0) LED_Proc(0x00);
			if(e_time >= 500) 
			{
				e_time = 0;
				LED_Proc(0x00);
				error_timesy = error_times;
			}
		}
		
		//串口部分
		k++;
		if(k == 10) 
		{
			if(rxp != 7 && rxp !=0)
			{
				for(int i=0;i<rxp;i++) rxdata[i] = 0;
				rxp = 0;
				HAL_UART_Transmit(&huart1,(unsigned char *)"Error",6,50);
			}
			k = 0;
		}
		
		//按键部分
		key[0].key_sta = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
		key[1].key_sta = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
		key[2].key_sta = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
		key[3].key_sta = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
		
		for(int i=0;i<4;i++)
		{
			switch(key[i].judge)
			{
				case 0:
				{
					if(key[i].key_sta == 0) key[i].judge = 1;
				}
				break;
				case 1:
				{
					if(key[i].key_sta == 0) key[i].judge = 2;
					else key[i].judge = 0;
				}
				break;
				case 2:
				{
					if(key[i].key_sta == 1)
					{
						key[i].single_sta = 1;
						key[i].judge = 0;
					}
				}
				break;
			}
		}
	}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			carr1 = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
			carr2 = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
			__HAL_TIM_SetCounter(htim,0);
			frq_r = (80000000/80)/carr1;
			duty_r = (carr2/carr1)*100;
			HAL_TIM_IC_Start(htim,TIM_CHANNEL_1);
			HAL_TIM_IC_Start(htim,TIM_CHANNEL_2);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	rxdata[rxp++] = rxdat;
	HAL_UART_Receive_IT(&huart1,&rxdat,1);
}



