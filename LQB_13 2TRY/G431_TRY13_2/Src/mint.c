#include "mint.h"
#include "string.h"
#include "stdio.h"
#include "tim.h"


unsigned char rxdat;
extern char text_uart[20];
extern double Money_X,Money_Y;

struct keys key[4] = { 0 };

extern int time_led1,time_led2;
extern uint8_t Num_X,Num_Y,flag,flag2;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{	
	if(htim->Instance == TIM6)
	{
		//货物为空，闪烁
		if(Num_X == 0 && Num_Y == 0) 
		{
			time_led2++;
			if(time_led2 == 21) time_led2 = 1;
			
			if(time_led2 % 20 == 0) flag2 = 1;
			else if(time_led2 % 10 == 0) flag2 = 0;
		}
		else 
		{
			flag2 = 0;
			time_led2 = 1;
		}
		//客户购买，持续5s
		if(flag == 1)
		{
			time_led1++;
			if(time_led1 == 500)
			{
				time_led1 = 0;
				flag = 0;
			}
		}
		//按键部分
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


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(rxdat == '?') 
	{
		sprintf(text_uart,"X:%.2f,Y:%.2f\r\n",Money_X,Money_Y);
		HAL_UART_Transmit(huart,(unsigned char *)text_uart,strlen(text_uart),50);
	}
	else HAL_UART_Transmit(huart,(unsigned char *)"Error\r\n",strlen("Error"),50);
	HAL_UART_Receive_IT(huart,&rxdat,1);
}




