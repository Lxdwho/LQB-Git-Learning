#include "mint.h"
#include "usart.h"

keys key[4] = { 0 };

extern uint8_t rxdat,time_flag;
extern bool page,mode;
uint8_t time=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		//0.1msÂö³åtime_flag
		time++;
		if(time >= 10) time_flag = (time_flag>1)?0:(1-time_flag);
		//°´¼ü²¿·Ö
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
					if(key[i].key_sta == 0) key[i].judge_sta =1;
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(mode == 1)
	{
		if(rxdat == '@') page = 0;
		else if(rxdat == '#') page = 1;
		else HAL_UART_Transmit(&huart1,(unsigned char *)"ERROR\r\n",7,50);
	}
	else HAL_UART_Transmit(&huart1,(unsigned char *)"KEY CONTROL\r\n",13,50);
	HAL_UART_Receive_IT(&huart1,&rxdat,1);
}

void LED_Proc(uint8_t led,uint8_t state)
{
	static unsigned char LED_State = 0xff;
	if(led>8 || led<1) return;
	
	if(state == 0) LED_State |= (GPIO_PIN_0 << (led-1));
	else if(state == 1) LED_State &= ~(GPIO_PIN_0 << (led-1));
	
	GPIOC->ODR = LED_State << 8;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}














