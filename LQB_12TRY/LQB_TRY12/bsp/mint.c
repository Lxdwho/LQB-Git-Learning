#include "mint.h"
#include "usart.h"
#include "main.h"
#include "stdio.h"
#include "string.h"

struct keys key[4] = {0,0,0};
char rxdata[30];
uint8_t rxdat;
uint8_t rxp=0;
extern char rxdata[30];
extern uint8_t rxp;
extern uint8_t rxdat;
char text1[30];
struct cars car[9];
extern double money_c,money_v;
extern uint8_t num_C,num_V;
int k=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		k++;
		if(k == 10)
		{
			k = 0;
			if(rxp != 0 && rxp != 22)
			{
				HAL_UART_Transmit(&huart1,(unsigned char *)"Error\r\n",strlen("Error\r\n"),50);
				for(int i=0;i<21;i++) rxdata[i] = 0;
				rxp = 0;
			}
		}
		goto Error;
		Error:
		usart1_Proc();
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
					if(key[i].key_sta == 1) 
					{
						key[i].judge_sta = 0;
						key[i].single_sta = 1;
					}
				}
				break;
			}
		}
		
		
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	k=0;
	rxdata[rxp++] = rxdat;
	HAL_UART_Receive_IT(&huart1,&rxdat,1);
}

void usart1_Proc(void)
{
	int JUDGE = 0;//合法标志位
	if(rxp == 22)
	{
		int flag=0,judge=0,T=0;
		double money=0;
		char TYPE1[5] = "VNBR",TYPE2[5] = "CNBR";
    sscanf(rxdata, "%4s:%4s:%2d%2d%2d%2d%2d%2d", car[0].car_type, car[0].car_num, &car[0].year, &car[0].month, &car[0].day, &car[0].hours, &car[0].minter, &car[0].second);
		//查看这辆车是否存在、合法
    for (int i = 1; i < 9; i++)
    {
        flag = 0;
        for (int j = 0; j < 4; j++)
        {
          if(car[0].car_num[j] != car[i].car_num[j] || car[0].car_type[j] != car[i].car_type[j]) flag = 1;//不存在这辆车，但合法
					if(car[0].car_type[j] != TYPE1[j] && car[0].car_type[j] != TYPE2[j]) JUDGE = 1;;//不合法类型
        }
        if (flag == 0) judge = i;
    }
    if (judge != 0 && JUDGE == 0) //存在该车且合法
    {
			car[judge].exit = 0;
			T = (car[0].year - car[judge].year) * 365 * 24 + (car[0].month - car[judge].month) * 30 * 24 + (car[0].day - car[judge].day) * 1 * 24 +
					(car[0].hours - car[judge].hours);
			if(T<0) JUDGE = 1;
			int i = (car[0].minter - car[judge].minter) * 60 + (car[0].second - car[judge].second);
			if (i > 0) T++;
			if(car[0].car_type[0] == 'C') 
			{
				money = (double)T * money_c;
				if((num_C-1)>=0) num_C--;
				else JUDGE = 1;
			}
			else if(car[0].car_type[0] == 'V') 
			{
				money = (double)T * money_v;
				if((num_V-1)>=0) num_V--;
				else JUDGE = 1;
			}
			if(JUDGE  == 0)
			{
				sprintf(text1,"%s:%s:%d:%.2f\r\n",car[0].car_type,car[0].car_num,T,money);
				HAL_UART_Transmit(&huart1,(unsigned char *)text1,strlen(text1),50);
				for(int m=0;m<4;m++) car[judge].car_num[m] = 0;//清除编号
			}
			else HAL_UART_Transmit(&huart1,(unsigned char *)"Error\r\n",strlen("Error\r\n"),50);
    }
		else if(flag == 1 && num_C+num_V<8 && JUDGE == 0)//不存在该车,且车库没满，且合法
		{
			int kong=0;
			for (int i = 1; i < 9; i++)
			{
				if(car[i].exit == 0) 
				{
					car[i].exit = 1;
					kong = i;
					break;
				}
			}
			sscanf(rxdata, "%04s:%04s:%2d%2d%2d%2d%2d%2d", car[kong].car_type, car[kong].car_num, &car[kong].year, &car[kong].month, &car[kong].day, &car[kong].hours, &car[kong].minter, &car[kong].second);
			if(car[0].car_type[0] == 'C') num_C++;
			else if(car[0].car_type[0] == 'V') num_V++;
		}
		else HAL_UART_Transmit(&huart1,(unsigned char *)"Error\r\n",strlen("Error\r\n"),50);
		rxp = 0;
		for(int i=0;i<21;i++) rxdata[i] = 0;
	}
	else;
}















