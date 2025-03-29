#include "mint.h"
#include "usart.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "time.h"

struct keys key[4] = {0,0,0};

char rxdata[30];
uint8_t rxdat;
uint8_t rxp=0;

char text1[30];
struct cars car[9];
int d_time=0;											//��¼����ʱ�䣬20ms��δ������ɣ���Ϊ�����ַ���������
long TIME,time_0,time_1;					//TIME����¼ͣ��ʱ�䣬time_0����¼ͣ������ʱ�䣬time_1����¼ͣ����ʼʱ��
uint8_t flag0,num,num1;						//flag0�������жϳ����Ƿ�Ϸ���num�����ڼ�¼�Ϸ�����λ�ã�num1����¼�ҵ��ĵ�һ�����г�λ
double MONEY;											//MONEY����¼ͣ������

extern double money_c,money_v;		//
extern uint8_t num_C,num_V;				//




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		//���ڽ������ݳ���������
		d_time++;
		if(d_time == 2)
		{
			d_time = 0;
			if(rxp != 0 && rxp != 22)
			{
				HAL_UART_Transmit(&huart1,(unsigned char *)"Error\r\n",strlen("Error\r\n"),50);
				for(int i=0;i<21;i++) rxdata[i] = 0;
				rxp = 0;
			}
		}
		usart1_Proc();//���ڽ������ݴ���
		/*************************�������*************************/
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
/*****************
*�����жϺ���
*
****************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	d_time=0;
	rxdata[rxp++] = rxdat;
	HAL_UART_Receive_IT(&huart1,&rxdat,1);
}

/*****************
*�������ݴ�����
*
****************/
void usart1_Proc(void)
{
	if(rxp == 22)
	{
		rxp = 0;
		sscanf(rxdata, "%4s:%4s:%2d%2d%2d%2d%2d%2d", 
		car[0].car_type, car[0].car_num, &car[0].year, &car[0].month, &car[0].day, &car[0].hours, &car[0].minter, &car[0].second);
		//�������ж�
		if(car[0].car_type[0] != 'C' && car[0].car_type[0] != 'V') goto Error;
		if(car[0].car_type[1] != 'N' || car[0].car_type[2] != 'B' || car[0].car_type[3] != 'R') goto Error; 
		//ʱ���ʽ�ж�
		if(car[0].month > 12 || car[0].month < 1 || car[0].day > 31 || car[0].day < 1 || car[0].hours > 23 || 
			 car[0].hours < 0 || car[0].minter > 59 || car[0].minter < 0 || car[0].second > 59 || car[0].second < 0) 
		goto Error; 
		
		struct tm tm_info;
		tm_info.tm_year = car[0].year - 1900;
		tm_info.tm_mon = car[0].month - 1;
		tm_info.tm_mday = car[0].day;
		tm_info.tm_hour = car[0].hours;
		tm_info.tm_min = car[0].minter;
		tm_info.tm_sec = car[0].second;
		tm_info.tm_isdst = -1; //����ʱ
		
		time_0 = mktime(&tm_info);
		if(time_0 == -1) goto Error; 
		
		//�鿴�������Ƿ���ڡ��Ϸ�
		num = 0;
    for (int i = 1; i < 9; i++)
    {
				flag0 = 0;
        for (int j = 0; j < 4; j++)
        {
					if(car[0].car_num[j] != car[i].car_num[j]) flag0 = 1;
        }
				if(flag0 == 0) 
				{
					num = i;
					break;
				}
    }
		if(num != 0) //���ڸó�
		{
			if(car[0].car_type[0] != car[num].car_type[0]) goto Error; 
			
			car[num].exit = 0; //��Ϊ������
			//��Ϣ�Ϸ�
			tm_info.tm_year = car[num].year - 1900;
			tm_info.tm_mon = car[num].month - 1;
			tm_info.tm_mday = car[num].day;
			tm_info.tm_hour = car[num].hours;
			tm_info.tm_min = car[num].minter;
			tm_info.tm_sec = car[num].second;
			tm_info.tm_isdst = -1; //����ʱ
			
			time_1 = mktime(&tm_info);
			TIME = time_0 - time_1;
			if(TIME < 0) goto Error; 
			
			TIME = TIME / 3600;
			if((time_0 - time_1) % 3600 > 0) TIME++;
			if(car[num].car_type[0] == 'C') 
			{
				MONEY = TIME * money_c;
				if(num_C == 0) goto Error;
				num_C--;
			}
			else 
			{
				MONEY = TIME * money_v;
				if(num_V == 0) goto Error;
				num_V--;
			}
			sprintf(text1,"%s:%s:%ld:%.2f\r\n",car[num].car_type,car[num].car_num,TIME,MONEY);
			HAL_UART_Transmit(&huart1,(unsigned char *)text1,strlen(text1),50);
			for(int i=0;i<4;i++) car[num].car_num[i] = 0;
			goto Yes;
		}
		else //�����ڸó�
		{
			if(num_C+num_V >= 8) goto Error; //��������
			
			if(car[num].car_type[0] == 'C') num_C++;
			else num_V++;
			
			for (num1 = 1; num1 < 9; num1++)
			{
				if(car[num1].exit == 0) 
				{
					car[num1].exit = 1;
					break;
				}
			}
			sscanf(rxdata, "%4s:%4s:%2d%2d%2d%2d%2d%2d", 
			car[num1].car_type, car[num1].car_num, &car[num1].year, &car[num1].month, &car[num1].day,
			&car[num1].hours, &car[num1].minter, &car[num1].second);
			goto Yes;
		}
		Error:
		HAL_UART_Transmit(&huart1,(unsigned char *)"Error\r\n",strlen("Error\r\n"),50);
		Yes:
		;
	}
}















