#include "mint.h"
#include "string.h"

keys key[4] = { 0 };
char text1[20];

extern char data[60][20];
unsigned char p=0;

unsigned char t=0,sta=0;
double val=0,frq=0;
extern double T,H,TS,T_U,H_U;

extern bool page;

extern RTC_AlarmTypeDef sAlarm;
extern RTC_TimeTypeDef GetTime;
extern RTC_DateTypeDef GetDate;

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
	if(htim->Instance == TIM17)
	{
		val = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
		__HAL_TIM_SetCounter(htim,0);
		frq = 1000000/val;
		HAL_TIM_IC_Start(htim,TIM_CHANNEL_1);
//		HAL_TIM_IC_Start_IT(htim,TIM_CHANNEL_1);
	}
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	HAL_RTC_GetTime(hrtc,&GetTime,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc,&GetDate,RTC_FORMAT_BIN);
	//Show Time
	sprintf(text1,"    Time:%02d-%02d-%02d      ",GetTime.Hours,GetTime.Minutes,GetTime.Seconds);
	if(page == 0) LCD_DisplayStringLine(Line5,(u8 *)text1);
	t++;
	if(t == (int)TS)
	{
		T = GetADC()*80/3.3-20;
		H = frq/1000/9*80+10/9;
		LED_Proc(3,sta);
		sta = 1-sta;
		t = 0;
		if(p<59) sprintf(data[p++],"%02d-%02d-%02d,T:%02d,H:%02d\r\n",GetTime.Hours,GetTime.Minutes,GetTime.Seconds,(int)T,(int)H);
		else
		{
			for(int j=0;j<59;j++)
			strcpy(data[j],data[j+1]);
			sprintf(data[59],"%02d-%02d-%02d,T:%02d,H:%02d\r\n",GetTime.Hours,GetTime.Minutes,GetTime.Seconds,(int)T,(int)H);
		}
	}
	
	sAlarm.AlarmTime.Seconds = GetTime.Seconds + 1;
	if(sAlarm.AlarmTime.Seconds == 60) sAlarm.AlarmTime.Seconds = 0;
	HAL_RTC_SetAlarm_IT(hrtc,&sAlarm,RTC_FORMAT_BIN);
}



