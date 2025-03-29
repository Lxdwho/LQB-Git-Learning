#ifndef _KEY_H_
#define _KEY_H_

#include "main.h"
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

struct keys{
	uint8_t key_sta;      //为1表示检测到高电平
	uint8_t single_sta;   //为1表示确认按键按下
	uint8_t judge_sta;		//为1表示10ms后依旧检测到高电平
};

#endif
