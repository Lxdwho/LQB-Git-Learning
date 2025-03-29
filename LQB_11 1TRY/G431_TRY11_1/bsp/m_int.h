#ifndef _M_INT_H_
#define _M_INT_H_

#include "main.h"
#include "stdbool.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

struct keys
{
	bool key_sta;
	bool single_sta;
	uint8_t judge_flag;
};

#endif
