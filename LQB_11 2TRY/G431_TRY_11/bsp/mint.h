#ifndef _MINT_H_
#define _MINT_H_

#include "main.h"
#include "stdbool.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

struct keys{
	bool single_flag;
	bool key_sta;
	unsigned int judge_flag;
};


#endif
