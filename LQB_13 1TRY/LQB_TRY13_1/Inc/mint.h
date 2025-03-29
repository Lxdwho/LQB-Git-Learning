#ifndef _MINT_H_
#define _MINT_H_

#include "main.h"
#include "stdbool.h"

struct keys
{
	bool key_sta;
	bool single_sta;
	uint8_t judge;
};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void usart_proc(void);

#endif
