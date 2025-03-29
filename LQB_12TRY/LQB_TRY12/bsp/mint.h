#ifndef _MINT_H_
#define _MINT_H_

#include "main.h"
#include "stdbool.h"

struct keys
{
	bool key_sta;
	bool single_sta;
	uint8_t judge_sta;
};

struct cars
{
	char car_num[5];
	char car_type[5];
	int year;
	int month;
	int day;
	int hours;
	int minter;
	int second;
	bool exit;
};
void usart1_Proc(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif
