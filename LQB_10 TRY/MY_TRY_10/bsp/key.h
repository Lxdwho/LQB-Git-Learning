#ifndef _KEY_H_
#define _KEY_H_

#include "main.h"
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

struct keys{
	uint8_t key_sta;      //Ϊ1��ʾ��⵽�ߵ�ƽ
	uint8_t single_sta;   //Ϊ1��ʾȷ�ϰ�������
	uint8_t judge_sta;		//Ϊ1��ʾ10ms�����ɼ�⵽�ߵ�ƽ
};

#endif
