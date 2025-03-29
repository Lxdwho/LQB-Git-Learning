#ifndef _MINT_H_
#define _MINT_H_

#include "main.h"
#include "stdbool.h"

typedef struct 
{
	bool key_sta;
	uint8_t judge_sta;
	bool single_sta;
}keys;

void LED_Proc(uint8_t led,uint8_t state);

#endif
