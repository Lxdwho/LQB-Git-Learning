#include "dht11_hal.h"


dht11Data dht11;

//
static void usDelay(uint32_t us)
{
	uint16_t i = 0;
	while(us--){
		i = 30;
		while(i--);
	}
}

//
void outDQ(uint8_t i)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	
	GPIO_InitStructure.Pin = HDQ;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	if(!i) 
		HAL_GPIO_WritePin(GPIOA, HDQ, GPIO_PIN_RESET);
	else 
		HAL_GPIO_WritePin(GPIOA, HDQ, GPIO_PIN_SET);
}

//
void inDQ(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.Pin = HDQ;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
}

//
void dht11Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
	outDQ(1);			
}

//
uint8_t recData(void)
{
	uint8_t i,temp=0,j=220;
	
	for(i=0; i<8; i++){
		
		while(!HAL_GPIO_ReadPin(GPIOA,HDQ));
		usDelay(40);
		if(HAL_GPIO_ReadPin(GPIOA,HDQ))
		{
			temp=(temp<<1)|1;
			while(HAL_GPIO_ReadPin(GPIOA,HDQ)&&(j--));	
		}	
		else
		{
			temp=(temp<<1)|0;
		}
	}
	return temp;
}
