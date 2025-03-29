#ifndef __DHT11_HAL_H
#define __DHT11_HAL_H

#include "stm32g4xx_hal.h"

#define HDQ		GPIO_PIN_7

typedef struct {
	uint8_t humidity_high;
	uint8_t humidity_low;
	uint8_t temperature_high;
	uint8_t temperature_low;
}dht11Data;

void dht11Init(void);
dht11Data* dht11Read(void);


#endif
