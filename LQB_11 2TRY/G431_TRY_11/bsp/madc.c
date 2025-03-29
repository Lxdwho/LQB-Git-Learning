#include "madc.h"

double Getadc(ADC_HandleTypeDef * pin)
{
	double adc;
	HAL_ADC_Start(pin);
	adc = HAL_ADC_GetValue(pin);
	return adc*3.3/4095;
}
