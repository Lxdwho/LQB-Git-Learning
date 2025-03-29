#include "ds18b20_hal.h"

#define delay_us(X)  delay((X)*80/5)

//
void delay(unsigned int n)
{
    while(n--);
}

//
void ds18b20_init_x(void)
{
    GPIO_InitTypeDef	GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();


    GPIO_InitStruct.Pin = OW_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH ;
    GPIO_InitStruct.Pull = GPIO_PULLUP ;
    HAL_GPIO_Init(OW_PIN_PORT, &GPIO_InitStruct);
}

//
void mode_input1(void )
{

    GPIO_InitTypeDef	GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */

    GPIO_InitStruct.Pin = OW_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(OW_PIN_PORT, &GPIO_InitStruct);
}

//
void mode_output1(void )
{

    GPIO_InitTypeDef	GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = OW_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL ;
    HAL_GPIO_Init(OW_PIN_PORT, &GPIO_InitStruct);
}

//
uint8_t ow_reset(void)
{
    uint8_t err;

    OW_DIR_OUT(); 
    OW_OUT_LOW(); 

    delay_us(400);	  

    OW_DIR_IN(); 

    delay_us(66);
    err = OW_GET_IN();		

    delay_us(480 - 66);
    if( OW_GET_IN() == 0 )		
        err = 1;

    return err;
}

//
uint8_t ow_bit_io( uint8_t b )
{
    OW_DIR_OUT(); 
    OW_OUT_LOW();
    delay_us(1); 

    if ( b ) OW_DIR_IN(); 

#define  OW_CONF_DELAYOFFSET  5
    delay_us(15 - 1 - OW_CONF_DELAYOFFSET);

    if( OW_GET_IN() == 0 ) b = 0;  

    delay_us(60 - 15);
    OW_DIR_IN();

    return b;
}

//
uint8_t ow_byte_wr( uint8_t b )
{
    uint8_t i = 8, j;
    do
    {
        j = ow_bit_io( b & 1 );
        b >>= 1;
        if( j ) b |= 0x80;
    }
    while( --i );
    return b;
}

//
uint8_t ow_byte_rd( void )
{
    return ow_byte_wr( 0xFF );
}

double TEMP_READ(void)
{
	unsigned char h,l;
	uint16_t tem;
	
	ow_reset();
	ow_byte_wr(OW_SKIP_ROM);
	ow_byte_wr(DS18B20_CONVERT);
	
	ow_reset();
	ow_byte_wr(OW_SKIP_ROM);
	ow_byte_wr(DS18B20_READ);
	
	l = ow_byte_rd();
	h = ow_byte_rd();
	
	tem = (h<<8) + l;
	return tem * 0.0625;
	
}


















