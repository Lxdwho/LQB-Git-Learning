#include "ds18b20.h"

#define delay_us(X)  delay((X)*80/5)

void delay(unsigned int n)
{
	while(n--);
}

void ds18b20_init_x(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitStructure.Pin = GPIO_PIN_6;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
}

//
void mode_input1(void )
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = GPIO_PIN_6;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
}

void mode_output1(void )
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = GPIO_PIN_6;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
}

//
uint8_t ow_reset(void)
{ 
	uint8_t err;

   	OW_DIR_OUT(); // pull OW-Pin low for 480us
   	OW_OUT_LOW(); // disable internal pull-up (maybe on from parasite)

   	delay_us(400);	  //about 480us
   
   	// set Pin as input - wait for clients to pull low
   	OW_DIR_IN(); // input

   	delay_us(66);
   	err = OW_GET_IN();		// no presence detect
   	// nobody pulled to low, still high


   	// after a delay the clients should release the line
  	// and input-pin gets back to high due to pull-up-resistor
   	delay_us(480-66);
   	if( OW_GET_IN() == 0 )		// short circuit
      err = 1;

   	return err;
 }

uint8_t ow_bit_io( uint8_t b )
{ 
 	OW_DIR_OUT(); // drive bus low
 	OW_OUT_LOW();	
 	delay_us(1); // Recovery-Time wuffwuff was 1

 	if ( b ) OW_DIR_IN(); // if bit is 1 set bus high (by ext. pull-up)
	
#define  OW_CONF_DELAYOFFSET  5
 	delay_us(15-1-OW_CONF_DELAYOFFSET);
      
 	if( OW_GET_IN() == 0 ) b = 0;  // sample at end of read-timeslot
	
 	delay_us(60-15);
 	OW_DIR_IN();

 	return b;
}

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

double Read_temperature(void)
{
	uint8_t h,l;
	uint16_t tem;
	double Tem;
	
	ow_reset();
	ow_byte_wr(OW_SKIP_ROM);
	ow_byte_wr(DS18B20_CONVERT);
	
	ow_reset();
	ow_byte_wr(OW_SKIP_ROM);
	ow_byte_wr(DS18B20_READ);
	
	l = ow_byte_rd();
	h = ow_byte_rd();
	tem = (h<<8) + l;
	if((tem & 0xf800) == 0xf800)
	{
		tem = (~tem) + 1;
		Tem = tem * -0.0625;
	}
	else Tem = tem * 0.0625;
	return Tem;
}














