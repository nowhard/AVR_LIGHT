#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

#include "rc5.h"


 
volatile uint8_t rc5_flag = 0;
volatile rc5data data;

inline void RC5_interrupt_init(void)
{
	PCMSK|= _BV(0);
	GIMSK|= _BV(PCIE);
}

static uint8_t pos=0;

ISR(PCINT0_vect)
{
 
	if (rc5_flag==RC5_MSG_NONE) 
	{
		data.raw = 0;
		rc5_flag = RC5_MSG_PROCESS;
	} 
			
	
	if (pos < 14) 
	{
		data.raw = (data.raw << 1) | ((~PINB) & 0x01);
		_delay_ms(1.8);
		pos++;
	}
	else
	{
		rc5_flag=RC5_MSG_NEW_MSG;
	}
}
