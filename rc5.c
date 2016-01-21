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
	rc5_flag==RC5_MSG_NONE;
}

inline void Interrupts_Disable(void)
{
	ADCSRA &=  ~_BV(ADIE);	
	TIMSK  &=  ~_BV(TOIE0);
	GIMSK  &=  ~_BV(PCIE);
}

inline void Interrupts_Enable(void)
{
	ADCSRA |=  _BV(ADIE);
	TIMSK  |=  _BV(TOIE0);
	GIMSK  |=  _BV(PCIE);
}

static uint8_t pos=0;

ISR(PCINT0_vect)
{
	GIFR  |=  _BV(PCIF);
	Interrupts_Disable();
	
 
	if (rc5_flag==RC5_MSG_NONE) 
	{

		data.raw = 0;
		pos=0;
		rc5_flag = RC5_MSG_PROCESS;
//		PORTB &= ~_BV(4);		
	} 
			
	
	if (pos < 14) 
	{
		data.raw = (data.raw << 1) | ((~PINB) & 0x01);
//		PORTB ^= _BV(4);
		pos++;
	}
/*	else
	{
		rc5_flag=RC5_MSG_NEW_MSG;
	//	PORTB |= _BV(4);
		PORTB &= ~_BV(4);
		Interrupts_Enable();
	}*/

	if(pos==14)
	{
		rc5_flag=RC5_MSG_NEW_MSG;
/*		PORTB &= ~_BV(4);
		_delay_ms(5);
		PORTB |= _BV(4);
		_delay_ms(5);
		PORTB &= ~_BV(4);*/
	}

	_delay_ms(1.4);
	GIFR  |=  _BV(PCIF);

    Interrupts_Enable();
}
