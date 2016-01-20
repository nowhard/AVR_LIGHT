/*
*/

#define F_CPU  1200000 
#define T0_OVF_US	((uint8_t)(1000000 / (F_CPU / 8 / 256)))
#define MS_10	(6)	//((uint8_t)(10000 / T0_OVF_US))
//#define BRIGHT_MEAS_PERIOD_IN_MS	
#define BRIGHT_0	(0)
//#define BRIGHT_LOW	((uint8_t)(220 / 100 *10))
#define BRIGHT_100	(225)
//#define triggerPhotoLevel	(100)
//#define WLag 254


#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "rc5.h"


uint8_t EEMEM triggerPhotoLevel_EEMEM = 180; //
uint8_t EEMEM triggerSoundLevel_EEMEM = 125;
uint16_t EEMEM WLag_EEMEM = 4090; //
uint8_t EEMEM BRIGHT_LOW_EEMEM = 20; //
uint16_t EEMEM timer_ON_MODE_EEMEM = 6000; //


uint8_t triggerPhotoLevel; //
uint8_t triggerSoundLevel;
uint16_t WLag; //
uint8_t HYST;
uint8_t BRIGHT_LOW;
uint16_t timer_ON_MODE;

uint16_t adc_microphone,adc_microphone_prev,adc_photosensor;


uint8_t photoLevel, photoLevelLPF;

enum {MODE_OFF, POWER_ON, MODE_NORMAL, MODE_TIMER, MODE_2};
uint8_t mode = POWER_ON;
enum {SUBMODE_WORK, SUBMODE_BLINK, SUBMODE_WAIT};
uint8_t submode = SUBMODE_WORK;

volatile struct 
{
	uint8_t tic		:1;
	uint8_t ms10	:1;
	uint8_t adc		:1;
	uint8_t sound	:1;
	uint8_t blink_end	:1;
	uint8_t bright_meas_md	:1;
	uint8_t bright_meas_rq	:1;
} evt;

typedef struct 
{
	uint8_t cnt;
	uint8_t tOff;
	uint8_t tOn;
	uint8_t brOn;
	uint8_t brOff;
	uint8_t state;
} sBlink;

sBlink blink;

enum
{
	ADC0=0,
	ADC1,
	ADC2,
	ADC3
};


// фильтр первого порядка с коэффициентом усиления = 4096 (иначе на малых сигналах возможна самоблокировка фильтра по ограниченной разрядности)

//#define WLAG 4095
#define FILTER_TCONST	4096
#if 1
uint8_t filtr_1 (uint8_t in)
{
/*	static uint16_t lastOut=0;
	uint16_t out = in * (256 - WLag) + ((WLag * (uint32_t)lastOut) / 256);
	lastOut = out;
	return out / 256; */

	static uint32_t lastOut=0;
	uint32_t out = in * (FILTER_TCONST - WLag) + ((WLag * lastOut) / FILTER_TCONST);
	lastOut = out;
	return (uint8_t)(out / FILTER_TCONST); 

}
#else
uint8_t filtr_1 (uint8_t in)
{
	static uint8_t lastOut=0;
	uint16_t out = in * (255 - WLag) + (WLag * lastOut);
	lastOut = out / 256;
	return lastOut; 
}
#endif

void startT1_div8_fastPWM_B(void)
{
	TCCR0B |= _BV(CS01);	// div_8 
	TCCR0A = _BV(WGM00)|_BV(WGM01);	// fast PWM 0 to FF
	TCCR0A |= _BV(COM0B1) | _BV(COM0B0);		// inverted PWM to pin to light control
}

inline void stop_fastPWM_B(void)
{
	TCCR0A &= ~(_BV(COM0B1) | _BV(COM0B0));
}

void setLightBright(uint8_t val)
{
	if(val == 0)
	{
		stop_fastPWM_B();
	}
	else 
	{
		startT1_div8_fastPWM_B();
	}
	OCR0B = ~val;
}

/*inline uint8_t getLightBright(void)
{
	return ~OCR0B;
}*/


inline uint8_t findSoundFront(void)
{
	uint8_t  ret=0;
	uint8_t  sound_val=(uint8_t)(adc_microphone>>2);

	if(sound_val>triggerSoundLevel)
	{
		ret = 1;
	}

	return ret;
}

ISR(TIM0_OVF_vect)
{
	static uint8_t ms10_timer = MS_10;
//	uint16_t val;


	ms10_timer--;
	if(ms10_timer == 0)
	{
		ms10_timer = MS_10;
		evt.ms10 = 1;
	}

    ADMUX&=~(0x3);
    ADMUX|=ADC3;
	ADCSRA |= _BV(ADSC); // start ADC conversion (Light from PWM is OFF!)

	//PORTB |= _BV(0);

	
	if(adc_photosensor > 255)
	{
		photoLevel = 255;
	}
	else
	{
		photoLevel = adc_photosensor;
	}
//	photoLevelLPF = filtr_1(photoLevel);


	
	evt.tic = 1;
}


uint8_t mirophone_cycle=0;
#define MICROPHONE_CYCLE_NUM	2

ISR(ADC_vect)
{
	switch(ADMUX&0x3)
	{
		case ADC3:
		{
			//PORTB &= ~_BV(0);
			adc_photosensor=ADC;

			mirophone_cycle=0;
			ADMUX&=~(0x3);
			ADMUX|=ADC1;
			ADCSRA |= _BV(ADSC);
		}
		break;

		case ADC1:
		{
			
			adc_microphone=ADC;

			if(mirophone_cycle<MICROPHONE_CYCLE_NUM)
			{
				ADMUX&=~(0x3);
				ADMUX|=ADC1;
				ADCSRA |= _BV(ADSC);
				mirophone_cycle++;
			}
		}
		break;

		default:
		{
		}
	}
}

void stateMashine(void)
{
	static uint16_t timer = 200;	// POWER_ON mode timer
	
//	photoLevelLPF = filtr_1(photoLevel);
	if(photoLevelLPF > (triggerPhotoLevel + HYST))
	{	// при освешенности большей порога однозначно выключаем свет из любого режима
		setLightBright(BRIGHT_0);    // turn the LED off
		mode = MODE_OFF;
	}
	switch (mode)
	{
		case MODE_OFF:	// свет выключен т.к. внешняя освещенность больше порога. При снижении освещенности переходим в нормальный режим 
			if(photoLevelLPF  <= (triggerPhotoLevel - HYST))
			{
		    	setLightBright(BRIGHT_LOW);    // turn the LED off by making the voltage LOW
				mode = MODE_NORMAL;
				evt.sound = 0;
		  	}
		break;

		case POWER_ON:
	    	if(timer)
			{
				timer--;
			}
			else
			{
		    	setLightBright(BRIGHT_0);    // turn the LED off by making the voltage LOW
				mode = MODE_OFF;
		  	}
		break;

		case MODE_NORMAL: // освещенность в этом режиме - 20%. при срабатывании датчика звука - ставим яркость 100 и уходим в отработку таймера 
			//setLightBright(photoLevelLPF);
//			setLightBright(BRIGHT_100);
			if(evt.sound)
			{
	    		timer = timer_ON_MODE;
	    		setLightBright(BRIGHT_100);    // turn the LED off by making the voltage LOW
				mode = MODE_TIMER;
			
			}
		break;

		case MODE_TIMER:
	    	if(timer)
			{
				timer--;

				if(evt.sound)
				{
		    		timer = timer_ON_MODE;
					evt.sound = 0;
				}
			}
			else
			{
		    	setLightBright(BRIGHT_LOW);    // turn the LED to 20%
				mode = MODE_NORMAL;
				evt.sound = 0;
		  	}
		break;

		case MODE_2:
		break;
	}
}

inline void init_var(void)
{
	triggerPhotoLevel = eeprom_read_byte(&triggerPhotoLevel_EEMEM); // 
	triggerSoundLevel = eeprom_read_byte(&triggerSoundLevel_EEMEM); // 
	WLag = eeprom_read_word(&WLag_EEMEM); // постоянная времени фильтра
	BRIGHT_LOW = eeprom_read_byte(&BRIGHT_LOW_EEMEM); // уровень пониженной яркости (из максимума = 250)
	timer_ON_MODE = eeprom_read_word(&timer_ON_MODE_EEMEM); // время удержания при срабатывании звука
	HYST = triggerPhotoLevel/16;	// гистерезис день/ночь
	mode = POWER_ON;
}

inline void timer_init(void)
{
	//Setup timer interrupt and PWM pins
	/*TIMSK0 |= 2;*/						// and OVF interrupt
	TIMSK |= 2;
	TCNT0=0; 
	setLightBright(BRIGHT_100);	// on
	startT1_div8_fastPWM_B();		// PWM to pin to light control
}



inline void ADC_init(void)
{
	ADMUX&=~(0x3);
	ADCSRA |= _BV(ADEN) | _BV(ADIE) | _BV(ADPS1) | _BV(ADPS0);	// около 40 мкс на преобразование (8 бит) при 1 200 000
	DIDR0|=_BV(ADC1D)|_BV(ADC3D);
}

int main()
{
	init_var();
	//DDRB |=0b00000011;//Led=portB.1
	DDRB |=0b00010010;//Led=portB.1
	ADC_init();
	RC5_interrupt_init();
	timer_init();
	sei();
	while(1)
	{
		while(rc5_flag==RC5_MSG_PROCESS);

		

		if(rc5_flag==RC5_MSG_NEW_MSG)
		{
			//handle rc5
			rc5_flag=RC5_MSG_NONE;	
		}
		
		if(evt.ms10)
		{
			stateMashine();
			evt.ms10 = 0;
		}

		if(evt.tic)
		{
			photoLevelLPF = filtr_1(photoLevel);
			evt.tic=0;
		}

		if(findSoundFront()) evt.sound = 1;
	}
}
