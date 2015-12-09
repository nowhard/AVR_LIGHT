/*
*/

#define F_CPU  1200000 
#define T0_OVF_US	((uint8_t)(1000000 / (F_CPU / 8 / 256)))
#define MS_10	(6)	//((uint8_t)(10000 / T0_OVF_US))
//#define BRIGHT_MEAS_PERIOD_IN_MS	
#define BRIGHT_0	(0)
//#define BRIGHT_LOW	((uint8_t)(220 / 100 *10))
#define BRIGHT_100	(220)
//#define triggerPhotoLevel	(100)
//#define WLag 254


#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

//uint16_t EEMEM eeprom_var = 0xaa2; //459 ; // 3 16-ричных цифры кода кнопки (до 4096 кодов)

uint8_t EEMEM triggerPhotoLevel_EEMEM = 80; //
uint8_t EEMEM WLag_EEMEM = 254; //
uint8_t EEMEM BRIGHT_LOW_EEMEM = 20; //
uint16_t EEMEM timer_ON_MODE_EEMEM = 6000; //


uint8_t triggerPhotoLevel; //
uint8_t WLag; //
uint8_t HYST;
uint8_t BRIGHT_LOW;
uint16_t timer_ON_MODE;


uint8_t photoLevel, photoLevelLPF;
uint8_t extBright;
volatile uint32_t ovrf=0;
volatile uint8_t brMeasRq;
uint8_t sound;
enum {MODE_OFF, POWER_ON, MODE_NORMAL, MODE_TIMER, MODE_2};
uint8_t mode = POWER_ON;
enum {SUBMODE_WORK, SUBMODE_BLINK, SUBMODE_WAIT};
uint8_t submode = SUBMODE_WORK;

volatile struct {
	uint8_t tic		:1;
	uint8_t ms10	:1;
	uint8_t adc		:1;
	uint8_t sound	:1;
	uint8_t blink_end	:1;
	uint8_t bright_meas_md	:1;
	uint8_t bright_meas_rq	:1;
} evt;

typedef struct {
	uint8_t cnt;
	uint8_t tOff;
	uint8_t tOn;
	uint8_t brOn;
	uint8_t brOff;
	uint8_t state;
} sBlink;

sBlink blink;

// фильтр первого пор€дка с коэффициентом усилени€ = 256 (иначе на малых сигналах возможна самоблокировка фильтра по ограниченной разр€дности)
#if 1
uint8_t filtr_1 (uint8_t in)
{
	static uint16_t lastOut=0;
	uint16_t out = in * (256 - WLag) + ((WLag * (uint32_t)lastOut) / 256);
	lastOut = out;
	return out / 256; 
}
#else
uint8_t filtr_1 (uint8_t in)
{
	static uint8_t lastOut=0;
	uint16_t out = in * (256 - WLag) + (WLag * lastOut);
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

void setLightBright(uint8_t val){
	if(val == 0){
		stop_fastPWM_B();
	}
	else {
		startT1_div8_fastPWM_B();
	}
	OCR0B = ~val;
}

inline uint8_t getLightBright(void){
	return ~OCR0B;
}

inline uint8_t findSoundFront(void)
{
	static uint8_t prevSound;
	uint8_t sound, ret=0;
	sound = PINB & _BV(2);
  	if((sound) && prevSound==0){
		ret = 1;
	}
	prevSound = sound;
	return ret;
}

/*inline void setBlinkCnt(uint8_t cnt)
{
	blink.cnt = cnt+1;
}

void blink_clk(void)
{
	static uint8_t timer;
	if(blink.cnt){
		if(timer){
			timer--;
		}
		else{
			if(blink.state){
				blink.state = 0;
				timer = blink.tOff;
			}
			else{
				blink.cnt--;
				if(blink.cnt){
					blink.state = 1;
					timer = blink.tOn;
				}
				else{				
					evt.blink_end = 1;
				}
			}
		}
	}
}
*/
ISR(TIM0_OVF_vect){
static uint8_t ms10_timer = MS_10;
	uint16_t val;
	ovrf++; //Increment counter every 256 clock cycles
	ms10_timer--;
	if(ms10_timer == 0){
		ms10_timer = MS_10;
//		blink_clk();
		evt.ms10 = 1;
	}
	// значение ј÷ѕ тем более, чем темнее, поэтому проинвертируем и ограничим €ркость 8-ю битами
	val = (1023 - ADC);
	if(val > 255){
		photoLevel = 255;
	}
	else{
		photoLevel = val;
	}
	photoLevelLPF = filtr_1(photoLevel);
	PORTB |= _BV(0);	// for test
	evt.tic = 1;
	ADCSRA |= _BV(ADSC); // start ADC conversion (Light from PWM is OFF!)
}

ISR(ADC_vect){
/*static uint8_t prevBright;
static uint8_t pause;
	if(evt.bright_meas_md){
		pause--;
		if(!pause){
			extBright = ~((uint8_t)(ADC >> 2));
			setLightBright(prevBright);
			evt.bright_meas_md = 0;
		}
	}
	else{
		photoLevel = ~((uint8_t)(ADC >> 2));
	}
//	photoLevel = ADC;
	if(evt.bright_meas_rq){
		prevBright = getLightBright();
		setLightBright(BRIGHT_0);
		evt.bright_meas_md = 1;
		evt.bright_meas_rq = 0;
		pause = 2;	// пауза дл€ гашени€ светильника, прежде, чем замерим внешнюю освещенность (в количестве преобразований ј÷ѕ)
	}
	evt.adc = 1;
	ADCSRA |= _BV(ADSC); // start ADC conversion
*/
	PORTB &= ~_BV(0);	// for test
}


void stateMashine(void)
{
	static uint16_t timer = 200;	// POWER_ON mode timer
	int16_t level;
	int16_t dLevel;
	if(photoLevelLPF > triggerPhotoLevel + HYST){	// при освешенности большей порога однозначно выключаем свет из любого режима
    	setLightBright(BRIGHT_0);    // turn the LED off
		mode = MODE_OFF;
	}
	switch (mode){
		case MODE_OFF:	// свет выключен т.к. внешн€€ освещенность больше порога. ѕри снижении освещенности переходим в нормальный режим 
			if(photoLevelLPF <= triggerPhotoLevel - HYST){
		    	setLightBright(BRIGHT_LOW);    // turn the LED off by making the voltage LOW
				mode = MODE_NORMAL;
				evt.sound = 0;
		  	}
		break;
		case POWER_ON:
	    	if(timer){
				timer--;
			}
			else{
		    	setLightBright(BRIGHT_0);    // turn the LED off by making the voltage LOW
				mode = MODE_OFF;
		  	}
		break;
		case MODE_NORMAL: // освещенность в этом режиме - 20%. при срабатывании датчика звука - ставим €ркость 100 и уходим в отработку таймера 
//			setLightBright(photoLevelLPF);
			if(evt.sound){
	    		timer = timer_ON_MODE;
	    		setLightBright(BRIGHT_100);    // turn the LED off by making the voltage LOW
				mode = MODE_TIMER;
			
/*				blink.tOn = 2;
				blink.tOff = 2;
				blink.brOn = 200;
				blink.brOff = 0;
				setBlinkCnt(200);
				evt.sound = 0;
				submode = SUBMODE_BLINK;*/
			}
/*			switch (submode){
				case SUBMODE_WORK:
					
					dLevel = photoLevel - photoLevelLPF;
					level = photoLevelLPF;// +(dLevel*16);
					if(level < 0) level = 0;
					if(level > 255) level = 255;
					setLightBright((uint8_t)level);
				break;
*/				
/*				case SUBMODE_BLINK:
					if (blink.cnt){
						if(blink.state) 
							setLightBright(blink.brOn);
						else 
							setLightBright(blink.brOff);
					}
					else{
						submode = SUBMODE_WORK;
						evt.sound = 0;
				    	setLightBright(BRIGHT_0);    // turn the LED off by making the voltage LOW
					}
				break;*/
//			};
/*	    	if(timer){
				timer--;
			}
			else{
		    	setLightBright(BRIGHT_0);    //turn the LED to 0%
		  	}*/
		break;

		case MODE_TIMER:
	    	if(timer){
				timer--;
			}
			else{
		    	setLightBright(BRIGHT_LOW);    // turn the LED to 20%
				mode = MODE_NORMAL;
				evt.sound = 0;
		  	}
		break;

		case MODE_2:
		break;
	}
}

inline void init_var(void){
	triggerPhotoLevel = eeprom_read_byte(&triggerPhotoLevel_EEMEM); // 
	WLag = eeprom_read_byte(&WLag_EEMEM); // посто€нна€ времени фильтра
	BRIGHT_LOW = eeprom_read_byte(&BRIGHT_LOW_EEMEM); // уровень пониженной €ркости (из максимума = 250)
	timer_ON_MODE = eeprom_read_word(&timer_ON_MODE_EEMEM); // врем€ удержани€ при срабатывании звука
	HYST = triggerPhotoLevel / 16;	// гистерезис день/ночь
	mode = POWER_ON;
}

inline void timer_init(void){
	//Setup timer interrupt and PWM pins
	TIMSK0 |= 2;						// and OVF interrupt
	TCNT0=0; 
	setLightBright(BRIGHT_100);	// on
	startT1_div8_fastPWM_B();		// PWM to pin to light control
}

inline void ADC_init(void){
	ADMUX= /*_BV(ADLAR) |*/ 3;	// pinb.3, ref = external, left adj
	ADCSRA |= _BV(ADEN) | _BV(ADIE) | _BV(ADPS1) | _BV(ADPS0);	// около 40 мкс на преобразование (8 бит) при 1 200 000
}

int main()
{
	init_var();
	DDRB |=0b00000011;//Led=portB.1
	ADC_init();
	timer_init();
	sei();
	while(1){
		if(evt.ms10){
			stateMashine();
			evt.ms10 = 0;
		}

		if(findSoundFront()) evt.sound = 1;
	}
}
