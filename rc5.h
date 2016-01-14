#ifndef RC5_H
#define RC5_H

typedef union { 
	uint16_t raw;
	struct {
		unsigned cmd    : 6;	// LSB
		unsigned addr   : 5;
		unsigned toggle : 1;
		unsigned start  : 2;
		unsigned        : 2;	// MSB
	};
} rc5data;

enum
{
	RC5_MSG_NONE=0,
	RC5_MSG_PROCESS,
	RC5_MSG_NEW_MSG
};

extern volatile uint8_t rc5_flag;
extern volatile rc5data data;

void RC5_interrupt_init(void);

#endif
