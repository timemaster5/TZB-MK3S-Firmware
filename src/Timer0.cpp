//timer0.cpp
// timer0 is used for fast pwm (OC0B output)

#include "system_timer.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include "io_atmega2560.h"

void timer0_init(void)
{
	//save sreg
	uint8_t _sreg = SREG;
	//disable global interrupts
	cli();

	TCNT0  = 0;
	// Phase correct PWM duty (0-255). 
	// Set initial duty-cycle to zero (bed not heating).
	OCR0B = 0;
	// Set phase correct PWM mode clear OC0B on match when up-counting and set on match when down counting.
	TCCR0A = (1 << WGM00) | (1 << COM0B1);  
	TCCR0B = (1 << CS00) | (1 << CS02);    // 1024 (prescaler) to result in 30.51758Hz
	TIMSK0 |= (1 << TOIE0);  // enable timer overflow interrupt

  sei(); // enable global interrupts
	//restore sreg (enable interrupts)
	SREG = _sreg;
}
