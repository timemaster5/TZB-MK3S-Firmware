//timer2.h
// use atmega timer2 as main system timer instead of timer0
// timer0 is used for fast pwm (OC0B output)
// original OVF handler is disabled
#ifndef TIMER2_H
#define TIMER2_H

#include <inttypes.h>

///! Initializes TIMER2
extern void timer2_init(void);

///! Reimplemented original _millis() using timer2
extern unsigned long millis2(void);

///! Reimplemented original micros() using timer2
extern unsigned long micros2(void);

///! Reimplemented original delay() using timer2
extern void delay2(unsigned long ms);

///! Reimplemented original tone() using timer2
///! Does not perform any PWM tone generation, it just sets the beeper pin to 1
extern void tone2(uint8_t _pin, unsigned int frequency/*, unsigned long duration*/);

///! Turn off beeping - set beeper pin to 0
extern void noTone2(uint8_t _pin);

#endif //TIMER2_H
