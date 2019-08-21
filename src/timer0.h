//timer0.h
// timer0 is used for fast pwm (OC0B output)
#ifndef TIMER0_H
#define TIMER0_H

#include <inttypes.h>

///! Initializes TIMER0 for fast PWM mode-driven bed heating
extern void timer0_init(void);

#endif //TIMER02_H
