#ifndef UTILITIES__H__
#define UTILITIES__H__

#include <xc.h> // processor SFR definitions
#include <sys/attribs.h> // __ISR macro

#include "NU32DIP.h"

enum mode {IDLE, PWM, ITEST, HOLD, TRACK}; 

volatile enum mode get_mode(void);
void set_mode(enum mode input);

#endif // UTILITIES__H__
