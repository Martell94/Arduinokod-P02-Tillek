
#ifndef _FUNCTIONS_h
#define _FUNCTIONS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define BUTTON_PRESSED buttonState == HIGH
#define LCD_BACKL_ON digitalWrite(8,HIGH)
#define LCD_BACKL_OFF digitalWrite(8,LOW)
#define CURRENT_AMPS currentAvg>=1000
#define CURRENT_MAMPS currentAvg<1000
#define POWER_WATTS powerMoment >=1
#define POWER_MWATTS powerMoment <1

void check_button();
float get_voltage();
long get_current();
void print_voltage_amps();
void print_power();
void init_powermeasure();
void lcd_sleep();


#endif

