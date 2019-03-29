#include <atmel_start.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "i2c_lcd.h"

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	sei();//enable interrupt service
	int t_out=11, t_in=22, hum=33, ev=44;
	//Display all variables in a specified format used in "display_all_data.h" library
	display_all(&t_out, &t_in, &hum, &ev);
}
