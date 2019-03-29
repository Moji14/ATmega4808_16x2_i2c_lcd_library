/*
 * i2c_lcd.h
 *
 *DHT11 libraty for ATmega4808
 *
 * Created: 28/03/2019 18:06:29
 *  Author: Mohamed Lanjri El Halimi email: moji14@hotmail.com
 */ 
#include <avr/delay.h>
#include <stdio.h>
#include "display_all_data.h"

void display_all(int8_t *To, int8_t *Ti, int8_t *Hum, int8_t *Ev){
	i2c_lcd_init();
	i2c_lcd_led_on();
	i2c_lcd_set_cursor(0,0);

	char buffer[20];
	//int t_out=11, t_in=22, hum=33, ev=44;

	i2c_lcd_clear();
	_delay_us(1500);

	i2c_lcd_write_text("To:   C H:   %");
	itoa(*To, buffer, 10);//Temperature conversion to text
	i2c_lcd_set_cursor(3,0);//Set curso position
	i2c_lcd_write_text(buffer);//Write on specified position
	itoa(*Hum, buffer, 10);//Temperature conversion to text
	i2c_lcd_set_cursor(10,0);//Set curso position
	i2c_lcd_write_text(buffer);//Write on specified position

	i2c_lcd_set_cursor(0,1);
	i2c_lcd_write_text("Ti:   C L:   lx");
	itoa(*Ti, buffer, 10);//Temperature conversion to text
	i2c_lcd_set_cursor(3,1);//Set curso position
	i2c_lcd_write_text(buffer);//Write on specified position
	itoa(*Ev, buffer, 10);//Temperature conversion to text
	i2c_lcd_set_cursor(10,1);//Set curso position
	i2c_lcd_write_text(buffer);//Write on specified position
	
	_delay_ms(1500);	
}