/*
 * i2c_lcd.h
 *
 *DHT11 libraty for ATmega4808
 *
 * Created: 28/03/2019 18:06:29
 *  Author: Mohamed Lanjri El Halimi email: moji14@hotmail.com
 *References:
 * - Patrick Taling on 20/02/2017
 * - Noel200 from www.elektroda.pl/rtvforum/topic2756081.html
 * - w.e.dolman w.e.dolman@hva.nl
 * - Obs³uga wyœwietlacza HD44780 po I2C za pomoc¹ PCF8574T. 2015-01-DASEJ , dasej(at)wp.pl 
 */ 

#include "i2c_lcd.h"
#include <util/delay.h>
#include "avr/io.h"

char i2c_lcd_status = 0X00;
volatile uint8_t lcd_line = 0;

void i2c_lcd_write_status(void){
	I2C_0_write1ByteRegister(i2c_lcd_ADDR,&TWIE,i2c_lcd_status);
}

void i2c_lcd_data_part(char data_part)
{
	i2c_lcd_status &= ~i2c_lcd_DB4 & ~i2c_lcd_DB5 & ~i2c_lcd_DB6 & ~i2c_lcd_DB7;
	if(data_part & 0x01) i2c_lcd_status |= i2c_lcd_DB4;
	if(data_part & 0x02) i2c_lcd_status |= i2c_lcd_DB5;
	if(data_part & 0x04) i2c_lcd_status |= i2c_lcd_DB6;
	if(data_part & 0x08) i2c_lcd_status |= i2c_lcd_DB7;
}

void i2c_lcd_write(char data)
{
	i2c_lcd_e_hi();
	i2c_lcd_data_part(data >> 4);
	i2c_lcd_write_status();
	i2c_lcd_e_lo();
	i2c_lcd_write_status();
	i2c_lcd_e_hi();
	i2c_lcd_data_part(data);
	i2c_lcd_write_status();
	i2c_lcd_e_lo();
	i2c_lcd_write_status();
	_delay_ms(20);
}

void i2c_lcd_write_instruction(char instruction)
{
	i2c_lcd_rw_lo();
	i2c_lcd_rs_lo();
	i2c_lcd_write(instruction);
}

void i2c_lcd_write_data(char data)
{
	i2c_lcd_rs_hi();
	
	switch (data) {
		case '\f':
		i2c_lcd_clear();
		lcd_line = 0;
		break;
		case '\n':
		if (++lcd_line==LCD_LINES) lcd_line = 0;
		i2c_lcd_set_cursor(0, lcd_line);
		break;
		default:
		i2c_lcd_write(data);
		break;
	}
}

void i2c_lcd_write_text(char *text)
{
	while(*text) i2c_lcd_write_data(*text++);
}

void i2c_lcd_clear(void)
{
	lcd_line = 0;
	i2c_lcd_write_instruction(0x01);
}

void i2c_lcd_set_cursor(char x, char y)
{
	uint8_t address;

	#if LCD_LINES==1
	address = LCD_START_LINE1;
	#elif LCD_LINES==2
	if ( y==0 ) {
		address = LCD_START_LINE1;
		} else {
		address = LCD_START_LINE2;
	}
	#else
	if ( y==0 ) {
		address = LCD_START_LINE1;
		} else if ( y==1) {
		address = LCD_START_LINE2;
		} else if ( y==2) {
		address = LCD_START_LINE3;
		} else {
		address = LCD_START_LINE4;
	}
	#endif
	
	i2c_lcd_write_instruction(HD44780_DDRAM_SET | (x + (address * y)));
}

void i2c_lcd_led_on(void)
{
	i2c_lcd_led_hi();
	i2c_lcd_write_status();
}

void i2c_lcd_led_off(void)
{
	i2c_lcd_led_lo();
	i2c_lcd_write_status();
}

void i2c_lcd_home(void)
{
	lcd_line = 0;
	i2c_lcd_set_cursor(0,0);
}

void i2c_lcd_init(void)
{
	char i;
	_delay_ms(150);
	for(i = 0; i < 3; i++)
	{
		i2c_lcd_data_part(0x03);
		i2c_lcd_e_hi();
		i2c_lcd_write_status();
		
		i2c_lcd_e_lo();
		i2c_lcd_write_status();
		_delay_ms(40);
	}
	i2c_lcd_data_part(0x02);
	i2c_lcd_e_hi();
	i2c_lcd_write_status();
	
	i2c_lcd_e_lo();
	i2c_lcd_write_status();
	_delay_ms(10);
	
	i2c_lcd_write_instruction(HD44780_FUNCTION_SET | HD44780_FONT5x10 | HD44780_TWO_LINE | HD44780_4_BIT); // interfejs 4-bity, 2-linie, znak 5x7
	i2c_lcd_write_instruction(HD44780_DISPLAY_ONOFF | HD44780_DISPLAY_OFF); // wy³¹czenie wyswietlacza
	i2c_lcd_write_instruction(HD44780_CLEAR); // czyszczenie zawartosæi pamieci DDRAM
	_delay_ms(20);
	i2c_lcd_write_instruction(HD44780_ENTRY_MODE | HD44780_EM_SHIFT_CURSOR | HD44780_EM_INCREMENT);// inkrementaja adresu i przesuwanie kursora
	i2c_lcd_write_instruction(HD44780_DISPLAY_ONOFF | HD44780_DISPLAY_ON | HD44780_CURSOR_OFF | HD44780_CURSOR_NOBLINK); // w³¹cz LCD, bez kursora i mrugania
}
