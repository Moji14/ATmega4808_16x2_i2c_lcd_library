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

#include <util/delay.h>

#include "i2c_master.h"
#include "i2c_simple_master.h"


#define LCD_LINES          2             //!< Number of visible lines of the display
#define LCD_DISP_LENGTH    16            //!< Visible characters per line of the display

#if LCD_DISP_LENGTH==16
#define LCD_START_LINE1    0x00          //!< DDRAM address of first char of line 1
#define LCD_START_LINE2    0x40          //!< DDRAM address of first char of line 2
#define LCD_START_LINE3    0x10          //!< DDRAM address of first char of line 3
#define LCD_START_LINE4    0x50          //!< DDRAM address of first char of line 4
#else
#define LCD_START_LINE1    0x00          //!< DDRAM address of first char of line 1
#define LCD_START_LINE2    0x40          //!< DDRAM address of first char of line 2
#define LCD_START_LINE3    0x14          //!< DDRAM address of first char of line 3
#define LCD_START_LINE4    0x54          //!< DDRAM address of first char of line 4
#endif

#define i2c_lcd_RS						(1 << 0)
#define i2c_lcd_RW						(1 << 1)
#define i2c_lcd_E						(1 << 2)
#define i2c_lcd_LED						(1 << 3)
#define i2c_lcd_DB4						(1 << 4)
#define i2c_lcd_DB5						(1 << 5)
#define i2c_lcd_DB6						(1 << 6)
#define i2c_lcd_DB7						(1 << 7)



#define i2c_lcd_rs_lo()					i2c_lcd_status &= ~i2c_lcd_RS
#define i2c_lcd_rs_hi()					i2c_lcd_status |= i2c_lcd_RS

#define i2c_lcd_rw_lo()					i2c_lcd_status &= ~i2c_lcd_RW
#define i2c_lcd_rw_hi()					i2c_lcd_status |= i2c_lcd_RW


#define i2c_lcd_e_lo()					i2c_lcd_status &= ~i2c_lcd_E
#define i2c_lcd_e_hi()					i2c_lcd_status |= i2c_lcd_E

#define i2c_lcd_led_lo()				i2c_lcd_status &= ~i2c_lcd_LED
#define i2c_lcd_led_hi()				i2c_lcd_status |= i2c_lcd_LED


#define HD44780_ENTRY_MODE				0x04
	#define HD44780_EM_SHIFT_CURSOR		0
	#define HD44780_EM_SHIFT_DISPLAY	1
	#define HD44780_EM_DECREMENT		0
	#define HD44780_EM_INCREMENT		2

#define HD44780_DISPLAY_ONOFF			0x08
	#define HD44780_DISPLAY_OFF			0
	#define HD44780_DISPLAY_ON			4
	#define HD44780_CURSOR_OFF			0
	#define HD44780_CURSOR_ON			2
	#define HD44780_CURSOR_NOBLINK		0
	#define HD44780_CURSOR_BLINK		1

#define HD44780_DISPLAY_CURSOR_SHIFT	0x10
	#define HD44780_SHIFT_CURSOR		0
	#define HD44780_SHIFT_DISPLAY		8
	#define HD44780_SHIFT_LEFT			0
	#define HD44780_SHIFT_RIGHT			4

#define HD44780_FUNCTION_SET			0x20
	#define HD44780_FONT5x7				0
	#define HD44780_FONT5x10			4
	#define HD44780_ONE_LINE			0
	#define HD44780_TWO_LINE			8
	#define HD44780_4_BIT				0
	#define HD44780_8_BIT				16

#define HD44780_CGRAM_SET				0x40

#define HD44780_DDRAM_SET				0x80

#define HD44780_CLEAR					0x01

#define i2c_lcd_ADDR					0x27 //I2C LCD adres.

char i2c_lcd_status;

void i2c_lcd_write_status(void);
void i2c_lcd_data_part(char data_part);
void i2c_lcd_write(char data);
void i2c_lcd_write_instruction(char instruction);
void i2c_lcd_write_data(char data);
void i2c_lcd_write_text(char *text);
void i2c_lcd_clear(void);
void i2c_lcd_set_cursor(char x, char y);
void i2c_lcd_led_on(void);
void i2c_lcd_led_off(void);
void i2c_lcd_init(void);






#define TWIE                  (*(TWI_t *) 0x04A0) /* Two-Wire Interface */
/* Master Bus State */
typedef enum TWI_MASTER_BUSSTATE_enum
{
	TWI_MASTER_BUSSTATE_UNKNOWN_gc = (0x00<<0),  /* Unknown Bus State */
	TWI_MASTER_BUSSTATE_IDLE_gc = (0x01<<0),  /* Bus is Idle */
	TWI_MASTER_BUSSTATE_OWNER_gc = (0x02<<0),  /* This Module Controls The Bus */
	TWI_MASTER_BUSSTATE_BUSY_gc = (0x03<<0),  /* The Bus is Busy */
} TWI_MASTER_BUSSTATE_t;


typedef enum TWI_MASTER_CMD_enum
{
	TWI_MASTER_CMD_NOACT_gc = (0x00<<0),  /* No Action */
	TWI_MASTER_CMD_REPSTART_gc = (0x01<<0),  /* Issue Repeated Start Condition */
	TWI_MASTER_CMD_RECVTRANS_gc = (0x02<<0),  /* Receive or Transmit Data */
	TWI_MASTER_CMD_STOP_gc = (0x03<<0),  /* Issue Stop Condition */
} TWI_MASTER_CMD_t;

/* TWI - Two-Wire Interface */
/* TWI_MASTER.CTRLA  bit masks and bit positions */
#define TWI_MASTER_ENABLE_bm  0x08  /* Enable TWI Master bit mask. */
#define TWI_MASTER_ENABLE_bp  3  /* Enable TWI Master bit position. */
#define TWI_MASTER_WIEN_bm  0x10  /* Write Interrupt Enable bit mask. */
#define TWI_MASTER_WIEN_bp  4  /* Write Interrupt Enable bit position. */
#define TWI_MASTER_RIEN_bm  0x20  /* Read Interrupt Enable bit mask. */
#define TWI_MASTER_RIEN_bp  5  /* Read Interrupt Enable bit position. */
#define TWI_MASTER_INTLVL_gm  0xC0  /* Interrupt Level group mask. */
#define TWI_MASTER_INTLVL_gp  6  /* Interrupt Level group position. */
#define TWI_MASTER_INTLVL0_bm  (1<<6)  /* Interrupt Level bit 0 mask. */
#define TWI_MASTER_INTLVL0_bp  6  /* Interrupt Level bit 0 position. */
#define TWI_MASTER_INTLVL1_bm  (1<<7)  /* Interrupt Level bit 1 mask. */
#define TWI_MASTER_INTLVL1_bp  7  /* Interrupt Level bit 1 position. */

/* TWI_MASTER.CTRLB  bit masks and bit positions */
#define TWI_MASTER_SMEN_bm  0x01  /* Smart Mode Enable bit mask. */
#define TWI_MASTER_SMEN_bp  0  /* Smart Mode Enable bit position. */
#define TWI_MASTER_QCEN_bm  0x02  /* Quick Command Enable bit mask. */
#define TWI_MASTER_QCEN_bp  1  /* Quick Command Enable bit position. */
#define TWI_MASTER_TIMEOUT_gm  0x0C  /* Inactive Bus Timeout group mask. */
#define TWI_MASTER_TIMEOUT_gp  2  /* Inactive Bus Timeout group position. */
#define TWI_MASTER_TIMEOUT0_bm  (1<<2)  /* Inactive Bus Timeout bit 0 mask. */
#define TWI_MASTER_TIMEOUT0_bp  2  /* Inactive Bus Timeout bit 0 position. */
#define TWI_MASTER_TIMEOUT1_bm  (1<<3)  /* Inactive Bus Timeout bit 1 mask. */
#define TWI_MASTER_TIMEOUT1_bp  3  /* Inactive Bus Timeout bit 1 position. */

/* TWI_MASTER.CTRLC  bit masks and bit positions */
#define TWI_MASTER_CMD_gm  0x03  /* Command group mask. */
#define TWI_MASTER_CMD_gp  0  /* Command group position. */
#define TWI_MASTER_CMD0_bm  (1<<0)  /* Command bit 0 mask. */
#define TWI_MASTER_CMD0_bp  0  /* Command bit 0 position. */
#define TWI_MASTER_CMD1_bm  (1<<1)  /* Command bit 1 mask. */
#define TWI_MASTER_CMD1_bp  1  /* Command bit 1 position. */
#define TWI_MASTER_ACKACT_bm  0x04  /* Acknowledge Action bit mask. */
#define TWI_MASTER_ACKACT_bp  2  /* Acknowledge Action bit position. */

/* TWI_MASTER.STATUS  bit masks and bit positions */
#define TWI_MASTER_BUSSTATE_gm  0x03  /* Bus State group mask. */
#define TWI_MASTER_BUSSTATE_gp  0  /* Bus State group position. */
#define TWI_MASTER_BUSSTATE0_bm  (1<<0)  /* Bus State bit 0 mask. */
#define TWI_MASTER_BUSSTATE0_bp  0  /* Bus State bit 0 position. */
#define TWI_MASTER_BUSSTATE1_bm  (1<<1)  /* Bus State bit 1 mask. */
#define TWI_MASTER_BUSSTATE1_bp  1  /* Bus State bit 1 position. */
#define TWI_MASTER_BUSERR_bm  0x04  /* Bus Error bit mask. */
#define TWI_MASTER_BUSERR_bp  2  /* Bus Error bit position. */
#define TWI_MASTER_ARBLOST_bm  0x08  /* Arbitration Lost bit mask. */
#define TWI_MASTER_ARBLOST_bp  3  /* Arbitration Lost bit position. */
#define TWI_MASTER_RXACK_bm  0x10  /* Received Acknowledge bit mask. */
#define TWI_MASTER_RXACK_bp  4  /* Received Acknowledge bit position. */
#define TWI_MASTER_CLKHOLD_bm  0x20  /* Clock Hold bit mask. */
#define TWI_MASTER_CLKHOLD_bp  5  /* Clock Hold bit position. */
#define TWI_MASTER_WIF_bm  0x40  /* Write Interrupt Flag bit mask. */
#define TWI_MASTER_WIF_bp  6  /* Write Interrupt Flag bit position. */
#define TWI_MASTER_RIF_bm  0x80  /* Read Interrupt Flag bit mask. */
#define TWI_MASTER_RIF_bp  7  /* Read Interrupt Flag bit position. */