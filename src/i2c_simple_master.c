/**
 * \file
 *
 * \brief I2C Simple master driver.
 *
 (c) 2018 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms,you may use this software and
    any derivatives exclusively with Microchip products.It is your responsibility
    to comply with third party license terms applicable to your use of third party
    software (including open source software) that may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 */

/**
 * \defgroup doc_driver_i2c_simple_master I2C Simple Master Driver
 * \ingroup doc_driver_i2c
 *
 * \section doc_driver_i2c_simple_master_rev Revision History
 * - v0.0.0.1 Initial Commit
 *
 *@{
 */

#include <i2c_master.h>
#include <i2c_simple_master.h>

#include <i2c_types.h>//My code

#define slave_adr 0x4f//my code
#define slave_reg_adr 0x0// my code

//************************************************************************my code
uint8_t      read_data[2];

/** Structure passed into read_handler to describe the actions to be performed by the handler */
typedef struct {
	uint8_t *data;
	uint8_t  size;
} transfer_descriptor_t;


i2c_operations_t read_handler(void *d)
{
	transfer_descriptor_t *desc = (transfer_descriptor_t *)d;
	I2C_0_set_buffer((void *)desc->data, desc->size);
	// Set callback to terminate transfer and send STOP after read is complete
	I2C_0_set_data_complete_callback(i2c_cb_return_stop, NULL);
	return i2c_restart_read;  // Send REPEATED START before read
}




static i2c_operations_t I2C_0_wr1RegCompleteHandler(void *p)
{
	I2C_0_set_buffer(p, 1);
	I2C_0_set_data_complete_callback(NULL, NULL);
	return i2c_continue;
}

void I2C_0_write1ByteRegister(i2c_address_t address, uint8_t reg, uint8_t data)
{
	while (!I2C_0_open(address))
		; // sit here until we get the bus..
	I2C_0_set_data_complete_callback(I2C_0_wr1RegCompleteHandler, &data);
	I2C_0_set_buffer(&reg, 1);
	I2C_0_set_address_nack_callback(i2c_cb_restart_write, NULL); // NACK polling?
	I2C_0_master_write();
	while (I2C_BUSY == I2C_0_close())
		; // sit here until finished.
}


void I2C_0_writeNBytes(i2c_address_t address, void *data, size_t len)
{
	while (!I2C_0_open(address))
		; // sit here until we get the bus..
	I2C_0_set_buffer(data, len);
	I2C_0_set_address_nack_callback(i2c_cb_restart_write, NULL); // NACK polling?
	I2C_0_master_write();
	while (I2C_BUSY == I2C_0_close())
		; // sit here until finished.
}

static i2c_operations_t I2C_0_rd1RegCompleteHandler(void *p)
{
	I2C_0_set_buffer(p, 1);
	I2C_0_set_data_complete_callback(NULL, NULL);
	return i2c_restart_read;
}

uint8_t I2C_0_read1ByteRegister(i2c_address_t address, uint8_t reg)
{
	uint8_t     d2 = 42;
	i2c_error_t e;
	int         x;

	for (x = 2; x != 0; x--) {
		while (!I2C_0_open(address))
			; // sit here until we get the bus..
		I2C_0_set_data_complete_callback(I2C_0_rd1RegCompleteHandler, &d2);
		I2C_0_set_buffer(&reg, 1);
		I2C_0_set_address_nack_callback(i2c_cb_restart_write, NULL); // NACK polling?
		I2C_0_master_write();
		while (I2C_BUSY == (e = I2C_0_close()))
			; // sit here until finished.
		if (e == I2C_NOERR)
			break;
	}

	return d2;
}

static i2c_operations_t I2C_0_rd2RegCompleteHandler(void *p)
{
	I2C_0_set_buffer(p, 2);
	I2C_0_set_data_complete_callback(NULL, NULL);
	return i2c_restart_read;
}

uint16_t I2C_0_read2ByteRegister(i2c_address_t address, uint8_t reg)
{
	// result is little endian
	uint16_t result;

	while (!I2C_0_open(address))
		; // sit here until we get the bus..
	I2C_0_set_data_complete_callback(I2C_0_rd2RegCompleteHandler, &result);
	I2C_0_set_buffer(&reg, 1);
	I2C_0_set_address_nack_callback(i2c_cb_restart_write, NULL); // NACK polling?
	I2C_0_master_write();
	while (I2C_BUSY == I2C_0_close())
		; // sit here until finished.

	return (result << 8 | result >> 8);
}

/****************************************************************/
static i2c_operations_t I2C_0_wr2RegCompleteHandler(void *p)
{
	I2C_0_set_buffer(p, 2);
	I2C_0_set_data_complete_callback(NULL, NULL);
	return i2c_continue;
}

void I2C_0_write2ByteRegister(i2c_address_t address, uint8_t reg, uint16_t data)
{
	while (!I2C_0_open(address))
		; // sit here until we get the bus..
	I2C_0_set_data_complete_callback(I2C_0_wr2RegCompleteHandler, &data);
	I2C_0_set_buffer(&reg, 1);
	I2C_0_set_address_nack_callback(i2c_cb_restart_write, NULL); // NACK polling?
	I2C_0_master_write();
	while (I2C_BUSY == I2C_0_close())
		; // sit here until finished.
}

/****************************************************************/
typedef struct {
	size_t len;
	char * data;
} I2C_0_buf_t;

static i2c_operations_t I2C_0_rdBlkRegCompleteHandler(void *p)
{
	I2C_0_set_buffer(((I2C_0_buf_t *)p)->data, ((I2C_0_buf_t *)p)->len);
	I2C_0_set_data_complete_callback(NULL, NULL);
	return i2c_restart_read;
}

void I2C_0_readDataBlock(i2c_address_t address, uint8_t reg, void *data, size_t len)
{
	// result is little endian
	I2C_0_buf_t d;
	d.data = data;
	d.len  = len;

	while (!I2C_0_open(address))
		; // sit here until we get the bus..
	I2C_0_set_data_complete_callback(I2C_0_rdBlkRegCompleteHandler, &d);
	I2C_0_set_buffer(&reg, 1);
	I2C_0_set_address_nack_callback(i2c_cb_restart_write, NULL); // NACK polling?
	I2C_0_master_write();
	while (I2C_BUSY == I2C_0_close())
		; // sit here until finished.
}

void I2C_0_readNBytes(i2c_address_t address, void *data, size_t len)
{
	while (!I2C_0_open(address))
		; // sit here until we get the bus..
	I2C_0_set_buffer(data, len);
	I2C_0_master_read();
	while (I2C_BUSY == I2C_0_close())
		; // sit here until finished.
}


//**************************************************************************************************************************************************************************************//
/*This transfer sequence is typically done to first write to the slave the address in
the slave to read from, thereafter to read N bytes from this address.
*/
void do_transfer(uint8_t adr, uint8_t *data, uint8_t size)
{
transfer_descriptor_t d = {data, size};
while (!I2C_0_open(slave_adr))
; // sit here until we get the bus..
// This callback specifies what to do after the first write operation has completed
// The parameters to the callback are bundled together in the aggregate data type d.
I2C_0_set_data_complete_callback((void *)read_handler, &d);
// If we get an address NACK, then try again by sending SLA+W
I2C_0_set_address_nack_callback((void *)i2c_cb_restart_write, NULL);
// Transmit one byte
I2C_0_set_buffer((void *)&adr, 1);
// Start a Write operation
I2C_0_master_operation(false);
while (I2C_BUSY == I2C_0_close())
; // sit here until the entire chained operation has finished
}

