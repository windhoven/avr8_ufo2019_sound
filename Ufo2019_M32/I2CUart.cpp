#include <avr/io.h>
#include <stdio.h>
#include <util/twi.h>
#include <string.h>
#include "I2CUart.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define _BV(bit) (1 << (bit))

// See section 8.4 of the datasheet for definitions
// of bits in the Line Control Register (LCR)
#define LCR_ENABLE_DIVISOR_LATCH 1 << 7
#define XTAL_FREQUENCY 14745600UL // oscillator

// See datasheet section 7.8 for configuring the
// "Programmable baud rate generator"
#define BAUD_RATE_DIVISOR(baud) (XTAL_FREQUENCY/(baud*16UL))

void twi_init(void) {
	/* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
#if defined(TWPS0)
	/* has prescaler (mega128 & newer) */
	TWSR = 0;
#endif
	TWBR = (F_CPU / 100000UL - 16) / 2;
}

void twi_wait(void) {
	while ((TWCR & _BV(TWINT)) == 0){}; /* wait for transmission */
}
int twi_send_start(void) {
	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); /* send start condition */
	twi_wait();
	return TW_STATUS;
}

int twi_send_byte(uint8_t out) {
	TWDR = out;
	TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */
	twi_wait();
	return TW_STATUS;
}

void twi_send_stop(void) {
	TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); /* send stop condition */
}

int twi_receive_bytes(uint8_t*buf,int len) {
	int rv = 0, twst;
	uint8_t twcr;
	for (twcr = _BV(TWINT) | _BV(TWEN) | _BV(TWEA) /* Note [13] */;
	     len > 0;
	     len--) {
		if (len == 1)
			twcr = _BV(TWINT) | _BV(TWEN); /* send NAK this time */
		TWCR = twcr;		/* clear int to start transmission */
		twi_wait();
		switch ((twst = TW_STATUS)) {
		case TW_MR_DATA_NACK:
			len = 0;		/* force end of loop */
			/* FALLTHROUGH */
		case TW_MR_DATA_ACK:
			*buf++ = TWDR;
			rv++;
			break;
		
		default:
			return -1;
		}
	}
	return rv;
}

uint8_t sc16is_address = 0x0;
void sc16is_set_addr(uint8_t address) {
	sc16is_address = address;
}
int sc16is_write(uint8_t reg,uint8_t data) {
	return sc16is_write_buf(reg,1,&data);
}
int sc16is_write_buf(uint8_t reg,int len,uint8_t*buf) {
	uint8_t done = 0,i;
	while (!done) {
		switch (twi_send_start()) {
		case TW_REP_START:
		case TW_START:
			break;
			
		/* on a multimaster bus and lost arbitration */
		case TW_MT_ARB_LOST:	
			continue;
			
		default:
			return -1;		/* error: not in start condition, do /not/ send stop condition */
		}
		
		/* send address+W */
		switch (twi_send_byte(sc16is_address | TW_WRITE)) {
		case TW_MT_SLA_ACK:
			break;
		
		default:
			done = 1;
		case TW_MT_SLA_NACK:	/* nack during select: device busy writing */
			printf("friggin nack\n\r");
		case TW_MT_ARB_LOST:	/* re-arbitrate */
			continue;		/* must send stop condition */
		}

		/* send register to write */
		switch (twi_send_byte(reg)) {
		case TW_MT_DATA_ACK:
			break;
		
		case TW_MT_DATA_NACK:
		default:
			done = 1;
		case TW_MT_ARB_LOST:
			continue;		/* must send stop condition */
		}

		/* send data for selected register */
		for (i=0;i<len;i++) {
			switch (twi_send_byte(buf[i])) {
			case TW_MT_DATA_ACK:
			case TW_MT_DATA_NACK:
				break;
			
			default:
				done = 1;
			case TW_MT_ARB_LOST:
				continue;		/* must send stop condition */
			}
		}

		/* send stop */
		twi_send_stop();
		return 1;
	}
	/* error section */
	twi_send_stop();
	return 0;
}

int sc16is_read_buf(uint8_t reg, int len, uint8_t*buf) {
	int rv = 0;
	uint8_t done = 0;

	while (!done) {
		/* must write register before reading data */
		switch (twi_send_start()) {
		case TW_REP_START:		/* OK, but should not happen */
		case TW_START:
			break;
			
		case TW_MT_ARB_LOST:
			continue;
			
		default:
			return -1;		/* error: not in start condition */
			    			/* NB: do /not/ send stop condition */
		}
		
		/* send SLA+W */
		switch (twi_send_byte(sc16is_address | TW_WRITE)) {
		case TW_MT_SLA_ACK:
			break;
		
		default:
			done = 1;
		case TW_MT_SLA_NACK:	/* nack during select: device busy writing */
		case TW_MT_ARB_LOST:	/* re-arbitrate */
			continue;
		}
	
		switch (twi_send_byte(reg)) {
		case TW_MT_DATA_ACK:
			break;
		
		case TW_MT_DATA_NACK:
		default:
			done = 1;
		case TW_MT_ARB_LOST:
			continue;
		}
		
		/* swap to master receiver mode */
		switch (twi_send_start()) {
		case TW_START:
		case TW_REP_START:
			break;
		
		default:
			done = 1;
		case TW_MT_ARB_LOST:
			continue;
		}
		
		/* send SLA+R */
		switch (twi_send_byte(sc16is_address | TW_READ)) {
		case TW_MR_SLA_ACK:
			break;
		
		case TW_MR_SLA_NACK:
		default:
			done = 1;
		case TW_MR_ARB_LOST:
			continue;
		}
		
		rv = twi_receive_bytes(buf,len);
		twi_send_stop();
		return rv;
	}
	/* error condition */	
	twi_send_stop();
	return -1;
}

int sc16is_read(uint8_t reg) {
	uint8_t buf;
	switch(sc16is_read_buf(reg,1,&buf)) {
	case 0:
	case -1:
	default:
		return 0;
	case 1:
		return buf;
	}
	return 0;
}

void sc16is_init(unsigned long baudrate) {
	twi_init();
	sc16is_configure(baudrate);
	
	if(!sc16is_connected()){ 
	  while(1) {
	    // Lock up if we fail to initialise I2C UART bridge.
	  } 
	}
	
	// The I2C UART bridge is now successfully initialised.
}

void sc16is_set_baud(unsigned long baudrate) {
	unsigned long divisor = BAUD_RATE_DIVISOR(baudrate);
	
	sc16is_write(LCR, LCR_ENABLE_DIVISOR_LATCH); // "Program baudrate"
	sc16is_write(DLL, (uint8_t)divisor);
	sc16is_write(DLM, (uint8_t)(divisor>>8)); 
}


/* Configure the settings of the UART. */
void sc16is_configure(unsigned long baudrate) {
	// TODO: Improve with use of constants and calculations.
	sc16is_set_baud(baudrate);
	
	sc16is_write(LCR, 0b00000011); // 8 data bit, 1 stop bit, no parity
	sc16is_write(FCR, 0x06); // reset TXFIFO, reset RXFIFO, non FIFO mode
	sc16is_write(FCR, 0x01); // enable FIFO mode   
}


int sc16is_connected(void) {
	// Perform read/write test to check if UART is working
	const char TEST_CHARACTER = 'H';
	char buf;
  
	sc16is_write(SPR, TEST_CHARACTER);
	buf = sc16is_read(SPR);
	return (buf == TEST_CHARACTER);
}

/* Get the number of bytes (characters) available for reading.

   This is data that's already arrived and stored in the receive
   buffer (which holds 64 bytes).
 */
uint8_t sc16is_available(void) {
	return sc16is_read(RXLVL);
}

/* Read byte from UART.
   Returns byte read or or -1 if no data available.
 */
uint8_t sc16is_uart_read(void) {
	while (!sc16is_available()) {
	}
	return sc16is_read(RHR);
}

/* Read into buffer from the remote UART */
int sc16is_uart_read_buf(uint8_t *buffer, int size) {
	// returns the number of bytes read
	return sc16is_read_buf(RHR,size,buffer);
}

/* Write byte to UART. */
void sc16is_uart_write(uint8_t value) {
	// Wait for space in TX buffer
	while (sc16is_read(TXLVL) == 0) ;
	sc16is_write(THR, value); 
}

/* Write string to UART. */
void sc16is_uart_write_str(char *str) {
	sc16is_uart_write_buf((uint8_t *) str, strlen(str));  
	while (sc16is_read(TXLVL) < 64) ;
}

/* Write buffer to UART. */
void sc16is_uart_write_buf(uint8_t *buffer, int size) {
	// break this into chunks of 16?
	sc16is_write_buf(THR,size,buffer);
}

/* Flush characters from SC16IS750 receive buffer. */
void sc16is_flush(void) {
	//sc16is_write(FCR, 0x06); // reset TXFIFO, reset RXFIFO, non FIFO mode
	//sc16is_write(FCR, 0x01); // enable FIFO mode   
	while(sc16is_available() > 0) {
		sc16is_uart_read();
	}
}

