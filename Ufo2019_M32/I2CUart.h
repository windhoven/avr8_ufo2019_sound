
#ifndef __SPIUART_H__
#define __SPIUART_H__

// SC16IS750 Register definitions
// TODO: Don't bit shift these here, do it in the read/write register routines
#define THR        0x00 << 3
#define RHR        0x00 << 3
#define IER        0x01 << 3
#define FCR        0x02 << 3
#define IIR        0x02 << 3
#define LCR        0x03 << 3
#define MCR        0x04 << 3
#define LSR        0x05 << 3
#define MSR        0x06 << 3
#define SPR        0x07 << 3
#define TXLVL      0x08 << 3
#define RXLVL      0x09 << 3
#define DLAB       0x80 << 3
#define IODIR      0x0A << 3
#define IOSTATE    0x0B << 3
#define IOINTMSK   0x0C << 3
#define IOCTRL     0x0E << 3
#define EFCR       0x0F << 3

#define DLL        0x00 << 3
#define DLM        0x01 << 3
#define EFR        0x02 << 3
#define XON1       0x04 << 3  
#define XON2       0x05 << 3
#define XOFF1      0x06 << 3
#define XOFF2      0x07 << 3

void twi_init(void);
int twi_send_start(void);
int twi_send_byte(uint8_t out);
void twi_send_stop(void);
int twi_receive_bytes(uint8_t*buf,int len);
void sc16is_set_addr(uint8_t address);
int sc16is_write(uint8_t reg,uint8_t data);
int sc16is_write_buf(uint8_t reg,int len,uint8_t*buf);
int sc16is_read_buf(uint8_t reg, int len, uint8_t*buf);
int sc16is_read(uint8_t reg);
void sc16is_init(unsigned long baudrate);
void sc16is_set_baud(unsigned long baudrate);
void sc16is_configure(unsigned long baudrate);
int sc16is_connected(void);
uint8_t sc16is_available(void);
uint8_t sc16is_uart_read(void);
int sc16is_uart_read_buf(uint8_t *buffer, int size);
void sc16is_uart_write(uint8_t value);
void sc16is_uart_write_str(char *str);
void sc16is_uart_write_buf(uint8_t*buffer, int size);
void sc16is_flush(void);
#endif 
