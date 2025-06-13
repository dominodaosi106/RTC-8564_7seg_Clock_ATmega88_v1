#ifndef I2C_H
#define I2C_H

#include <util/twi.h>

#define i2c_wait(p) loop_until_bit_is_set(TWCR, p)
#define i2c_status() (TWSR & 0xf8)

extern void i2c_init(void);
extern int i2c_start(unsigned char adr);
extern void i2c_stop(void);
extern int i2c_send(unsigned char data);
extern int i2c_recv(unsigned char ack);

#endif