#include <avr/io.h>
#include "i2c.h"

void i2c_init(void) {
	DDRC &= ~((1 << PC4) | (1 << PC5)); // SDA/SCL入力モード
	PORTC |= (1 << PC4) | (1 << PC5); // プルアップ有効
	TWCR = 0x00; // TWIリセット
	TWSR = 0x00; // 
	TWBR = 0; // 56.7kHz(実測）
	TWDR = 0x00; // TWDRクリア
	TWCR = (1 << TWEN); // TWI有効
}

int i2c_start(unsigned char adr) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // START condition
    i2c_wait(TWINT);
    if ((i2c_status() != TW_START) && (i2c_status() != TW_REP_START)) return -1;

    TWDR = adr; // SLA+R/W をセット
    TWCR = (1 << TWINT) | (1 << TWEN); // 送信開始
    i2c_wait(TWINT);
    if ((i2c_status() != TW_MT_SLA_ACK) && (i2c_status() != TW_MR_SLA_ACK)) return -1;

    return 0;
}

void i2c_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

int i2c_send(unsigned char data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	i2c_wait(TWINT);
	return (i2c_status() == TW_MT_DATA_ACK) ? 0 : -1;
}

int i2c_recv(unsigned char ack) {
	TWCR = (1 << TWINT) | (ack ? (1 << TWEA) : 0) | (1 << TWEN);
	i2c_wait(TWINT);
	return TWDR;
}