// ATmega88P 7seg LED Clock
#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "i2c.h"

#define MODE_NORMAL     0
#define MODE_SET_HOUR   1
#define MODE_SET_MIN    2
#define MODE_SET_SEC    3
#define MODE_SAVE       4
#define MODE_DATE_DISP  5 // �N�����\��
#define MODE_SET_YEAR   6 // �N�ݒ�
#define MODE_SET_MONTH  7 // ���ݒ�
#define MODE_SET_DAY    8 // ���ݒ�

// �O���[�o���ϐ�
volatile uint8_t seg[8];         // 7�Z�O�{COM�\���f�[�^
volatile uint8_t mx = 0;         // ���d���X���b�g
volatile uint8_t hour = 12, min = 34, sec = 36;
volatile uint8_t year = 25, month = 6, day = 9; // �N�����iBCD�`���̉�2���j
volatile uint8_t mode = MODE_NORMAL;
volatile uint16_t switch_press_time = 0;
volatile uint8_t is_am = 1;      // AM/PM�t���O
volatile uint8_t is_24hour = 1;  // 24���ԕ\�L�t���O�i1:24����, 0:12���ԁj
volatile uint8_t rtc_update_flag; // INT0�����݃t���O
volatile uint8_t colon_blink_state = 0; // �R�����̓_�ŏ�ԁi0:����, 1:�_���j
volatile uint16_t colon_timer = 0;      // �R�����_�����ԃJ�E���^
volatile uint8_t led8_state = 0;        // LED8 (segDP) �̏�ԁi0:����, 1:�_���j
volatile uint16_t led8_timer = 0;       // LED8�_�����ԃJ�E���^
volatile uint8_t waiting_for_release = 0; // �X�C�b�`����ҋ@�t���O
volatile uint16_t switch2_hold_time = 0; // S2���������ԃJ�E���^
volatile uint8_t blink_enabled = 1;      // �_�ŗL���t���O
volatile uint16_t led7_timer = 0;        // LED7�_�����ԃJ�E���^
volatile uint16_t buzzer_timer = 0;      // �u�U�[�^�C�}�[�i100ms?0�j
volatile uint16_t date_display_timer = 0; // �N�����\����2�b�^�C�}�[
volatile uint8_t switch2_pressed = 0;     // S2������ԁi�N�������[�h�p�j
volatile uint8_t last_mode = MODE_NORMAL; // �O��̃��[�h�i�ۑ����p�j
volatile uint8_t led7_always_on = 0;     // LED7�펞�_���t���O

// �萔��`
#define BLINK_CYCLES       250  // 0.25s (4Hz)
#define COLON_CYCLES       500  // 0.5s
#define LED8_CYCLES        50   // 16ms
#define BUZZER_CYCLES      100  // 100ms
#define BUZZER_ON1_START   100  // 0-10ms
#define BUZZER_OFF1        80   // 10-90ms
#define BUZZER_ON2         20   // 90-100ms
#define BUZZER_OFF2        1    // 100ms-
#define COLON_MASK         0x1E // segC,D,E,F (1<<1)|(1<<2)|(1<<3)|(1<<4)
#define AM_MASK            (1<<6) // segA
#define PM_MASK            (1<<7) // segB
#define LED8_MASK          (1<<5) // segDP
#define LED7_MASK          (1<<0) // segG
#define DATE_DISP_TIME     2000 // �N�����\�����ԁi2�b�j

// �_�Ŕ͈̓e�[�u���i�J�nmx, �I��mx�j
static const uint8_t blink_range[9][2] = {
	{0, 6}, // MODE_NORMAL: �S���\��
	{4, 6}, // MODE_SET_HOUR: ���imx=4,5�j
	{2, 4}, // MODE_SET_MIN: ���imx=2,3�j
	{0, 2}, // MODE_SET_SEC: �b�imx=0,1�j
	{0, 6}, // MODE_SAVE: �S���\��
	{0, 6}, // MODE_DATE_DISP: �S���\���i�_�łȂ��j
	{4, 6}, // MODE_SET_YEAR: �N�imx=4,5�j
	{2, 4}, // MODE_SET_MONTH: ���imx=2,3�j
	{0, 2}  // MODE_SET_DAY: ���imx=0,1�j
};

// �֐��v���g�^�C�v
uint8_t mask(uint8_t num);
uint8_t dec2bcd(uint8_t v);
uint8_t bcd2dec(uint8_t v);
void rtc_init_full(void);
void rtc_read_time(volatile uint8_t *h, volatile uint8_t *m, volatile uint8_t *s);
void rtc_read_date(volatile uint8_t *y, volatile uint8_t *m, volatile uint8_t *d);
void rtc_write_date(uint8_t y, uint8_t m, uint8_t d);
void process_rtc_update(void);
void read_switches(void);
void set_rtc_time(uint8_t h, uint8_t m, uint8_t s);
void buzzer_start(void);
void buzzer_stop(void);

// 7�Z�O�����g�}�X�N
uint8_t mask(uint8_t num) {
	switch (num) {
		case 0:  return 0x21;
		case 1:  return 0x77;
		case 2:  return 0x2A;
		case 3:  return 0x26;
		case 4:  return 0x74;
		case 5:  return 0xA4;
		case 6:  return 0xA0;
		case 7:  return 0x35;
		case 8:  return 0x20;
		case 9:  return 0x24;
		case 99: return 0xFF;
		default: return 0xEF;
	}
}

// 10�i��BCD
uint8_t dec2bcd(uint8_t v) { return ((v/10)<<4)|(v%10); }
// BCD��10�i
uint8_t bcd2dec(uint8_t v) { return ((v>>4)*10)+(v&0x0F); }

// RTC-8564 ������
void rtc_init_full(void) {
	// RTC��~ (STOP=1)
	i2c_start(0xA2);
	i2c_send(0x00);
	i2c_send(0x20); // STOP=1
	i2c_stop();

	// Control2: �t���O�N���A�E�����L��
	i2c_start(0xA2);
	i2c_send(0x01);
	i2c_send(0x11); // AIE=1, TIE=1
	i2c_stop();

	// CLKOUT Frequency
	i2c_start(0xA2);
	i2c_send(0x0D);
	i2c_send(0x83); //FE=1,1Hz
	i2c_stop();

	// Timer Clock (1Hz)
	i2c_start(0xA2);
	i2c_send(0x0E);
	i2c_send(0x82); // TE=1, 1Hz
	i2c_stop();

	// Timer Value = 1
	i2c_start(0xA2);
	i2c_send(0x0F);
	i2c_send(0x01);
	i2c_stop();
	
	// RTC�ăX�^�[�g (STOP=0, TI/TP=1)
	i2c_start(0xA2);
	i2c_send(0x00);
	i2c_send(0x10); // STOP=0, TI/TP=1
	i2c_stop();
}

// RTC�����ǂݎ��
void rtc_read_time(volatile uint8_t *h, volatile uint8_t *m, volatile uint8_t *s) {
	i2c_start(0xA2);
	i2c_send(0x02);
	i2c_stop();

	i2c_start(0xA3);
	*s = bcd2dec(i2c_recv(1) & 0x7F); // �b��0x7F�Ń}�X�N����VL�r�b�g����
	*m = bcd2dec(i2c_recv(1) & 0x7F); // ����0x7F�Ń}�X�N
	*h = bcd2dec(i2c_recv(0) & 0x3F); // ����0x3F�Ń}�X�N
	i2c_stop();
}

// RTC�N�����ǂݎ��
void rtc_read_date(volatile uint8_t *y, volatile uint8_t *m, volatile uint8_t *d) {
	i2c_start(0xA2);
	i2c_send(0x05); // day���W�X�^����J�n
	i2c_stop();

	i2c_start(0xA3);
	*d = bcd2dec(i2c_recv(1) & 0x3F); // day (�}�X�N: 0x3F)
	i2c_recv(1); // weekday (�X�L�b�v)
	*m = bcd2dec(i2c_recv(1) & 0x1F); // months (�}�X�N: 0x1F)
	*y = bcd2dec(i2c_recv(0)); // years
	i2c_stop();
}

// RTC�N������������
void rtc_write_date(uint8_t y, uint8_t m, uint8_t d) {
	if (y > 99) y = 0; // �͈͕␳
	if (m > 12) m = 1;
	if (d > 31) d = 1;
	i2c_start(0xA2);
	i2c_send(0x05); // day���W�X�^����J�n
	i2c_send(dec2bcd(d)); // day
	i2c_send(0x00); // weekday (���g�p)
	i2c_send(dec2bcd(m)); // months (century=0)
	i2c_send(dec2bcd(y)); // years
	i2c_stop();
}

// RTC-8564��0D���W�X�^��ǂݏo���֐�
uint8_t rtc_read_reg0D(void) {
	i2c_start(0xA2); // RTC�������݃A�h���X
	i2c_send(0x0D); // 0D���W�X�^�w��
	i2c_stop();
	
	i2c_start(0xA3); // RTC�ǂݏo���A�h���X
	uint8_t data = i2c_recv(0); // 0D���W�X�^�ǂݏo��
	i2c_stop();
	
	return data;
}

// INT0���荞�݁F�t���O�Z�b�g�A�R������LED8�_���J�n�A�u�U�[�J�n
ISR(INT0_vect) {
	rtc_update_flag = 1;
	colon_blink_state = 1; // �R�����_���J�n
	colon_timer = 0;
	led8_state = 1;       // LED8�_���J�n
	led8_timer = 0;
	//buzzer_timer = 20; //���b�炷�ݒ�
	
	// 0���i00:00:00�j�܂���12���i12:00:00�j�Ńu�U�[�J�n
	if ((hour == 23 || hour == 11) && min == 59 && sec == 59) {
		buzzer_timer = 100; // �u�U�[100ms�i10ms����+80ms����+10ms�����j
	}
}

// RTC�X�V����
void process_rtc_update(void) {
	rtc_read_time(&hour, &min, &sec);
	is_am = (hour < 12);
}

// �u�U�[�J�n
void buzzer_start(void) {
	// CTC���[�h�iWGM01 = 1�j�AToggle OC0A on compare match�iCOM0A0 = 1�j
	TCCR0A = (1 << COM0A0) | (1 << WGM01);
	TCCR0B = (1 << CS00);  // �v���X�P�[�� 1
	// ��r��v�l�̐ݒ�i��4kHz�jFout = F_CPU/(2*Prescaler*(1+OCR0A)) = 1000000/(2*1*(1+124))?4000Hz
	OCR0A = 124;
}

// �u�U�[��~
void buzzer_stop(void) {
	TCCR0A = 0; // PWM����
	TCCR0B = 0; // �N���b�N��~
	PORTD &= ~(1<<PD6); // PD6��LOW
}

// �X�C�b�`�ǂݎ��
void read_switches(void) {
	static uint8_t last_s1 = 0, last_s2 = 0; // �����l��0�i�������j��
	uint8_t s1 = !(PINC & (1 << PC2)); // Switch1 (PC2)
	uint8_t s2 = !(PINC & (1 << PC3)); // Switch2 (PC3)
	static uint16_t auto_count_timer = 0; // �����J�E���g�p�^�C�}�[
	static uint16_t date_long_press_time = 0; // S2�������i�N�����ݒ�p�j
	static uint8_t skip_next_s2_release = 0; // S2������̒l�ύX���X�L�b�v����t���O
	static uint8_t skip_next_s1_release = 0; // S1������̃��[�h�J�ڂ��X�L�b�v����t���O

	// MODE_NORMAL: 2�b���������Ŏ����ݒ�AS1��12/24�؂�ւ��AS2�ŔN�����\��
	if (mode == MODE_NORMAL) {
		if (s1 && s2 && last_s1 && last_s2) {
			if (++switch_press_time >= 2000) { // 2�b���������Ŏ����ݒ�
				mode = MODE_SET_HOUR;
				switch_press_time = 0;
				waiting_for_release = 1; // �X�C�b�`����ҋ@
				skip_next_s2_release = 1; // ����S2������X�L�b�v
				skip_next_s1_release = 1; // ����S1������X�L�b�v
				rtc_read_time(&hour, &min, &sec);
				is_am = (hour < 12);
			}
			} else {
			switch_press_time = 0;
			// S1�P�Ɖ�����12/24���Ԑ؂�ւ��i����������j
			if (s1 == 0 && last_s1 == 1 && !s2) {
				is_24hour ^= 1;
				if (!is_24hour) led7_timer = 2000; // 24��12��LED7�_��2�b
				_delay_us(100); // �ȈՃf�o�E���X
			}
			// S2�P�Ɖ����ŔN�����\���i����������j
			if (s2 == 0 && last_s2 == 1 && !s1) {
				mode = MODE_DATE_DISP;
				date_display_timer = DATE_DISP_TIME; // 2�b�^�C�}�[�J�n
				rtc_read_date(&year, &month, &day); // RTC����N�����ǂݍ���
				_delay_us(100); // �ȈՃf�o�E���X
			}
		}
	}
	// MODE_DATE_DISP: S2�������ŔN�����ݒ�A2�b�Œʏ탂�[�h��
	else if (mode == MODE_DATE_DISP) {
		if (s2 && last_s2) {
			switch2_pressed = 1; // S2������
			if (++date_long_press_time >= 2000) { // 2�b�������ŔN�ݒ�
				mode = MODE_SET_YEAR;
				date_long_press_time = 0;
				waiting_for_release = 1; // �X�C�b�`����ҋ@
				skip_next_s2_release = 1; // ����S2������X�L�b�v
				rtc_read_date(&year, &month, &day); // �����l�ǂݍ���
			}
			} else {
			switch2_pressed = 0; // S2�����ꂽ
			date_long_press_time = 0;
		}
		if (!s2 && !last_s2) { // S2�������łȂ��ꍇ�A�^�C�}�[����
			if (date_display_timer > 0) date_display_timer--;
			if (date_display_timer == 0) mode = MODE_NORMAL; // 2�b�Œʏ탂�[�h
		}
	}
	// ����/�N�����ݒ胂�[�h
	else {
		// �X�C�b�`����ҋ@
		if (waiting_for_release) {
			if (s1 == 0 && s2 == 0) {
				waiting_for_release = 0;
				_delay_us(100); // �ȈՃf�o�E���X
			}
			return; // ����ҋ@���͏������X�L�b�v
		}
		// �ݒ葀��
		// S1: ���[�h�J�ځi����������j
		if (s1 == 0 && last_s1 == 1 && !skip_next_s1_release) {
			if (mode == MODE_SET_DAY) { // ���ݒ��ɕۑ����[�h��
				mode = MODE_SAVE;
				} else if (mode == MODE_SET_SEC) { // �b�ݒ��ɕۑ����[�h��
				mode = MODE_SAVE;
				} else {
				mode++; // ���̃��[�h�ցiMODE_DATE_DISP�̓X�L�b�v�j
				if (mode == MODE_DATE_DISP) mode = MODE_NORMAL;
			}
			if (mode == MODE_SAVE) {
				// �ۑ�����
				if (last_mode >= MODE_SET_HOUR && last_mode <= MODE_SET_SEC) {
					set_rtc_time(hour, min, sec); // ������������
					} else if (last_mode >= MODE_SET_YEAR && last_mode <= MODE_SET_DAY) {
					rtc_write_date(year, month, day); // �N������������
					_delay_us(100); // I2C�������݊�����ۏ�
				}
				mode = MODE_NORMAL;
				waiting_for_release = 1; // �ۑ���ɉ���ҋ@
				skip_next_s2_release = 1; // �ۑ����S2������X�L�b�v
				skip_next_s1_release = 1; // �ۑ����S1������X�L�b�v
			}
			switch2_hold_time = 0;
			_delay_us(100); // �ȈՃf�o�E���X
		}
		// S1�����̃X�L�b�v�t���O�����Z�b�g
		if (s1 == 0 && last_s1 == 1) {
			skip_next_s1_release = 0;
		}
		// S2: �l�ύX�i����������܂��͒������j- ����ҋ@���̓X�L�b�v
		if (!waiting_for_release) {
			if (s2 == 0 && last_s2 == 1 && !skip_next_s2_release) {
				if (mode == MODE_SET_HOUR) {
					hour = (hour + 1) % 24;
					is_am = (hour < 12);
					} else if (mode == MODE_SET_MIN) {
					min = (min + 1) % 60;
					} else if (mode == MODE_SET_SEC) {
					sec = (sec + 1) % 60;
					} else if (mode == MODE_SET_YEAR) {
					year = (year + 1) % 100;
					} else if (mode == MODE_SET_MONTH) {
					month = (month % 12) + 1; // 1-12
					} else if (mode == MODE_SET_DAY) {
					day = (day % 31) + 1; // 1-31
				}
				switch2_hold_time = 0;
				_delay_us(100); // �ȈՃf�o�E���X
			}
			// S2�������i10Hz�J�E���g�A�b�v�j
			if (s2 == 1) {
				switch2_hold_time++;
				if (switch2_hold_time >= 500) { // 0.5�b������
					if (++auto_count_timer >= 100) { // 100ms���Ɓi10Hz�j
						if (mode == MODE_SET_HOUR) {
							hour = (hour + 1) % 24;
							is_am = (hour < 12);
							} else if (mode == MODE_SET_MIN) {
							min = (min + 1) % 60;
							} else if (mode == MODE_SET_SEC) {
							sec = (sec + 1) % 60;
							} else if (mode == MODE_SET_YEAR) {
							year = (year + 1) % 100;
							} else if (mode == MODE_SET_MONTH) {
							month = (month % 12) + 1;
							} else if (mode == MODE_SET_DAY) {
							day = (day % 31) + 1;
						}
						auto_count_timer = 0;
					}
				}
				} else {
				switch2_hold_time = 0;
				auto_count_timer = 0;
			}
			// S2�����̃X�L�b�v�t���O�����Z�b�g
			if (s2 == 0 && last_s2 == 1) {
				skip_next_s2_release = 0;
			}
		}
		// �_�Ő���iS2��������̂ݓ_�ŁA�������Œ�~�j
		blink_enabled = (s2 == 0 && switch2_hold_time < 500); // S2��������̂ݓ_��
	}

	last_s1 = s1;
	last_s2 = s2;
	last_mode = mode;
}

// RTC�����ݒ�
void set_rtc_time(uint8_t h, uint8_t m, uint8_t s) {
	if (h > 23) h = 0; // �͈͊O�␳
	if (m > 59) m = 0;
	if (s > 59) s = 0;
	i2c_start(0xA2);
	i2c_send(0x02);
	i2c_send(dec2bcd(s)); // VL�r�b�g�͏������݂�0�ɂȂ�͂�
	i2c_send(dec2bcd(m));
	i2c_send(dec2bcd(h));
	i2c_stop();

	// VL�r�b�g�m�F
	i2c_start(0xA2); // RTC�������݃A�h���X
	i2c_send(0x02); // �b���W�X�^�w��
	i2c_stop();
	
	i2c_start(0xA3); // RTC�ǂݏo���A�h���X
	uint8_t sec_data = i2c_recv(0); // �b���W�X�^�ǂݏo��
	i2c_stop();
	
	led7_always_on = (sec_data & (1 << 7)) ? 1 : 0; // VL�r�b�g�Ɋ�Â��t���O�X�V
}

// Timer1���荞�݁F�_�Ő���ƃ^�C�}�[�Ǘ��A�X�C�b�`�ǂݎ��
ISR(TIMER1_COMPA_vect) {
	static uint16_t blink = 0;
	static uint8_t blink_state = 1;

	// �^�C�}�[�X�V�i1ms�P�ʁj
	if (++blink >= BLINK_CYCLES) { // 0.25s (4Hz)
		blink_state ^= 1;
		blink = 0;
	}
	if (colon_blink_state && ++colon_timer >= COLON_CYCLES) {
		colon_blink_state = 0;
		colon_timer = 0;
	}
	if (led8_state && ++led8_timer >= LED8_CYCLES) {
		led8_state = 0;
		led8_timer = 0;
	}
	if (led7_timer) led7_timer--;
	if (buzzer_timer) {
		uint8_t t = buzzer_timer--;
		if (t == BUZZER_ON1_START || t == BUZZER_ON2) buzzer_start();
		else if (t == BUZZER_OFF1 || t == BUZZER_OFF2) buzzer_stop();
	}

	// �X�C�b�`�ǂݎ����^�C�}�[1���Ŏ��s
	read_switches();

	// 7�Z�O�\���f�[�^�X�V
	if (mode == MODE_DATE_DISP || mode == MODE_SET_YEAR || mode == MODE_SET_MONTH || mode == MODE_SET_DAY) {
		// �N�����\���i25.06.09�j
		seg[0] = mask(day % 10); // ����̈�
		seg[1] = mask(day / 10); // ���\�̈�
		seg[2] = mask(month % 10) & ~LED8_MASK; // ����̈ʁi�h�b�g�_���j
		seg[3] = mask(month / 10); // ���\�̈�
		seg[4] = mask(year % 10) & ~LED8_MASK; // �N��̈ʁi�h�b�g�_���j
		seg[5] = mask(year / 10); // �N�\�̈�
		} else {
		// �����\��
		uint8_t display_hour;
		if (is_24hour) {
			display_hour = hour;
			} else {
			if (hour < 12) display_hour = hour; // �ߑO: 0?11
			else if (hour == 12) display_hour = hour; // �ߌ�12:00?12:59
			else display_hour = hour - 12; // �ߌ�01:00?11:59
		}
		if (mode == MODE_NORMAL || mode == MODE_SAVE) {
			seg[0] = mask(sec % 10);
			seg[1] = mask(sec / 10);
			seg[2] = mask(min % 10);
			seg[3] = mask(min / 10);
			seg[4] = mask(display_hour % 10);
			seg[5] = mask(display_hour / 10 ? display_hour / 10 : 99);
			} else if (mode == MODE_SET_HOUR) {
			seg[0] = mask(sec % 10);
			seg[1] = mask(sec / 10);
			seg[2] = mask(min % 10);
			seg[3] = mask(min / 10);
			seg[4] = mask(hour % 10); // �ݒ蒆��0?23��\��
			seg[5] = mask(hour / 10 ? hour / 10 : 99);
			} else if (mode == MODE_SET_MIN) {
			seg[0] = mask(sec % 10);
			seg[1] = mask(sec / 10);
			seg[2] = mask(min % 10);
			seg[3] = mask(min / 10);
			seg[4] = mask(hour % 10);
			seg[5] = mask(hour / 10 ? hour / 10 : 99);
			} else if (mode == MODE_SET_SEC) {
			seg[0] = mask(sec % 10);
			seg[1] = mask(sec / 10);
			seg[2] = mask(min % 10);
			seg[3] = mask(min / 10);
			seg[4] = mask(hour % 10);
			seg[5] = mask(hour / 10 ? hour / 10 : 99);
		}
	}

	// �_�Ő���: �K�v�ɉ�����seg[]������
	if (blink_enabled && mode != MODE_NORMAL && mode != MODE_SAVE && mode != MODE_DATE_DISP && !blink_state) {
		for (uint8_t i = blink_range[mode][0]; i < blink_range[mode][1]; i++) {
			seg[i] = 0xFF; // ����
		}
	}

	// COM6�i�R�����AAM/PM�ALED7/8�j�̍X�V
	uint8_t com = 0xFF;
	if (mode == MODE_NORMAL || mode == MODE_SET_HOUR || mode == MODE_SET_MIN || mode == MODE_SET_SEC || mode == MODE_SAVE) {
		if (mode != MODE_NORMAL || colon_blink_state) com &= ~COLON_MASK;
		com &= is_am ? ~AM_MASK : ~PM_MASK;
	}
	if (led8_state) com &= ~LED8_MASK;
	if (led7_timer || led7_always_on) com &= ~LED7_MASK; // LED7�펞�_���܂��̓^�C�}�[�_��
	seg[6] = com;
}

// Timer2���荞�݁F7�Z�O���d��
ISR(TIMER2_COMPA_vect) {
	// �S������
	PORTB = 0xFF;
	PORTD &= 0x4C; // seg,com low 0b01001100
	PORTC &= 0xFC; // seg low 0b11111100

	// �Z�O�����g�f�[�^�o��
	PORTB = seg[mx];
	// �N�����\�����Ƀh�b�g�iPB5�j��_���imx=2,4�j
	if (mode == MODE_DATE_DISP || mode == MODE_SET_YEAR || mode == MODE_SET_MONTH || mode == MODE_SET_DAY) {
		if (mx == 2 || mx == 4 || mx == 0) {
			PORTB &= ~_BV(PORTB5); // PB5��LOW�i�h�b�g�_���j
		}
	}
	// COM�s���Z�b�g
	switch (mx) {
		case 0: PORTD |= (1 << PD0); break; // COM0: �b��̈�
		case 1: PORTD |= (1 << PD1); break; // COM1: �b�\�̈�
		case 2: PORTC |= (1 << PC0); break; // COM2: ����̈�
		case 3: PORTC |= (1 << PC1); break; // COM3: ���\�̈�
		case 4: PORTD |= (1 << PD4); break; // COM4: ����̈�
		case 5: PORTD |= (1 << PD5); break; // COM5: ���\�̈�
		case 6: PORTD |= (1 << PD7); break; // COM6: �R�����EAM/PM�ELED7/8
	}

	// �C���f�b�N�X�X�V
	mx = (mx + 1) % 7;
}

int main(void) {
	// �s��������
	DDRB = 0xFF;
	DDRC = 0x33; //0b0011 0011
	DDRD = 0xFB; //PD2���� ����ȊO�͏o�� PD6��Timer0�Ő��� 0b1111 1011
	PORTC = 0;
	PORTD = 0;

	// Timer2: ���d����1000Hz
	TCCR2A = (1<<WGM21);
	TCCR2B = (1<<CS21);
	OCR2A = (F_CPU/(8*1000)-1); // ��1000Hz
	TIMSK2 = (1<<OCIE2A);

	// Timer1: 1ms�����i1000Hz�j
	TCCR1A = 0;
	TCCR1B = (1<<WGM12) | (1<<CS11); // CTC���[�h�A�v���X�P�[��8
	OCR1A = (F_CPU/8/1000) - 1; // 1ms = 1000000/8/1000 = 124
	TIMSK1 = (1<<OCIE1A);

	// INT0: �����������1Hz�X�V
	EICRA = (1<<ISC01);
	EIMSK = (1<<INT0);

	// �u�U�[�������i��~��ԁj
	buzzer_stop();
	
	//�ċN�����Ƀu�U�[��炷(20ms1��)
	//buzzer_timer = 100;

	// I2C & RTC
	i2c_init();
	
	// �b���W�X�^�i0x02�j��VL�r�b�g�m�F
	i2c_start(0xA2); // RTC�������݃A�h���X
	i2c_send(0x02); // �b���W�X�^�w��
	i2c_stop();
	
	i2c_start(0xA3); // RTC�ǂݏo���A�h���X
	uint8_t sec_data = i2c_recv(0); // �b���W�X�^�ǂݏo��
	i2c_stop();
	
	if (sec_data & (1 << 7)) { // VL�r�b�g�ibit7�j��1�̏ꍇ
		led7_always_on = 1; // LED7���펞�_��
		set_rtc_time(0, 0, 0); // ������00:00:00�ɐݒ�
		rtc_write_date(25, 1, 1); // �N������2025.01.01�ɐݒ�
	}
	
	// 0D���W�X�^��ǂݏo���A�}�X�N����
	uint8_t reg0D = rtc_read_reg0D() & 0x83; // 0x83�Ń}�X�N
	if (reg0D == 0x80) { // FD1=0, FD0=0, FE=1
		rtc_init_full();
		buzzer_timer = 20;	//�����d��������20ms�̃u�U�[��炷
		} else if(reg0D == 0x83){
		buzzer_timer = 100;	//�d����������20ms 2��̃u�U�[��炷
		} else {
		// CLKOUT Frequency
		i2c_start(0xA2);
		i2c_send(0x0D);
		i2c_send(0x83); //FE=1,1Hz
		i2c_stop();
		buzzer_timer = 20;
	}
	_delay_ms(1);
	
	process_rtc_update();
	sei();

	while (1) {
		if (rtc_update_flag && mode == MODE_NORMAL) { // �ݒ胂�[�h����RTC�ǂݍ��ݒ�~
			rtc_update_flag = 0;
			process_rtc_update();
		}
	}
}