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
#define MODE_DATE_DISP  5 // 年月日表示
#define MODE_SET_YEAR   6 // 年設定
#define MODE_SET_MONTH  7 // 月設定
#define MODE_SET_DAY    8 // 日設定

// グローバル変数
volatile uint8_t seg[8];         // 7セグ＋COM表示データ
volatile uint8_t mx = 0;         // 多重化スロット
volatile uint8_t hour = 12, min = 34, sec = 36;
volatile uint8_t year = 25, month = 6, day = 9; // 年月日（BCD形式の下2桁）
volatile uint8_t mode = MODE_NORMAL;
volatile uint16_t switch_press_time = 0;
volatile uint8_t is_am = 1;      // AM/PMフラグ
volatile uint8_t is_24hour = 1;  // 24時間表記フラグ（1:24時間, 0:12時間）
volatile uint8_t rtc_update_flag; // INT0割込みフラグ
volatile uint8_t colon_blink_state = 0; // コロンの点滅状態（0:消灯, 1:点灯）
volatile uint16_t colon_timer = 0;      // コロン点灯時間カウンタ
volatile uint8_t led8_state = 0;        // LED8 (segDP) の状態（0:消灯, 1:点灯）
volatile uint16_t led8_timer = 0;       // LED8点灯時間カウンタ
volatile uint8_t waiting_for_release = 0; // スイッチ解放待機フラグ
volatile uint16_t switch2_hold_time = 0; // S2長押し時間カウンタ
volatile uint8_t blink_enabled = 1;      // 点滅有効フラグ
volatile uint16_t led7_timer = 0;        // LED7点灯時間カウンタ
volatile uint16_t buzzer_timer = 0;      // ブザータイマー（100ms?0）
volatile uint16_t date_display_timer = 0; // 年月日表示の2秒タイマー
volatile uint8_t switch2_pressed = 0;     // S2押下状態（年月日モード用）
volatile uint8_t last_mode = MODE_NORMAL; // 前回のモード（保存時用）
volatile uint8_t led7_always_on = 0;     // LED7常時点灯フラグ

// 定数定義
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
#define DATE_DISP_TIME     2000 // 年月日表示時間（2秒）

// 点滅範囲テーブル（開始mx, 終了mx）
static const uint8_t blink_range[9][2] = {
	{0, 6}, // MODE_NORMAL: 全桁表示
	{4, 6}, // MODE_SET_HOUR: 時（mx=4,5）
	{2, 4}, // MODE_SET_MIN: 分（mx=2,3）
	{0, 2}, // MODE_SET_SEC: 秒（mx=0,1）
	{0, 6}, // MODE_SAVE: 全桁表示
	{0, 6}, // MODE_DATE_DISP: 全桁表示（点滅なし）
	{4, 6}, // MODE_SET_YEAR: 年（mx=4,5）
	{2, 4}, // MODE_SET_MONTH: 月（mx=2,3）
	{0, 2}  // MODE_SET_DAY: 日（mx=0,1）
};

// 関数プロトタイプ
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

// 7セグメントマスク
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

// 10進→BCD
uint8_t dec2bcd(uint8_t v) { return ((v/10)<<4)|(v%10); }
// BCD→10進
uint8_t bcd2dec(uint8_t v) { return ((v>>4)*10)+(v&0x0F); }

// RTC-8564 初期化
void rtc_init_full(void) {
	// RTC停止 (STOP=1)
	i2c_start(0xA2);
	i2c_send(0x00);
	i2c_send(0x20); // STOP=1
	i2c_stop();

	// Control2: フラグクリア・割込有効
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
	
	// RTC再スタート (STOP=0, TI/TP=1)
	i2c_start(0xA2);
	i2c_send(0x00);
	i2c_send(0x10); // STOP=0, TI/TP=1
	i2c_stop();
}

// RTC時刻読み取り
void rtc_read_time(volatile uint8_t *h, volatile uint8_t *m, volatile uint8_t *s) {
	i2c_start(0xA2);
	i2c_send(0x02);
	i2c_stop();

	i2c_start(0xA3);
	*s = bcd2dec(i2c_recv(1) & 0x7F); // 秒を0x7FでマスクしてVLビット除去
	*m = bcd2dec(i2c_recv(1) & 0x7F); // 分を0x7Fでマスク
	*h = bcd2dec(i2c_recv(0) & 0x3F); // 時を0x3Fでマスク
	i2c_stop();
}

// RTC年月日読み取り
void rtc_read_date(volatile uint8_t *y, volatile uint8_t *m, volatile uint8_t *d) {
	i2c_start(0xA2);
	i2c_send(0x05); // dayレジスタから開始
	i2c_stop();

	i2c_start(0xA3);
	*d = bcd2dec(i2c_recv(1) & 0x3F); // day (マスク: 0x3F)
	i2c_recv(1); // weekday (スキップ)
	*m = bcd2dec(i2c_recv(1) & 0x1F); // months (マスク: 0x1F)
	*y = bcd2dec(i2c_recv(0)); // years
	i2c_stop();
}

// RTC年月日書き込み
void rtc_write_date(uint8_t y, uint8_t m, uint8_t d) {
	if (y > 99) y = 0; // 範囲補正
	if (m > 12) m = 1;
	if (d > 31) d = 1;
	i2c_start(0xA2);
	i2c_send(0x05); // dayレジスタから開始
	i2c_send(dec2bcd(d)); // day
	i2c_send(0x00); // weekday (未使用)
	i2c_send(dec2bcd(m)); // months (century=0)
	i2c_send(dec2bcd(y)); // years
	i2c_stop();
}

// RTC-8564の0Dレジスタを読み出す関数
uint8_t rtc_read_reg0D(void) {
	i2c_start(0xA2); // RTC書き込みアドレス
	i2c_send(0x0D); // 0Dレジスタ指定
	i2c_stop();
	
	i2c_start(0xA3); // RTC読み出しアドレス
	uint8_t data = i2c_recv(0); // 0Dレジスタ読み出し
	i2c_stop();
	
	return data;
}

// INT0割り込み：フラグセット、コロン＆LED8点灯開始、ブザー開始
ISR(INT0_vect) {
	rtc_update_flag = 1;
	colon_blink_state = 1; // コロン点灯開始
	colon_timer = 0;
	led8_state = 1;       // LED8点灯開始
	led8_timer = 0;
	//buzzer_timer = 20; //毎秒鳴らす設定
	
	// 0時（00:00:00）または12時（12:00:00）でブザー開始
	if ((hour == 23 || hour == 11) && min == 59 && sec == 59) {
		buzzer_timer = 100; // ブザー100ms（10ms発音+80ms消音+10ms発音）
	}
}

// RTC更新処理
void process_rtc_update(void) {
	rtc_read_time(&hour, &min, &sec);
	is_am = (hour < 12);
}

// ブザー開始
void buzzer_start(void) {
	// CTCモード（WGM01 = 1）、Toggle OC0A on compare match（COM0A0 = 1）
	TCCR0A = (1 << COM0A0) | (1 << WGM01);
	TCCR0B = (1 << CS00);  // プリスケーラ 1
	// 比較一致値の設定（約4kHz）Fout = F_CPU/(2*Prescaler*(1+OCR0A)) = 1000000/(2*1*(1+124))?4000Hz
	OCR0A = 124;
}

// ブザー停止
void buzzer_stop(void) {
	TCCR0A = 0; // PWM無効
	TCCR0B = 0; // クロック停止
	PORTD &= ~(1<<PD6); // PD6をLOW
}

// スイッチ読み取り
void read_switches(void) {
	static uint8_t last_s1 = 0, last_s2 = 0; // 初期値を0（未押下）に
	uint8_t s1 = !(PINC & (1 << PC2)); // Switch1 (PC2)
	uint8_t s2 = !(PINC & (1 << PC3)); // Switch2 (PC3)
	static uint16_t auto_count_timer = 0; // 自動カウント用タイマー
	static uint16_t date_long_press_time = 0; // S2長押し（年月日設定用）
	static uint8_t skip_next_s2_release = 0; // S2解放時の値変更をスキップするフラグ
	static uint8_t skip_next_s1_release = 0; // S1解放時のモード遷移をスキップするフラグ

	// MODE_NORMAL: 2秒同時押しで時刻設定、S1で12/24切り替え、S2で年月日表示
	if (mode == MODE_NORMAL) {
		if (s1 && s2 && last_s1 && last_s2) {
			if (++switch_press_time >= 2000) { // 2秒同時押しで時刻設定
				mode = MODE_SET_HOUR;
				switch_press_time = 0;
				waiting_for_release = 1; // スイッチ解放待機
				skip_next_s2_release = 1; // 次のS2解放をスキップ
				skip_next_s1_release = 1; // 次のS1解放をスキップ
				rtc_read_time(&hour, &min, &sec);
				is_am = (hour < 12);
			}
			} else {
			switch_press_time = 0;
			// S1単独押しで12/24時間切り替え（立ち下がり）
			if (s1 == 0 && last_s1 == 1 && !s2) {
				is_24hour ^= 1;
				if (!is_24hour) led7_timer = 2000; // 24→12でLED7点灯2秒
				_delay_us(100); // 簡易デバウンス
			}
			// S2単独押しで年月日表示（立ち下がり）
			if (s2 == 0 && last_s2 == 1 && !s1) {
				mode = MODE_DATE_DISP;
				date_display_timer = DATE_DISP_TIME; // 2秒タイマー開始
				rtc_read_date(&year, &month, &day); // RTCから年月日読み込み
				_delay_us(100); // 簡易デバウンス
			}
		}
	}
	// MODE_DATE_DISP: S2長押しで年月日設定、2秒で通常モードへ
	else if (mode == MODE_DATE_DISP) {
		if (s2 && last_s2) {
			switch2_pressed = 1; // S2押下中
			if (++date_long_press_time >= 2000) { // 2秒長押しで年設定
				mode = MODE_SET_YEAR;
				date_long_press_time = 0;
				waiting_for_release = 1; // スイッチ解放待機
				skip_next_s2_release = 1; // 次のS2解放をスキップ
				rtc_read_date(&year, &month, &day); // 初期値読み込み
			}
			} else {
			switch2_pressed = 0; // S2離された
			date_long_press_time = 0;
		}
		if (!s2 && !last_s2) { // S2押下中でない場合、タイマー減少
			if (date_display_timer > 0) date_display_timer--;
			if (date_display_timer == 0) mode = MODE_NORMAL; // 2秒で通常モード
		}
	}
	// 時刻/年月日設定モード
	else {
		// スイッチ解放待機
		if (waiting_for_release) {
			if (s1 == 0 && s2 == 0) {
				waiting_for_release = 0;
				_delay_us(100); // 簡易デバウンス
			}
			return; // 解放待機中は処理をスキップ
		}
		// 設定操作
		// S1: モード遷移（立ち下がり）
		if (s1 == 0 && last_s1 == 1 && !skip_next_s1_release) {
			if (mode == MODE_SET_DAY) { // 日設定後に保存モードへ
				mode = MODE_SAVE;
				} else if (mode == MODE_SET_SEC) { // 秒設定後に保存モードへ
				mode = MODE_SAVE;
				} else {
				mode++; // 次のモードへ（MODE_DATE_DISPはスキップ）
				if (mode == MODE_DATE_DISP) mode = MODE_NORMAL;
			}
			if (mode == MODE_SAVE) {
				// 保存処理
				if (last_mode >= MODE_SET_HOUR && last_mode <= MODE_SET_SEC) {
					set_rtc_time(hour, min, sec); // 時刻書き込み
					} else if (last_mode >= MODE_SET_YEAR && last_mode <= MODE_SET_DAY) {
					rtc_write_date(year, month, day); // 年月日書き込み
					_delay_us(100); // I2C書き込み完了を保証
				}
				mode = MODE_NORMAL;
				waiting_for_release = 1; // 保存後に解放待機
				skip_next_s2_release = 1; // 保存後のS2解放をスキップ
				skip_next_s1_release = 1; // 保存後のS1解放をスキップ
			}
			switch2_hold_time = 0;
			_delay_us(100); // 簡易デバウンス
		}
		// S1解放後のスキップフラグをリセット
		if (s1 == 0 && last_s1 == 1) {
			skip_next_s1_release = 0;
		}
		// S2: 値変更（立ち下がりまたは長押し）- 解放待機中はスキップ
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
				_delay_us(100); // 簡易デバウンス
			}
			// S2長押し（10Hzカウントアップ）
			if (s2 == 1) {
				switch2_hold_time++;
				if (switch2_hold_time >= 500) { // 0.5秒長押し
					if (++auto_count_timer >= 100) { // 100msごと（10Hz）
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
			// S2解放後のスキップフラグをリセット
			if (s2 == 0 && last_s2 == 1) {
				skip_next_s2_release = 0;
			}
		}
		// 点滅制御（S2押下直後のみ点滅、長押しで停止）
		blink_enabled = (s2 == 0 && switch2_hold_time < 500); // S2押下直後のみ点滅
	}

	last_s1 = s1;
	last_s2 = s2;
	last_mode = mode;
}

// RTC時刻設定
void set_rtc_time(uint8_t h, uint8_t m, uint8_t s) {
	if (h > 23) h = 0; // 範囲外補正
	if (m > 59) m = 0;
	if (s > 59) s = 0;
	i2c_start(0xA2);
	i2c_send(0x02);
	i2c_send(dec2bcd(s)); // VLビットは書き込みで0になるはず
	i2c_send(dec2bcd(m));
	i2c_send(dec2bcd(h));
	i2c_stop();

	// VLビット確認
	i2c_start(0xA2); // RTC書き込みアドレス
	i2c_send(0x02); // 秒レジスタ指定
	i2c_stop();
	
	i2c_start(0xA3); // RTC読み出しアドレス
	uint8_t sec_data = i2c_recv(0); // 秒レジスタ読み出し
	i2c_stop();
	
	led7_always_on = (sec_data & (1 << 7)) ? 1 : 0; // VLビットに基づきフラグ更新
}

// Timer1割り込み：点滅制御とタイマー管理、スイッチ読み取り
ISR(TIMER1_COMPA_vect) {
	static uint16_t blink = 0;
	static uint8_t blink_state = 1;

	// タイマー更新（1ms単位）
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

	// スイッチ読み取りをタイマー1内で実行
	read_switches();

	// 7セグ表示データ更新
	if (mode == MODE_DATE_DISP || mode == MODE_SET_YEAR || mode == MODE_SET_MONTH || mode == MODE_SET_DAY) {
		// 年月日表示（25.06.09）
		seg[0] = mask(day % 10); // 日一の位
		seg[1] = mask(day / 10); // 日十の位
		seg[2] = mask(month % 10) & ~LED8_MASK; // 月一の位（ドット点灯）
		seg[3] = mask(month / 10); // 月十の位
		seg[4] = mask(year % 10) & ~LED8_MASK; // 年一の位（ドット点灯）
		seg[5] = mask(year / 10); // 年十の位
		} else {
		// 時刻表示
		uint8_t display_hour;
		if (is_24hour) {
			display_hour = hour;
			} else {
			if (hour < 12) display_hour = hour; // 午前: 0?11
			else if (hour == 12) display_hour = hour; // 午後12:00?12:59
			else display_hour = hour - 12; // 午後01:00?11:59
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
			seg[4] = mask(hour % 10); // 設定中は0?23を表示
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

	// 点滅制御: 必要に応じてseg[]を消灯
	if (blink_enabled && mode != MODE_NORMAL && mode != MODE_SAVE && mode != MODE_DATE_DISP && !blink_state) {
		for (uint8_t i = blink_range[mode][0]; i < blink_range[mode][1]; i++) {
			seg[i] = 0xFF; // 消灯
		}
	}

	// COM6（コロン、AM/PM、LED7/8）の更新
	uint8_t com = 0xFF;
	if (mode == MODE_NORMAL || mode == MODE_SET_HOUR || mode == MODE_SET_MIN || mode == MODE_SET_SEC || mode == MODE_SAVE) {
		if (mode != MODE_NORMAL || colon_blink_state) com &= ~COLON_MASK;
		com &= is_am ? ~AM_MASK : ~PM_MASK;
	}
	if (led8_state) com &= ~LED8_MASK;
	if (led7_timer || led7_always_on) com &= ~LED7_MASK; // LED7常時点灯またはタイマー点灯
	seg[6] = com;
}

// Timer2割り込み：7セグ多重化
ISR(TIMER2_COMPA_vect) {
	// 全灯消し
	PORTB = 0xFF;
	PORTD &= 0x4C; // seg,com low 0b01001100
	PORTC &= 0xFC; // seg low 0b11111100

	// セグメントデータ出力
	PORTB = seg[mx];
	// 年月日表示時にドット（PB5）を点灯（mx=2,4）
	if (mode == MODE_DATE_DISP || mode == MODE_SET_YEAR || mode == MODE_SET_MONTH || mode == MODE_SET_DAY) {
		if (mx == 2 || mx == 4 || mx == 0) {
			PORTB &= ~_BV(PORTB5); // PB5をLOW（ドット点灯）
		}
	}
	// COMピンセット
	switch (mx) {
		case 0: PORTD |= (1 << PD0); break; // COM0: 秒一の位
		case 1: PORTD |= (1 << PD1); break; // COM1: 秒十の位
		case 2: PORTC |= (1 << PC0); break; // COM2: 分一の位
		case 3: PORTC |= (1 << PC1); break; // COM3: 分十の位
		case 4: PORTD |= (1 << PD4); break; // COM4: 時一の位
		case 5: PORTD |= (1 << PD5); break; // COM5: 時十の位
		case 6: PORTD |= (1 << PD7); break; // COM6: コロン・AM/PM・LED7/8
	}

	// インデックス更新
	mx = (mx + 1) % 7;
}

int main(void) {
	// ピン初期化
	DDRB = 0xFF;
	DDRC = 0x33; //0b0011 0011
	DDRD = 0xFB; //PD2入力 それ以外は出力 PD6はTimer0で制御 0b1111 1011
	PORTC = 0;
	PORTD = 0;

	// Timer2: 多重化約1000Hz
	TCCR2A = (1<<WGM21);
	TCCR2B = (1<<CS21);
	OCR2A = (F_CPU/(8*1000)-1); // 約1000Hz
	TIMSK2 = (1<<OCIE2A);

	// Timer1: 1ms周期（1000Hz）
	TCCR1A = 0;
	TCCR1B = (1<<WGM12) | (1<<CS11); // CTCモード、プリスケーラ8
	OCR1A = (F_CPU/8/1000) - 1; // 1ms = 1000000/8/1000 = 124
	TIMSK1 = (1<<OCIE1A);

	// INT0: 立ち下がりで1Hz更新
	EICRA = (1<<ISC01);
	EIMSK = (1<<INT0);

	// ブザー初期化（停止状態）
	buzzer_stop();
	
	//再起動時にブザーを鳴らす(20ms1回)
	//buzzer_timer = 100;

	// I2C & RTC
	i2c_init();
	
	// 秒レジスタ（0x02）のVLビット確認
	i2c_start(0xA2); // RTC書き込みアドレス
	i2c_send(0x02); // 秒レジスタ指定
	i2c_stop();
	
	i2c_start(0xA3); // RTC読み出しアドレス
	uint8_t sec_data = i2c_recv(0); // 秒レジスタ読み出し
	i2c_stop();
	
	if (sec_data & (1 << 7)) { // VLビット（bit7）が1の場合
		led7_always_on = 1; // LED7を常時点灯
		set_rtc_time(0, 0, 0); // 時刻を00:00:00に設定
		rtc_write_date(25, 1, 1); // 年月日を2025.01.01に設定
	}
	
	// 0Dレジスタを読み出し、マスク処理
	uint8_t reg0D = rtc_read_reg0D() & 0x83; // 0x83でマスク
	if (reg0D == 0x80) { // FD1=0, FD0=0, FE=1
		rtc_init_full();
		buzzer_timer = 20;	//初期電源投入時20msのブザーを鳴らす
		} else if(reg0D == 0x83){
		buzzer_timer = 100;	//電源投入時に20ms 2回のブザーを鳴らす
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
		if (rtc_update_flag && mode == MODE_NORMAL) { // 設定モード中はRTC読み込み停止
			rtc_update_flag = 0;
			process_rtc_update();
		}
	}
}