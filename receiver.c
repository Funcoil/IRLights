#define F_CPU 1000000UL                                    /* Clock Frequency = 1Mhz */

#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> 
#include <avr/sleep.h>
#include <avr/eeprom.h> // For debugging

#include "common.h"

#define UCSRB UCSR0B

#define RED_BUTTON	0b00010000		// PB4 (physical pin 3); INPUT;
#define GREEN_BUTTON	0b00001000		// PB1 (physical pin 6); INPUT;
#define IR_INPUT	0b00000001		// PB0 (physical pin 5; PCINT0); INPUT;
						// TSOP 4838 is connected here
#define INTERNAL_PULLUPS GREEN_BUTTON | RED_BUTTON
#define ALLOWED_BITS (RED_LED | GREEN_LED) // IR controllers are allowed to change LEDs

#define TIMER_SPEED ((1<<CS02) | (1<<CS00)) // 1/256 MHz
#define BASE_LEN 6 // Length of shortest pulse in clock ticks
#define TOLERANCE 1 / 3

//#define DEBUG_STATES

#ifdef DEBUG_PULSE_LENGTHS
	#define DEBUG
#endif

#ifdef DEBUG_STATES
	#define DEBUG
#endif

volatile char run;
volatile char firsttime;
volatile char bit_state = 0;
volatile char byte_state = 0;
volatile char recvd_data = 0; // Received bits
volatile char recvd_bit = 0;  // On which bit position we are
volatile int last_pulse_length;
volatile uint8_t *mem_pos = 0;

#ifdef DEBUG
void append_byte(char byte) {
	if(mem_pos < (uint8_t *)64) {
		eeprom_write_byte((uint8_t *)mem_pos, byte);
		++mem_pos;
	}
}
#endif

signed char rangecmp(int time, int length) {
	if(time < length * BASE_LEN - BASE_LEN * TOLERANCE) return -1;
	if(time > length * BASE_LEN + BASE_LEN * TOLERANCE) return 1;
	return 0;
}

void process_byte() {
	// Byte states:
	// 0 - waiting for device address
	// 1 - our device address set
	// 2 - ignore bytes until reset
	switch(byte_state) {
		case 0:
			if(recvd_data == DEVICE_ADDRESS) {
				byte_state = 1;
			} else {
				byte_state = 0;
			}
			break;
		case 1:
			PORTB = (PORTB & ~ALLOWED_BITS) | (recvd_data & ALLOWED_BITS); // Set outputs
			++byte_state;
			break;
	}
}

void reset_bit_state() {
	recvd_data = 0;
	recvd_bit = 0;
	bit_state = 0;
	byte_state = 0;
}

void read_bit(int ticks) {
	signed char r = rangecmp(ticks, 1);
	signed char r2 = rangecmp(ticks, 2);
	if(r < 0 || r2 > 0) { // Protocol error - reset state
		reset_bit_state();
	}

	if(!(PINB & IR_INPUT)) { // There was space
		if(!r && !rangecmp(last_pulse_length, 2)) { // Correct signal of bit "1"
			recvd_data <<= 1;
			recvd_data |= 1;
			++recvd_bit;
		} else if(!r2 && !rangecmp(last_pulse_length, 1)) {// Correct signal of bit "0"
			recvd_data <<= 1;
			++recvd_bit;
		} else { // error
			reset_bit_state();
		}
		if(recvd_bit == 8) { // We've got whole byte
			process_byte();
			// NOT doing reset_bit_state, because there may come new data
			recvd_data = 0;
			recvd_bit = 0;
		}

	} else { // There was signal
		last_pulse_length = ticks;
	}
}

ISR(PCINT0_vect) {
	// Test if buttons are hold down
	if(!(PINB & GREEN_BUTTON)) {
		PORTB = (PORTB & ~RED_LED) | GREEN_LED;
		return;
	}
	if(!(PINB & RED_BUTTON)) {
		PORTB = (PORTB & ~GREEN_LED) | RED_LED;
		return;
	}
	int ticks = TCNT0; // Savetimer value
	TCNT0 = 0;	    // Reset timer
	if(!(TCCR0B & TIMER_SPEED)) {
		TCCR0B |= TIMER_SPEED; // Start timer (in case it's not running)
		return;
	}

#ifdef DEBUG_PULSE_LENGTHS
	if(mem_pos < (uint8_t *)64) {
		eeprom_write_byte((uint8_t *)mem_pos, ticks);
		++mem_pos;
#ifdef VISUAL_DEBUG
		PORTB |= GREEN_LED;
	} else PORTB &= ~GREEN_LED;
#else
	}
#endif
#endif

	switch(bit_state) {
		case 0:
			if((PINB & IR_INPUT) && !rangecmp(ticks, 3)) ++bit_state;
#ifdef DEBUG_STATES
			append_byte(bit_state);
#endif
			break;
		case 1:
			if(!(PINB & IR_INPUT) && !rangecmp(ticks, 3))
				++bit_state;
			else
				bit_state = 0;
#ifdef DEBUG_STATES
			append_byte(bit_state);
#endif
			break;
		case 2:
			read_bit(ticks);
#ifdef DEBUG_STATES
			append_byte(byte_state);
			append_byte(recvd_bit);
			append_byte(recvd_data);
#endif
			break;
	}
#ifdef VISUAL_DEBUG
	PORTB = (PORTB & ~RED_LED) | ((~PINB & IR_INPUT) << 3);
#endif
}

ISR(TIM0_OVF_vect) {
	TCCR0B &= ~((1 << CS00) | (1 << CS01) | (1 << CS02)); // Stop timer
	TCNT0 = 0; // Too long - reset timer

#ifdef DEBUG_PULSE_LENGTHS
	if(mem_pos < (uint8_t *)64) { // Log overflow
		eeprom_write_byte((uint8_t *)mem_pos, 0);
		++mem_pos;
	}
#endif

}

int main() {
	// ================== IO Setup =================

	DDRB = GREEN_LED | RED_LED;  // Set IO pins. GREEN_LED and RED_LED are outputs
	PORTB = INTERNAL_PULLUPS; // Init internal pullups

	PORTB |= GREEN_LED | RED_LED;
	_delay_ms(250);
	PORTB &= ~(GREEN_LED | RED_LED);

	// ================ Timer Setup ================

	TCCR0A = 0; // Normal mode
	//TCCR0B |= (1 << CS00); // Configure prescaler to 1MHz (one tick == one microsecond)
	TIMSK0 |= (1 << TOIE0); // Enable overflow interrupt
	TCNT0 = 0;

	// ============== Interrupt Setup ==============
	
	MCUCR |= (1<<ISC00); // PCINT0 is triggered on any edge
	PCMSK = IR_INPUT | RED_BUTTON | GREEN_BUTTON; // Enable interrupt on IR_INPUT
	GIMSK |= (1<<PCIE); // enable pin change interrupts

	//Enable global interrupts
	sei();

	run = 1;
	while (run) {
	}

	sleep_enable();
	sleep_cpu();

	return 0;
}
