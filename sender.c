/* IR Sender
 * Author: Martin Habovštiak <martin.habovstiak@gmail.com>
 * This document is published under terms and conditions of GPL
 * (Do what the f*** you want public license)
 *
 * This file is part of IR sender/receiver project made for UPeCe Bratislava.
 * Project consists of one sender with ATTiny13, three buttons and one IR diode
 * (possibly connected using transistor) which transmits commands to IR receiver
 * depending on which button is pressed. (Exactly like TV remote control).
 * IR receiver then turns on/off desired LEDs. (For example green for "Free
 * sign and red for "Do not disturb" sign)
 *
 * Receiver code, schematics and other documentation should be included with
 * this project
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/wdt.h>

#include "common.h"

#define IR_LED 0b00000001

// Buttons are on physical pins 2, 3 and 6 mapped to PB3, PB4 and PB1
#define GREEN_BUTTON	0b00010000
#define RED_BUTTON	0b00001000
#define OFF_BUTTON	0b00000010
#define BUTTONS (GREEN_BUTTON | RED_BUTTON | OFF_BUTTON)

#define BASE_LEN 4096 // in microseconds TODO: try 6144
#define CARRIER_PERIOD 13 // in microseconds
#define CYCLES_PER_PULSE BASE_LEN / (CARRIER_PERIOD * 2)
#define DELAY_BETWEEN_SIGNALS 255 // in microseconds

#define ANTIBUG_DELAY 100

#define TIMER_PRESCALE (1<<CS00) // No prescaling

// Uncoment do debug pulse lengths
//#define DEBUG_PULSES

/* Determines which command to send depending on pressed buttons
 * \return Byte representing command to be sent to remote device.
 * \return 255 if no button is pressed.
 */
uint8_t get_cmd() {
	// Prevent problems, when pins are changed during code execution
	uint8_t tmp = PINB;

	// Test which button is pressed
	if(!(tmp & OFF_BUTTON)) return 0;
	if(!(tmp & RED_BUTTON)) return RED_LED;
	if(!(tmp & GREEN_BUTTON)) return GREEN_LED;

	// No button pressed
	return 255;
}

/*! Sends one logic pulse modulated on frequency based on CARRIER_PERIOD (mostly 38 KHz)
 * \param state If true, pulse is actually sent, if false, function just delays
 */
void onePulse(uint8_t state) 
{ 
	int i;
	// One pulse is made of CYCLES_PER_PULSE cycles
	for(i = 0; i < CYCLES_PER_PULSE; i++) 
	{ 
		// Turn on IR LED if we are sending pulse
		if(state) PORTB |= IR_LED;

		// Delay 1 / (modulating frequency * 2) seconds
		_delay_us(CARRIER_PERIOD);

		// Turn off IR LED
		PORTB &= ~IR_LED;

		// Delay 1 / (modulating frequency * 2) seconds
		// With first delay it's 1 / modulating frequency
		_delay_us(CARRIER_PERIOD); 
	}
} 

/*! Sends given byte bit-by-bit. Most significant bit first
 * \param byte Data to be sent
 */
void sendByte(uint8_t byte) 
{
	uint8_t i;
	for(i = 0; i < 8; ++i) // 8 bits of byte
	{ 
		// "1" = 1T on + 3T off
		// "0" = 1T on + 1T off 
		onePulse(1); // Preceding pulse
		onePulse(byte & 0b10000000); // Actual bit
		onePulse(0); // Ending space
		byte <<= 1; // shift bite, so we have next bit on first position
	} 
}

ISR(PCINT0_vect) {
}

/* Main function is executed when microcontroller is powered on or after reset
 */
int main() {
	// Shut down unneccesary stuff
	cli(); // just in case
	ADCSRA = 0;
	ACSR = (1<<ACD);
	wdt_reset();
	MCUSR &= ~(1<<WDRF);
	WDTCR |= (1<<WDCE) | (1<<WDE);
	WDTCR = 0x00; // Shut down watchdog

	// Set Sleep mode to power down
	MCUCR |= (1<<SM1);

	// ================== IO Setup =================

	// IR_LED is the only output we use
	DDRB = IR_LED;

	// Setup internal pullup resistors
	// Pullup resistors filter out electrical noise.
	// For simplicity, we use internal pullups but we have to set them up
	// Internal pullups are set by writting ones to PORTB inputs.
	// (same as setting outpus high)
	// We're just oring all input buttons
	PORTB = GREEN_BUTTON | RED_BUTTON | OFF_BUTTON;

	// ================ Timer Setup ================
	// Count up to CARRIER_PERIOD
	/*
	OCR0A = CARRIER_PERIOD;
	// Togle output pin on match and enable PWM mode
	TCCR0A = (1<<COM0A0) | (1<<WGM00);
	TCCR0B = (1<<WGM02); // We don't want tu run timer yet
*/
	// ============== Interrupt Setup ==============

	// Enable interrupts on button presses
	PCMSK = BUTTONS;

	// Enable timer interrupt on B compare register
	//TIMSK0 |= OCIE0B;

	// enable pin change interrupts
	GIMSK |= (1<<PCIE);

	//Enable global interrupts
	sei();

	// Endless loop makes sure controller can always send commands
	while(1) {
#ifdef DEBUG_PULSES
		// For debugging pulse lengths
		onePulse(1);
		onePulse(0);
#else
		cli();
		// Find out cmd
		uint8_t cmd = get_cmd();

		// If any button was actually pressed
		if(cmd < 255) {
			sei();
			// Send one pulse and one space to wake up processor,
			// so it can quickly react on our request
			onePulse(1);
			onePulse(0);
			// Send preamble
			// Preamble is made of one pulse and
			// one space each three times longer than BASE_LEN
			uint8_t i;
			for(i = 0; i < 3; ++i) onePulse(1);
			for(i = 0; i < 3; ++i) onePulse(0);

			// Send device address
			// Device address identifies device
			// within our protocol.
			sendByte(DEVICE_ADDRESS);

			// Send command
			// Command tells device what to do.
			// In this case, we are telling receiver, which output
			// pins should go HIGH.
			// (our byte will be masked and written to PORTB)
			sendByte(cmd);

			// Send end bit
			// Communication must end with pulse of any reasonable length.
			// We use BASE_LEN for convenience.
			onePulse(1);

			// Delay before next communication
			// Communication sessions should be divided with space of
			// reasonable length. We use 50 ms
			_delay_ms(DELAY_BETWEEN_SIGNALS);
		} else {
			// Turn LED off in case it is still on.
			// Delay is needed due to unknown reason (else it's not reliable)
			// If anynone knows better solution, please let us know at martin.habovstiak@gmail.com
			_delay_ms(ANTIBUG_DELAY);
			PORTB &= ~IR_LED;
			_delay_ms(ANTIBUG_DELAY);

			// "Magic sleep routine"
			sleep_enable();
#ifdef sleep_bod_disable
			sleep_bod_disable(); // Disable brown out detection, if present.
#endif
			sei(); // Turn on interrupts (to be able to wake up CPU)
			sleep_cpu(); // Sleep (Power down)
			sleep_disable(); // Prevent sleeping after CPU wakes up
		}
#endif
	}

	// To avoid compiler warnings
	return 0;
}
