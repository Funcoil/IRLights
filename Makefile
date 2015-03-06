#default values
CONFIG=./avrdude.conf
PORT=/dev/ttyUSB0
BAUDRATE=19200
PROCESSOR=attiny13
PROGRAMMER=stk500v1
VERBOSE=
CC=avr-gcc

all: receiver.hex sender.hex
	@echo "Compiled. Use make flash-sender or make flash-receiver to program the microcontroler"

%.hex: %.elf
	avr-objcopy -j .text -j .data -O ihex $< $@

%.elf: %.c
	$(CC) -W -Wall -mmcu=$(PROCESSOR) -Os -o $@ $<

%.s: %.c
	$(CC) -W -Wall -mmcu=$(PROCESSOR) -S -Os -o $@ $<

flash-%: %.hex
	avrdude $(VERBOSE) -C $(CONFIG) -P $(PORT) -b $(BAUDRATE) -p $(PROCESSOR) -c $(PROGRAMMER) -U flash:w:$<:i

read-fuses:
	avrdude $(VERBOSE) -C $(CONFIG) -P $(PORT) -b $(BAUDRATE) -p $(PROCESSOR) -c $(PROGRAMMER) -U lfuse:r:-:h -U hfuse:r:-:h

read-eeprom:
	avrdude $(VERBOSE) -C $(CONFIG) -P $(PORT) -b $(BAUDRATE) -p $(PROCESSOR) -c $(PROGRAMMER) -U eeprom:r:-:h

.PHONY: all flash flash-% read-fuses read-eeprom
