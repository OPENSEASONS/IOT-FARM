#Main application file name

MAIN_APP = blink
MAIN_HEX_PATH = $(MAIN_APP).hex

# Compiler and tools
CC = avr-gcc
OBJCOPY = avr-objcopy

# Compiler flags
CFLAGS = -g -Os -mmcu=atmega328p -DF_CPU=8000000UL
LFLAGS = -mmcu=atmega328p
HFLAGS = -j .text -j .data -O ihex

# AVRDude configuration
AVRDUDE := avrdude -C "C:\Users\SoftClansUser\AppData\Local\Arduino15\packages\arduino\tools\avr\etc\avrdude.conf"
DUDEFLAGS = -c avrisp -p m328p -P COM3 -b 19200

# Source files
SRC = $(MAIN_APP).c

# Build rules
all: $(MAIN_APP).hex

$(MAIN_APP).hex: $(MAIN_APP).elf
	$(OBJCOPY) $(HFLAGS) $< $@

$(MAIN_APP).elf: $(MAIN_APP).o
	$(CC) $(LFLAGS) -o $@ $^

$(MAIN_APP).o: $(SRC)
	$(CC) $(CFLAGS) -c -o $@ $^

flash: $(MAIN_APP).hex
	$(AVRDUDE) $(DUDEFLAGS) -U flash:w:$<:i

clean:
	rm -f *.o *.elf *.hex