MCU = atmega328p
CPU_FREQ = 20000000
TARGET_FILE_NAME = main

INCLUDE_FILES  = spi.c lcd.c
#INCLUDE_FILES += nrf24l01/nrf24l01.c

INCLUDE_OBJECTS  = spi.o lcd.o
#INCLUDE_OBJECTS += nrf24l01.o

OUTPUT_FORMAT = ihex
OPTIMIZATION_LEVEL = s # [0, 1, 2, 3, s]
DEBUG_FORMAT = dwarf-2

CSTANDARD = -std=gnu99

AVRDUDE_PROGRAMMER = usbtiny
AVRDUDE_PORT = usb

###############################################

CC = avr-gcc
REMOVE = rm -f

CDEFS = -mmcu=$(MCU)

COMPILER_FLAGS  = -g$(DEBUG_FORMAT) -O$(OPTIMIZATION_LEVEL)
COMPILER_FLAGS += $(CDEFS)
COMPILER_FLAGS += -DF_CPU=$(CPU_FREQ)UL
COMPILER_FLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
COMPILER_FLAGS += -Wall -Wstrict-prototypes
COMPILER_FLAGS += $(CSTANDARD)
COMPILER_FLAGS += -lm # math library

AVRDUDE       = avrdude
AVRDUDE_FLAGS = -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER)

###############################################
MESSAGE_BEGIN       = ------- Start
MESSAGE_END         = ------- End
MESSAGE_OBJECT      = ------- Building object files
MESSAGE_ELF         = ------- Building elf file
MESSAGE_HEX         = ------- Building hex file
MESSAGE_PROGRAM     = ------- Programming
MESSAGE_FUSES       = ------- Burning fuses
MESSAGE_CLEAN       = ------- Cleaning files
MESSAGE_SHOW_FOLDER = ------- Folder content
###############################################

all: begin build end

build: obj elf hex

obj: $(TARGET_FILE_NAME).c
	@echo
	@echo $(MESSAGE_OBJECT)
	$(CC) $(COMPILER_FLAGS) -c $(TARGET_FILE_NAME).c $(INCLUDE_FILES)

elf: $(TARGET_FILE_NAME).o
	@echo
	@echo $(MESSAGE_ELF)
	$(CC) -g $(CDEFS) -o $(TARGET_FILE_NAME).elf $(TARGET_FILE_NAME).o $(INCLUDE_OBJECTS)

hex: $(TARGET_FILE_NAME).elf
	@echo
	@echo $(MESSAGE_HEX)
	avr-objcopy -j .text -j .data -O ihex $(TARGET_FILE_NAME).elf $(TARGET_FILE_NAME).hex

begin:
	@echo
	@echo $(MESSAGE_BEGIN)

end:
	@echo
	@echo $(MESSAGE_END)
	@echo
	
program: $(TARGET_FILE_NAME).hex
	@echo
	@echo $(MESSAGE_FLASH)  
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U flash:w:$(TARGET_FILE_NAME).hex
	
readfuse:
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U lfuse:r:-:h -U hfuse:r:-:h -U efuse:r:-:h -U lock:r:-:h

writefuse:
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U lfuse:w:0xEF:m -U hfuse:w:0xD9:m -U efuse:w:0xFF:m

clean:
	@echo
	@echo $(MESSAGE_CLEAN)
	$(REMOVE) $(TARGET_FILE_NAME).hex
	$(REMOVE) $(TARGET_FILE_NAME).elf
	$(REMOVE) *.o
	@echo
