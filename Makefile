DEVICE = atmega32p
MCU = atmega328p
AVRDUDE_DEVICE = m328p


TARGET = main
CC = avr-gcc
CFLAGS = -g -Wall -Os -mmcu=atmega328p
ADFLAGS = -c usbtiny -p atmega328p
OBJCOPY= avr-objcopy

.PHONY : flash erase clean fuses

$(TARGET).obj : $(TARGET).o
	$(CC) $(CFLAGS) lcd.c functions.c $< -o $@
	
$(TARGET).hex: $(TARGET).obj
	$(OBJCOPY) -R .eeprom -O ihex $< $@

$(TARGET).eeprom : $(TARGET).obj
	$(OBJCOPY) -j .eeprop -O ihex $< $@
	
flash: $(TARGET).hex $(TARGET).eeprom
	avrdude $(ADFLAGS) -U flash:w:$(TARGET).hex:i -F -v -v -v -v
	
erase:
	avrdude $(ADFLAGS) -F -e -v -v

clean:
	rm -f *~ *.hex *.obj *.o *.eeprom

fusesINT:
	avrdude $(ADFLAGS) -U lfuse:w:0x62:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m
	
fusesEXT:
	avrdude $(ADFLAGS) -U lfuse:w:0xf7:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m