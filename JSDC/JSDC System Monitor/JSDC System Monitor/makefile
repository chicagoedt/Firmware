makeCompile: JSDC\ System\ Monitor.c
	avr-gcc -g -O3 -mmcu=atmega32u4 -c JSDC\ System\ Monitor.c 
	avr-gcc -g -O3 -mmcu=atmega32u4 -o JSDC\ System\ Monitor.elf JSDC\ System\ Monitor.o
	avr-objcopy -j .text -j .data -O ihex JSDC\ System\ Monitor.elf JSDC\ System\ Monitor.hex

program: JSDC\ System\ Monitor.hex 
	avrdude -c avr109 -p atmega32u4 -P /dev/ttyACM0 -U flash:w:JSDC\ System\ Monitor.hex:i
setfuse: 
	avrdude -c avr109 -p atmega32u4 -P /dev/ttyACM0 -U lfuse:w:0xde:m -U hfuse:w:0x99:m -U efuse:w:0xf3:m 

