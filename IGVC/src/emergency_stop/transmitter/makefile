makeCompile: transmitter.c
	avr-gcc -g -O3 -mmcu=atmega8 -c transmitter.c 
	avr-gcc -g -O3 -mmcu=atmega8 -o transmitter.elf ./transmitter.o
	avr-objcopy -j .text -j .data -O ihex transmitter.elf transmitter.hex
upload: transmitter.hex
	sudo avrdude -c USBasp -p atmega8 -U flash:w:transmitter.hex:i
setfuse: 
	sudo avrdude -c USBasp -p atmega8 -U lfuse:w:0xd2:m -U hfuse:w:0xd9:m 

	
