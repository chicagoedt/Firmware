makeCompile: transmitter.c
	avr-gcc -g -O3 -mmcu=atmega8 -c transmitter.c 
	avr-gcc -g -O3 -mmcu=atmega8 -o transmitter.elf ./transmitter.o
	avr-objcopy -j .text -j .data -O ihex transmitter.elf transmitter.hex
	
