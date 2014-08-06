makeCompile: receiver.c
	avr-gcc -g -O3 -mmcu=atmega8 -c receiver.c 
	avr-gcc -g -O3 -mmcu=atmega8 -o receiver.elf ./receiver.o
	avr-objcopy -j .text -j .data -O ihex receiver.elf receiver.hex
	
