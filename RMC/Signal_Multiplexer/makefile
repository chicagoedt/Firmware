makeCompile: Sigmux.c
	avr-gcc -g -O3 -mmcu=atmega32u4 -c Sigmux.c 
	avr-gcc -g -O3 -mmcu=atmega32u4 -o Sigmux.elf ./Sigmux.o ./CC3000HostDriver/spi.o ./CC3000HostDriver/socket.o ./CC3000HostDriver/wlan.c ./CC3000HostDriver/nvmem.o ./CC3000HostDriver/cc3000_common.o ./CC3000HostDriver/evnt_handler.o ./CC3000HostDriver/hci.o ./CC3000HostDriver/netapp.o ./CC3000HostDriver/security.o
	avr-objcopy -j .text -j .data -O ihex Sigmux.elf Sigmux.hex

upload: Sigmux.hex
	sudo avrdude -c USBasp -p atmega32u4 -U flash:w:Sigmux.hex:i		
