; Anup Kotadia
; Auto Iris lenses control
; 12-16-2007

; Define symbolic constants

;PD0 = RxD
;PD1 = TxD
;PD2 = SERIAL IN sipo
;PD3 = SERIAL OUT sipo
;PD4 = SERIAL CLOCK sipo
;PA4 = SIPO REGISTER ENABLE

;Circuit Setup
;Serial Cable from computer to MAX232 Chip
;Max 232 Chip to Stamp11
;Stamp11 to SIPO Register
;SIPO Register to Digital to Analog Converter
;D/A to camera and other biasing circuitry



Regbase	EQU	$0000		; Register block starts at $0000 
PortA	EQU	$00		; PortA Address (relative to Regbas)
Config	EQU	$3F		; Configuration control register 
PortD	EQU	$08
DDRD	EQU	$09
PACTL	EQU	$26

SPCR EQU $28
SPSR EQU $29
SPDR EQU $2A

BAUD EQU $2B
SCCR1 EQU $2C
SCCR2 EQU $2D
SCSR EQU $2E
SCDR EQU $2F

INITVAL EQU $80; value that lenses will start at

SCI_READ EQU $40; if 0, we did not read from SCI, otherwise we did

; Begin code
; Initialize the 68HC11:
 
	ORG	$E000		; Place code in EEPROM starting at $E000

Start:	LDS	#$00FF		; Initialize stack pointer 
	LDX	#Regbase	; Initialize register base address ptr. 
	LDAA	#$04
	STAA	Config,X	; Disable "COP" watchdog timer 
        LDY	#Regbase
	LDAA	#$1A		; Make PortD1,3,4 output
	STAA	DDRD

	;LDAA #$50 ; enable SPI
	;STAA SPCR;


	LDAA #$30 ; enable SCI
	STAA BAUD
	LDAA #$00
	STAA SCCR1
	LDAA #$0C
	STAA SCCR2

	;LDAA #INITVAL; these two lines will start up the lenses at 50% of max voltage
	;JSR WriteRegister

Loop:
	JSR ReadSCI; gets the input from computer
	JSR WriteSCI; writes the input to the SIPO register
	JMP Loop


;This subroutine writes Accumulator A to SIPO
WriteRegister:
	STAA SPDR; sends command to send out the data
	BRCLR SPSR, Y, $80, *;
	BSET PortA, Y, $10; sets PA4 to fire register enable
	NOP;
	BCLR PortA, Y, $10; clears PA4
	NOP;
	RTS

;This subroutine loads Accumulator A from SCI input
ReadSCI:
	BRCLR SCSR,Y, $20, *;
	LDAA SCDR;
	RTS

;This subroutine loads Accumulator A from SCI input without waiting
ReadSCINoBlocking:
	PSHB
	LDAB #$00
	STAB SCI_READ
	BRCLR SCSR, Y, $20, SkipReadSCI;
	LDAB #$01
	STAB SCI_READ
	LDAA SCDR;
SkipReadSCI:
	PULB
	RTS

;This subroutine writes Accumulator A to SCI
WriteSCI:
	BRCLR SCSR, Y, $80, *;
	STAA SCDR
	RTS


; End of code
; Define Power-On Reset Interrupt Vector

	ORG	$FFFE		; $FFFE, $FFFF = Power-On Reset Int. Vector Location
	FDB	Start		; Specify instruction to execute on power up
