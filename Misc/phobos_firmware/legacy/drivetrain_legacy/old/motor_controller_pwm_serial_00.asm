; University of Illinois at Chicago
; ECE 367 - Microprocessor-Based Design
; Experiment #2: Morse Code ID tag
; Nick Novak Feburaray 8 2007

; Define symbolic constants
Regbas EQU $0000 	; Register block starts at $0000
PortA EQU $00 		; PortA Address (relative to Regbas)
PortD EQU $08
Config EQU $3F 		; Configuration control register
PACTL EQU $26
DDRD EQU $09
PERIOD EQU 40000
DTYCYC EQU 3200
TCTL1 EQU $20
TFLG1 EQU $23
TOC1 EQU $16
TOC3 EQU $1A
TCNT EQU $0E
OC1M EQU $0C
OC1D EQU $0D
VAR1 EQU $45		;store duty cycle
VAR2 EQU $48		;store delay



;SPCR EQU $28
;SPSR EQU $29
;SPDR EQU $2A

BAUD EQU $2B
SCCR1 EQU $2C
SCCR2 EQU $2D
SCSR EQU $2E
SCDR EQU $2F

INITVAL EQU $80; value that lenses will start at

SCI_READ EQU $40; if 0, we did not read from SCI, otherwise we did







; Begin code
; Initialize the 68HC11: 
	
	ORG $E000 	; Place code in EEPROM starting at $FF00

Start:  LDS #$00FF 	; Initialize stack pointer
	LDX #Regbas 	; Initialize register base address ptr.
	LDAA #$04	; Imediate addressing mode $04 in A
	STAA Config,X 	; Disable "COP" watchdog timer

	LDAA #$00	; Load into A 0
   	STAA PortA,X 	; Initialize output lines of PORT A to 0's

	LDD TCNT,X
	STD TOC3,X	;prevent premature capture of OC1 and OC3
	STD TOC1,X
	LDAB #$A0
	STAB TFLG1,X	;clear the oc1 flag and oc3 flag
	LDAA #$20	;setup oc3 to go low on oc3 compare
	STA OC1M,X	;oc1 controls oc3
	STA OC1D,X	;oc1 compare sets oc3 high
	LDAA #$20
	STA TCTL1,X

	LDAA	#$1A		; Make PortD1,3,4 output
	STAA	DDRD

	LDAA #$30 ; enable SCI
	STAA BAUD
	LDAA #$00
	STAA SCCR1
	LDAA #$0C
	STAA SCCR2


	;default duty cycle is 3000 netural
	LDY #3000
	STY VAR1


	;should start the beginggingof the .1 sec loop here
	;for now lets start o by checking the serial register and
	;if it has been written to

Top
	;code to read from serial input
	;once we have the input number
	;load it in A then us a look up table to find the duty cycle 
	;store tha into VAR1

	;JMP HERE11

ReadSCINoBlocking:
	;PSHB			;protect contents of B
	;LDAB #$00		;load 0
	;STAB SCI_READ		;clear flag
	BRCLR SCSR, X, $20, SkipReadSCI ;check is something was written
	;LDAB #$01			;something written set flag
	;STAB SCI_READ			;set flag
	LDAB SCDR,X;			;load number written into B
	
	;we now have the 8bit number, we have to use this to go to the look up table
	PSHX			;save X we are gonna use for the look up table
	LDY #$EFD0
	ABY			;add b and Y, should now point to look up table
	LDX $00,Y		;X has value from look up table
	STX VAR1		;the duty cycle is now updated
	PULX
	
SkipReadSCI:			;nothing to read do nothing
	;PULB


;HERE11
	
	LDAA #255
	STAA VAR2
	

Loop3
	

	;LDY #3500 	;min speed is 3028
	;STY VAR1

	JSR PWM		;call pwm routine

	DEC VAR2	;delay counter
	BNE Loop3

	;we are done with this pwm time, loop and see
	;if we need to update, go back to beginning


	
	JMP TOP

	
PWM

	LDD TCNT,X	;get intitial value
	ADDD #$0A	;add 10 cycles
	STD TOC1,X	;one e-clock later have TOC1 compare
			;oc3 will go high
	LDAB #$80
	STAB TFLG1,X	;clear oc1 flag
	LDD TOC1,X	;get intial value of time

	

Loop1	ADDD VAR1	;add the duty cycle
	STD TOC3,X
	LDD TOC1,X	;get last tcnt value
	ADDD #PERIOD	;increase by one period
	STD TOC1,X	;store it in TOC1

WAIT	BRCLR TFLG1,X $80 WAIT	;poll oc1 flag
	LDAB #$A0	;got it so clear oc1 lag
	STAB TFLG1,X	;clear both flags
	LDD TOC1,X	;get last tco1 count
	RTS


; End of code

;look up table or digits for encoder direction
	ORG $F000
	DC.B $0B,$B8,$0F,$A0,$07,$D0


; Define Power-On Reset Interrupt Vector

	ORG $FFFE ; $FFFE, $FFFF = Power-On Reset Int. Vector Location
	FDB Start ; Specify instruction to execute on power up