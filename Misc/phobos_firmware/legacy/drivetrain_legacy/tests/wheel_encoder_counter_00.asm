; University of Illinois at Chicago
; EDT IGVC 2008 achilles
; Experiment #2: Morse Code ID tag
; Nick Novak Feburaray 8 2007
; encoder for dreive, get the number of ticks per unit time
; Define symbolic constants

Regbas EQU $0000 	; Register block starts at $0000
PortA EQU $00 		; PortA Address (relative to Regbas)
Config EQU $3F 		; Configuration control register
PACTL EQU $26		;port a control register
PACNT EQU $27		;pulse accumulator register PA7
TFLG2 EQU $25		;bit 5 is the pulse acc flag overflow
TCTL2 EQU $21		;use this to set what will trigger input capture
TFLG1 EQU $23		;flags for input capture, will go high on trigger

VAR1 EQU $45
VAR2 EQU $48


; Begin code
; Initialize the 68HC11: 
	
	ORG $E000 	; Place code in EEPROM starting at $FF00

Start:  LDS #$00FF 	; Initialize stack pointer
	LDX #Regbas 	; Initialize register base address pit5tr.
	LDAA #$04	; Imediate addressing mode $04 in A
	STAA Config,X 	; Disable "COP" watchdog timer

	;first, setup the pulse accumaltor and have it inc.
	;on a rising edge

	LDAA #$50	;load into a 01010000
	STAA PACTL,X	;enable PA and have it inc. on rising edge PAEN = PEDGE = 1

	;setup the input capture system use PA2/IC1 and PA1/IC2
	;when the the state change at the input, a flag will go off

	LDAA #$3C	;00111100 setup up PA2/IC1 and PA1/IC2 to flag when toggled
	STAA TCTL2,X	;store that in the control register
	
	;lets test out the system, have the system count pulses from the encoder
	;once we overflow get more than 256 pulse, we will set pin A6 high

	LDAA #$00
	STAA VAR1,X


Loop0
	;clear over flow flag just in case
	LDAA #$20	;0010000
	STAA TFLG2,X	;write a 1 to clear the flag

	LDAA #$00
	STAA PACNT	; clear the PA


	;lets figure out the direction
	;we can rememeber our current state 
	;and use the input capture to sense a change
	
	
	;c



Loop1	BRCLR TFLG2,X $20 Loop1

	;we get here only if we overflow
	;set PA6 high

	LDAA #$40
	EORA VAR1,X
	STAA VAR1,X	

	STAA PORTA,X

	;stop

	BRA Loop0	


; End of code


; Define Power-On Reset Interrupt Vector

	ORG $FFFE ; $FFFE, $FFFF = Power-On Reset Int. Vector Location
	FDB Start ; Specify instruction to execute on power up