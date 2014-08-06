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
PREV EQU $50

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

	;setup portA let PA3 be output
	LDAA #$08	;00001000 this will set PA3 for outbound
	STAA PACTL,X	;store it in porta control reg
	BCLR PORTA,X $FF	;clear all of porta

	;setup the input capture system use PA2/IC1 and PA1/IC2
	;when the the state change at the input, a flag will go off

	LDAA #$3C	;00111100 setup up PA2/IC1 and PA1/IC2 to flag when toggled
	STAA TCTL2,X	;store that in the control register
	
	;lets test out the system, have the system count pulses from the encoder
	;once we overflow get more than 256 pulse, we will set pin A6 high

	LDAA #$00
	STAA VAR1,X


	;test the red yellow and green leds
	;BSET PORTA,X $38	;set on PA345 
	;BRA *		;stop here


Loop0
	;clear over flow flag just in case
	LDAA #$20	;0010000
	STAA TFLG2,X	;write a 1 to clear the flag

	LDAA #$00
	STAA PACNT	; clear the PA

THERE1
	;lets figure out the direction
	;we can rememeber our current state 
	;and use the input capture to sense a change
	
	
	;clear the flags 
	LDAA #$06	;load 00000110 into A
	STAA TFLG1,X	;clear the IC1 and IC2 flags

	;wait for a flag to go off

	;LDY #60000
HERE1
	;DEY				;dec y, if we are not moving then lets not wait here
	;BEQ THERE2
	BRCLR TFLG1,X $06 HERE1		;wait until a IC1 or IC2 flag is high

THERE2:
	;if we are here than a flag has been sent, remember the state of encoder
	LDAA PORTA,X	;store the contents or porta into the accumulator
	ANDA #$06	;and A with 00000110 to clear everything but encode input
	LSLA		;move it over so its in bits 3 and 2
	STAA PREV,X	;store the previous encoder state here

	;we have save the previous state
	;clear the flags and wait for the next change

	;clear the flags 
	LDAA #$06	;load 00000110 into A
	STAA TFLG1,X	;clear the IC1 and IC2 flags

	;wait for change and store state
	;LDY #6000
HERE2
	;DEY				;dec y, if we are not moving then lets not wait here
	;BEQ THERE3
	BRCLR TFLG1,X $06 HERE2		;wait until a IC1 or IC2 flag is high

THERE3
	;if we are here than a flag has been sent, remember the state of encoder
	LDAB PORTA,X	;store the contents or porta into the accumulator
	ANDB #$06	;and A with 00000110 to clear everything but encode input
	LSRB	;move it over so its in bits 1 and 0
	
	ORAB PREV,X	;or a with prev encoder state gives XXXXPPCC

	LDY #$F000
	ABY
	LDAA $00,Y
	STAA PORTA,X

	




	LDY #60000		
HERE5	DEY
	BNE HERE5

	;LDAA #$10
	;STAA PORTA,X

	;BRA *


	JMP THERE1

	;go to look up table to get direction
	
	
	

	;test the red yellow and green leds
	;BSET PORTA,X $38	;set on PA345 
	;BRA *		;stop here





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

	;look up table or digits for encoder direction
	ORG $F000
	DC.B $10,$20,$08,$00,$08,$10,$00,$20,$20,$00,$10,$08,$00,$08,$20,$10


; Define Power-On Reset Interrupt Vector

	ORG $FFFE ; $FFFE, $FFFF = Power-On Reset Int. Vector Location
	FDB Start ; Specify instruction to execute on power up