; University of Illinois at Chicago
; ECE 367 - Microprocessor-Based Design
; Experiment #2: Morse Code ID tag
; Nick Novak Feburaray 8 2007

; Define symbolic constants
Regbas EQU $0000 	; Register block starts at $0000
PortA EQU $00 		; PortA Address (relative to Regbas)
Config EQU $3F 		; Configuration control register
PERIOD EQU 40000
DTYCYC EQU 3200
TCTL1 EQU $20
TFLG1 EQU $23
TOC1 EQU $16
TOC3 EQU $1A
TCNT EQU $0E
OC1M EQU $0C
OC1D EQU $0D
VAR1 EQU $45



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
	LDD TCNT,X	;get intitial value
	ADDD #$0A	;add 10 cycles
	STD TOC1,X	;one e-clock later have TOC1 compare
			;oc3 will go high

	LDAB #$80
	STAB TFLG1,X	;clear oc1 flag
	LDD TOC1,X	;get intial value of time

	LDY #4000
	STY VAR1

Loop1	ADDD VAR1	;add the duty cycle
	STD TOC3,X
	LDD TOC1,X	;get last tcnt value
	ADDD #PERIOD	;increase by one period
	STD TOC1,X	;store it in TOC1

WAIT	BRCLR TFLG1,X $80 WAIT	;poll oc1 flag
	LDAB #$A0	;got it so clear oc1 lag
	STAB TFLG1,X	;clear both flags
	LDD TOC1,X	;get last tco1 count
	

	BRA Loop1


	





; End of code


; Define Power-On Reset Interrupt Vector

	ORG $FFFE ; $FFFE, $FFFF = Power-On Reset Int. Vector Location
	FDB Start ; Specify instruction to execute on power up