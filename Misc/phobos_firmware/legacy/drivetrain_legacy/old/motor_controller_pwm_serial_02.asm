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
	LDY #$F000
	ABY
	ABY			;add b and Y, should now point to look up table
	LDX $00,Y		;X has value from look up table
	STX VAR1		;the duty cycle is now updated
	PULX
	
SkipReadSCI:			;nothing to read do nothing
	;PULB


;HERE11
	
	LDAA #10
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

;this is routine that creates the pwm and send its to wheels	
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
	DC.B $0B, $B8
        DC.B $0B, $BF
        DC.B $0B, $C6
        DC.B $0B, $CD
        DC.B $0B, $D4
        DC.B $0B, $DB
        DC.B $0B, $E2
        DC.B $0B, $E9
        DC.B $0B, $F0
        DC.B $0B, $F7
        DC.B $0B, $FE
        DC.B $0C, $05
        DC.B $0C, $0C
        DC.B $0C, $13
        DC.B $0C, $1A
        DC.B $0C, $21
        DC.B $0C, $28
        DC.B $0C, $2F
        DC.B $0C, $36
        DC.B $0C, $3D
        DC.B $0C, $44
        DC.B $0C, $4B
        DC.B $0C, $52
        DC.B $0C, $59
        DC.B $0C, $60
        DC.B $0C, $67
        DC.B $0C, $6E
        DC.B $0C, $75
        DC.B $0C, $7C
        DC.B $0C, $83
        DC.B $0C, $8A
        DC.B $0C, $91
        DC.B $0C, $98
        DC.B $0C, $9F
        DC.B $0C, $A6
        DC.B $0C, $AD
        DC.B $0C, $B4
        DC.B $0C, $BB
        DC.B $0C, $C2
        DC.B $0C, $C9
        DC.B $0C, $D0
        DC.B $0C, $D7
        DC.B $0C, $DE
        DC.B $0C, $E5
        DC.B $0C, $EC
        DC.B $0C, $F3
        DC.B $0C, $FA
        DC.B $0D, $01
        DC.B $0D, $08
        DC.B $0D, $0F
        DC.B $0D, $16
        DC.B $0D, $1D
        DC.B $0D, $24
        DC.B $0D, $2B
        DC.B $0D, $32
        DC.B $0D, $39
        DC.B $0D, $40
        DC.B $0D, $47
        DC.B $0D, $4E
        DC.B $0D, $55
        DC.B $0D, $5C
        DC.B $0D, $63
        DC.B $0D, $6A
        DC.B $0D, $71
        DC.B $0D, $78
        DC.B $0D, $7F
        DC.B $0D, $86
        DC.B $0D, $8D
        DC.B $0D, $94
        DC.B $0D, $9B
        DC.B $0D, $A2
        DC.B $0D, $A9
        DC.B $0D, $B0
        DC.B $0D, $B7
        DC.B $0D, $BE
        DC.B $0D, $C5
        DC.B $0D, $CC
        DC.B $0D, $D3
        DC.B $0D, $DA
        DC.B $0D, $E1
        DC.B $0D, $E8
        DC.B $0D, $EF
        DC.B $0D, $F6
        DC.B $0D, $FD
        DC.B $0E, $04
        DC.B $0E, $0B
        DC.B $0E, $12
        DC.B $0E, $19
        DC.B $0E, $20
        DC.B $0E, $27
        DC.B $0E, $2E
        DC.B $0E, $35
        DC.B $0E, $3C
        DC.B $0E, $43
        DC.B $0E, $4A
        DC.B $0E, $51
        DC.B $0E, $58
        DC.B $0E, $5F
        DC.B $0E, $66
        DC.B $0E, $6D
        DC.B $0E, $74
        DC.B $0E, $7B
        DC.B $0E, $82
        DC.B $0E, $89
        DC.B $0E, $90
        DC.B $0E, $97
        DC.B $0E, $9E
        DC.B $0E, $A5
        DC.B $0E, $AC
        DC.B $0E, $B3
        DC.B $0E, $BA
        DC.B $0E, $C1
        DC.B $0E, $C8
        DC.B $0E, $CF
        DC.B $0E, $D6
        DC.B $0E, $DD
        DC.B $0E, $E4
        DC.B $0E, $EB
        DC.B $0E, $F2
        DC.B $0E, $F9
        DC.B $0F, $00
        DC.B $0F, $07
        DC.B $0F, $0E
        DC.B $0F, $15
        DC.B $0F, $1C
        DC.B $0F, $23
        DC.B $0F, $2A
        DC.B $0F, $31
        DC.B $08, $38
        DC.B $08, $3F
        DC.B $08, $46
        DC.B $08, $4D
        DC.B $08, $54
        DC.B $08, $5B
        DC.B $08, $62
        DC.B $08, $69
        DC.B $08, $70
        DC.B $08, $77
        DC.B $08, $7E
        DC.B $08, $85
        DC.B $08, $8C
        DC.B $08, $93
        DC.B $08, $9A
        DC.B $08, $A1
        DC.B $08, $A8
        DC.B $08, $AF
        DC.B $08, $B6
        DC.B $08, $BD
        DC.B $08, $C4
        DC.B $08, $CB
        DC.B $08, $D2
        DC.B $08, $D9
        DC.B $08, $E0
        DC.B $08, $E7
        DC.B $08, $EE
        DC.B $08, $F5
        DC.B $08, $FC
        DC.B $09, $03
        DC.B $09, $0A
        DC.B $09, $11
        DC.B $09, $18
        DC.B $09, $1F
        DC.B $09, $26
        DC.B $09, $2D
        DC.B $09, $34
        DC.B $09, $3B
        DC.B $09, $42
        DC.B $09, $49
        DC.B $09, $50
        DC.B $09, $57
        DC.B $09, $5E
        DC.B $09, $65
        DC.B $09, $6C
        DC.B $09, $73
        DC.B $09, $7A
        DC.B $09, $81
        DC.B $09, $88
        DC.B $09, $8F
       	DC.B $09, $96
        DC.B $09, $9D
        DC.B $09, $A4
        DC.B $09, $AB
        DC.B $09, $B2
        DC.B $09, $B9
        DC.B $09, $C0
        DC.B $09, $C7
        DC.B $09, $CE
        DC.B $09, $D5
        DC.B $09, $DC
        DC.B $09, $E3
        DC.B $09, $EA
        DC.B $09, $F1
        DC.B $09, $F8
        DC.B $09, $FF
        DC.B $0A, $06
        DC.B $0A, $0D
        DC.B $0A, $14
        DC.B $0A, $1B
        DC.B $0A, $22
        DC.B $0A, $29
        DC.B $0A, $30
        DC.B $0A, $37
        DC.B $0A, $3E
        DC.B $0A, $45
        DC.B $0A, $4C
        DC.B $0A, $53
        DC.B $0A, $5A
        DC.B $0A, $61
        DC.B $0A, $68
        DC.B $0A, $6F
        DC.B $0A, $76
        DC.B $0A, $7D
        DC.B $0A, $84
        DC.B $0A, $8B
        DC.B $0A, $92
        DC.B $0A, $99
        DC.B $0A, $A0
        DC.B $0A, $A7
        DC.B $0A, $AE
        DC.B $0A, $B5
        DC.B $0A, $BC
        DC.B $0A, $C3
        DC.B $0A, $CA
        DC.B $0A, $D1
        DC.B $0A, $D8
        DC.B $0A, $DF
        DC.B $0A, $E6
        DC.B $0A, $ED
        DC.B $0A, $F4
        DC.B $0A, $FB
        DC.B $0B, $02
        DC.B $0B, $09
        DC.B $0B, $10
        DC.B $0B, $17
        DC.B $0B, $1E
        DC.B $0B, $25
        DC.B $0B, $2C
        DC.B $0B, $33
        DC.B $0B, $3A
        DC.B $0B, $41
        DC.B $0B, $48
        DC.B $0B, $4F
        DC.B $0B, $56
        DC.B $0B, $5D
        DC.B $0B, $64
        DC.B $0B, $6B
        DC.B $0B, $72
        DC.B $0B, $79
        DC.B $0B, $80
        DC.B $0B, $87
        DC.B $0B, $8E
        DC.B $0B, $95
        DC.B $0B, $9C
        DC.B $0B, $A3
        DC.B $0B, $AA
        DC.B $0B, $B1

; Define Power-On Reset Interrupt Vector

	ORG $FFFE ; $FFFE, $FFFF = Power-On Reset Int. Vector Location
	FDB Start ; Specify instruction to execute on power up