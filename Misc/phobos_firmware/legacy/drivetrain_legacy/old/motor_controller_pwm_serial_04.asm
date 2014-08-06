;Engineering Desing Team
;Nick Novak 4-14-08
;IGVC 2008 Achilles
;Version 1.1
;---------------------------------------------------------------------
;This program serves as interface between the computer control 
;system and the drive train.  The control system will send out
;an 8-bit number, through the serial port.  The uC has an automated
;serial interface that will read the incoming 8bit number.  The 8-bit 
;number will control the length of the high pulse of the PWM.  Ultimately
;this controls the speed of the motor.  Attach near one of the sprockets
;on the drive train is a quadrature wheel encoder.  This device will allow
;us to detect the sped of the wheel and the direction.  The direction and 
;speed of the wheel will be sent out as an 8-bit number through the serial 
;port back to the control program.
;---------------------------------------------------------------------
;               CHANGES FROM PREVIOUS VERSION
;The previous version used pin3 on the uC to gernerate the output
;PWM for the motorcontroller, when we made the circuit boards
;we made them to use pin2 not pin3.  The only change is that the PWM
;will use pin2 instead of pin.
;---------------------------------------------------------------------


;Define symbolic constants
Regbas EQU $0000 		; Register block starts at $0000
PortA EQU $00 			; PortA Address 
PortD EQU $08			; PortD Address 
PACNT EQU $27		    	; pulse accumulator register PA7
Config EQU $3F 			; Configuration control register
PACTL EQU $26			; PortA control register
DDRD EQU $09			; PortD control register
PERIOD EQU 40000		; variable to store period of PWM 40,000 counts = 20ms
DTYCYC EQU 3200			; variable to store the duty cycle of PWM
TCTL1 EQU $20			; timer control register 1
TCTL2 EQU $21			; use this to set what will trigger input capture
TFLG1 EQU $23			; timer flag register
TFLG2 EQU $25			; bit 5 is the pulse acc flag overflow
TOC1 EQU $16			; output compare 1 timer register
TOC2 EQU $18			; output compare 2 timer register
TOC3 EQU $1A			; output compare 3 timer regsiter
TCNT EQU $0E			; timer count register
OC1M EQU $0C			; output compare mask register
OC1D EQU $0D			; output compare data register
BAUD EQU $2B			; register to control baud rate of serial port
SCCR1 EQU $2C			; serial control register 1
SCCR2 EQU $2D			; serial control register 2		
SCSR EQU $2E			; serial flag register
SCDR EQU $2F			; serial data register
VAR1 EQU $45			; store duty cycle
VAR2 EQU $48			; store delay
PREV EQU $50			; store previous state of wheel encoder, for direction

;INITVAL EQU $80; value that lenses will start at

;SCI_READ EQU $40; if 0, we did not read from SCI, otherwise we did


; Begin code
; Initialize the 68HC11: 
	
	ORG $E000 	; Place code in EEPROM starting at $FF00

Start:  LDS #$00FF 	; Initialize stack pointer
	LDX #Regbas 	; Initialize register base address ptr.
	LDAA #$04	; Imediate addressing mode $04 in A
	STAA Config,X 	; Disable "COP" watchdog timer

	LDAA #$00	; Load into A 0
   	STAA PortA,X 	; Initialize output lines of PORT A to 0's

	;setup the pulse accumaltor and have it inc.
	;on a rising edge
	LDAA #$50	;load into a 01010000
	STAA PACTL,X	;enable PA and have it inc. on rising edge PAEN = PEDGE = 1

	; setup portA let PA3 be output
	LDAA #$58	;01011000 this will set porta for outbound
	STAA PACTL,X	;store it in porta control reg
	BCLR PORTA,X $FF	;clear all of porta

	;setup the input capture system use PA2/IC1 and PA1/IC2
	;when the the state change at the input, a flag will go off

	LDAA #$3C	;00111100 setup up PA2/IC1 and PA1/IC2 to flag when toggled
	STAA TCTL2,X	;store that in the control register


	;setup the output timer controls
	LDD TCNT,X	;read TCNT register
	STD TOC2,X	;prevent premature capture of OC1 and OC2
	STD TOC1,X
	LDAB #$C0	;load 11000000
	STAB TFLG1,X	;clear the oc1 flag and oc2 flag
	LDAA #$40	;01000000
	STA OC1M,X	;oc1 controls oc2
	STA OC1D,X	;oc1 compare sets oc2 high
	LDAA #$80	;10000000
	STA TCTL1,X	;drive oc2 low on oc2 compare

	LDAA	#$1A		; Make PortD1,3,4 output
	STAA	DDRD

	LDAA #$30 	; enable SCI
	STAA BAUD	; set the baurd rate	
	LDAA #$00	;
	STAA SCCR1	;
	LDAA #$0C	;
	STAA SCCR2	;


	;default duty cycle is 3000 netural
	LDY #3000
	STY VAR1

	;clear the pulse accumulator and start it off as zero
	LDAA #$00
	STAA PACNT	; clear the PA

	;clear over flow flag for PA just in case - not using over flow lag
	;LDAA #$20	;0010000
	;STAA TFLG2,X	;write a 1 to clear the flag

;code to setup the timers for PWM, only do this part once

	LDD TCNT,X	;get intitial value
	ADDD #$0A	;add 10 cycles
	STD TOC1,X	;one e-clock later have TOC1 compare
			;oc2 will go high
	LDAB #$80
	STAB TFLG1,X	;clear oc1 flag
	LDD TOC1,X	;get intial value of time


;this is where the loop will start
Loop1	

;1111111111111111111111111111111111111111111111111111111111111111111
	ADDD VAR1	;add the duty cycle
	STD TOC2,X
	LDD TOC1,X	;get last tcnt value
	ADDD #PERIOD	;increase by one period
	STD TOC1,X	;store it in TOC1

	;waiting here for the oc1 flag to go high
	;that is waiting for the rising edge of PWM
	;lets update the serial input here


	;code to read from serial input
	;once we have the input number
	;load it in A then us a look up table to find the duty cycle 
	;store tha into VAR1


	BRCLR SCSR, X, $20, SkipReadSCI ;check is something was written
	
	;something was written so we are here
	;update var1 with new input
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

	;the velocity has been updated, time send out the 8-bit number
	;that will give the measure veloicyt from the encoders
	

	;take counts from PA and store in A
	LDAB #$00			;load 0 so that after we read PA we can clear it
	LDAA PACNT, X			;move contents for PA to accum A
	STAB PACNT, X			;clear PA to 0
	
	;this might need to be changed, pitfall for infinte loop maybe
	BRCLR SCSR, X, $80, *;		;when its 1, this means its empty and we can write to it
	STAA SCDR			;write to serial, automatically sends out data



WAIT1	BRCLR TFLG1,X $80 WAIT1		;poll oc1 flag
	LDAB #$C0			;got it so clear oc1 oc2 flags
	STAB TFLG1,X			;clear both flags
	LDD TOC1,X			;get last tco1 count
	
;1111111111111111111111111111111111111111111111111111111111111111111

;2222222222222222222222222222222222222222222222222222222222222222222
	ADDD VAR1	;add the duty cycle
	STD TOC2,X
	LDD TOC1,X	;get last tcnt value
	ADDD #PERIOD	;increase by one period
	STD TOC1,X	;store it in TOC1

WAIT2	BRCLR TFLG1,X $80 WAIT2		;poll oc1 flag
	LDAB #$C0			;got it so clear oc1 lag
	STAB TFLG1,X			;clear both flags
	LDD TOC1,X			;get last tco1 count
	

;2222222222222222222222222222222222222222222222222222222222222222222

;3333333333333333333333333333333333333333333333333333333333333333333

	ADDD VAR1	;add the duty cycle
	STD TOC2,X
	LDD TOC1,X	;get last tcnt value
	ADDD #PERIOD	;increase by one period
	STD TOC1,X	;store it in TOC1

;lets figure out the direction-------------------------------------------------------------------------
	;we can rememeber our current state 
	;and use the input capture to sense a change	
	
	;clear the flags 
	LDAA #$06	;load 00000110 into A						COUNTS --2
	STAA TFLG1,X	;clear the IC1 and IC2 flags					COUNTS --4
	;wait for a flag to go off

	LDY #1300	;2500								COUNTS --4
HERE1
	DEY		;								COUNTS --4
	BEQ OVER2	;								COUNTS --3
	BRCLR TFLG1,X $06 HERE1		;wait until a IC1 or IC2 flag is high		COUNTS --7
	
	;if we are here than a flag has been sent, remember the state of encoder
	LDAA PORTA,X	;store the contents or porta into the accumulator 		COUNTS --2                    
	ANDA #$06	;and A with 00000110 to clear everything but encode input	COUNTS --2
	LSLA		;move it over so its in bits 3 and 2				COUNTS --2
	STAA PREV,X	;store the previous encoder state here				COUNTS --4

	;we have save the previous state
	;clear the flags and wait for the next change

	;clear the flags 
	LDAA #$06	;load 00000110 into A						COUNTS --2
	STAA TFLG1,X	;clear the IC1 and IC2 flags					COUNTS --4

	;wait for change and store state
	LDY #1300	;								COUNTS --4
HERE2
	DEY		;								COUNTS --4
	BEQ OVER2	;								COUNTS --3
	BRCLR TFLG1,X $06 HERE2		;wait until a IC1 or IC2 flag is high		COUNTS --7

	;if we are here than a flag has been sent, remember the state of encoder
	LDAB PORTA,X	;store the contents or porta into the accumulator		COUNTS --4		
	

	ANDB #$06	;and A with 00000110 to clear everything but encode input	COUNTS --2
	LSRB	;move it over so its in bits 1 and 0					COUNTS --2
	
	ORAB PREV,X	;or a with prev encoder state gives XXXXPPCC			COUNTS --4

	LDY #$FF00	;								COUNTS --4
	ABY		;								COUNTS --4
	LDAA $00,Y	;								COUNTS --5
	STAA PORTA,X	;								COUNTS --4
	JMP WAIT3	;								COUNTS --3
OVER2			;								TOTAL  -87
	LDAA #$00
	STAA PORTA,X	;								COUNTS --4

;lets figure out the direction------end----------------------------------------------------------------

WAIT3	BRCLR TFLG1,X $80 WAIT3		;poll oc1 flag					COUNTS --7
	LDAB #$C0			;got it so clear oc1 lag			COUNTS --2
	STAB TFLG1,X			;clear both flags				COUNTS --4
	LDD TOC1,X			;get last tco1 count				COUNTS --5
	

;3333333333333333333333333333333333333333333333333333333333333333333

;4444444444444444444444444444444444444444444444444444444444444444444
	ADDD VAR1	;add the duty cycle
	STD TOC2,X
	LDD TOC1,X	;get last tcnt value
	ADDD #PERIOD	;increase by one period
	STD TOC1,X	;store it in TOC1

WAIT4	BRCLR TFLG1,X $80 WAIT4		;poll oc1 flag
	LDAB #$C0			;got it so clear oc1 lag
	STAB TFLG1,X			;clear both flags
	LDD TOC1,X			;get last tco1 count
	

;4444444444444444444444444444444444444444444444444444444444444444444

;5555555555555555555555555555555555555555555555555555555555555555555


	ADDD VAR1	;add the duty cycle
	STD TOC2,X
	LDD TOC1,X	;get last tcnt value
	ADDD #PERIOD	;increase by one period
	STD TOC1,X	;store it in TOC1

WAIT5	BRCLR TFLG1,X $80 WAIT5		;poll oc1 flag
	LDAB #$C0			;got it so clear oc1 lag
	STAB TFLG1,X			;clear both flags
	LDD TOC1,X			;get last tco1 count
	
;55555555555555555555555555555555555555555555555555555555555555555

	;end of 0.1s loop, start over again
	JMP Loop1

	
; End of code

;look up table or digits for encoder direction
	ORG $F000
	DC.B $0B, $B8 ; 3000 : 0
	DC.B $0B, $BF ; 3007 : 1
	DC.B $0B, $C6 ; 3014 : 2
	DC.B $0B, $CD ; 3021 : 3
	DC.B $0B, $D4 ; 3028 : 4
	DC.B $0B, $DB ; 3035 : 5
	DC.B $0B, $E2 ; 3042 : 6
	DC.B $0B, $E9 ; 3049 : 7
	DC.B $0B, $F0 ; 3056 : 8
	DC.B $0B, $F7 ; 3063 : 9
	DC.B $0B, $FE ; 3070 : 10
	DC.B $0C, $05 ; 3077 : 11
	DC.B $0C, $0C ; 3084 : 12
	DC.B $0C, $13 ; 3091 : 13
	DC.B $0C, $1A ; 3098 : 14
	DC.B $0C, $21 ; 3105 : 15
	DC.B $0C, $28 ; 3112 : 16
	DC.B $0C, $2F ; 3119 : 17
	DC.B $0C, $36 ; 3126 : 18
	DC.B $0C, $3D ; 3133 : 19
	DC.B $0C, $44 ; 3140 : 20
	DC.B $0C, $4B ; 3147 : 21
	DC.B $0C, $52 ; 3154 : 22
	DC.B $0C, $59 ; 3161 : 23
	DC.B $0C, $60 ; 3168 : 24
	DC.B $0C, $67 ; 3175 : 25
	DC.B $0C, $6E ; 3182 : 26
	DC.B $0C, $75 ; 3189 : 27
	DC.B $0C, $7C ; 3196 : 28
	DC.B $0C, $83 ; 3203 : 29
	DC.B $0C, $8A ; 3210 : 30
	DC.B $0C, $91 ; 3217 : 31
	DC.B $0C, $98 ; 3224 : 32
	DC.B $0C, $9F ; 3231 : 33
	DC.B $0C, $A6 ; 3238 : 34
	DC.B $0C, $AD ; 3245 : 35
	DC.B $0C, $B4 ; 3252 : 36
	DC.B $0C, $BB ; 3259 : 37
	DC.B $0C, $C2 ; 3266 : 38
	DC.B $0C, $C9 ; 3273 : 39
	DC.B $0C, $D0 ; 3280 : 40
	DC.B $0C, $D7 ; 3287 : 41
	DC.B $0C, $DE ; 3294 : 42
	DC.B $0C, $E5 ; 3301 : 43
	DC.B $0C, $EC ; 3308 : 44
	DC.B $0C, $F3 ; 3315 : 45
	DC.B $0C, $FA ; 3322 : 46
	DC.B $0D, $01 ; 3329 : 47
	DC.B $0D, $08 ; 3336 : 48
	DC.B $0D, $0F ; 3343 : 49
	DC.B $0D, $16 ; 3350 : 50
	DC.B $0D, $1D ; 3357 : 51
	DC.B $0D, $24 ; 3364 : 52
	DC.B $0D, $2B ; 3371 : 53
	DC.B $0D, $32 ; 3378 : 54
	DC.B $0D, $39 ; 3385 : 55
	DC.B $0D, $40 ; 3392 : 56
	DC.B $0D, $47 ; 3399 : 57
	DC.B $0D, $4E ; 3406 : 58
	DC.B $0D, $55 ; 3413 : 59
	DC.B $0D, $5C ; 3420 : 60
	DC.B $0D, $63 ; 3427 : 61
	DC.B $0D, $6A ; 3434 : 62
	DC.B $0D, $71 ; 3441 : 63
	DC.B $0D, $78 ; 3448 : 64
	DC.B $0D, $7F ; 3455 : 65
	DC.B $0D, $86 ; 3462 : 66
	DC.B $0D, $8D ; 3469 : 67
	DC.B $0D, $94 ; 3476 : 68
	DC.B $0D, $9B ; 3483 : 69
	DC.B $0D, $A2 ; 3490 : 70
	DC.B $0D, $A9 ; 3497 : 71
	DC.B $0D, $B0 ; 3504 : 72
	DC.B $0D, $B7 ; 3511 : 73
	DC.B $0D, $BE ; 3518 : 74
	DC.B $0D, $C5 ; 3525 : 75
	DC.B $0D, $CC ; 3532 : 76
	DC.B $0D, $D3 ; 3539 : 77
	DC.B $0D, $DA ; 3546 : 78
	DC.B $0D, $E1 ; 3553 : 79
	DC.B $0D, $E8 ; 3560 : 80
	DC.B $0D, $EF ; 3567 : 81
	DC.B $0D, $F6 ; 3574 : 82
	DC.B $0D, $FD ; 3581 : 83
	DC.B $0E, $04 ; 3588 : 84
	DC.B $0E, $0B ; 3595 : 85
	DC.B $0E, $12 ; 3602 : 86
	DC.B $0E, $19 ; 3609 : 87
	DC.B $0E, $20 ; 3616 : 88
	DC.B $0E, $27 ; 3623 : 89
	DC.B $0E, $2E ; 3630 : 90
	DC.B $0E, $35 ; 3637 : 91
	DC.B $0E, $3C ; 3644 : 92
	DC.B $0E, $43 ; 3651 : 93
	DC.B $0E, $4A ; 3658 : 94
	DC.B $0E, $51 ; 3665 : 95
	DC.B $0E, $58 ; 3672 : 96
	DC.B $0E, $5F ; 3679 : 97
	DC.B $0E, $66 ; 3686 : 98
	DC.B $0E, $6D ; 3693 : 99
	DC.B $0E, $74 ; 3700 : 100
	DC.B $0E, $7B ; 3707 : 101
	DC.B $0E, $82 ; 3714 : 102
	DC.B $0E, $89 ; 3721 : 103
	DC.B $0E, $90 ; 3728 : 104
	DC.B $0E, $97 ; 3735 : 105
	DC.B $0E, $9E ; 3742 : 106
	DC.B $0E, $A5 ; 3749 : 107
	DC.B $0E, $AC ; 3756 : 108
	DC.B $0E, $B3 ; 3763 : 109
	DC.B $0E, $BA ; 3770 : 110
	DC.B $0E, $C1 ; 3777 : 111
	DC.B $0E, $C8 ; 3784 : 112
	DC.B $0E, $CF ; 3791 : 113
	DC.B $0E, $D6 ; 3798 : 114
	DC.B $0E, $DD ; 3805 : 115
	DC.B $0E, $E4 ; 3812 : 116
	DC.B $0E, $EB ; 3819 : 117
	DC.B $0E, $F2 ; 3826 : 118
	DC.B $0E, $F9 ; 3833 : 119
	DC.B $0F, $00 ; 3840 : 120
	DC.B $0F, $07 ; 3847 : 121
	DC.B $0F, $0E ; 3854 : 122
	DC.B $0F, $15 ; 3861 : 123
	DC.B $0F, $1C ; 3868 : 124
	DC.B $0F, $23 ; 3875 : 125
	DC.B $0F, $2A ; 3882 : 126
	DC.B $0F, $31 ; 3889 : 127
	DC.B $08, $38 ; 2104 : -128
	DC.B $08, $3F ; 2111 : -127
	DC.B $08, $46 ; 2118 : -126
	DC.B $08, $4D ; 2125 : -125
	DC.B $08, $54 ; 2132 : -124
	DC.B $08, $5B ; 2139 : -123
	DC.B $08, $62 ; 2146 : -122
	DC.B $08, $69 ; 2153 : -121
	DC.B $08, $70 ; 2160 : -120
	DC.B $08, $77 ; 2167 : -119
	DC.B $08, $7E ; 2174 : -118
	DC.B $08, $85 ; 2181 : -117
	DC.B $08, $8C ; 2188 : -116
	DC.B $08, $93 ; 2195 : -115
	DC.B $08, $9A ; 2202 : -114
	DC.B $08, $A1 ; 2209 : -113
	DC.B $08, $A8 ; 2216 : -112
	DC.B $08, $AF ; 2223 : -111
	DC.B $08, $B6 ; 2230 : -110
	DC.B $08, $BD ; 2237 : -109
	DC.B $08, $C4 ; 2244 : -108
	DC.B $08, $CB ; 2251 : -107
	DC.B $08, $D2 ; 2258 : -106
	DC.B $08, $D9 ; 2265 : -105
	DC.B $08, $E0 ; 2272 : -104
	DC.B $08, $E7 ; 2279 : -103
	DC.B $08, $EE ; 2286 : -102
	DC.B $08, $F5 ; 2293 : -101
	DC.B $08, $FC ; 2300 : -100
	DC.B $09, $03 ; 2307 : -99
	DC.B $09, $0A ; 2314 : -98
	DC.B $09, $11 ; 2321 : -97
	DC.B $09, $18 ; 2328 : -96
	DC.B $09, $1F ; 2335 : -95
	DC.B $09, $26 ; 2342 : -94
	DC.B $09, $2D ; 2349 : -93
	DC.B $09, $34 ; 2356 : -92
	DC.B $09, $3B ; 2363 : -91
	DC.B $09, $42 ; 2370 : -90
	DC.B $09, $49 ; 2377 : -89
	DC.B $09, $50 ; 2384 : -88
	DC.B $09, $57 ; 2391 : -87
	DC.B $09, $5E ; 2398 : -86
	DC.B $09, $65 ; 2405 : -85
	DC.B $09, $6C ; 2412 : -84
	DC.B $09, $73 ; 2419 : -83
	DC.B $09, $7A ; 2426 : -82
	DC.B $09, $81 ; 2433 : -81
	DC.B $09, $88 ; 2440 : -80
	DC.B $09, $8F ; 2447 : -79
	DC.B $09, $96 ; 2454 : -78
	DC.B $09, $9D ; 2461 : -77
	DC.B $09, $A4 ; 2468 : -76
	DC.B $09, $AB ; 2475 : -75
	DC.B $09, $B2 ; 2482 : -74
	DC.B $09, $B9 ; 2489 : -73
	DC.B $09, $C0 ; 2496 : -72
	DC.B $09, $C7 ; 2503 : -71
	DC.B $09, $CE ; 2510 : -70
	DC.B $09, $D5 ; 2517 : -69
	DC.B $09, $DC ; 2524 : -68
	DC.B $09, $E3 ; 2531 : -67
	DC.B $09, $EA ; 2538 : -66
	DC.B $09, $F1 ; 2545 : -65
	DC.B $09, $F8 ; 2552 : -64
	DC.B $09, $FF ; 2559 : -63
	DC.B $0A, $06 ; 2566 : -62
	DC.B $0A, $0D ; 2573 : -61
	DC.B $0A, $14 ; 2580 : -60
	DC.B $0A, $1B ; 2587 : -59
	DC.B $0A, $22 ; 2594 : -58
	DC.B $0A, $29 ; 2601 : -57
	DC.B $0A, $30 ; 2608 : -56
	DC.B $0A, $37 ; 2615 : -55
	DC.B $0A, $3E ; 2622 : -54
	DC.B $0A, $45 ; 2629 : -53
	DC.B $0A, $4C ; 2636 : -52
	DC.B $0A, $53 ; 2643 : -51
	DC.B $0A, $5A ; 2650 : -50
	DC.B $0A, $61 ; 2657 : -49
	DC.B $0A, $68 ; 2664 : -48
	DC.B $0A, $6F ; 2671 : -47
	DC.B $0A, $76 ; 2678 : -46
	DC.B $0A, $7D ; 2685 : -45
	DC.B $0A, $84 ; 2692 : -44
	DC.B $0A, $8B ; 2699 : -43
	DC.B $0A, $92 ; 2706 : -42
	DC.B $0A, $99 ; 2713 : -41
	DC.B $0A, $A0 ; 2720 : -40
	DC.B $0A, $A7 ; 2727 : -39
	DC.B $0A, $AE ; 2734 : -38
	DC.B $0A, $B5 ; 2741 : -37
	DC.B $0A, $BC ; 2748 : -36
	DC.B $0A, $C3 ; 2755 : -35
	DC.B $0A, $CA ; 2762 : -34
	DC.B $0A, $D1 ; 2769 : -33
	DC.B $0A, $D8 ; 2776 : -32
	DC.B $0A, $DF ; 2783 : -31
	DC.B $0A, $E6 ; 2790 : -30
	DC.B $0A, $ED ; 2797 : -29
	DC.B $0A, $F4 ; 2804 : -28
	DC.B $0A, $FB ; 2811 : -27
	DC.B $0B, $02 ; 2818 : -26
	DC.B $0B, $09 ; 2825 : -25
	DC.B $0B, $10 ; 2832 : -24
	DC.B $0B, $17 ; 2839 : -23
	DC.B $0B, $1E ; 2846 : -22
	DC.B $0B, $25 ; 2853 : -21
	DC.B $0B, $2C ; 2860 : -20
	DC.B $0B, $33 ; 2867 : -19
	DC.B $0B, $3A ; 2874 : -18
	DC.B $0B, $41 ; 2881 : -17
	DC.B $0B, $48 ; 2888 : -16
	DC.B $0B, $4F ; 2895 : -15
	DC.B $0B, $56 ; 2902 : -14
	DC.B $0B, $5D ; 2909 : -13
	DC.B $0B, $64 ; 2916 : -12
	DC.B $0B, $6B ; 2923 : -11
	DC.B $0B, $72 ; 2930 : -10
	DC.B $0B, $79 ; 2937 : -9
	DC.B $0B, $80 ; 2944 : -8
	DC.B $0B, $87 ; 2951 : -7
	DC.B $0B, $8E ; 2958 : -6
	DC.B $0B, $95 ; 2965 : -5
	DC.B $0B, $9C ; 2972 : -4
	DC.B $0B, $A3 ; 2979 : -3
	DC.B $0B, $AA ; 2986 : -2
	DC.B $0B, $B1 ; 2993 : -1

	;look up table or digits for encoder direction
	ORG $FF00
	DC.B $00,$10,$08,$00,$08,$00,$00,$10,$10,$00,$00,$08,$00,$08,$10,$00

; Define Power-On Reset Interrupt Vector

	ORG $FFFE ; $FFFE, $FFFF = Power-On Reset Int. Vector Location
	FDB Start ; Specify instruction to execute on power up