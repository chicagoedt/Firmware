/*
 * EstopUART.c
 *
 * Created: 5/22/2018 10:30:32 AM
 * Author : Zach
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 1000000
#define BUAD 4800
#define BRC ((F_CPU/16/BUAD) - 1)
#define TIMEOUTSECONDS 2

//Output pins
#define OUTPUTESTOP PINC5
#define TRANSESTOP PINC4

//Input pins
#define MODE PIND3 // INT1
#define ROBOTESTOP PIND2 // INT0

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#include <util/delay.h>

void init_UART(void);
void USART_Transmit(unsigned char data);
void init_Timer0(void);
void checkTimeOut(void);
void checkCode(void);
void init_External(void);
void checkEstop(void);

volatile int quaterSeconds = 0;
volatile int estop = 0;
volatile unsigned char code = 0;
volatile int estop_r = 0;
volatile int estop_w = 0;
volatile int modePin = 0;


int main(void)
{
    DDRC = 0xFF; // Set PORTC as output
    DDRD &= ~((1<<PIND3)|(1<<PIND2)) ; // Set PORTD pins 2 and 3 as input
	
	init_UART();
	init_Timer0();
	init_External();

	sei();

    while (1) 
    {
		checkEstop();
    }
}


ISR(TIMER0_OVF_vect)
{
	quaterSeconds++; // 0.2604 seconds has elapsed increment counter

	checkTimeOut();
}


ISR(USART_RXC_vect)
{
	code = UDR; // store the RX data

	checkCode();
}


ISR(INT0_vect)
{
	//Estop button changed states
	if(PIND & (1<<PIND2))
	{
		estop_r = 0;
	}
	else
	{
		estop_r = 1;
	}

	checkEstop();
}


ISR(INT1_vect)
{
	//Mode changed states
	if(PIND & (1<<PIND3))
	{
		modePin = 1;
	}
	else
	{
		modePin = 0;
	}
}


void init_External(void)
{
	GICR = (1<<INT1)|(1<<INT0);

	MCUCR = (1<<ISC10)|(1<<ISC00);
}


void checkCode(void)
{
	if(code == 1)
	{
		quaterSeconds = 0; //Reset timeout timer
		estop_w = 0;
		//sbi(PORTC, PINC4);
	}
	else if(code == 2)
	{
		estop_w = 1;
		//cbi(PORTC, PINC4);
	}
}


void checkTimeOut(void)
{
	if(quaterSeconds >= (TIMEOUTSECONDS << 2)) // << 2 = * 4 convert seconds to quaterseconds
	{
		estop_w = 1;
		//cbi(PORTC, PINC4);
	}
}


void checkEstop(void)
{
	if(estop_r || estop_w)
	{
		estop = 1;
		sbi(PORTC, PINC5);
	}

	if(!estop_r && !estop_w)
	{
		estop = 0;
		cbi(PORTC, PINC5);
	}

	if(estop_r)
	{
		cbi(PORTC, PINC3);
	}
	else
	{
		sbi(PORTC, PINC3);
	}

	if(estop_w)
	{
		cbi(PORTC, PINC4);
	}
	else
	{
		sbi(PORTC, PINC4);
	}
}


void init_Timer0(void)
{
	TCCR0 = (1<<CS02)|(1<<CS00); //Scaler of 1024

	TIMSK = (1<<TOIE0); // Enable overflow interrupt
}


void init_UART(void)
{
	// Shift in BRC based on controller frequency and BUAD
	UBRRH = (BRC >> 8);
	UBRRL =  BRC;

	// Enable transmitter and receiver
	UCSRB = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE);

	// Set to 8bit frame with 1 stop bit
	UCSRC = (1<<URSEL)|(3<<UCSZ0);
}


void USART_Transmit(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSRA & (1<<UDRE)) );

	/* Put data into buffer, sends the data */
	UDR = data;
}

