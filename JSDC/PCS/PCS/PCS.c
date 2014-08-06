/*
 * PCS.c
 *
 * Created: 2/26/2014 11:27:40 AM
 *  Author: Zach
	This program uses a simple PID look to act as a controller for the EDT mechanisms.
	The position in captured via the Atmega's internal ADC, which is attached to a 10K SoftPot
	on the Mech (acting as a voltage divider).
	The position is then determined and compared to the desired position which is an input from
	the user. This input is captured on Timer Channel 1.
	The error between these two is then fed into the PID controller, which will determine
	how much the mechanism must be adjusted.
	
	**If the code isnt working, there is a good chance that the SoftPot is plugged in backwards**
	*/

#define F_CPU 8000000UL
//The following are general libraries
#include <stdint.h>			//General C library
#include <avr/io.h>			//For running C on Atmel Chips
#include <avr/interrupt.h>	//For executing interrupts on Atmel Chips
#include <avr/delay.h>		//For using an internal or external clock as a delay mechanism


// Pulse length in timer1 units
static uint16_t tPulse;
// Convert readings to timer1 ticks
// For F_CPU=8MHz and /8 prescaler no conversion is needed
#define US2TIMER1(x) (x)
void PWM_init(void)
{
	//Initialize Timer to PWM mode
	TCCR1A |= (1<<COM1B1)|(1<<WGM11)|(1<<WGM10);
	TCCR1B |= (1<<WGM13)|(1<<WGM12)|(1 << CS11);

	//Make sure to make OC0 pin (pin PB3 for atmega32) as output pin
	OCR1A = 20000;
	DDRD |= (1<<PD4);
	
}
void Capture_init(void)
{
	//TCCR1B |= (1 << CS10);	// /8 prescaler
	TIFR1 = (1 << ICF1);			// clear interrupt-flag
	TIMSK1 |= (1 << ICIE1);		// enable Timer1 input capture interrupt
}
void ADC_init(void)
{
	ADMUX |= (1<<REFS0);
	ADCSRA |= (1<<ADATE)|(1<<ADPS1)|(1<<ADPS0);
	ADCSRA |= (1<<ADEN);
	ADMUX |=(1<<MUX0);
	ADCSRA|=(1<<ADSC);
}
ISR(TIMER1_CAPT_vect)
{
	static uint16_t tStart;
	uint16_t t = ICR1;
	if (TCCR1B & (1<<ICES1))
	{
		// falling edge next
		TCCR1B &= ~(1<<ICES1);
		tStart = t;
	}
	else
	{
		// rising edge next
		TCCR1B |= (1<<ICES1);
		tPulse = t - tStart;
	}
}
int main(void)
{
	DDRB = 0xff;			//Debugging LEDs
	PORTB = 0x00;			//LEDs off
	int positionOut = 500;	//This is where the arm will start with no external input captured
	int prevPosition = 500; //This is to prevent an error in a future loop (before input is captured)
	int prevError = 0;		//This is initially zero since we haven't had a difference in current position vs desired position
	int iError = 0;			//Integral error is also zero
	float KP = 2.5;			//Proportional Gain Factor
	float KD = 0;			//Derivative Gain Factor
	float KI = 0;			//Integral Gain Factor
	
	Capture_init();		//Setup the system to be ready for input
	PWM_init();			//Get ready to go grab the desired position
	ADC_init();			//Get ready to go grab the current position
	DDRD |= (1<<PIND5);
	sei();
	while(1)
	{
		// Get current pulse time
		uint16_t t;
		cli();
		t = tPulse;
		sei();
		// Filter out errors (like the first pulse)
		if (t >= US2TIMER1(750) && t <= US2TIMER1(2250))
		{
			unsigned int setPosition = ( ((unsigned int)t - US2TIMER1(1000)));
			if ((setPosition < 0)||(setPosition> 800)) //These are the two endpoint that you will want to adjust.
			setPosition = prevPosition;				//Assuming you are currently within an accepted range,
			//you can then move to a new position
			
			int potPosition = ADC;		//Grab position from ADC
			if (ADC>=125)
			{
				PORTD |= (1<<PIND5);
				
			}
			else
			{
				PORTD &= ~(1<<PIND5);
			}
				//_delay_ms(1000);
			//Calculate errors
			int error = setPosition-potPosition;
			int dError = error - prevError;
			int PID = ((KP*error)+(KD*dError)+(KI*iError));	//Term to move
			
			//These both make sure you don't get something erratic when moving to desired position
			if ((PID)>500)
			PID=500;
			if ((PID)<-500)
			PID=-500;
			
			positionOut= 500 + PID;	//Position for the system to go to
			
			PORTB = PID/4;			//Allow us to get there in 4 steps
			//This is done in case a new desired position is detected
			//Now recalculate the error
			OCR1B = (positionOut+1000);
			prevError = error;
			prevPosition=setPosition;
			iError = iError+error;
		}
	}
}