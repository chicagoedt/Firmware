/*
 *  		JSDC_System_Monitor.c
 *  Rev: 		2.0.1
 *  Microcontroller:	Atmega32U4
 *  Created: 		03/02/2014 11:16:45 PM
 *  Last Modified:	06/07/2014 12:55:00 PM
 *  Author: 		Zach Quinn
 *  Modified by: 	John Sabino, Zach Quinn
 *
 *  Revisions:	
 *	0.0.1 - (03/02/2014) - File Created.
 *	1.0.0 - (??/??/2014) - JSDC system finished.
 * 	1.1.0 - (05/29/2014) - John Sabino added preprocessor definitions, and added serial communications.
 *			       Code modified for Atmega32U4 instead of Atmega8. 
 *	2.0.0 - (06/01/2014) - Removed color functions to optimize for a single color function. 
 *			       Moved all initialization code to Initialization inline function for readability.
 *			       Reduced the number of global variables down to one.  
 *	2.0.1 - (06/07/2014) - Added a debug preprocessor flag and added debugging code. Also 0x10 is 
 *			       transmitted anytime that carrige return is detected.
 *
 *  Description:
 *	This system reads the 12V and 24V battery systems and reports an error if either system is running low. 
 *	It also reads two temperature sensors, which should be placed on a drive motor and a mechanism motor, 
 *	to ensure no overheating.
 */ 


//*******************************
//   Preprocessor Flags:
//*******************************
#define RS232_ENABLED
#define SERIAL_CONVERT

#undef REVISED_HARDWARE

#undef DEBUG_ENABLED
//#######################################

		//green and red 255 =yellow
		//red and blue = pink

//*******************************
//   Preprocessor Definitions:
//*******************************

#define F_CPU 				16000000UL

#define Blank_LEDS 			PINF0		//PINB1
#define Shift_Latch 			PINF1		//PINB2
#define Serial_Data 			PINB2		//PINB3
#define Serial_Clock 			PINB1		//PINB5

#define regularBright 			(255)
#define shiftedBright 			(255)

#define RED				(1)
#define BLUE				(2)
#define GREEN				(3)
#define YELLOW				(4)
#define ORANGE				(5)
#define PINK				(6)
#define WHITE				(7)
#define BLANK				(8)
#define ALTERNATE_BLUE_AND_RED 		(9)

#define MAX_RPM 			(70)
#define FORWARD_THRESHOLD_RPM 		(MAX_RPM * 0.9)

#define Initial_Delay_Time		(1000)	//milliseconds

#ifdef RS232_ENABLED
	#define BAUD 			115200
#endif	//End RS232_ENABLED

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//	Libraries:
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdbool.h>

#ifdef RS232_ENABLED
	#include <util/setbaud.h>
#endif	//End RS232_ENABLED


//######################################
//	Macros:
//######################################
/**
 *      @brief This function takes in data and transmit it over serial.
 *      @author John Sabino johntsabino@gmail.com
*/
#ifdef RS232_ENABLED
	#ifdef SERIAL_CONVERT
        	#define USART_Transmit(_Data, _Index) {uint8_t _j; for (_j = 0; _j < _Index; ++_j) { while (!( UCSR1A & (1<<UDRE1))); UDR1 = ((_Data[_j] << 1) ^ 0xff);}}

	#else
        	#define USART_Transmit(_Data, _Index) {uint8_t _j; for (_j = 0; _j < _Index; ++_j) { while (!( UCSR1A & (1<<UDRE1))); UDR1 = _Data[_j];}}
	#endif //SERIAL_CONVERT

#endif //RS232_ENABLED

/**
 *	@brief This is used to simplify repeated code fragments making them easier to read. 
 *	@author Zach Quinn , John Sabino johntsabino@gmail.com
*/
#define SPI_SEND(_DATA) {SPDR = _DATA;	while ((SPSR & (1 << SPIF)) == 0);}

//######################################

//-------------------------------------
//	Function Prototypes:
//-------------------------------------
inline void Initialization	(void);

#ifdef RS232_ENABLED
	inline void Read_USART	(void);	
#endif //End RS232_ENABLED

void Select_Color		(uint16_t * Flags);
void Set_LEDS 			(uint8_t  * index);
//-------------------------------------


/////////////////////////////////////
//	Global Variables:
/////////////////////////////////////
static uint8_t Current_Color;
/////////////////////////////////////

int main(void)
{	
	Initialization();

//_delay_ms(5000);

    	while(1)
    	{	
		#ifdef RS232_ENABLED
			Read_USART();	
		#endif	//End RS232_ENABLED
	}//End while loop		
}//End main
		
//---------------------------------------------
//	Functions:
//---------------------------------------------

inline void Initialization(void)
{
	//Initialize the global variable:
	Current_Color = 0;		

	//SPI Setup:
	DDRB = (1 << Serial_Clock) | (1 << Serial_Data) | (1 << PORTB0);
	//Enable SPI, Master Mode, Fosc/16
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);

	//Setup Shift and Blank Pins:
	DDRF = (1 << Blank_LEDS) | (1 << Shift_Latch);

	_delay_ms(1);		//Wait 1 ms for the SPI system to catch up.

	#ifdef DEBUG_ENABLED	
		uint16_t init = 234;
		Select_Color(&init);	//Blanks the LEDs.

		_delay_ms(5000);
	
		init = 255;
		Select_Color(&init);
	#else
		uint16_t init = 1;
		Select_Color(&init);	//Set the LEDs to Blue and Red to identify that the system has not
					// received data yet, but is functional.

		_delay_ms(Initial_Delay_Time);
	#endif //End DEBUG_ENABLED

	
	//UART Setup:
	#ifdef RS232_ENABLED
		DDRD |= (1 << DDD3);
		
		UBRR1H = UBRRH_VALUE; /*Set baud rate*/
        	UBRR1L = UBRRL_VALUE; /*Set baud rate*/

		//Defined in util/setbaud.h:
		#if USE_2X
        		UCSR1A |= (1 << U2X1);  //Double the baud rate for asynchronous communication.
		#else
        		UCSR1A &= ~(1 << U2X1);
		#endif //End USE_2X

        	//Set to no parity and in Asynchronous mode.
        	//1 Stop bit.
        	//1 Start bit.
        	//Set to 8-bit data.
        	UCSR1C |= (1 << UCSZ11) | (1 << UCSZ10);

		//Enable the Rx and Tx lines.
	        UCSR1B |= (1 << TXEN1) | (1 << RXEN1);

		_delay_ms(10);	//Hold for ten milliseconds to make sure all serial systems are online.
	
		#ifdef REVISED_HARDWARE
			PCICR  = PCIE0;		//Enable Pin Change Global Interrupt. 
			PCMSK0 = PCINT4;	//Enable Pin Change Interrupt system.

			sei();	//Enable global interrupts.

			//Setup the PCINT4 (PINB4) interrupt to re-initialize telemetry.

			//Wait until the RoboteQ has signaled the power LED signifying that the unit is online:
			while(!(PINB & PINB4));		

		#endif //End REVISED_HARDWARE

		//Setup Telemetry String in the RoboteQ:
		///NOTE: IF WE SET THE TELEMETRY UP THIS WAY REMOVE THE TELEMETRY FROM THE RoboteQ driver.
		///Also this would prevent any issues from occuring in the instance that the RoboteQ is rebooted.

		const unsigned char Telemetry [] = "# C\r?S\r# 200\r";
		USART_Transmit(Telemetry, 13);

		_delay_ms(10);	//Wait for ten milliseconds to make sure everything went thourgh.

	#endif	//End RS232_ENABLED
}//End Initialization

//----------------------------------------------------------

#ifdef RS232_ENABLED
	inline void Read_USART(void)
	{
		uint8_t Received_Data;
		int16_t Temp_Value;	
		int16_t Temp   = 0;
		uint8_t Number = 0;
		uint16_t Flags = 0;

		//Flags bit definitions:
		//Bit   (Meaning)
		//------------------------- 
		//15	(Reserved)
		//14	(Reserved)
		//13	(Reserved)
		//12	(Reserved)
		//11	(Wheel 1 > Wheel 2)
		//10	(Wheel 1 < Wheel 2)
		//9	(Wheel 1 is stopped)
		//8	(Wheel 2 is stopped)
		//7	('S' Found) 
		//6	('=' Found)
		//5 	(':' Found) 
		//4	(Wheel 1 is negative) 
		//3	(Wheel 2 is negative) 
		//2	('\r' Found)
		//1	(Wheel 1 above threshold)
		//0	(Wheel 2 above threshold)
		//=========================	

		while (1)
		{
			while (!(UCSR1A & (1<<RXC1)));  /*Wait for data to be recieved.*/
			Received_Data = UDR1;
		
			#ifdef DEBUG_ENABLED
				while (!( UCSR1A & (1<<UDRE1))); 
				UDR1 = ((Received_Data << 1) ^ 0xff);		
			
			#endif	//End DEBUG_ENABLED

			switch(Received_Data)
			{
		//		case 'S':
				case 0x56:	//'S'
					Flags = 128;
					Temp_Value = 0;	
				break;

				//case '=':
				case 0x61:	// '='
					Flags |= 64;
				break;

				//case ':':
				case 0x31:	//':'
					Flags |= 32;
					if (Temp_Value > FORWARD_THRESHOLD_RPM)		{Flags |= 2;}
					else if (Temp_Value == 0)			{Flags |= 512;}
					
					if (Flags & 16)					{Temp_Value *= -1;}	
					Temp = Temp_Value;	//Move over to run a comparison.
					Temp_Value = 0;	//Clear the temporary value for use with the next wheel.
				break;

				//case '-':
				case 0x69:	//'-'
					if (Flags & 32)		{Flags |= 8;}
					else			{Flags |= 16;}
				break;

				//case '\r':
				case 0x79:	//'\r'
					#ifdef DEBUG_ENABLED
						//Send 0x10 anytime that carrige return is detected.
						while (!( UCSR1A & (1<<UDRE1)));
						UDR1 = ((0x10 << 1) ^ 0xff);
					#endif //End DEBUG_ENABLED
					
					Flags |= 4;	
					if (Temp_Value > FORWARD_THRESHOLD_RPM) 	{Flags |= 1;}
					else if (Temp_Value == 0)			{Flags |= 256;}

                                        if (Flags & 8)                                  {Temp_Value *= -1;}

					if (Temp_Value > Temp)				{Flags |= 1024;}
					else if (Temp_Value < Temp)			{Flags |= 2048;}
			
						//Temp_Value will be cleared on the next evaluation.
					Select_Color(&Flags);
				break;
	
				//All possible numbers:
				case 0x63:	//9
					if (!(Flags & 192))     {break;}
                                        Temp_Value *= 10;
                                        Temp_Value += 9;
				break;

				case 0x0c:	//8
					if (!(Flags & 192))     {break;}
                                        Temp_Value *= 10;
                                        Temp_Value += 8;
				break;

				case 0x64:	//7
					if (!(Flags & 192))     {break;}
                                        Temp_Value *= 10;
                                        Temp_Value += 7;
				break;

				case 0x32:	//6
					if (!(Flags & 192))     {break;}
                                        Temp_Value *= 10;
                                        Temp_Value += 6;
				break;

				case 0x65:	//5
					if (!(Flags & 192))     {break;}
                                        Temp_Value *= 10;
                                        Temp_Value += 5;
				break;

				case 0x19:	//4
					if (!(Flags & 192))     {break;}
                                        Temp_Value *= 10;
                                        Temp_Value += 4;
				break;

				case 0x66:	//3
					if (!(Flags & 192))     {break;}
                                        Temp_Value *= 10;
                                        Temp_Value += 3;
				break;

				case 0x33:	//2
					if (!(Flags & 192))     {break;}
                                        Temp_Value *= 10;
                                        Temp_Value += 2;
				break;

				case 0x67:	//1
					if (!(Flags & 192))     {break;}
                                        Temp_Value *= 10;
                                        Temp_Value += 1;
				break;

				case 0x06:	//0	
					if (!(Flags & 192))	{break;}
					Temp_Value *= 10;
					Temp_Value += 0;	
				break;

				//All other characters:
				default:
					//Ignore and continue.
				break;
			}//End switch
		}//End while loop
	}//End Read_USART
#endif 	//End RS232_ENABLED

//--------------------------------------------------------------


/*Select_Color*/
void Select_Color(uint16_t * Flags)
{
	uint8_t New_Color;	
	uint8_t Sequence [18] = {0, 0, 0, 0, 0, 0,
				 0, 0, 0, 0, 0, 0,
				 0, 0, 0, 0, 0, 0};
		
	switch((*Flags))
	{

		//RED (Full Forward):
		case 231:
		case 1255:
		case 2279:
			Sequence [1]  = regularBright;
			Sequence [6]  = shiftedBright;
			Sequence [10] = regularBright;
			Sequence [15] = shiftedBright;
			New_Color     = RED;
		break;
		
		//BLUE (Full Reverse):
		case 255:
		case 1279:
		case 2303:
			Sequence [3]  = shiftedBright;
			Sequence [7]  = regularBright;
			Sequence [12] = shiftedBright;
			Sequence [16] = regularBright;
			New_Color     = BLUE;
		break;

		//GREEN (Stopped):
		case 996:
			Sequence [0]  = regularBright;
			Sequence [4]  = shiftedBright;
			Sequence [9]  = regularBright;
			Sequence [13] = shiftedBright;
			New_Color     = GREEN;
		break;

		//YELLOW (Reverse):
		case 252:
		case 1276:
		case 2300:
			Sequence [0]  = 255;
			Sequence [1]  = 255;
			Sequence [4]  = 255;
			Sequence [6]  = 255;
			Sequence [9]  = 255;
			Sequence [10] =	255;
			Sequence [13] = 255;
			Sequence [15] = 255;
			New_Color     = YELLOW;
		break;
		
		//PINK (Right Turn):
		case 1268:
		case 1269:
		case 1524:
		case 1526:
		case 1764:
		case 1765:
			Sequence [1]  = 255;
			Sequence [3]  = 255;
			Sequence [6]  = 255;
			Sequence [7]  = 255;
			Sequence [10] = 255;
			Sequence [12] =	255;
			Sequence [15] =	255;
			Sequence [16] = 255;
			New_Color     = PINK;
		break;

		//ORANGE (Forward):
		case 228:
		case 1252:
		case 2276:
			Sequence [0]  = 125;
			Sequence [1]  = regularBright;
			Sequence [2]  =  30;
			Sequence [4]  = 125;
			Sequence [6]  = regularBright;
			Sequence [9]  = 125;
			Sequence [10] = regularBright;
			Sequence [13] = 125;
			Sequence [15] = shiftedBright;
			New_Color     = ORANGE;
		break;

		//WHITE (Left Turn):
		case 2284:
		case 2286:
		case 2532:
		case 2534:
		case 2796:
		case 2797:
			Sequence [0]  = 255;
			Sequence [1]  = 255;
                        Sequence [2]  = 255;
                        Sequence [3]  = 255;
                        Sequence [4]  = 255;
                        Sequence [5]  = 255;
                        Sequence [6]  = 255;
                        Sequence [7]  = 255;
                        Sequence [8]  = 255;
                        Sequence [9]  = 255;
                        Sequence [10] = 255;
                        Sequence [11] = 255;
                        Sequence [12] = 255;
                        Sequence [13] = 255;
                        Sequence [14] = 255;
                        Sequence [15] = 255;
                        Sequence [16] = 255;
                        Sequence [17] = 255;
			New_Color     = WHITE;
		break;

		 //BLANK (Initial Setup):
                case 0:
                        //Send all zeros from the initialization at the start of the function.
                        New_Color     = BLANK;
                break;  

		//ALTERNATE_BLUE_AND_RED (All other inputs):
		default:
			Sequence [0]  = 10;
			Sequence [1]  = 10;
                        Sequence [2]  = 10;
                        Sequence [3]  = 10;
                        Sequence [4]  = 10;
                        Sequence [5]  = 10;
                        Sequence [6]  = 10;
                        Sequence [7]  = 10;
                        Sequence [8]  = 10;
                        Sequence [9]  = 10;
                        Sequence [10] = 10;
                        Sequence [11] = 10;
                        Sequence [12] = 10;
                        Sequence [13] = 10;
                        Sequence [14] = 10;
                        Sequence [15] = 10;
                        Sequence [16] = 10;
                        Sequence [17] = 10;
			New_Color     = ALTERNATE_BLUE_AND_RED;		
		break;		
	}//End switch 

	if (New_Color == Current_Color)			{return;} 
	
	Current_Color = New_Color;

	Set_LEDS(Sequence);

}//End Select_Color

//--------------------------------------------------------------


/*Set_LEDS*/
void Set_LEDS (uint8_t * index)
{
	PORTB |= (1<<Blank_LEDS);
	_delay_ms(1);
	PORTB &= ~(1<<Blank_LEDS);

	//----------------------------
	
	SPI_SEND(index[0]);
	
	SPI_SEND(index[1]);

	SPI_SEND(index[2]);
	
	SPI_SEND(index[3]);
	
	SPI_SEND(index[4]);	
	
	SPI_SEND(index[5]);

	//----------------------------
	
	SPI_SEND(index[6]);
	
	SPI_SEND(index[7]);

	SPI_SEND(index[8]);
	
	SPI_SEND(index[9]);
	
	SPI_SEND(index[10]);	
	
	SPI_SEND(index[11]);

	//----------------------------
	
	SPI_SEND(index[12]);
	
	SPI_SEND(index[13]);

	SPI_SEND(index[14]);
	
	SPI_SEND(index[15]);
	
	SPI_SEND(index[16]);	
	
	SPI_SEND(index[17]);

	//----------------------------
	
	SPI_SEND(index[0]);
	
	SPI_SEND(index[1]);

	SPI_SEND(index[2]);
	
	SPI_SEND(index[3]);
	
	SPI_SEND(index[4]);	
	
	SPI_SEND(index[5]);

	//----------------------------
	
	SPI_SEND(index[6]);
	
	SPI_SEND(index[7]);

	SPI_SEND(index[8]);
	
	SPI_SEND(index[9]);
	
	SPI_SEND(index[10]);	
	
	SPI_SEND(index[11]);

	//----------------------------
	
	SPI_SEND(index[12]);
	
	SPI_SEND(index[13]);

	SPI_SEND(index[14]);
	
	SPI_SEND(index[15]);
	
	SPI_SEND(index[16]);	
	
	SPI_SEND(index[17]);

	//-----------------------------

	PORTB |= (1<<Shift_Latch);
	_delay_ms(1);
	PORTB &= ~(1<<Shift_Latch);
	
}//End Set_LEDS

//===========================================
//		Interrupts:
//===========================================

#ifdef REVISED_HARDWARE
	ISR(PCINT0_vect)
	{
		
	}//End Pin Change Event Interrupt
#endif //End REVISED_HARDWARE
