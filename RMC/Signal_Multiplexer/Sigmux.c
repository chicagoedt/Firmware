/*
 *Organization:		University of Illinois at Chicago Engineering Design Team
 *Engineer(s):		John Sabino
 *E-Mail(s):		johntsabino@gmail.com
 *
 *Project Title:	Sigmux.c
 *Microcontroller:	Atmega32U4
 *
 *
 *Date Created:		09/05/2013
 *Last Date Modified:	11/24/2014
 *Last Modified by:	John Sabino
 *
 *
 *Fuse Configuration:
 *
 *-U lfuse:w:0x82:m 
 *-U hfuse:w:0x99:m 
 *-U efuse:w:0xf3:m 
 *
 *All Fuse calculations from http://www.engbedded.com/fusecalc/
 *
 *Purpose/Description:
 *	The purpose of the Sigmux is to be the central decision system of the robot. Its task is to constantly
 * check on the Rx and Tx pins for communications over the wi-fi, and appropriately changing modes, or forwarding * communications. At the same time the Sigmux is to listen for physical button presses on pins (TO BE ADDED) and * changing which mode it is in. Added functionality at a later time will be to analyze power consumption voltage * analysis and DC current analysis through a DC current transducer on pins(TO BE ADDED).
 * Focusing more on the actual multiplexing component, the Sigmux is designed to forward any and all motor
 * controller commands that are sent through the wi-fi interface, if they are from a controller, or from the
 * on-board computer system if the unit is set into 'Autonomous Mode'. 
 *
 *
 *Procedure:
 *	1)
 *
 *	2)
 *
 *	3) 
 *
 *
 *
 *Revisions:
 * 	0.1 (09/05/2013) - File created.
 *	0.2 (09/11/2013) - Micro-controller revised, not using Atmega8_16PU. Now using Atmega16U4.	
 *	0.3 (09/12/2013) - Created all compiler flags, function prototypes, and main.
 *	0.4 (09/15/2013) - Created an initialization inline function for reading simplicity.
 *	0.5 (09/18/2013) - Halting of all energy analysis code (Design change).
 *	0.6 (09/19/2013) - Begin CC3000 code.
 *	0.7 (10/01/2013) - Begin the initialization of the CC3000 through wlan_init(). 
 *	0.8 (10/07/2013) - Continue converting ccspi adafruit library to an AVR library in SPI.
 *	0.9 (10/08/2013) - Finished converting ccspi adafruit library to an AVR library in SPI.
 *	1.0 (10/09/2013) - Set up socket for UDP communication over the CC3000, and finished draft of wlan 
 *                         intitialization.
 *	1.1 (10/11/2013) - Begining setup of serial communication for motor controllers.
 *	1.2 (10/16/2013) - Added function to control muxes and demuxes for motor select.
 *	1.3 (10/30/2013) - Modified the Initialization for spi_init, since it is no longer a function, 
 *                         but an inline.
 *	1.4 (11/05/2013) - Disabled all Energy Analysis systems. 
 *	1.5 (11/06/2013) - Set up the Timer 1 Compare B interrupt and configured its initialization. 
 *	1.6 (11/22/2013) - CC3000 communications finished, the device now initializes without any issues, 
 *                         though more testing needs to be done to make sure the device is connecting 
 *                         appropriately.
 *	1.7 (11/26/2013) - Discovered that the MAC address of the CC3000 unit is:  08:00:28:57:55:16.
 *	1.8 (12/20/2013) - Reconfigured the wlan_connect() parameters (in initialization) based of TI examples.
 *	1.9 (01/03/2014) - Removed all the static functions since they only waste space and unused static 
 *                         variables. Also added the function Recieve_Wifi_Data() to collect the UDP packets 
 *                         and decipher them. Was added to main. 
 *	2.0 (01/03/2014) - Added a new volatile global variable to keep track of if DHCP is complete or not.
 *			   now we do not need to wait five seconds to see if DHCP completed.
 *	2.1 (01/07/2014) - Finished USART_Transmit macro (in macro section). Now transmits a char array.
 *	2.2 (01/08/2014) - Finished USART_Recieve macro (in macro section). 
 *	2.3 (01/09/2014) - Modified USART_Recieve macro to take a pointer to hold the number of characters
 *			   that were recieved. Also corrected the registers that were used to non-arbitrary 
 *		 	   registers (they were 'n's instead of '1's).
 *	2.4 (01/14/2014) - Modified UDP protocol to be one byte of data.
 *	2.5 (01/15/2014) - Added some preprocessor flag if statements to wi-fi functions. Included a USART baud 
 * 			   configuration library to minimize the design. And removed USART debug code in the
 *			   initialization code. 	
 *	2.6 (01/16/2014) - Determined the proper operating frequency for both the CC3000 and USART to be 
 *			   8MHz from the internal clock.
 *	2.7 (01/17/2014) - Set up USART debug test by broadcasting "GO!" on boot. Still does not work unless
 *			   data is sent in. So the RX line has been disabled to try to fix it. 
 *	2.8 (01/17/2014) - Added the fuse configuration to this file right after 'Last Date Modified'.
 *	2.9 (01/17/2014) - Added SERIAL_CONVERT preprocessor flag. Modified USART_Transmit to convert the input 
 *			   to be usable with transmission.
 *	3.0 (01/21/2014) - Commented out the serial initialization code and made a few corrections to make sure
 *			   the serial transmits only up to the '\r' character.
 *	3.1 (01/22/2014) - Fixed the issue with the socket timing out. The wireless device now will not need 
 *			   to communicate to sustain connection.
 *	3.2 (02/05/2014) - Added a Power LED on port E2 and a status LED on port C7.
 *	3.3 (02/06/2014) - Removed the Power LED. It is now based on the power rail.
 *	3.4 (03/12/2014) - Included the kill to linear actuators in safety mode.
 *	3.5 (03/27/2014) - Redesigned the decrypting protocols to accomodate the MDC22XX series.
 *	3.6 (04/02/2014) - Disabled Rx line to be repurposed as the external interrupt for the CC3000. The old 
 * 			   interrupt pin is to be repurposed for Two Wire Interface (I2C).
 *	3.7 (05/06/2014) - Removed all unused global variables to reduce space consumption. 
 *			   Modified Decrypt_Data to utilize one switch statement and to fit the 
 *			   two dual-channeled RoboteQ design.
 *	3.8 (05/07/2014) - Optimized the Decrypt_Data to run faster and consume less space. 
 *			   Also removed some unused pins.		
 *	3.9 (05/15/2014) - Redesigned Decrypt_Data to accomodate the operation of new dual channeled RoboteQ. 
 *	4.0 (05/17/2014) - Added the Manual E-GO command to the case statement in Decrypt_Data.
 *	4.1 (05/21/2014) - Added a pseudo watchdog system that uses timer 1 to reset the microcontroller if 
 *			   there is no activity for 30 seconds. Also optimized the modes using ECE 465 state
 *			   assignment methods (look in preprocessor definitions). Removed current_motor global
 *			   variable due to its lack of utility. Changed DHCP_Complete to a boolean definition
 *			   rather than making it a global variable. Added an Unassigned_Mode definition to 
 *			   be used on initialization. Updated Pin Assignments. 
 *	4.2 (05/22/2014) - Moved the initial Safety_Mode configuration to after UART initialization. Also 
 *			   a one second delay before it enters.	
 *	4.3 (06/27/2014) - Added a preprocessor flag for the TWI system called TWI_ENABLED. Added TWI related
 *			   macros. Added PinB4 as TWI Error LED.
 *	4.4 (11/24/2014) - Modified Decrypt_Data() to accomodate for a command change.
 *
 *	
 *TODO/NOTES:
 *	- See if there is a faster way to do delay commands rather than through 'util/delay.h' (if needed).  
 *	- Should the Sigmux do the power consumption check? Regardless, if we are going to do it would be best
 * 	  done with a microcontroller with a LARGE resolution with a small prescaler, an off-unit voltage source
 *	  (other than the battery) for reference voltage, and a DC current transducer to calculate power.
 *	- We will be using an external oscillator at 16 MHz.
 *
 *
 *
 *------------------------------------------------------------------------------------------------------*/

/*======================================================================================================
 *Pin Assignments:
 *
 * PB0 (SS)   :				[OUTPUT]  	{CC3000 CS} 
 * PB1 (SCLK) :				[OUTPUT]  	{CC3000 SCK}
 * PB2 (MOSI) :				[OUTPUT]  	{CC3000 MOSI}
 * PB3 (MISO) :				[INPUT]   	{CC3000 MISO}
 * PB5 (PCINT5/OC1A/!OC4B/ADC12) :	[OUTPUT]  	{CC3000 VBEN}
 * PB4 () :				[OUTPUT]	{TWI Error LED}
 * PB6 (...) :				[OUTPUT]	{CC3000 WLAN_Enabled LED}
 * PB7 () :				[OUTPUT]	{Demux Select Line 1}
 *
 * PC6 (OC3A/!OC4B) :  			[OUTPUT]  	{CC3000 DHCP Complete LED}
 * PC7 (ICP3/CLK0/OC4A) :		[OUTPUT]	{Status 0 LED}
 * 
 * PD0 (OC0B/SCL/INT0) :		[UNUSED]	{TWI SCL Line}
 * PD1 (SDA/INT1) :			[UNUSED]	{TWI SDA Line}
 * PD2 (RXD1/INT2) :			[INPUT]  	{CC3000 IRQ}
 * PD3 (TXD1) : 			[OUTPUT] 	{RoboteQ RS232 Tx}
 * PD5 (XCK1/!) : 			[OUTPUT] 	{Demux Select Line 0}
 * PD6 (T1/OC4D/ADC9) :			[UNUSED]	{Status 1 LED}
 * PD7 (T0/OC4D/ADC10) :		[UNUSED]	{Status 2 LED}
 *
 * PF0 (ADC0) :				[OUTPUT]	{Mux Select Line}
 * PF1 (ADC1) :				[UNUSED]  	{Battery Voltage}
 *
 * Pin 2 (UVcc) :			[USB ]
 * Pin 3 (D-  ) :			[USB Data]
 * Pin 4 (D+  ) :			[USB Data]
 * Pin 5 (UGnd) :			[USB ]
 * Pin 6 (UCap) :			[USB ]
 * Pin 7 (VBus) :			[USB ]
 *
 * Pin 16 (XTAL_2) :			[External Clock] {NOT USED}
 * Pin 17 (XTAL_1) :			[External Clock] {NOT USED}
 *
 * Pin 42 (ARef  ) :			[Reference Voltage]
 *
 *======================================================================================================*/

//----------------------------------------------------------------------------------------------------------
//Preprocesor Definitions:
//----------------------------------------------------------------------------------------------------------

/**
 * @brief Onboard Operating Frequency of uC in Hertz. This is required to be defined before importing the delay header to prevent warnings on compilation.
 *	View pages 29, 209, and 348 to determine the default configurations. The device is prescaled by 8 by default. 
*/
#ifndef F_CPU
	//#define F_CPU 32000000UL
        //#define F_CPU 16000000UL
	#define F_CPU 8000000UL
	//#define F_CPU 4000000
	//#define F_CPU 2000000
#endif
//----------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//Preprocessor Compiling Flags:
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
/**
	@breif DEBUG_ENABLED is a preprocessor flag that is used to identify what fragments of code are used for debugging and should be included in the code for compilation, otherwise it will entirely ignore (or remove) the code fragments (i.e. not compile them). 
	By default this flag is enabled. To disable all debugging code fragments, replace #define with #undef. 
*/
#define DEBUG_ENABLED


/**
	@breif WATCHDOG_ENABLED is a preprocessor flag that is used to enable/disable the watchdog timer and any calls to feed it. 
	By default this flag is enabled. To disable all watchdog related code, replace #define with #undef. 
*/
#define WATCHDOG_ENABLED


/**
	@breif SKIP_BOOT is a preprocessor flag that is used to skip the boot sequence to identify that the unit is 
operating appropriately when turned on. Though it may consume time that can be better spent running the core code.
	By default this flag is disabled. To enable the boot sequence, replace #undef with # define.
*/
#define SKIP_BOOT

/**
	@breif ENERGY_ANALYSIS_ENABLED is a preprocessor flag that is ued to enable/disable power analysis of the
battery system periodically.
	By default this flag is enabled. To disable power analysis, replace #define with #undef.
*/
#undef ENERGY_ANALYSIS_ENABLED

/**
	@breif POWER_SAVING is a preprocessor flag that is used to enable/disable MCU sleep commands for power consumption saving.
	By default this flag is disabled. To enable sleep commands, replace #undef with #define
*/
#undef POWER_SAVING

/**
	@breif CC3000_ENABLED is a preprocessor flag that is used to enable/disable the Adafruit CC3000 WiFi breakout board for debugging purposes.
	By default this flag is enabled. To disable the wifi, replace #define with #undef
*/
#define CC3000_ENABLED

/**
	@breif USB_ENABLED is a preprocessor flag that is used to enable/diable all USB communications (i.e. diable the autonomous mode).
	By default this flag is enabled. To disable the USB controller, replace #define with #undef
*/
#undef USB_ENABLED

/**

	@brief PWM_ENABLED is a preprocessor flag that is used to enable/disable PWM communication to the motor controllers. This will override the serial communication for controling the motor controller, though the serial lines are still used to query the motor controller.
	By default this flag is disabled. To use the PWM, replace #undef with #define
*/
#undef PWM_ENABLED

/**

	@breif SERIAL_CONVERT is a preprocessor flag that is used to convert the inputs for RS-232 into a usable output. 
	By default this flag is enabled. To take ascii value in as they are with no modification, replace #define with #undef

*/
#define SERIAL_CONVERT

/**
	@brief ROUTER_WATCHDOG_ENABLED is a preprocessor flag that enables a timer interrupt system that counts
up the time without a command to determine if the connection to the router has been lost if after thirty seconds
no command has come in, then assume that the router has rebooted and reboot the microcontroller.

	By default this flag is enabled. To disable this pseudo-watchdog system, replace #define with #undef
*/

#undef ROUTER_WATCHDOG_ENABLED

/**
	@brief TWI_ENABLED is a preprocessor flag that enables the Two-Wire-Interface (Atmel's equivalent 
to I2C). This is used to interface the Sigmux to the energy monitor system and any other peripherals over TWI.
	
	By default this flag is enabled. To disable the TWI system, replace #define with #undef
*/

#define TWI_ENABLED


/**
	@brief TEST_TWI is a preprocessor flag that is dependent upon TWI_ENABLED preprocessor flag. If TEST_TWI
is enabled then test code will be compiled otherwise it will be ignored.

	By default this flag is disabled. To enable the TWI test code, replace #undef with #define
*/

#undef TEST_TWI

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

//------------------------------------------------------------------------------------------------------------
//Preprocessor Definitions:
//------------------------------------------------------------------------------------------------------------
#define Energy_Analysis_Interval 		(100)		//100ms
#define Energy_Analysis_Reference 		(12)		//12V
#define Energy_Analysis_Resistance 		(10)   		//10 Ohms

//UART Definitions:
#define BAUD 					115200		//115200 bits/second


//CC3000 Definitions:
#define NETAPP_IPCONFIG_MAC_OFFSET		(20)
#define PORT					(2390)		//0x0956 //Port 2390
#define HEX_PORT_1				(0x09)
#define HEX_PORT_2				(0x56)
#define Source_IP				"192.168.2.10"	//Set this to whatever the IP of the transmitter is.
#define ROUTER_SSID				"Team_27"	//"Tomato24"
#define SSID_LENGTH				(7)	
#define CC3000_APP_BUFFER_SIZE			(2)		//(5)
#define CC3000_RX_BUFFER_OVERHEAD_SIZE		(20)
#define DHCP_Complete				(PORTC & (1 << PC6))	

//TWI Definitions:
#define Energy_Monitor_Address			(0x00)

//Sigmux Mode Definitions:
#define Unassigned_Mode				(4) //00000100			
#define Safety_Mode               		(0) //00000000		//(1)		//00000001
#define Autonomous_Mode           		(3) //00000011		//(2)		//00000010
#define RC_Mode               	  		(1) //00000001		//(4)		//00000100
#define Upload_Mode 		  		(2) //00000010		//(8)		//00001000
	
//------------------------------------------------------------------------------------------------------------



//Define all libraries that we are using:
#include <avr/io.h>                     //Needed for standard Micorcontroller I/O operations.
#include <stdbool.h>                    //Needed to use Boolean data types.
#include <avr/interrupt.h>              //Needed to use all types of interrupts.
#include <util/delay.h>                 //Needed to call the delay command.
#include <util/setbaud.h>		//Needed to test if we can set the baud rate correctly. 

#ifdef CC3000_ENABLED
	#include "CC3000HostDriver/spi.h"		//Needed for SPI communications.
	#include "CC3000HostDriver/hci.h"		//Needed for communication with the CC3000.
	#include "CC3000HostDriver/cc3000_common.h"
	#include "CC3000HostDriver/wlan.h"
	#include "CC3000HostDriver/socket.h"
	#include "CC3000HostDriver/evnt_handler.h"
	#include "CC3000HostDriver/netapp.h"
	#include "CC3000HostDriver/host_driver_version.h"
	#include "CC3000HostDriver/nvmem.h"		//If chaning profiles/modifying the nvmem of the CC3000
#endif	//End CC3000_ENABLED

#ifdef WATCHDOG_ENABLED
        #include <avr/wdt.h>
#endif	//End WATCHDOG_ENABLED

#ifdef POWER_SAVING
        #include <avr/sleep.h>
#endif	//End POWER_SAVING

#ifdef TWI_ENABLED
	#include <util/twi.h>				//Contains twi library
#endif 	//End TWI_ENABLED


//************************************************************************************************************
//Preprocessor Macros:
//************************************************************************************************************

/**
 *	@brief The SET macro is used as a means to simplify the code so that it is easier to read without
 *	having to make process consuming functions.
 *	
 *	The functionality of this is to set a register.
 *	
 *	Original design taken from signal_mux.c.
 *
 *	@author Anup Kotadia anup.kotadia@gmail.com
*/
#define SET(mask, reg) (reg |= (mask))

/**
 *	@brief The CLEAR macro is used as a means to simplify the code so that it is easier to read.
 *      @author Anup Kotadia anup.kotadia@gmail.com
*/
#define CLEAR(mask, reg) (reg &= ~(mask))

/**
 *	@brief This function takes in data and transmit it over serial.
 *	@author John Sabino johntsabino@gmail.com
*/
#ifdef SERIAL_CONVERT
	#define USART_Transmit(_Data, _Index) {uint8_t _j; for (_j = 0; _j < _Index; ++_j) { while (!( UCSR1A & (1<<UDRE1))); UDR1 = ((_Data[_j] << 1) ^ 0xff);}}

#else
	#define USART_Transmit(_Data, _Index) {uint8_t _j; for (_j = 0; _j < _Index; ++_j) { while (!( UCSR1A & (1<<UDRE1))); UDR1 = _Data[_j];}}
#endif
/**
 *	@brief This function scans for serial data input.
 *	@author John Sabino johntsabino@gmail.com
*/
#define USART_Recieve(_Output, _Num_Char) {uint8_t _k = 0; memset((unsigned char *)&_Output, 0, sizeof(_Output)); while(_Output[_k] != '\r') {while (!(UCSR1A & (1<<RXC1))); _Output[_k++] = UDR1;} *_Num_Char = k;}

/**
 *	@brief Feed_Watchdog() call the Watchdog Reset command from assemble to reset the watchdog timer.
 *	@author John Sabino johntsabino@gmail.com
*/
#define Feed_Watchdog()		{asm("WDR");}

/**
 *	@breif the TWI related macros are to better understand what the system is doing. See the datasheet page
 *	237 for more details.
 *	@author John Sabino johntsabino@gmail.com
*/
#define TWI_ERROR()		{PORTB |= (1 << PB4);}
#define TWI_SEND_START()	{TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);}
#define TWI_SEND_STOP()		{TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);}
#define TWI_TRANSMIT()          {TWCR = (1 << TWINT) | (1 << TWEN);}
#define TWI_RECIEVE()		{TWCR = (1 << TWINT) | (1 << TWEA)  | (1 << TWEN);}

#define TWI_WAIT_FOR_START()	{while (!(TWCR & (1<<TWINT)));}
#define TWI_CHECK_START()	{if ((TWSR & 0xF8) != 0x08) {TWI_ERROR();}}
#define TWI_CHECK_RECIEVE()	{if ((TWSR & 0xF8) != 0x40) {TWI_ERROR();}}
#define TWI_SEND_SLA_R()	{TWDR = Energy_Monitor_Address | TW_READ;}

//************************************************************************************************************

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//Type Definitions:
/*
typedef struct {
        uint8_t OpCode;
        int16_t Data [3];
        uint8_t Length;
}Motor_Controller_Command;

typedef struct {
	char Command     [5];
	char Parameter_1 [5];
	char Parameter_2 [5];
}Cached_Motor_Command;
*/
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//Function Prototypes:
inline void Initialization(void);
uint8_t Set_Mode(uint8_t Mode);
void Select_Motor_Controller(uint8_t Motor);

#ifdef USB_ENABLED
	inline void Enable_USB_Controller(void);
	inline void Disable_USB_Controller(void);
#endif

#ifdef CC3000_ENABLED
	//Required for CC3000 Initialization:	
	char* Send_Driver_Patch(unsigned long *usLength);
	char* Send_Boot_Loader_Patch(unsigned long *usLength);
	char* Send_WLFW_Patch(unsigned long *usLength);
	void CC3000_Unsynch_Call_Back(long event_type, char * data, unsigned char length);
	long Read_WLAN_Interrupt_Pin(void);
	void Write_WLAN_Pin(unsigned char val);
	void WLAN_Interrupt_Enable(void);
	void WLAN_Interrupt_Disable(void);
	
	//Functions to utilize the CC3000:
	uint8_t Recieve_WiFi_Data(void);
	uint8_t Decrypt_Data(unsigned char * Data);
#endif	//End CC3000_ENABLED

#ifdef TWI_ENABLED
	inline void Transmit_Energy_Data(void);
#endif  //End TWI_ENABLED
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//############################################################################################################
//Global Variables:
	
#ifdef ENERGY_ANALYSIS_ENABLED
//TODO: COULD IT BE EASIER TO INITIALIZE THE VARIABLES IN MAIN AND MAKE THESE POINTERS?
	static volatile uint32_t Samples_Taken = 0;	 
	static volatile uint32_t Total_Power   = 0;
#endif

#ifdef CC3000_ENABLED
	static long Socket_Handle;                    // = 0;
	//volatile unsigned char Data_Buffer[CC3000_APP_BUFFER_SIZE + CC3000_RX_BUFFER_OVERHEAD_SIZE];
	//static volatile long Length 	              = 0;
	//volatile uint8_t DHCP_Complete		      = 0;
	//static sockaddr Source_Address;
	//static socklen_t Source_Address_Size 	      = 0;  
	static sockaddr Mission_Control_Address;
//static unsigned long Rx_Packet_Length 	      = 0;
#endif

//THIS COULD BE MORE EFFICIENT:
static volatile uint8_t Current_Mode;		// = 0;
//Motor Controlling Variables:
//static volatile uint8_t Current_Motor	      	      = 0;

#ifdef ROUTER_WATCHDOG_ENABLED
	static volatile uint8_t Count;		// = 0;
#endif	//End ROUTER_WATCHDOG_ENABLED

//############################################################################################################

int main (void)
{
	Initialization();

	while(1)
	{
		#ifdef CC3000_ENABLED
			Recieve_WiFi_Data();
		#endif

		#ifdef ROUTER_WATCHDOG_ENABLED
			//Call the reset vector to restart the system from the beginning.
                	if (Count >= 6)         {((void (*)(void))0)();}
		/*	if (Count >= 6)		//{asm("JMP 0x0000");}
			{
				PORTC |= (1 << PORTC7);
				PORTB &= ~(1 << PORTB6);
				if (Count >= 8)	{asm("JMP 0x0000");}
			}*/
		#endif

		#ifdef POWER_SAVING
			//COULD IT BE BETTER TO NOT JUST SLEEP, BUT POWER DOWN THE CPU?
			sleep_cpu(); /*Enter sleep mode.*/
		#endif
	}//End while loop

	return 0;
}//End main


//------------------------------------------------------------------------------------------------------
//Functions:
//------------------------------------------------------------------------------------------------------



/*Initialization*/

/**
	@brief	Initialization is used to configure all of the registers of the microcontroller

		The purpose of this function is to improve readability. By making it an inline function the code is efficient as possible and 
	@author John Sabino
*/
inline void Initialization (void)
{
	 #ifdef WATCHDOG_ENABLED
		wdt_enable(WDTO_8S);
	 #endif	


	//Turn on the Power LED to identify that the device is on.
	//DDRE |= (1 << DDE2);		//POWER LED
	DDRC |= (1 << DDC7);		//STATUS LED
	//PORTE |= (1 << PORTE2);		//TURN ON POWER LED

        //Set up the LEDs for WLAN_ON and DHCP:
        DDRB |= (1 << DDB6);    //WLAN_INIT LED
        DDRC |= (1 << DDC6);    //DHCP_Complete LED


#ifndef SKIP_BOOT
	DDRB |= (1 << DDB4);
	DDRD |= (1 << DDD7);
	DDRD |= (1 << DDD6);

	PORTB |= (1 << PB4);
	_delay_ms(1000);
	PORTD |= (1 << PD7);
	_delay_ms(1000);
	PORTD |= (1 << PD6);
	_delay_ms(1000);
	PORTB &= ~(1 << PB4);
	_delay_ms(1000);
	PORTD &= ~(1 << PD7);
	_delay_ms(1000);
	PORTD &= ~(1 << PD6);
#endif

		
	//First thing, set the Sigmux to Safe Mode.
	//Set_Mode(Safety_Mode);

	#ifdef ENERGY_ANALYSIS_ENABLED
		//Enable Timer/Counter0 Interrupt on compare match of OCR0A:
		TIMSK0 = (1 << OCIE0A); 		

		//Set the Output Compare Register for the timer to compare against:
		OCR0A = Energy_Analysis_Interval;

		//Configure the ADC to have the reference pin be AREF on pin 21, and make sure everything is set to defaults:
		ADMUX = 0x00;	
		
		//Enable the Analog to Digital Conversion (ADC):
		ADCSRA = (1 << ADEN);		//25 Clock cycles to initialize.	


	#endif	//ENERGY_ANALYSIS_ENABLED





	#ifdef CC3000_ENABLED

		//Enable the CC3000, and setup the SPI configurations.
		init_spi();

		//Set up the CC3000 API for communication.
		wlan_init(CC3000_Unsynch_Call_Back, 
			  Send_WLFW_Patch, 
			  Send_Driver_Patch, 
			  Send_Boot_Loader_Patch, 
			  Read_WLAN_Interrupt_Pin, 
			  WLAN_Interrupt_Enable, 
			  WLAN_Interrupt_Disable, 
			  Write_WLAN_Pin);
 
		PORTB |= (1 << PB6);	//Set the WLAN_INIT LED on.

		//Set the connection policy:
//wlan_ioctl_set_connection_policy(0, 1, 1);      //Only auto-connect to profiles specified in the user profile.

///DEBUG LINE:
//PORTB |= (1 << PB4);

///DEBUG LINE:
//PORTB |= (1 << PB4);

		sei();
	
//PORTB |= (1 << PB4);

		//Enable the CC3000, and wait for initialization process to finish.
		wlan_start(0);
//PORTB &= ~(1 << PB4);
		wlan_set_event_mask(HCI_EVNT_WLAN_KEEPALIVE|HCI_EVNT_WLAN_UNSOL_INIT|HCI_EVNT_WLAN_ASYNC_PING_REPORT);

///DEBUG LINE:
//PORTD |= (1 << PD7);
//PORTD |= (1 << PD6);

/* //In case we need to have a char array.
		char * SSID 			= "Tomato24";
		unsigned long SSID_Length 	= 8;
*/
		//if (wlan_connect(WLAN_SEC_WPA, "chicagoedt", 10, "chicagoedt", "notrightnow", 11) == 0)
		//if (wlan_connect(WLAN_SEC_UNSEC, "chicagoedt", 10, "chicagoedt", NULL, 0) == 0)	
		//Make sure we disconnect from any previous routers before we connect to a new one to prevent confusion on the device.
		wlan_disconnect();
//PORTD |= (1 << PD6);
		wlan_connect(WLAN_SEC_UNSEC, ROUTER_SSID, SSID_LENGTH, NULL, NULL, 0);
//PORTD &= ~(1 << PD7);	
		while(!DHCP_Complete)					{_delay_ms(1000);}//PORTB ^= (1 << PB4);
		
	        #ifdef WATCHDOG_ENABLED
			wdt_reset();
		#endif
/*
		//This is the socket address required to send and receive data.
		sockaddr_in Mission_Control_Address;
		//Clear it out to prevent any issues if I forget anything.
		memset((char *) &Mission_Control_Address, 0, sizeof(Mission_Control_Address));
		Mission_Control_Address.sin_family = AF_INET;
		Mission_Control_Address.sin_port   = htons(PORT);
		//This requires the use of arpa/inet.h library, so to minimize design we are not doing this.
		inet_aton(Source_IP, &Mission_Control_Address.sin_addr);
*/

		//Bind a socket to receive data:
//sockaddr Mission_Control_Address;
		memset((char *) &Mission_Control_Address, 0, sizeof(Mission_Control_Address));
		Mission_Control_Address.sa_family = AF_INET;
		
		//The Source Port:
		Mission_Control_Address.sa_data[0] = (char)HEX_PORT_1;		//(char)0x09;
		Mission_Control_Address.sa_data[1] = (char)HEX_PORT_2;		//(char)0x56;

		//Configure the socket to not time out to keep the connection active.
		//--------------------------------------------------------------------
   		unsigned long aucDHCP       = 14400;
        	unsigned long aucARP        = 3600;
        	unsigned long aucKeepalive  = 10;
        	unsigned long aucInactivity = 0;

		netapp_timeout_values(&aucDHCP, &aucARP, &aucKeepalive, &aucInactivity);
                //--------------------------------------------------------------------

		//Only necessary if CC3000 is set to a static IP.
		//netapp_dhcp(NULL, NULL, NULL, NULL);		//Sets the CC3000 to automatic DHCP

///////////////////

	//TODO:
	//Should check the CC3000's profiles. In the case that there are no profiles found, then 
	//inform the PC system, or use an LED.

		//Open a UDP socket that grabs datagram:
		Socket_Handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	
		switch(Socket_Handle)
		{
			case -1:		//Error
				//Flag somehow.
			break;

			default:		//Success
				//Set the socket configuration for blocking (since it is the only thing that is allowed).
				switch(bind(Socket_Handle, &Mission_Control_Address, sizeof(sockaddr)))
                		{
                        		case -1:
                                		//Flag as ERROR.
                        		break;

                        		default:
                                		//Flag as good.
                        		break;
                		}//End switch

			break;

		}//End switch
	
		//Clear out the buffer:
		//memset((char *) &Data_Buffer, 0, sizeof(Data_Buffer));

//SEND DATA (DEBUG TEST):

/*
		char MESSAGE [] = "HELLO";
		sockaddr Socket_Address;
		
		//Set the family to AF_INET:
		Socket_Address.sa_family = 2;
		
		//The destination port:
		Socket_Address.sa_data[0] = (char)0x09;
		Socket_Address.sa_data[1] = (char)0x56;
		
		//The destination IP:
		Socket_Address.sa_data[2] = (char)0xC0;
		Socket_Address.sa_data[3] = (char)0xA8;
		Socket_Address.sa_data[4] = (char)0x02;
		Socket_Address.sa_data[5] = (char)0x31;
		memset(&Socket_Address.sa_data[6], 0, 7);

		
		while(sendto(Socket_Handle, MESSAGE, 5, 0, &Socket_Address, sizeof(sockaddr)) != -1);
	
		PORTD |= (1 << PD7);

*/

	#endif  //CC3000_ENABLED


	#ifdef PWM_ENABLED

		//Put code to initialize the PWM on OC4A[PC7], OC3A[PC6], and OC1B[PB5].

	#endif //PWM_ENABLED

//NEED TO SETUP A QUICK REMOVAL FLAG FOR THIS CODE TO TEST THE CC3000.
//#ifdef MOTOR_CONTROL_FLAG
	//Set up our Motor Controller Selection lines and the output for the RS232 lines:
	//DDRD |= (1 << DDD3) | (1 << DDD4) | (1 << DDD5);
	DDRD |= (1 << DDD3) | (1 << DDD5);

	//Initialize the UART (RS-232 communications) for the motor controller interface:
	
	//Set the Baud rate to 115200 bits/s.  ((System Oscillator clock frequency / (2 * BAUD) ) - 1)
//NOTE: The value may not be correct, according to the data sheet (pg. 213).
	//With the value 16, the error is 2.1% (lower than 8, being -3.5%).

	UBRR1H = UBRRH_VALUE; /*Set baud rate*/
	UBRR1L = UBRRL_VALUE; /*Set baud rate*/

//Defined in util/setbaud.h:
	#if USE_2X
		UCSR1A |= (1 << U2X1);	//Double the baud rate for asynchronous communication.
	#else
		UCSR1A &= ~(1 << U2X1);
	#endif	//End USE_2X    

	//Set to no parity and in Asynchronous mode.
        //1 Stop bit.
        //1 Start bit.
        //Set to 8-bit data.
        UCSR1C |= (1 << UCSZ11) | (1 << UCSZ10); 

        //Enable the Rx and Tx lines.
        UCSR1B |= (1 << TXEN1);



#ifdef TWI_ENABLED
	//Set the SCL frequency to 200 KHz. From the equation: f(SCL) = F_CPU/(16 + (2*TWBR) * (4^TWPS))
	TWBR = 12;		
	DDRB |= (1 << DDB4);	//Setup PortB4 as the TWI error LED.
#endif	//End TWI_ENABLED

/*
//DEBUG:
char SEND = 0x31;
char REC = 0;
while(1)
{
*/
//while (!( UCSR1A & (1<<UDRE1))); 
//UDR1 = SEND;
//_delay_ms(500);
/*
//unsigned char SEND = '1';
//loop_until_bit_is_set(UCSR1A, RXC1); // Wait until data register empty.
//	unsigned char REC = UDR1; 
loop_until_bit_is_set(UCSR1A, UDRE1);
	UDR1 = 0x7d;
loop_until_bit_is_set(UCSR1A, UDRE1); // Wait until data register empty.
//loop_until_bit_is_set(UCSR1A, TXC1); // Wait until transmission ready.
        UDR1 = 0x7b;
loop_until_bit_is_set(UCSR1A, UDRE1); // Wait until data register empty.
//loop_until_bit_is_set(UCSR1A, TXC1); // Wait until transmission ready.
        UDR1 = 0x79;
loop_until_bit_is_set(UCSR1A, UDRE1); // Wait until data register empty.
//loop_until_bit_is_set(UCSR1A, TXC1); // Wait until transmission ready.
        UDR1 = 0xbd;
*/
/*
unsigned char SEND [] = "Hello";

USART_Transmit(SEND, sizeof(SEND));

//Serial has completed initialization.
PORTD |= (1 << PD7);
*/

	//Setup the Timer Compare Match to loop every 10ms:
	/*
	TCCR1B = (1 << WGM12) | (1 << CS12);	//Set to CTC mode and Prescale by 256.

	OCR1B = 625;		//Set the top value to compare against. Used the equation 1 sec = 16000000Hz.
	*/
	//PRESCALER:
		//TODO: If I feel that it should be lower.
//#endif
 	
///THIS SHOULD BE THE LAST LINE OF CODE IN THE INITIALIZATION.
/*
	#ifdef WATCHDOG_ENABLED      
          
	      //Enable the watchdog timer.
               //wdt_enable(WDTO_2S);    //Enable the watchdog timer with a 2 second Time Out.
		
		//cli();	//Temporarily disable global interrupts.
		
		Feed_Watchdog();//Reset the watchdog timer.

		//Enable the watchdog timer, set it to reset mode, and set the timer for eight seconds:
		WDTCSR = (1 << WDE) | (1 << WDP3) | (1 << WDP0);

	//	sei(); //Re-enable global interrupts.
		wdt_enable(WDTO_8S);
	#endif  //WATCHDOG_ENABLED
	*/

	//sei();	//Enable Global Interrupts.	


//QUICKLY REQUEST WHAT PATCH VERSION IS ON THE CC3000 AND FORWARD IT TO USART:
/*
unsigned char * OUT = NULL;
nvmem_read_sp_version(OUT);
USART_Transmit(OUT, sizeof(OUT));
*/

	_delay_ms(1000);	//Wait for one second for the RoboteQs to finish booting.
	//First thing, set the Sigmux to Safe Mode.
	Set_Mode(Safety_Mode);




	#ifdef ROUTER_WATCHDOG_ENABLED
		Count  = 0;					//Clear the Count variable out.
		TCNT1  = 0;					//Clear the TCNT register.
		TCCR1B = (1 << CS12) | (1 << CS10);		//Set the prescaler for 1024.
		TIMSK1 = (1 << OCIE1A);				//Enable output compare for 1A.
		OCR1A  = 39063;					//Set the system to interrupt every 5 seconds.
	
		//OCR1A = (Multiplier) * (F_CPU) / (Prescaler)		
		//39063 = (5) * (8000000) / (1024) 	

	#endif


}//End Initialization


//==========================================================================================================
//==========================================================================================================

/*Set_Mode*/
/**
  * @breif 
**/
uint8_t Set_Mode(uint8_t New_Mode)
{

	if (New_Mode == Current_Mode)	{return Current_Mode;}

	unsigned char Kill_Command[] = {'@', '0', '0', '!', 'E', 'X', '\r'};//"!EX\r";
	unsigned char Go_Command[] = {'@', '0', '0', '!', 'M', 'G', '\r'};//"!MG\r";

	switch (New_Mode)
	{
		case Safety_Mode:
			//Disable USB communications (i.e. disable autonomous):
			#ifdef USB_ENABLED
				Disable_USB_Controller();
			#endif
			//Send neutral signal to motor controllers (or if we could disable them...).
			switch(Current_Mode)
			{
				case Safety_Mode:
					return Safety_Mode;
				break;
				default:
					//unsigned char Kill_Command[] = "!EX\r";
			
					Select_Motor_Controller(1);
					USART_Transmit(Kill_Command, 7);
			
					Select_Motor_Controller(2);
                        		USART_Transmit(Kill_Command, 7);
					
					//Stop the Linear Actuator 01/11
                                        //PORTD &= ~(1 << PD7);
                                        //PORTD &= ~(1 << PD6);

					//Signal that the unit is in safety mode.
					PORTC &= ~(1 << PORTC7);
				break;
			}//End switch
		break;

		case Autonomous_Mode:            
//		case Upload_Mode:
			//Enable USB communications:
			#ifdef USB_ENABLED
				Enable_USB_Controller();
			#endif
			//Listen to all communications by the computer.
			//Do not forward any CC3000 motor controller commands.
		break;

		case RC_Mode:           
			
			//Disable USB communications:
			#ifdef USB_ENABLED
				Disable_USB_Controller();
			#endif
			//Forward all CC3000 motor controller commands.    

			switch(Current_Mode)
			{
				case RC_Mode:
					return RC_Mode;
				break;
				default:
					//unsigned char Go_Command[] = "!MG\r";
                        
                        		Select_Motor_Controller(1);
                        		USART_Transmit(Go_Command, 7);

                        		Select_Motor_Controller(2);
                        		USART_Transmit(Go_Command, 7);				
					
					PORTC |= (1 << PORTC7);
				break;
			}//End switch
		break;

		default:
			//Invalid input: 
			return -1;
		break;
	}//End switch
	
	Current_Mode = New_Mode;
	
	return New_Mode;	
}//End Switch_Mode
//==========================================================================================================
//==========================================================================================================

#ifdef USB_ENABLED
	inline void Enable_USB_Controller()
	{
		USBCON = (1 << USBE);		//Enable the USB Controller.
		UDCON &= ~(1 << LSM);		//Make sure the USB is in full speed mode.
		UDCON &= ~(1 << DETACH);	//Re-attach all devices.
	}//End Enable_USB_Controller

	inline void Disable_USB_Controller()
	{
		UDCON |= (1 << DETACH);		//Detach all devices on the USB line.
	}//End Diable_USB_Controller
#endif

//==========================================================================================================
//==========================================================================================================

#ifdef CC3000_ENABLED

char* Send_Driver_Patch(unsigned long *usLength)
{
	*usLength = 0;
	return NULL;
}//End Send_Driver_Patch

//--------------------------------------------------------

char* Send_Boot_Loader_Patch(unsigned long *usLength)
{
	*usLength = 0;
	return NULL;
}//End Send_Boot_Loader_Patch

//--------------------------------------------------------


char* Send_WLFW_Patch(unsigned long *usLength)
{
	*usLength = 0;
	return NULL;
}//End Send_WLFW_Patch

//--------------------------------------------------------


void CC3000_Unsynch_Call_Back(long Event_Type, char * Data, unsigned char Length)
{
	switch (Event_Type)
	{
		case HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE:	//First-time configuration process is complete.
			break;
		case HCI_EVNT_WLAN_KEEPALIVE:			//Periodic keep-alive event.
			break;
		case HCI_EVNT_WLAN_UNSOL_CONNECT:		//WLAN-connected event.
			break;
		case HCI_EVNT_WLAN_UNSOL_DISCONNECT:		//CC3000 disconnected from AP.
			break;
////COMPILER CANNOT FIND THESE.
		case HCI_EVNT_WLAN_UNSOL_DHCP:			//DHCP state change.
				if ( * (Data + NETAPP_IPCONFIG_MAC_OFFSET) == 0)
				{
					//DHCP_Complete = 1;
					PORTC |= (1 << PC6);
				}//End if
				else
				{
					//DHCP_Complete = 0;
					PORTC &= ~(1 << PC6);
				}//End else			

			break;
//case HCI_EVENT_CC300_CAN_SHUT_DOWN:
//break;	
		case HCI_EVNT_WLAN_ASYNC_PING_REPORT:		//Notification of ping results.
			break;
//case HCI_EVNT_BSD_TCP_CLOSE_WAIT:	
//break;	
		case HCI_EVNT_WLAN_UNSOL_INIT:			//CC3000 finished the initialization process.

			break;	
		default:
			break;
	}//End switch
}//End CC3000_Unsynch_Call_Back

//--------------------------------------------------------

/*Read_Interrupt_Pin*/
/**
  *	@breif This function listens to the interrupt pin, and if it is high, return 1, or low return 0.
**/
long Read_WLAN_Interrupt_Pin()
{
	return bit_is_set(PIND, PIND2);
}//End Read_Interrupt_Pin 

//--------------------------------------------------------

void Write_WLAN_Pin(unsigned char val)
{
	switch (val)
	{
		case 0:
			PORTB &= ~(1 << PORTB5);
			break;
		case 1:
			PORTB |= (1 << PORTB5);
			break;
	}//End switch
}//End Write_WLAN_Pin

//--------------------------------------------------------

void WLAN_Interrupt_Enable()
{
	//Set the interrupt to occur when the pinout is falling:
	EICRA = (1 << ISC21);
	EIMSK |= (1 << INT2);

}//End WLAN_Interrupt_Enable

//--------------------------------------------------------

void WLAN_Interrupt_Disable()
{
	EIMSK &= ~(1 << INT2);
}//End WLAN_Interrupt_Disable


//--------------------------------------------------------

uint8_t Recieve_WiFi_Data()
{
	unsigned char Temp_Buffer[2];
        unsigned long Rx_Packet_Length = 0;
	
	switch(Socket_Handle)
	{
		case -1:
			return -1;
		break;

		case 0:
		break;
	}//End switch

	Temp_Buffer[0] = 0;
	Temp_Buffer[1] = 0;

        //Begin receiving data:
        recvfrom(Socket_Handle, Temp_Buffer, CC3000_APP_BUFFER_SIZE, 0, &Mission_Control_Address, &Rx_Packet_Length);	
	
	//Decipher the data transmitted to see if it matches the protocol (if it doesn't ERROR):
	if(Temp_Buffer[0]) {Decrypt_Data(Temp_Buffer);}
	return 0;
}//End Recieve_WiFi_Data
#endif

//////////////////////////////////////////////////
///REMOVE?
//////////////////////////////////////////////////
void Select_Motor_Controller(uint8_t Motor)
{
	switch(Motor)
	{
		case 0:		//Wheel Controller
		case 1:		
			PORTD &= ~(1 << PORTD5);
		break;

		case 2:		//Mechanism & Linear Actuator Controller
			PORTD |= (1 << PORTD5);
		break;

		default:	//Invalid intput
			return;
		break;

	}//End switch


	_delay_ms(10);
	//asm("nop");
	//asm("nop");

	//Current_Motor = Motor;

}//End Select_Motor_Controller


/*
uint8_t Decrypt_Data(unsigned char Data)
{
	if (Data == 0)	{return;}
	//|E|O|M|M|D|P|P|P|
	unsigned char Parsed_Data = 0;
	uint8_t index = 0;

	Parsed_Data = Data >> 7;

	if (Parsed_Data)
	{
		//Expansion bit is true.
		return 0;	
	}//End if

	Parsed_Data = Data >> 6;
	
	if (!Parsed_Data)
	{
		//Operation bit is false, so set to eStop. 
		Set_Mode(Safety_Mode);	
		return 1;
	}//End if
	
	Set_Mode(RC_Mode);

	unsigned char Output_Command[20];
        memset((unsigned char *) &Output_Command, 0, sizeof(Output_Command));

	Parsed_Data = Data << 2;
	Parsed_Data >>= 6;

	Select_Motor_Controller(Parsed_Data);

	switch(Parsed_Data)
	{
		case 0:		//00		Left Wheel
		case 2:		//10		Mechanism
			strcpy(Output_Command, "!M ");
			index = 3;
		break;

		case 1:		//01		Right Wheel
                        strcpy(Output_Command, " ");
			index = 1;
		break;

                case 3:		//11		Linear Actuator
			//Continue code to set output pins to RoboteQ to control the Linear Actuator.
			switch((Data & 0x0c) >> 2)
			{
				case 0:		//Positive direction of motion (up). 00
					PORTD &= ~(1 << PD6);
					PORTD |= (1 << PD7);
				break;
				
				case 2:		//Negative direction of motion (down). 10
					PORTD |= (1 << PD6);
					PORTD |= (1 << PD7);
				break;
	
				default:	//Stop the Linear Actuator 01/11
					PORTD &= ~(1 << PD6);
					PORTD &= ~(1 << PD7);	
				break;
			}//End switch	

			return 0;
		break;

		default:
			//Something has gone terribly wrong!!!
			return -1;
		break;

	}//End switch

	Parsed_Data = Data << 4;
	Parsed_Data >>= 4;	

	switch (Parsed_Data)
	{	
                case 0:			//0000	0		0		0%	
                case 8:                 //1000  8               -8              0%		
			strcpy(&Output_Command[index],"0\r");
		break;
		
		case 1:			//0001  1		1		14.3%
			strcpy(&Output_Command[index],"143\r");
		break;
                
		case 2:			//0010  2		2		28.6%
                        strcpy(&Output_Command[index],"286\r");
		break;
                
		case 3:			//0011	3		3		42.9%
                        strcpy(&Output_Command[index],"429\r");                
		break;			
                
		case 4:			//0100	4		4		57.2%		
                        strcpy(&Output_Command[index],"572\r");                
		break;
                
		case 5:			//0101	5		5		71.5%
                        strcpy(&Output_Command[index],"715\r");                	
		break;
                
		case 6:			//0110	6		6		85.8%
                        strcpy(&Output_Command[index],"858\r");                
		break;
                
		case 7:			//0111	7		7		100%
                        strcpy(&Output_Command[index],"1000\r");                
		break;
              
		case 9:			//1001	9		-7		-14.3%
                        strcpy(&Output_Command[index],"-143\r");                
		break;
                
		case 10:		//1010	10		-6		-28.6%
                        strcpy(&Output_Command[index],"-286\r");                
		break;
                
		case 11:		//1011	11		-5		-42.9%
                        strcpy(&Output_Command[index],"-429\r");     
	        break;
                
		case 12:		//1100	12		-4		-57.2%
                        strcpy(&Output_Command[index],"-572\r");                
		break;
                
		case 13:		//1101	13		-3		-71.5%
                        strcpy(&Output_Command[index],"-715\r");                
		break;
                
		case 14:		//1110	14		-2		-85.8%
                        strcpy(&Output_Command[index],"-858\r");                
		break;
                
		case 15:		//1111	15		-1		-100%
                        strcpy(&Output_Command[index],"-1000\r");                
		break;
		
		default:
			//Something has gone terribly wrong!!!
			return -1;
		break;
	}//End switch	

	Parsed_Data = Data << 2;
        Parsed_Data >>= 6;

        switch(Parsed_Data)
        {
		case 1:		//01		Right Wheel
                case 2:         //10            Mechanism
			USART_Transmit(Output_Command, (((unsigned char *)strchr(Output_Command, '\r')) - Output_Command + 1));
		break;

		default:
			 USART_Transmit(Output_Command, (((unsigned char *)strchr(Output_Command, '\r')) - Output_Command));
		break;
	}//End switch statement


	//USART_Transmit(Output_Command, (((unsigned char *)strchr(Output_Command, '\r')) - Output_Command + 1)); 

	return 0;
}//End Decrypt_Data
*/

/*
uint8_t Decrypt_Data(unsigned char Data)
{
	int Wheel_Speed = 0;
	unsigned char Output_Command[10];

	switch(Data)
	{

		//Left Wheel Forward:
		case 71:
			Wheel_Speed = 142;
		case 70:
			Wheel_Speed += 143;
		case 69:
			Wheel_Speed += 143;
		case 68:
			Wheel_Speed += 143;
		case 67:
			Wheel_Speed += 143;
		case 66:
			Wheel_Speed += 143;
		case 65:
			Wheel_Speed += 143;
		case 64:
			Set_Mode(RC_Mode);
			Select_Motor_Controller(1);		
			sprintf(Output_Command, "!M %d\r", Wheel_Speed);
			USART_Transmit(Output_Command, (((unsigned char *)strchr(Output_Command, '\r')) - Output_Command));		
		break;

		//Left Wheel Reverse:
		case 79:
			Wheel_Speed = -142;
		case 78:
			Wheel_Speed -= 143;
		case 77:
			Wheel_Speed -= 143;
		case 76:
			Wheel_Speed -= 143;
		case 75:
			Wheel_Speed -= 143;
		case 74:
			Wheel_Speed -= 143;
		case 73:
			Wheel_Speed -= 143;
		case 72:
			Set_Mode(RC_Mode);
                        Select_Motor_Controller(1);
			sprintf(Output_Command, "!M %d\r", Wheel_Speed);
			USART_Transmit(Output_Command, (((unsigned char *)strchr(Output_Command, '\r')) - Output_Command));			
		break;	
	
		//Right Wheel Forward:
		case 87:
			Wheel_Speed = 142;
		case 86:
			Wheel_Speed += 143;
		case 85:
			Wheel_Speed += 143;
		case 84:
			Wheel_Speed += 143;
		case 83:
			Wheel_Speed += 143;
		case 82:
			Wheel_Speed += 143;
		case 81:
			Wheel_Speed += 143;
		case 80:
			Set_Mode(RC_Mode);
			Select_Motor_Controller(1);
			sprintf(Output_Command, " %d\r", Wheel_Speed);
			USART_Transmit(Output_Command, (((unsigned char *)strchr(Output_Command, '\r')) - Output_Command + 1));		
		break;
	
		//Right Wheel Reverse:
		case 95:
			Wheel_Speed = -142;
		case 94:
			Wheel_Speed -= 143;
		case 93:
			Wheel_Speed -= 143;
		case 92:
			Wheel_Speed -= 143;
		case 91:
			Wheel_Speed -= 143;
		case 90:
			Wheel_Speed -= 143;
		case 89:
			Wheel_Speed -= 143;
		case 88:
			Set_Mode(RC_Mode);
			Select_Motor_Controller(1);
			sprintf(Output_Command, " %d\r", Wheel_Speed);
			USART_Transmit(Output_Command, (((unsigned char *)strchr(Output_Command, '\r')) - Output_Command + 1));	
		break;
	
		//Digging Mechanism Forward:
		case 111:
			Wheel_Speed = 142;
		case 110:
			Wheel_Speed += 143;
		case 109:
			Wheel_Speed += 143;
		case 108:
			Wheel_Speed += 143;
		case 107:
			Wheel_Speed += 143;
		case 106:
			Wheel_Speed += 143;
		case 105:
			Wheel_Speed += 143;
		case 104:
			Set_Mode(RC_Mode);
			Select_Motor_Controller(2);
			sprintf(Output_Command, "!M %d\r", Wheel_Speed);
			USART_Transmit(Output_Command, (((unsigned char *)strchr(Output_Command, '\r')) - Output_Command));	
		break;

		//Digging Mechanism Reverse:
		case 103:
			Wheel_Speed = -142;
		case 102:
			Wheel_Speed -= 143;
		case 101:
			Wheel_Speed -= 143;
		case 100:
			Wheel_Speed -= 143;
		case 99:
			Wheel_Speed -= 143;
		case 98:
			Wheel_Speed -= 143;
		case 97:
			Wheel_Speed -= 143;
		case 96:
			Set_Mode(RC_Mode);
			Select_Motor_Controller(2);
			sprintf(Output_Command, "!M %d\r", Wheel_Speed);
			USART_Transmit(Output_Command, (((unsigned char *)strchr(Output_Command, '\r')) - Output_Command));
		break;

		//Linear Actuator Up:
		case 112:
		case 113:
		case 114:
		case 115:
			Set_Mode(RC_Mode);
			Select_Motor_Controller(2);
			strcpy(Output_Command, " 1000\r");
			USART_Transmit(Output_Command, (((unsigned char *)strchr(Output_Command, '\r')) - Output_Command + 1));		
		break;

		//Linear Actuator Down:	
		case 120:
		case 121:
		case 122:
		case 123:
			Set_Mode(RC_Mode);
			Select_Motor_Controller(2);
			strcpy(Output_Command," -1000\r");
			USART_Transmit(Output_Command, (((unsigned char *)strchr(Output_Command, '\r')) - Output_Command + 1));		
		break;

		//Linear Actuator OFF:
		case 116:
		case 117:
		case 118:
		case 119:
		case 124:
		case 125:
		case 126:
		case 127:
			Set_Mode(RC_Mode);
			Select_Motor_Controller(2);
			strcpy(Output_Command, " 0\r");
			USART_Transmit(Output_Command, (((unsigned char *)strchr(Output_Command, '\r')) - Output_Command + 1));		
		break;

		//All other commands:
		default:
			if (Data & 128)	{return 0;}
			Set_Mode(Safety_Mode);
			return 1;
		break;
	}//End switch

	return 0;
}//End Decrypt_Data
*/

uint8_t Decrypt_Data(unsigned char * Data)
{
	//EECMDPPPPPDPPPPP
	//EECMDPPP
	//XX1XXXMM
	
	unsigned char Size = 0;
	unsigned char Output_Command[14] = {'@', '0', ' ', '!', 'G', ' ', '1', ' '};

	#ifdef WATCHDOG_ENABLED
		//Feed_Watchdog();
		wdt_reset();
	#endif 	//End WATCHDOG_ENABLED

	//Is this a mode change command?
	if (*Data & 0x20)
	{
		Set_Mode(*Data & 0x03);
		return 0;
	}//End if

	//If we are not in RC Mode then don't process the rest of the command.
	if (Current_Mode != RC_MODE)
	{
		return 2;	
	}//End if


	switch(*Data)
	{
	//NEW
		case 0x01:

		//Digging Mechanism Forward:
		//	&		
		//Left Wheel Forward:
		case 111:		
		case 71:
			Output_Command[2] 	= '1';
			Output_Command[8] 	= '1';
			Output_Command[9] 	= '0';
			Output_Command[10] 	= '0';
			Output_Command[11] 	= '0';
			Output_Command[12]	= '\r';
			Size 			= 13;
		break;

		case 110:
		case 70:
			Output_Command[2] 	= '1';
			Output_Command[8] 	= '8';
			Output_Command[9] 	= '5';
			Output_Command[10] 	= '8';
			Output_Command[11]	= '\r';
			Size			= 12;
		break;

		case 109:
		case 69:
			Output_Command[2] 	= '1';
			Output_Command[8] 	= '7';
			Output_Command[9] 	= '1';
			Output_Command[10] 	= '5';
			Output_Command[11]	= '\r';
			Size			= 12;
		break;	

		case 108:
		case 68:
			Output_Command[2] 	= '1';
			Output_Command[8] 	= '5';
			Output_Command[9] 	= '7';
			Output_Command[10] 	= '2';
			Output_Command[11]	= '\r';
			Size			= 12;
		break;

		case 107:
		case 67:
			Output_Command[2] 	= '1';
			Output_Command[8] 	= '4';
			Output_Command[9] 	= '2';
			Output_Command[10] 	= '9';
			Output_Command[11]	= '\r';
			Size			= 12;
		break;

		case 106:
		case 66:
			Output_Command[2] 	= '1';
			Output_Command[8] 	= '2';
			Output_Command[9] 	= '8';
			Output_Command[10] 	= '6';
			Output_Command[11]	= '\r';
			Size			= 12;
		break;

		case 105:
		case 65:
			Output_Command[2] 	= '1';
			Output_Command[8] 	= '1';
			Output_Command[9] 	= '4';
			Output_Command[10] 	= '3';
			Output_Command[11]	= '\r';
			Size			= 12;
		break;

	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	
		//Digging Mechanism OFF:
		case 104:			
		case 96:

		//Left Wheel Stop:
		case 72:
		case 64:
			Output_Command[2] 	= '1';
			Output_Command[8] 	= '0';
			Output_Command[9]	= '\r';
			Size			= 10;
		break;
		
		//Digging Mechanism Reverse:
		//	&		
		//Left Wheel Reverse:
		case 103:
		case 79:
			Output_Command[2] 	= '1';
			Output_Command[8] 	= '-';
			Output_Command[9] 	= '1';
			Output_Command[10] 	= '0';
			Output_Command[11]	= '0';
			Output_Command[12]	= '0';
			Output_Command[13]	= '\r';
			Size			= 14;
		break;

		case 102:
		case 78:	
			Output_Command[2] 	= '1';
			Output_Command[8] 	= '-';
			Output_Command[9] 	= '8';
			Output_Command[10] 	= '5';
			Output_Command[11]	= '8';
			Output_Command[12]	= '\r';
			Size			= 13;
		break;
	
		case 101:
		case 77:
			Output_Command[2] 	= '1';
			Output_Command[8] 	= '-';
			Output_Command[9] 	= '7';
			Output_Command[10] 	= '1';
			Output_Command[11]	= '5';
			Output_Command[12]	= '\r';
			Size			= 13;
		break;

		case 100:
		case 76:
			Output_Command[2] 	= '1';
			Output_Command[8] 	= '-';
			Output_Command[9] 	= '5';
			Output_Command[10] 	= '7';
			Output_Command[11]	= '2';
			Output_Command[12]	= '\r';
			Size			= 13;
		break;

		case 99:
		case 75:
			Output_Command[2] 	= '1';
			Output_Command[8] 	= '-';
			Output_Command[9] 	= '4';
			Output_Command[10] 	= '2';
			Output_Command[11]	= '9';
			Output_Command[12]	= '\r';
			Size			= 13;
		break;

		case 98:
		case 74:
			Output_Command[2] 	= '1';
			Output_Command[8] 	= '-';
			Output_Command[9] 	= '2';
			Output_Command[10] 	= '8';
			Output_Command[11]	= '6';
			Output_Command[12]	= '\r';
			Size			= 13;
		break;
		
		case 97:
		case 73:
			Output_Command[2] 	= '1';
			Output_Command[8] 	= '-';
			Output_Command[9] 	= '1';
			Output_Command[10] 	= '4';
			Output_Command[11]	= '3';
			Output_Command[12]	= '\r';
			Size			= 13;	
		break;

	
		//Linear Actuator Up:
		case 112:
		case 113:
		case 114:
		case 115:
	
		//Right Wheel Forward:
		case 87:
			Output_Command[2] 	= '2';
			Output_Command[8] 	= '1';
			Output_Command[9] 	= '0';
			Output_Command[10] 	= '0';
			Output_Command[11] 	= '0';
			Output_Command[12]	= '\r';
			Size 			= 13;
		break;

		case 86:
			Output_Command[2]	= '2';
			Output_Command[8] 	= '8';
			Output_Command[9] 	= '5';
			Output_Command[10] 	= '8';
			Output_Command[11] 	= '\r';
			Size 			= 12;
		break;

		case 85:
			Output_Command[2]	= '2';
			Output_Command[8] 	= '7';
			Output_Command[9] 	= '1';
			Output_Command[10] 	= '5';
			Output_Command[11] 	= '\r';
			Size 			= 12;
		break;

		case 84:
			Output_Command[2]	= '2';
			Output_Command[8] 	= '5';
			Output_Command[9] 	= '7';
			Output_Command[10] 	= '2';
			Output_Command[11] 	= '\r';
			Size 			= 12;
		break;

		case 83:
			Output_Command[2]	= '2';
			Output_Command[8] 	= '4';
			Output_Command[9] 	= '2';
			Output_Command[10] 	= '9';
			Output_Command[11] 	= '\r';
			Size 			= 12;
		break;

		case 82:
			Output_Command[2]	= '2';
			Output_Command[8] 	= '2';
			Output_Command[9] 	= '8';
			Output_Command[10] 	= '6';
			Output_Command[11] 	= '\r';
			Size 			= 12;
		break;

		case 81:
			Output_Command[2]	= '2';
			Output_Command[8] 	= '1';
			Output_Command[9] 	= '4';
			Output_Command[10] 	= '3';
			Output_Command[11] 	= '\r';
			Size 			= 12;
		break;
		
		//Linear Actuator OFF:
		case 116:
		case 117:
		case 118:
		case 119:
		case 124:
		case 125:
		case 126:
		case 127:

		//Right Wheel Stop:
		case 80:
		case 88:
			Output_Command[2] 	= '2';
			Output_Command[8]	= '0';
			Output_Command[9]	= '\r';
			Size 			= 10;	
		break;
	

		//Linear Actuator Down:	
		case 120:
		case 121:
		case 122:
		case 123:		
		
		//Right Wheel Reverse:
		case 95:
			Output_Command[3]	= '2';	
			Output_Command[5] 	= '-';
			Output_Command[6] 	= '1';
			Output_Command[7] 	= '0';
			Output_Command[8] 	= '0';
			Output_Command[9]	= '0';
			Output_Command[10]	= '\r';
			Size 			= 11;
		break;

		case 94:
			Output_Command[3]	= '2';	
			Output_Command[5] 	= '-';
			Output_Command[6] 	= '8';
			Output_Command[7] 	= '5';
			Output_Command[8] 	= '8';
			Output_Command[9]	= '\r';
			Size 			= 10;
		break;	
	
		case 93:
			Output_Command[3]	= '2';	
			Output_Command[5] 	= '-';
			Output_Command[6] 	= '7';
			Output_Command[7] 	= '1';
			Output_Command[8] 	= '5';
			Output_Command[9]	= '\r';
			Size 			= 10;
		break;

		case 92:
			Output_Command[3]	= '2';	
			Output_Command[5] 	= '-';
			Output_Command[6] 	= '5';
			Output_Command[7] 	= '7';
			Output_Command[8] 	= '2';
			Output_Command[9]	= '\r';
			Size 			= 10;
		break;
		
		case 91:
			Output_Command[3]	= '2';	
			Output_Command[5] 	= '-';
			Output_Command[6] 	= '4';
			Output_Command[7] 	= '2';
			Output_Command[8] 	= '9';
			Output_Command[9]	= '\r';
			Size 			= 10;
		break;
		
		case 90:
			Output_Command[3]	= '2';	
			Output_Command[5] 	= '-';
			Output_Command[6] 	= '2';
			Output_Command[7] 	= '8';
			Output_Command[8] 	= '6';
			Output_Command[9]	= '\r';
			Size 			= 10;
		break;

		case 89:
			Output_Command[3]	= '2';	
			Output_Command[5] 	= '-';
			Output_Command[6] 	= '1';
			Output_Command[7] 	= '4';
			Output_Command[8] 	= '3';
			Output_Command[9]	= '\r';
			Size 			= 10;
		break;			

		//Manual E-GO:
		case 192:
                       	Select_Motor_Controller(1);
                        USART_Transmit("!MG\r", 4);
                        asm("nop");
                        asm("nop");
                        Select_Motor_Controller(2);
                        USART_Transmit("!MG\r", 4);
                        asm("nop");
                        asm("nop");
   			return 0;
		break;

		//Autonomous Mode:
		case 255:
			Set_Mode(Autonomous_Mode);
			return 0;
		break;

		//All other commands:
		default:
			if (*Data & 0x80 || *Data & 0x40) {return 0;}
			Set_Mode(Safety_Mode);
			return 1;
		break;
	}//End switch

	Set_Mode(RC_Mode);

	if (Data < 96)	{Select_Motor_Controller(1);}
	else		{Select_Motor_Controller(2);}

	USART_Transmit(Output_Command, Size);

	return 0;
}//End Decrypt_Data

#ifdef TWI_ENABLED
/*Transmit_Energy_Data*/
inline void Transmit_Energy_Data()
{
	unsigned char Energy_Data [4];
	
	//Use TWI to recieve data from energy monitor:

	TWI_SEND_START();
	TWI_WAIT_FOR_START();
	TWI_CHECK_START();
	
	TWI_SEND_SLA_R();
	TWI_TRANSMIT();
	TWI_WAIT_FOR_START();
	TWI_CHECK_RECIEVE();

//FOR THIS CASE LET US ASSUME THAT THERE WILL BE NO ERRORS DURRING COMMUNICATION.
//--------------------------------------------------------------------------------
	TWI_RECIEVE();
	TWI_WAIT_FOR_START();
	Energy_Data[0] = TWDR;

	TWI_CHECK_RECIEVE();
	TWI_WAIT_FOR_START();
	Energy_Data[1] = TWDR;

	TWI_RECIEVE();
        TWI_WAIT_FOR_START();
	Energy_Data[2] = TWDR;
	
	TWI_RECIEVE();
        TWI_WAIT_FOR_START();
	Energy_Data[3] = TWDR;
//--------------------------------------------------------------------------------

	TWI_SEND_STOP();

	//Transmit Data from Energy Monitor:
	sendto(Socket_Handle, Energy_Data, 4, 0, &Mission_Control_Address, (socklen_t)sizeof(Mission_Control_Address));
	
			
}//End Transmit_Energy_Data
#endif //End TWI_ENABLED


//==========================================================================================================
//==========================================================================================================


//***********************************************************************************************************
//Interrupts:
//***********************************************************************************************************

///NEEDS TO BE REVISED:
//Energy Analysis:
#ifdef ENERGY_ANALYSIS_ENABLED
	ISR (TIMER0_COMPA_vect)
	{
	/*
		uint16_t Temp_Power = 0;
		//Set the ADMUX to collect data from ADC0 and compare it against AREF to find the battery voltage: 
		ADMUX = 0x00;

		ADCSRA |= (1 << ADSC); //Trigger ADC conversion.

		//Wait until the conversion is complete (ADSC = 0) then complete:
		while (ADCSRA & (1 << ADSC));

		//Collect the data:
		Temp_Power = 0;
	*/
	}//End Timer/Counter 0 Compare Match
#endif //End ENERGY_ANALYSIS_ENABLED
//------------------------------------------------------------------------------------------------------

//CC3000 Data Output Request:
ISR (INT2_vect)
{
	SPI_IRQ();
	return;
}//End External Interrupt (INT0) [CC3000 IRQ]

//------------------------------------------------------------------------------------------------------

//Pseudo_Watchdog_System:
#ifdef ROUTER_WATCHDOG_ENABLED
	ISR (TIMER1_COMPA_vect)
	{	
///DEBUG:
PORTC ^= (1 << PORTC7);	
///
		++Count;		//Multiply count by five to get the number of seconds that have passed.
	}//End Timer1_Compare_A Match 
#endif //End ROUTER_WATCHDOG_ENABLED
