/**
 *	Sonar Code
 *  author: Mayur Patel
 */

/*Include definition files*/
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <avr/pgmspace.h> 

/**
 * @brief Operating Frequency of uC in Hertz.
 */
#define F_CPU 16000000UL

/**
 * @brief Baud rate of RS-232 communication in bps.
 */
#define BAUD 38400
#include <util/setbaud.h>

/**
 * @brief Boolean true.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define TRUE 1

/**
 * @brief Boolean false
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define FALSE 0

/**
 * @brief Number of Timer2 ticks to wait before starting the next program loop
 * iteration.
 *
 * This constant is used to determine the delay between consecutive program
 * loop iterations. This constant is equal to (operating frequency) / (main
 * program loop frequency * 8 bit timer prescaler * NUM_8BIT_CYCLES).
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define CLK_TCKS_2 125

/**
 * @brief Number of program loop iterations before sending a tick count.
 *
 * This constant is used to determine how often a tick count is transmitted
 * to the main computers. This method is needed because the tick count
 * transmission is performed at a lower frequency than the PWM and
 * Timer2 does not have the range for such frequencies. The 16 bit timer
 * is being used for PWM generation and thus is not available for this purpose.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define NUM_8BIT_CYCLES 25

/**
 * @brief Constant to set 8 bit Timer0 prescaler.
 *
 * See page 72 of the ATmega8 data sheet for valid values.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define PRESCALER_0 4

/**
 * @brief Constant to set 16 bit Timer1 prescaler.
 *
 * See page 100 of the ATmega8 data sheet for valid values.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define PRESCALER_1 3




/**
 * @brief Value to set PWM frequency
 *
 * This value determines the TOP value used for 16 bit Phase and Frequency
 * Correct PWM based on clock frequency. See page 94 of the ATmega8 data sheet
 * for valid values.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define PWM_TOP 20000





/**
 * @brief I2C address of this module if it is attached to the left wheel.
 *
 */
#define LEFT_I2C_ADDR 0x20

/**
 * @brief I2C address of this module if it is attached to the right wheel.
 *
 */
#define RIGHT_I2C_ADDR 0x21

/**
 * @brief Response to a request to test the RS-232 connection.
 *
 */
#define TEST_RESPONSE 0x55

/**
 * @brief Command to test the RS-232 connection.
 *
 * When this command is received through the USART interface, #TEST_RESPONSE
 * will be returned. This is used to make sure the serial connection is working
 * without modifying any system.
 */
#define CMD_TEST 0xFF

/**
 * @brief Command to obtain this module's identification.
 *
 * When this command is received through the USART interface, this module's
 * I2C adress will be returned. This is used to allow the computer software to
 * determine which module is connected to which serial port.
 */
#define CMD_ID 0xC9


#define CMD_QUERY_SONAR_ALL 0xCA
#define CMD_QUERY_SONAR_1 0xC1
#define CMD_QUERY_SONAR_2 0xC2
#define CMD_QUERY_SONAR_3 0xC3
#define CMD_QUERY_SONAR_4 0xC4


/**
 * @brief Boolean logic data type.
 *
 * This type should only take on one of two values: TRUE or FALSE.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
typedef unsigned char BOOL_T;

/**
 * @brief List of states that the system can be in.
 *
 * Each of these states refers to a type of communication with the computer
 * that needs to be processed.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
typedef enum
{
	/** The system is not currently communicating with the computer.*/
	COMM_MODE_IDLE,
	/** The system is responding to the test command from the computer.*/
	COMM_MODE_TEST,
	/** The system is reporting its ID to the computer.*/
	COMM_MODE_ID,
	/** The system has received an invalid command, data or has timed out.*/
	COMM_MODE_ERROR,
	COMM_MODE_QUERY_SONAR_ALL,
	COMM_MODE_QUERY_SONAR_1,
	COMM_MODE_QUERY_SONAR_2,
	COMM_MODE_QUERY_SONAR_3,
	COMM_MODE_QUERY_SONAR_4,
} COMM_MODE;

typedef enum
{
	STATE_QRY_SONAR_ALL_1,
	STATE_QRY_SONAR_ALL_2,
	STATE_QRY_SONAR_ALL_3,
	STATE_QRY_SONAR_ALL_4,
	STATE_QRY_SONAR_ALL_5,
	STATE_QRY_SONAR_ALL_6,
	STATE_QRY_SONAR_ALL_7,
	STATE_QRY_SONAR_ALL_8,
} STATE_QRY_SONAR_ALL;

typedef enum
{
	STATE_QRY_SONAR_1_1,
	STATE_QRY_SONAR_1_2,
} STATE_QRY_SONAR_1;

typedef enum
{
	STATE_QRY_SONAR_2_1,
	STATE_QRY_SONAR_2_2,
} STATE_QRY_SONAR_2;

typedef enum
{
	STATE_QRY_SONAR_3_1,
	STATE_QRY_SONAR_3_2,
} STATE_QRY_SONAR_3;

typedef enum
{
	STATE_QRY_SONAR_4_1,
	STATE_QRY_SONAR_4_2,
} STATE_QRY_SONAR_4;


/**
 * @brief List of sonar states picking from Sonar 1 to Sonar 4.
 *
 * Each of these states refers a specific sonar module
 *
 * @author Mayur Patel mayur1130@gmail.com
 */
typedef enum
{
	SONAR_1,
	SONAR_2,
	SONAR_3,
	SONAR_4
} SONAR;



/**
 * @brief This macro sets bits in a variable or register.
 *
 * The bits masked will be changed to logic 1. The other bits will remain
 * unchanged.
 *
 * @warning No type checking or parameter validation is performed.
 *
 * @param mask The bits mask that specifies which bits should be set. The bits
 * that are set in the mask will set.
 * @param reg The register or variable that is to have its bits set.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define BSET(mask, reg) (reg |= (mask))

/**
 * @brief This macro clears bits in a variable or register.
 *
 * The bits masked will be changed to logic 0. The other bits will remain
 * unchanged.
 *
 * @warning No type checking or parameter validation is performed.
 *
 * @param mask The bits mask that specifies which bits should be cleared. The 
 * bits that are set in the mask will cleared.
 * @param reg The register or variable that is to have its bits cleared.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define BCLR(mask, reg) (reg &= ~(mask))


/**
 * @brief This variable maintains the current internal communication state.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile COMM_MODE commMode;



static volatile STATE_QRY_SONAR_ALL stateQrySonarAll;
static volatile STATE_QRY_SONAR_1 stateQrySonar1;
static volatile STATE_QRY_SONAR_2 stateQrySonar2;
static volatile STATE_QRY_SONAR_3 stateQrySonar3;
static volatile STATE_QRY_SONAR_4 stateQrySonar4;


/**
 * @brief This variable increases the maximum period of Timer2.
 *
 * Whenever Timer2 compare occurs, this variable gets incremented or reset to
 * zero so that longer periods of time can be controlled by Timer 2.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile unsigned char counter8bit;


//STUFF ADDED IN BY MAYUR
//
//
//
//
//
static volatile int16_t startTime;
static volatile int16_t endTime;
static volatile SONAR currentSonar;
static volatile SONAR sonar;
static volatile int16_t sonar_1_time;
static volatile int16_t sonar_2_time;
static volatile int16_t sonar_3_time;
static volatile int16_t sonar_4_time;

static volatile int16_t sonar_1_RS232;
static volatile int16_t sonar_2_RS232;
static volatile int16_t sonar_3_RS232;
static volatile int16_t sonar_4_RS232;



/**
 * @brief This variable contains the system's I2C address.
 *
 * This value is dependent on which side of the robot this system is
 * responsible for.
 */
static unsigned char address;


/**
 * @brief This function initializes the microcontroller.
 *
 * The 16 bit timer, the 8 bit timer, the RS232 interface, the outputs pins,
 * interrupts, and the watchdog timer of the microcontroller are initialized.
 * This should be the first function called in the program.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void init(void);


/**
 * @brief Determines if the byte is a data byte.
 *
 * A byte is a data byte if the most significant bit is 0.
 *
 * @param b The byte to check.
 *
 * @return TRUE if the byte is a data byte or FALSE otherwise.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */


/**
 * @brief Determines if the byte is a command byte.
 *
 * A byte is a command byte if the most significant bit is 1.
 *
 * @param b The byte to check.
 *
 * @return TRUE if the byte is a command byte or FALSE otherwise.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static BOOL_T isCommandByte(unsigned char b);

/**
 * @brief Transmits a byte through the USART
 *
 * @param data The byte to transmit.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void transmitByte(unsigned char data);

/**
 * @brief Transmits a status byte through the USART indicating failure.
 *
 * The function also stops the timeout.
 *
 * @param numDataBytes The number of following data bytes.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void transmitStatusFail(unsigned char numDataBytes);

/**
 * @brief Transmits a status byte through the USART indicating success.
 *
 * The function also stops the timeout.
 *
 * @param numDataBytes The number of following data bytes.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void transmitStatusSuccess(unsigned char numDataBytes);

/**
 * @brief Postpones the time until a timeout.
 *
 * When a timeout is postponed, more time is given to the system to finish
 * its task. The new amount of time is the timeout period.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void postponeTimeout();

/**
 * @brief Disables the timeout.
 *
 * When a timeout is disabled, the system no longer has to take further action
 * under a time limit. No error will be sent to the computer.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void startTimeout();

/**
 * @brief Disables the timeout.
 *
 * When a timeout is disabled, the system no longer has to take further action
 * under a time limit. No error will be sent to the computer.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void stopTimeout();

/**
 * @brief Interrupt handler for Timer0 Overflow event.
 *
 * Whenever this event happens, a failure status byte will be sent to the
 * computer to indicate a timeout condition. The system will then be reset to
 * the idle state.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
ISR(TIMER0_OVF_vect)
{
	BCLR(1 << TOIE0, TIMSK); /*Clear the interrupt enable for Timer0 overflow to disable timeouts.*/
	commMode = COMM_MODE_IDLE; /*Reset system to idle mode*/
	transmitStatusFail(0); /*Send a failure notice to the computers*/
}

/**
 * @brief Interrupt handler for Timer1 Compare event.
 *
 * Whenever this event happens, the handler gates the de-mux
 * for 24 uS to allow each Sonar to power on and auto calibrate
 *
 * @author Mayur Patel mayur1130@gmail.com
 */
ISR(TIMER1_COMPA_vect)
{
	if(1 << COM1A0 & TCCR1A) {
		OCR1A = OCR1A + 6;  //24 uS
	}
	BCLR(1 << COM1A0, TCCR1A);



}


/**
 * @brief Interrupt handler for Timer1 Compare event.
 *
 * Whenever this event happens, the handler will fire each sonar
 *
 * @author Mayur Patel mayur1130@gmail.com
 */
ISR(TIMER1_COMPB_vect)
{

	wdt_reset(); /*feed the watchdog*/

	/* set bits 6 and 7 of TCCR1A */
	BSET((1 << COM1A0) | (1 << COM1A1)  , TCCR1A);
	OCR1A = OCR1B + 500;  //  add 2ms delay 
	OCR1B = OCR1B + 25000;  // 100ms in future
	
	BSET(1 << ICES1, TCCR1B);

	/* Pick between sonars */
	switch(currentSonar)
	{
		case SONAR_1: /*fire sonar 1*/
				currentSonar = SONAR_2;
				BCLR((1 << PORTC2) | (1 << PORTC3) , PORTC);
				break;
		case SONAR_2: /*fire sonar 2*/
				currentSonar = SONAR_3;
				BSET(1 << PORTC2, PORTC);
				BCLR(1 << PORTC3, PORTC);
				break;
		case SONAR_3: /*fire sonar 3*/
				currentSonar = SONAR_4;
				BCLR(1 << PORTC2, PORTC);
				BSET(1 << PORTC3, PORTC);
				break;
		default: /*fire sonar 4*/
				currentSonar = SONAR_1;
				BSET((1 << PORTC2) | (1 << PORTC3) , PORTC);
				break;
	}
	


}


/**
 * @brief Interrupt handler for Input Capture event.
 *
 * Whenever this event happens, the handler will obtain a rising
 * edge time and a falling edge time for each sonars pulse.
 * It will then subtract the end time from the start time.
 *
 * @author Mayur Patel mayur1130@gmail.com
 */
ISR(TIMER1_CAPT_vect)
{

	if (TCCR1B & (1 << ICES1)) /*if looking for rising edge*/
	{
		startTime = ICR1;
		BCLR(1 << ICES1, TCCR1B);

	}
	else /*if looking for falling edge*/
	{
		endTime = ICR1;
		switch(currentSonar) 
		{
			case SONAR_1:
				sonar_1_time = endTime - startTime;
				break;
			case SONAR_2:
				sonar_2_time = endTime - startTime;
				break;
			case SONAR_3:
				sonar_3_time = endTime - startTime;
				break;
			default:
				sonar_4_time = endTime - startTime;
				break;

		}


	}



}

/**
 * @brief Interrupt handler for USART Data Receive event.
 *
 * This handler takes the appropriate action based on the incoming data and
 * current system state.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
ISR(USART_RXC_vect)
{
	unsigned char received = UDR;
	switch (commMode)
	{
		case COMM_MODE_IDLE:
			if (isCommandByte(received))
			{
				switch(received)
				{
					case CMD_TEST: /*if we are responding to a test connection request*/
						transmitStatusSuccess(1); /*tell the PC that whatever it wanted to do succeeded and 1 more data byte is coming*/
						commMode = COMM_MODE_TEST;
						break;
					case CMD_ID: /*if we are responding to a system ID request*/
						transmitStatusSuccess(1); /*tell the PC that whatever it wanted to do succeeded and 1 more data byte is coming*/
						commMode = COMM_MODE_ID;
						break;
					case CMD_QUERY_SONAR_ALL: /*if we are responding to a request for velocity tick count*/
						transmitStatusSuccess(8); /*tell the PC that whatever it wanted to do succeeded and 2 more data bytes are coming*/
						commMode = COMM_MODE_QUERY_SONAR_ALL;
						stateQrySonarAll = STATE_QRY_SONAR_ALL_1;
						sonar_1_RS232 = sonar_1_time;
						sonar_2_RS232 = sonar_2_time;
						sonar_3_RS232 = sonar_3_time;
						sonar_4_RS232 = sonar_4_time;
						break;
					case CMD_QUERY_SONAR_1: /*if we are responding to a request for velocity tick count*/
						transmitStatusSuccess(2); /*tell the PC that whatever it wanted to do succeeded and 2 more data bytes are coming*/
						commMode = COMM_MODE_QUERY_SONAR_1;
						stateQrySonar1 = STATE_QRY_SONAR_1_1;
						sonar_1_RS232 = sonar_1_time;
						break;
					case CMD_QUERY_SONAR_2: /*if we are responding to a request for velocity tick count*/
						transmitStatusSuccess(2); /*tell the PC that whatever it wanted to do succeeded and 2 more data bytes are coming*/
						commMode = COMM_MODE_QUERY_SONAR_2;
						stateQrySonar2 = STATE_QRY_SONAR_2_1;
						sonar_2_RS232 = sonar_2_time;
						break;
					case CMD_QUERY_SONAR_3: /*if we are responding to a request for velocity tick count*/
						transmitStatusSuccess(2); /*tell the PC that whatever it wanted to do succeeded and 2 more data bytes are coming*/
						commMode = COMM_MODE_QUERY_SONAR_3;
						stateQrySonar3 = STATE_QRY_SONAR_3_1;
						sonar_3_RS232 = sonar_3_time;
						break;
					case CMD_QUERY_SONAR_4: /*if we are responding to a request for velocity tick count*/
						transmitStatusSuccess(2); /*tell the PC that whatever it wanted to do succeeded and 2 more data bytes are coming*/
						commMode = COMM_MODE_QUERY_SONAR_4;
						stateQrySonar4 = STATE_QRY_SONAR_4_1;
						sonar_4_RS232 = sonar_4_time;
						break;
					default: /*If computer sends an unsupported command*/
						transmitStatusFail(0);/*tell the PC that whatever it wanted to do failed*/
						break;
				}
			}
			else /*if in idle mode, we shouldn't get a data byte*/
			{
				transmitStatusFail(0);
			}
			break;
		default:
			transmitStatusFail(0); /*tell the PC that whatever it wanted to do failed*/
			commMode = COMM_MODE_IDLE; /*reset the state machine*/
			break;
	}
}

/**
 * @brief Interrupt handler for USART Data Transmit event.
 *
 * This handler takes the appropriate action, depending on the current system
 * state, after sending a byte to the computer.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
ISR(USART_TXC_vect)
{
	switch (commMode)
	{
		case COMM_MODE_TEST: /*if we are responding to a test connection request*/
			transmitByte(TEST_RESPONSE); /*send the test response*/
			commMode = COMM_MODE_IDLE; /*reset the state machine*/
			break;
		case COMM_MODE_ID: /*if we are responding to a system ID request*/
			transmitByte(address); /*send the I2C address*/
			commMode = COMM_MODE_IDLE; /*reset the state machine*/
			break;
		case COMM_MODE_QUERY_SONAR_ALL: /*if we are responding to a request for velocity tick count*/
			switch (stateQrySonarAll)
			{
				case STATE_QRY_SONAR_ALL_1:
					transmitByte((sonar_1_RS232 >> 7) & 0x7F); /*send the upper part of the tick count, bits 13:7*/
					stateQrySonarAll = STATE_QRY_SONAR_ALL_2; /*go to next stage of sending ticks*/
					break;
				case STATE_QRY_SONAR_ALL_2:
					transmitByte(sonar_1_RS232 & 0x7F); /*send the lower part of the tick count, bits 6:0*/
					stateQrySonarAll = STATE_QRY_SONAR_ALL_3; /*reset the state machine*/
					break;
				case STATE_QRY_SONAR_ALL_3:
					transmitByte((sonar_2_RS232 >> 7) & 0x7F); /*send the upper part of the tick count, bits 13:7*/
					stateQrySonarAll = STATE_QRY_SONAR_ALL_4; /*go to next stage of sending ticks*/
					break;
				case STATE_QRY_SONAR_ALL_4:
					transmitByte(sonar_2_RS232 & 0x7F); /*send the lower part of the tick count, bits 6:0*/
					stateQrySonarAll = STATE_QRY_SONAR_ALL_5; /*reset the state machine*/
					break;
				case STATE_QRY_SONAR_ALL_5:
					transmitByte((sonar_3_RS232 >> 7) & 0x7F); /*send the upper part of the tick count, bits 13:7*/
					stateQrySonarAll = STATE_QRY_SONAR_ALL_6; /*go to next stage of sending ticks*/
					break;
				case STATE_QRY_SONAR_ALL_6:
					transmitByte(sonar_3_RS232 & 0x7F); /*send the lower part of the tick count, bits 6:0*/
					stateQrySonarAll = STATE_QRY_SONAR_ALL_7; /*reset the state machine*/
					break;
				case STATE_QRY_SONAR_ALL_7:
					transmitByte((sonar_4_RS232 >> 7) & 0x7F); /*send the upper part of the tick count, bits 13:7*/
					stateQrySonarAll = STATE_QRY_SONAR_ALL_8; /*go to next stage of sending ticks*/
					break;
				case STATE_QRY_SONAR_ALL_8:
					transmitByte(sonar_4_RS232 & 0x7F); /*send the lower part of the tick count, bits 6:0*/
					commMode = COMM_MODE_IDLE; /*reset the state machine*/
					break;
				default: /*shouldn't ever get here*/
					commMode = COMM_MODE_IDLE; /*reset the state machine*/
					break;
			}
			break;
		case COMM_MODE_QUERY_SONAR_1: /*if we are responding to a request for position tick count*/
			switch (stateQrySonar1)
			{
				case STATE_QRY_SONAR_1_1:
					transmitByte((sonar_1_RS232 >> 7) & 0x7F); /*send the upper part of the tick count, bits 13:7*/
					stateQrySonar1 = STATE_QRY_SONAR_1_2; /*go to next stage of sending ticks*/
					break;
				case STATE_QRY_SONAR_1_2:
					transmitByte(sonar_1_RS232 & 0x7F); /*send the lower part of the tick count, bits 6:0*/
					commMode = COMM_MODE_IDLE; /*reset the state machine*/
					break;
				default: /*shouldn't ever get here*/
					commMode = COMM_MODE_IDLE; /*reset the state machine*/
					break;
			}
			break;
		case COMM_MODE_QUERY_SONAR_2: /*if we are responding to a request for position tick count*/
			switch (stateQrySonar2)
			{
				case STATE_QRY_SONAR_2_1:
					transmitByte((sonar_2_RS232 >> 7) & 0x7F); /*send the upper part of the tick count, bits 13:7*/
					stateQrySonar2 = STATE_QRY_SONAR_2_2; /*go to next stage of sending ticks*/
					break;
				case STATE_QRY_SONAR_2_2:
					transmitByte(sonar_2_RS232 & 0x7F); /*send the lower part of the tick count, bits 6:0*/
					commMode = COMM_MODE_IDLE; /*reset the state machine*/
					break;
				default: /*shouldn't ever get here*/
					commMode = COMM_MODE_IDLE; /*reset the state machine*/
					break;
			}
			break;
		case COMM_MODE_QUERY_SONAR_3: /*if we are responding to a request for position tick count*/
			switch (stateQrySonar3)
			{
				case STATE_QRY_SONAR_3_1:
					transmitByte((sonar_3_RS232 >> 7) & 0x7F); /*send the upper part of the tick count, bits 13:7*/
					stateQrySonar3 = STATE_QRY_SONAR_3_2; /*go to next stage of sending ticks*/
					break;
				case STATE_QRY_SONAR_3_2:
					transmitByte(sonar_3_RS232 & 0x7F); /*send the lower part of the tick count, bits 6:0*/
					commMode = COMM_MODE_IDLE; /*reset the state machine*/
					break;
				default: /*shouldn't ever get here*/
					commMode = COMM_MODE_IDLE; /*reset the state machine*/
					break;
			}
			break;
		case COMM_MODE_QUERY_SONAR_4: /*if we are responding to a request for position tick count*/
			switch (stateQrySonar4)
			{
				case STATE_QRY_SONAR_4_1:
					transmitByte((sonar_4_RS232 >> 7) & 0x7F); /*send the upper part of the tick count, bits 13:7*/
					stateQrySonar4 = STATE_QRY_SONAR_4_2; /*go to next stage of sending ticks*/
					break;
				case STATE_QRY_SONAR_4_2:
					transmitByte(sonar_4_RS232 & 0x7F); /*send the lower part of the tick count, bits 6:0*/
					commMode = COMM_MODE_IDLE; /*reset the state machine*/
					break;
				default: /*shouldn't ever get here*/
					commMode = COMM_MODE_IDLE; /*reset the state machine*/
					break;
			}
			break;
		default:
			commMode = COMM_MODE_IDLE; /*reset the state machine*/
			break;
	}
}

/**
 * @brief This function is the program execution starting point.
 * 
 * This function runs the program loop that reports encoder ticks to the
 * controls software and controls motor power levels.
 *
 * @return Returns zero upon program termination.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com 
 */
int main()
{
	int result = 0;
	init(); /*initialize the uC*/

	wdt_reset(); /*feed the watchdog*/

	while (TRUE)
	{
		sleep_cpu(); /*enter sleep mode*/
	}

	return result;
	
}

/*See function prototype for documentation.*/
static void init(void)
{
	wdt_enable(WDTO_120MS); /*Initialize watchdog, timeout is 15 ms*/

	/*Initialize ports*/
	DDRD = (1 << DDD2);
	DDRC = (1 << DDC0) | (1 << DDC2) | (1 << DDC3) | (1 << DDC4);
	DDRB = (1 << DDB1) | (1 << DDB5);

	
	/*initialize variables*/
	sonar_1_RS232 = 0;
	sonar_2_RS232 = 0;
	sonar_3_RS232 = 0;
	sonar_4_RS232 = 0;
	startTime = 0; 
	endTime = 0;
	sonar_1_time = 0;
	sonar_2_time = 0;
	sonar_3_time = 0;
	sonar_4_time = 0;
	commMode = COMM_MODE_IDLE;
	sonar = SONAR_4;

	/*determine which side we are on*/
	if (PINC & (1 << PINC0))
	{
		address = LEFT_I2C_ADDR;
		BSET(1 << PORTD2, PORTD);
	}
	else
	{
		address = RIGHT_I2C_ADDR;
		BCLR(1 << PORTD2, PORTD);
	}

	

	/*Initialize Serial*/
	UBRRH = UBRRH_VALUE; /*Set baud rate*/
	UBRRL = UBRRL_VALUE; /*Set baud rate*/
	#if USE_2X
	UCSRA |= (1 << U2X);
	#else
	UCSRA &= ~(1 << U2X);
	#endif
	/*Set 8 data bits, 2 stop bits, no parity bits*/
	UCSRC = (1 << URSEL) | (3 << UCSZ0) | (1 << USBS);
	/*Enable receive, transmit, and interrupts*/
	UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE) | (1 << TXCIE);

	/*Setup sleep mode*/
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();

	/*Initialize PWM and input capture on timers. 16 bit Phase and Frequency
	Correct PWM will be used with OCR1A register having the TOP value.
	Interrupt for Input Capture will be enabled.*/
	TCCR0 = (PRESCALER_0 << CS00);

	TCNT1 = 0;
	//TCCR1A = 0;
	TCCR1A = (0 << COM1B0) | (0 << WGM10);
	TCCR1B = (0 << WGM12) | (PRESCALER_1 << CS10);
	OCR1A = 0; //pwm+top
	OCR1B = OCR1A + 25000;
	TIMSK = (1 << TICIE1) | (1 << OCIE2) | (1 << OCIE1A) | (1 << OCIE1B);



	sei(); /* set global interrupt enable */
	
}





/*See function prototype for documentation.*/
static BOOL_T isCommandByte(unsigned char b)
{
	BOOL_T result = FALSE;
	if ((b & 0x80) != 0) /*check that bit 7 is 1*/
	{
		result = TRUE;
	}
	return result;
}

/*See function prototype for documentation.*/
static void transmitByte(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while (!( UCSRA & (1 << UDRE)))
	{
	}

	/* Put data into buffer, send the data */
	UDR = data;
}

/*See function prototype for documentation.*/
static void transmitStatusFail(unsigned char numDataBytes)
{
	stopTimeout();
	transmitByte(0x80 | numDataBytes);
}

/*See function prototype for documentation.*/
static void transmitStatusSuccess(unsigned char numDataBytes)
{
	stopTimeout();
	transmitByte(0xC0 | numDataBytes);
}


/*See function prototype for documentation.*/
static void startTimeout()
{
	TCNT0 = 0; /*reset the counter so full timeout period can be used*/
	if ((1 << TOV0) & TIFR) /*checks for Timer0 overflow falg*/
	{
		BSET(1 << TOV0, TIFR); /*clear Timer0 overlow flag*/
	}
	BSET(1 << TOIE0, TIMSK); /*enable Timer0 interrupt*/
}

/*See function prototype for documentation.*/
static void postponeTimeout()
{
	TCNT0 = 0; /*reset the counter so its interrupt is postponed*/
}



/*See function prototype for documentation.*/
static void stopTimeout()
{
	BCLR(1 << TOIE0, TIMSK); /*turns off the Timer0 interrupt*/
}

/**
 * @}
 * */

