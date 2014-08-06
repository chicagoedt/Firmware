/**
 * @defgroup phobosfirmware_component_control_panel_master Control Panel Master
 * @ingroup phobosfirmware_component_control_panel
 *
 * @brief Control Panel controls the signal multiplexer and reports the system
 * status of the robot 
 *
 * This code is designed to run on an Atmel ATmega32 microcontroller running at
 * 16 MHz. Please refer to the Control Panel requirements
 * and design documents while examining or modifying this code.
 *
 * @author Marios Fanourakis aristogenes@gmail.com
 *
 * @version 1.0
 *
 * @{
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
 * @brief Number of Timer1 ticks to wait before starting the next program loop
 * iteration.
 *
 * This constant is used to determine the delay between consecutive program
 * loop iterations. This constant is equal to (operating frequency) / (main
 * program loop frequency * 16 bit timer prescaler).
 *
 * @author Marios Fanourakis aristogenes@gmail.com
 */
#define CLK_TCKS_2 125

/**
 * @brief Number of program loop iterations before updating the LCD.
 *
 * @author Marios Fanourakis aristogenes@gmail.com
 */
#define NUM_CYCLES 25

/**
 * @brief Constant to set 8 bit Timer0 prescaler.
 *
 * See page 80 of the ATmega32 data sheet for valid values.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
//#define PRESCALER_0 4

/**
 * @brief Constant to set 16 bit Timer1 prescaler.
 *
 * See page 110 of the ATmega32 data sheet for valid values.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
//#define PRESCALER_1 2

/**
 * @brief Constant to set 8 bit Timer2 prescaler.
 *
 * See page 125 of the ATmega32 data sheet for valid values.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define PRESCALER_2 5


/**
 * @brief I2C address of this module.
 *
 */
#define CONTROL_PANEL_ADDR 0x1B



 /**
 * @brief This macro sets bits in a variable or register.
 *
 * The bits masked will be changed to logic 1. The other bits will remain
 * unchanged.
 *
 * @warning No type checking or parameter validation is performed.
 *
 * @param mask The bits mask that specifies which bits should be set. The bits
 * that are set in the mask will be set.
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
 * bits that are set in the mask will be cleared.
 * @param reg The register or variable that is to have its bits cleared.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define BCLR(mask, reg) (reg &= ~(mask))

/**
 * @brief Boolean logic data type.
 *
 * This type should only take on one of two values: TRUE or FALSE.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
typedef unsigned char BOOL_T;


/**
 * @brief This variable contains the system's I2C address.
 *
 */
static unsigned char address;

 /**
 * @brief Variable that counts the number of loop iterations in the program.
 *
 * This variable is used to count how many program iterations have passed in
 * order to precisely determine the timeout interval. When this variable
 * equals #NUM_CYCLES, a timeout occurs.
 *
 * This variable is volatile because it is modified by interrupts.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static volatile unsigned char loopIterations;

/**
 * @brief Stores the state of the input switches.
 *
 * The PISO data is saved in this variable during each program iteration. This variable is then used to
 * determine which mode to switch the system to.
 *
 * Data format:
 * - Bit 0: Switch to select safety mode, low = selected
 * - Bit 1: Switch to select RC mode, low = selected
 * - Bit 2: Switch to select autonomous mode, low = selected
 * - Bit 3: Switch to select user select mode, low = selected
 *
 * This variable is volatile because it is modified by interrupts.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static volatile unsigned char switchesPISO;

/**
 * @brief Stores the state of the input switches.
 *
 * The signal multiplexer portion of the PISO data is saved in this variable during each program iteration.
 * This variable is then used to determine which mode to switch the system to.
 *
 * Data format:
 * - Bit 0: Switch to select safety mode, low = selected
 * - Bit 1: Switch to select RC mode, low = selected
 * - Bit 2: Switch to select autonomous mode, low = selected
 * - Bit 3: Switch to select user select mode, low = selected
 *
 * This variable is volatile because it is modified by interrupts.
 *
 * @author Marios Fanourakis aristogenes@gmail.com
 */
static volatile unsigned char sigMuxSwitches;

/**
 * @brief Stores the previous state of the sig mux input switches when something was pressed.
 *
 * Data format:
 * - Bit 0: Switch to select safety mode, low = selected
 * - Bit 1: Switch to select RC mode, low = selected
 * - Bit 2: Switch to select autonomous mode, low = selected
 * - Bit 3: Switch to select user select mode, low = selected
 *
 * This variable is volatile because it is modified by interrupts.
 *
 * @author Marios Fanourakis aristogenes@gmail.com
 */
static volatile unsigned char sigMuxSwitchesb;

/**
 * @brief This variable contains the selected mode of the signal multiplexer
 * - Bit 1:0 : Current selection; 00 safety, 01 RC, 10 auto, 11 usr select
 * This variable is volatile because it is modified by interrupts.
 */
 static volatile unsigned char sigMuxI2CwriteByte;

 /**
 * @brief This variable determines if there needs to be a change in the signal multiplexer
 *
 * This variable is volatile because it is modified by interrups.
 */
static volatile BOOL_T sigMuxChange;

/**
 * @brief Stores the state of the input switches.
 *
 * The signal multiplexer portion of the PISO data is saved in this variable during each program iteration.
 * This variable is then used to determine which mode to switch the system to.
 *
 * Data format:
 * - Bit 0: Switch to select safety mode, low = selected
 * - Bit 1: Switch to select RC mode, low = selected
 * - Bit 2: Switch to select autonomous mode, low = selected
 * - Bit 3: Switch to select user select mode, low = selected
 *
 * This variable is volatile because it is modified by interrupts.
 *
 * @author Marios Fanourakis aristogenes@gmail.com
 */
static volatile unsigned char sigMuxSwitches;

/**
 * @brief Resets the timer for the signal timeout.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void resetTimer();

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
 * @brief Interrupt handler for Timer1 Compare event.
 *
 * This handler is responsible for the main program loop. Whenever the 1
 * compare event occurs, the next iteration of the program is run.
 *
 * @author Marios Fanourakis aristogenes@gmail.com
 */
ISR(TIMER2_COMP_vect)
{
	//wdt_reset(); /*feed the watchdog*/
					
		/*Fire clock on PISO*/
		BCLR(1 << PD2, PORTD);
		BSET(1 << PD2, PORTD);

		while (loopIterations < 30)
		{
			loopIterations++;
		}
		resetTimer();

		SPDR = 0xFF;

		/*This block reads in the value of PISO from SPI*/
		while ((SPSR & (1 << SPIF)) == 0)
		{
		}
		
		//BCLR(1<<PC0,PORTC);
		switchesPISO = SPDR;
		sigMuxSwitches = switchesPISO & 0x0F;


		/*This block analyzes the sig mux switches on the PISO*/
			if (sigMuxSwitches == 0x0E)
			{
				//sigMuxI2CwriteByte = 0x00;
				BCLR(1 << PC0,PORTC);
				BCLR(1 << PC1,PORTC);

				BSET(1 << PB0,PORTB);
				BCLR(1 << PB1,PORTB);
			}
			else if (sigMuxSwitches == 0x0D)
			{
				//sigMuxI2CwriteByte = 0x02;
				BCLR(1 << PC0,PORTC);
				BSET(1 << PC1,PORTC);

				BSET(1 << PB0,PORTB);
				BCLR(1 << PB1,PORTB);
			}
			else if (sigMuxSwitches == 0x0B)
			{
				//sigMuxI2CwriteByte = 0x01;
				BSET(1 << PC0,PORTC);
				BCLR(1 << PC1,PORTC);
				
				BCLR(1 << PB0,PORTB);
				BSET(1 << PB1,PORTB);
			}
			else if (sigMuxSwitches == 0x07)
			{
				//sigMuxI2CwriteByte = 0x03;
				BSET(1 << PC0,PORTC);
				BSET(1 << PC1,PORTC);
				
				BCLR(1 << PB0,PORTB);
				BSET(1 << PB1,PORTB);
			}
			else
			{
				
			}



}

/**
 * @brief This function is the program execution starting point.
 * 
 * This function continuously runs the program loop that checks for a PWM
 * signal and sets the output pin values accordingly.
 *
 * @return Returns zero upon program termination.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com 
 */
int main()
{

	init();
	
	while (TRUE)
	{
		sleep_cpu(); /*enter sleep mode*/
			
	}
	return 0;
}

/*See function prototype for documentation.*/
void resetTimer(void)
{
	loopIterations = 0;
}

/*See function prototype for documentation.*/
static void init(void)
{
	//wdt_enable(WDTO_15MS); /*Initialize watchdog, timeout is 15 ms*/

	/*Initialize ports*/
//	DDRA = 0xFF;
	DDRB = (1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (1 << DDB4) | (1 << DDB5) | (0 << DDB6) | (1 << DDB7);
	DDRC = (1 << DDC0)| (1 << DDC1) | (1 << DDC2) | (1 << DDC3) | (1 << DDC4) | (1 << DDC5);
	DDRD = (1 << DDD1) | (1 << DDD2);


	/*Set variables to default values*/
	loopIterations = 0;

	/*Setup SPI*/
	BSET(1 << PD2, PORTD); /*PISO loads data when RCK low so we have to normally keep this pin high*/

	SPCR = (1 << SPE) | (1 << MSTR);

	/*Setup the Control Panel Address */
	address = CONTROL_PANEL_ADDR;


	/*Setup sleep mode*/
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();

	/*Configure Timers*/
	//TCCR0 = (PRESCALER_0 << CS00);

	TIMSK = (1 << OCIE2); /*Enable input capture interrupts*/


	TCCR2 = (PRESCALER_2 << CS20) | (1 << WGM21);
	OCR2 = CLK_TCKS_2 - 1; /*This value is subtracted by one since CTC mode will go for (OCR2 + 1) cycles before starting over*/
	TCNT2 = 0;
	TIFR = 1 << OCF2; /*Clear the time compare bit if set*/

//setup default sigmux mode
	BCLR(1<<PC0,PORTC);
	BCLR(1<<PC1,PORTC);


	sei(); /* set global interrupt enable */
}


/**
 * @}
 * */

