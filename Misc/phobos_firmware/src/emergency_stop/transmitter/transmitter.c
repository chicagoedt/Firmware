/**
 * @defgroup phobosfirmware_component_emergency_stop_transmitter Emergency Stop Transmitter
 * @ingroup phobosfirmware_component_emergency_stop
 *
 * @brief The emergency stop transmitter is designed to continuously send a signal to
 * the emergency stop receiver.
 *
 * The signal tells the receiver to either cut or provide power to the robot.
 * A switch determines the signal that is to be sent. If the switch is pushed
 * down, the signal is to stop the robot, otherwise the robot should keep
 * moving. The signal is sent to a RF transmitter, which transmits the signal
 * to the receiver.
 *
 * This code is designed to run on an Atmel ATmega8 microcontroller running at
 * 2 MHz. Please refer to the Emergency Stop requirements and design documents
 * when examining or modifying this code.
 *
 * Pin assignments:
 * - PB0: Red pushbutton switch - input, high = stop, low = go.
 * - PB1: Switch position status LED - output, high = stop, low = go.
 * - PB2: CTSO of radio unit - input, high = do not send data, low = send data.
 * - PD1: TxD - output
 * @author Anup Kotadia anup.kotadia@gmail.com
 *
 * @version 2.0
 *
 * @{
 */

/*Include definition files*/
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

/**
 * @brief Operating Frequency of uC in Hertz.
 */
#define F_CPU 2000000UL

/**
 * @brief Baud rate of RS-232 communication in bps.
 */
#define BAUD 19200
#include <util/setbaud.h>

/**
 * @brief Signal to stop the robot.
 *
 * This signal is sent to the emergency stop receiver to tell the receiver to
 * let the robot move.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define STOP_SIGNAL 'S'

/**
 * @brief Signal to let the robot have power.
 *
 * This signal is sent to the emergency stop receiver to tell the receiver to
 * provide power to the robot's motors.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define GO_SIGNAL 'G'

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
 * @brief Constant to set 16 bit Timer1 prescaler.
 *
 * See page 100 of the ATmega8 data sheet for valid values.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define PRESCALER_1 3

/**
 * @brief Number of timer ticks to wait before sending another signal.
 *
 * This constant is used to determine the delay between consecutive signal
 * transmissions. This constant is equal to (operating frequency) / (main
 * program loop frequency * prescaler).
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 *
 */
#define DELAY_TCKS 250

/**
 * @brief Boolean logic data type.
 *
 * This type should only take on one of two values: TRUE or FALSE.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
typedef unsigned char BOOL_T;

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
 * @brief This function polls the remote emergency stop switch.
 *
 * The wireless emergency stop switch is connected to PB0 and this function
 * polls this pin for the position of the switch. The switch is closed when the
 * switched is pulled up and open when the switch is pushed down.
 *
 * @return This function returns TRUE if the switch is pushed down or
 * FALSE if the switch is not pressed.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static BOOL_T isSwitchDown(void);

/**
 * @brief This function sends a byte to the RF transmitter.
 *
 * The RF transmitter sends the byte to the RF receiver on the receive unit of
 * the emergency stop. The receive unit will then read the byte and trigger the
 * emergency stop if necessary.
 *
 * @param signal A byte that is to be sent to the emergency stop receiver.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void sendSignal(unsigned char signal);

/**
 * @brief This function initializes the microcontroller.
 *
 * The 16 bit timer, the RS232 interface, the outputs pins, and the watchdog
 * timer of the microcontroller are initialized. This should be the first
 * function called in the program.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void init(void);

/**
 * @brief Interrupt handler for Timer1 Output Compare A event.
 *
 * Whenever Timer1 encounters a successful timer compare, this interrupt will
 * be called and wake the uC from sleep mode. This interrupt handler polls the
 * switch position, sends the command to the receiver, and then returns the uC
 * to sleep mode.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
ISR(TIMER1_COMPA_vect)
{
	wdt_reset(); /*feed the watchdog*/
	if (isSwitchDown())
	{
		sendSignal(STOP_SIGNAL);
		BSET(1 << PORTB1, PORTB); /*Turns on E-stop engaged indicator*/
	}
	else
	{
		sendSignal(GO_SIGNAL);
		BCLR(1 << PORTB1, PORTB); /*Turns off E-stop engaged indicator*/
	}
}


/**
 * @brief This function is the program execution starting point.
 *
 * This function initializes the transmitter and puts the uC to sleep. The uC
 * will then rely on timer interrupts to wake up periodically.
 *
 * @return Returns zero upon program termination.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
int main(void)
{
	int result = 0;
	init();
	wdt_reset(); /*feed the watchdog*/

	while (TRUE)
	{
		sleep_cpu(); /*enter sleep mode*/
	}
	return result;
}

/*See function prototype for documentation.*/
BOOL_T isSwitchDown(void)
{
	BOOL_T result = FALSE;
	if (PINB & (1 << PINB0))
	{
		result = TRUE;
	}
	return result;
}

/*See function prototype for documentation.*/
void sendSignal(unsigned char signal)
{
	if (!(PINB & (1 << PINB2)))
	{
		/* Wait for empty transmit buffer */
		while ( !( UCSRA & (1 << UDRE)) )
		{
		}
		/* Put data into buffer, sends the data */
		UDR = signal;
	}
}

/*See function prototype for documentation.*/
void init(void)
{
	/*Initialize watchdog*/
	wdt_enable(WDTO_15MS); /*Time is 15 ms*/

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
	UCSRB = 1 << TXEN; /*Enable transmit*/

	/*Initialize PORTB*/
	DDRB = 1 << DDB1; /*Set PB1 as output*/

	/*Setup sleep mode*/
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();

	/*Initialize timer*/
	TCCR1A = 0x00;
	TCCR1B = (1 << WGM12) | (PRESCALER_1 << CS10);
	OCR1A = DELAY_TCKS - 1; /*This value is subtracted by one since CTC mode will go for (OCR1A + 1) cycles before starting over*/
	TCNT1 = 0;
	TIFR = 1 << OCF1A; /*Clear the time compare bit if set*/
	TIMSK = (1 << OCIE1A); /*interrupt on Output Compare A*/

	sei(); /*Enable interrupts*/
}

/**
 * @}
 * */

