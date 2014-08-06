/**
 * @defgroup phobosfirmware_component_emergency_stop_receiver Emergency Stop Receiver
 * @ingroup phobosfirmware_component_emergency_stop
 *
 * @brief The emergency stop receiver is designed to stop the robot if signaled
 * to do so.
 *
 * The receiver looks for a signal from the emergency stop transmitter. The
 * signal tells the receiver to either cut or provide power to the robot. There
 * is also a switch on the robot which determines if the robot should receive
 * power. When the switch is pushed down, the robot should stop
 * and when the switch is not pressed, the robot motors should
 * have power. If either the received signal
 * from the transmitter or the switch on the robot gives a STOP command, the
 * receiver circuit will cut power to the robot. The receiver
 * gracefully handles short term wireless connection outages and stops the robot during
 * long outages.
 *
 * This code is designed to run on an Atmel ATmega8 microcontroller running at
 * 16 MHz. Please refer to the Emergency Stop requirements and design documents
 * when examining or modifying this code.
 *
 * Pin assignments:
 * - PB0: Onboard switch - input, high = stop, low = go
 * - PB1: Relay signal and Status LEDs for telling if robot is stopped or not - output, high = go, low = stop
 * - PB2: Status LED for wireless connection lost - output, high = lost connection
 * - PB3: Onboard switch position status LED - output, high = stop, low = go
 * - PB4: Status LED for signal from wireless connection - output, high = go
 * - PB5: Status LED for signal from wireless connection - output, high = stop
 * - PC4: SDA
 * - PC5: SCL
 * - PD0: RxD - input
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 *
 * Revised by: John Sabino johntsabino@gmail.com
 * Revision date: 06/11/2014
 *
 * @version 3.0
 *
 * @{
 */

/*Preprocessor Flags*/
#undef I2C_ENABLED

/*Include definition files*/
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

/**
 * @brief Operating Frequency of uC in Hertz.
 */
#define F_CPU 16000000UL

/**
 * @brief Baud rate of RS-232 communication in bps.
 */
#define BAUD 19200
#include <util/setbaud.h>

/**
 * @brief I2C address of this module.
 *
 */
#define I2C_ADDRESS 0x18

/**
 * @brief Signal to stop the robot.
 *
 * If the receiver receives this signal from the transmitter, the receiver
 * should cut power to the robot's motors.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define STOP_SIGNAL 'S'

/**
 * @brief Signal to let the robot have power.
 *
 * If the receiver receives this signal from the transmitter, the receiver
 * may provide power to the robot's motors if other factors allow it.
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
#define PRESCALER_1 4

/**
 * @brief Constant to set 8 bit Timer2 prescaler.
 *
 * See page 118 of the ATmega8 data sheet for valid values.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define PRESCALER_2 7

/**
 * @brief Number of Timer1 ticks to wait before timing out.
 *
 * This constant is used to determine the delay before the system decides that
 * the wireless signal has been lost. This constant is equal to (operating
 * frequency * timeout period) / prescaler.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define DELAY_TCKS_1 12500

/**
 * @brief Number of Timer2 ticks to wait before running next program loop.
 *
 * This constant is used to determine the delay between consecutive
 * program loops. This constant is equal to (operating frequency) / (main
 * program loop frequency * prescaler).
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define DELAY_TCKS_2 125

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


#ifdef I2C_ENABLED
        /**
 	* @brief Stores the information that is to be sent to the CP via I2C.
 	* 
 	* This variable should be sent when the I2C bus requests data. The format of
 	* this variable is:
 	* - Bit 0: Set if motors are powered or cleared if power is cut.
 	* - Bits 2:1:
 	*   - 00 if wireless stop signal.
 	*   - 01 if wireless go signal
 	*   - 10 if wireless signal lost 
 	* - Bit 3: Set if the on-board switch is enabling power to the motors or zero otherwise.
 	*	
 	* This variable is volatile because it is modified by interrupts.
	*/
	static volatile unsigned char statusI2C;
#endif	//End I2C_ENABLED

/**
 * @brief This function polls the onboard emergency stop switch.
 *
 * The onboard emergency stop switch is connected to PB0 and this function
 * polls this pin for the position of the switch. The switch is closed when the
 * switched is pulled up and open when the switch is pushed down.
 *
 * @return This function returns TRUE if the switch is pushed down or
 * FALSE if the switch is not pressed.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
BOOL_T isSwitchDown(void);

/**
 * @brief This function provides power to the robot's motors.
 *
 * This function uses PB1 to drive the power relay and the "power on" and "power
 * off" indicator lights.
 *
 * @param givePower Whether to give or cut power to the motors.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
void providePower(BOOL_T givePower);

/**
 * @brief This function manages the indicator lighta for the signal coming from the transmitter.
 *
 * This function uses PB2 to light the lost connection indicator light, PB4 to
 * light the wireless GO signal indicator light, and PB5 to light the wireless
 * STOP signal indicator light.
 *
 * @param connectionLost Whether or not the connection to the transmitter has
 * been lost.
 * @param rfStop Whether or not the RF STOP signal has been received.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
void setConnectionLight(BOOL_T connectionLost, BOOL_T rfStop);

/**
 * @brief This function resets the timeout period.
 *
 * This function is called when valid data from the transmitter has arrived so
 * that the next incoming signal has the full timeout period before a lost
 * connection to the transmitter is declared.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
inline void postponeTimeout();

/**
 * @brief This function initializes the microcontroller.
 *
 * The timers, the RS232 interface, the outputs pins, and the watchdog
 * timer of the microcontroller are initialized. This should be the first
 * function called in the program.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
inline void init(void);

/**
 * @brief Interrupt handler for Timer2 Output Compare event.
 *
 * Whenever Timer1 encounters a successful timer compare, this interrupt will
 * be called and wake the uC from sleep mode. This interrupt handler polls the
 * switch position and sets the relay based on the wireless and switch states,
 * The uC is then returned to sleep mode.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
ISR(TIMER2_COMP_vect)
{
	wdt_reset(); /*feed the watchdog*/
}//End Timer_2_Compare Interrupt

/**
 * @brief Interrupt handler for Timer1 Output Compare A event.
 *
 * Whenever the system timeout from not having received a valid wireless
 * signal, this handler is called. It states that the RF is is not present
 * and that the robot should stop.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
ISR(TIMER1_COMPA_vect)
{
	providePower(FALSE);
}//End Timer_1_A_Compare Interrupt

/**
 * @brief Interrupt handler for USART Data Receive event.
 *
 * This handler checks the incoming wireless data and determines if it is valid
 * or not. If it is valid, it postpones the timeout and notifies the system of
 * the wireless signal.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
ISR(USART_RXC_vect)
{
	unsigned char received = UDR;
	switch(received)
	{
		case GO_SIGNAL:
			postponeTimeout();
		default:
			break;
	}//End switch	
}//End USART_RXC_Interrupt

/**
 * @brief This function is the program execution starting point.
 * 
 * This function initializes the receiver and runs the program loop that
 * constantly checks input and controls power to the motors.
 *
 * @return Returns zero upon program termination.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com 
 */
int main(void)
{
	init();
	providePower(FALSE);

	wdt_reset(); /*feed the watchdog*/

	while (TRUE)		{sleep_cpu(); /*enter sleep mode*/}

	return main();	//In the case the code falls through this restarts it.
}

/*See function prototype for documentation.*/
BOOL_T isSwitchDown(void)
{
	BOOL_T result = FALSE;
	if (PINB & (1 << PINB0))
	{
		result = TRUE;
		BSET(1 << PORTB3, PORTB);
#ifdef I2C_ENABLED
		BCLR(1 << 3, statusI2C);
#endif	//End I2C_ENABLED
	}//End if
	else
	{
		BCLR(1 << PORTB3, PORTB);
#ifdef I2C_ENABLED
		BSET(1 << 3, statusI2C);
#endif	//End I2C_ENABLED
	}//End else
	return result;
}//End isSwitchDown

/*See function prototype for documentation.*/
void providePower(BOOL_T givePower)
{
	if (givePower)
	{
		BSET(1 << PORTB1, PORTB); /*Turns on relay*/
#ifdef I2C_ENABLED
		BSET(1 << 0, statusI2C);
#endif	//End I2C_ENABLED
		
		//Set LEDs:
		BSET(1 << PORTB4, PORTB);
                BCLR((1 << PORTB2) | (1 << PORTB5), PORTB);
	}//End if
	else
	{
		BCLR(1 << PORTB1, PORTB); /*Turns off relay*/
#ifdef I2C_ENABLED
		BCLR(1 << 0, statusI2C);
#endif	//End I2C_ENABLED
		//Set LEDs:
		BCLR((1 << PORTB2) | (1 << PORTB4), PORTB);
	}//End else
}//End providePower

/*See function prototype for documentation.*/
/*
void setConnectionLight(BOOL_T connectionLost, BOOL_T rfStop)
{
	if (connectionLost)
	{
		BSET(1 << PORTB2, PORTB);
		BCLR((1 << PORTB4) | (1 << PORTB5), PORTB);
		BCLR(1 << 1, statusI2C);
		BSET(1 << 2, statusI2C);
	}
	else if (rfStop)
	{
		BSET(1 << PORTB5, PORTB);
		BCLR((1 << PORTB2) | (1 << PORTB4), PORTB);
		BCLR((1 << 2) | (1 << 1), statusI2C);
	}
	else
	{
		BSET(1 << PORTB4, PORTB);
		BCLR((1 << PORTB2) | (1 << PORTB5), PORTB);
		BCLR(1 << 2, statusI2C);
		BSET(1 << 1, statusI2C);
	}
}
*/

/*See function prototype for documentation.*/
inline void postponeTimeout()
{
	TCNT1 = 0;
}//End postponeTimeout

/*See function prototype for documentation.*/
inline void init(void)
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
	
	/*Set 8 data bits, 2 stop bit, no parity bits*/
	UCSRC = (1 << URSEL) | (3 << UCSZ0) | (1 << USBS);
	UCSRB = (1 << RXEN) | (1 << RXCIE); /*Enable receive and receive interrupt*/

	/*Initialize PORTB*/
	DDRB = (1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (1 << DDB4) | (1 << DDB5);

	/*Setup sleep mode*/
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();

	/*Setup I2C*/


	/*Initialize timer*/
	TCCR1A = 0x00;
	TCCR1B = (1 << WGM12) | (PRESCALER_1 << CS10);
	OCR1A = DELAY_TCKS_1 - 1; /*This value is subtracted by one since CTC mode will go for (OCR1A + 1) cycles before starting over*/
	TCNT1 = 0;
	TIFR = 1 << OCF1A; /*Clear the time compare bit if set*/

	TCCR2 = (PRESCALER_2 << CS20) | (1 << WGM21);
	OCR2 = DELAY_TCKS_2 - 1; /*This value is subtracted by one since CTC mode will go for (OCR2 + 1) cycles before starting over*/
	TCNT2 = 0;
	TIFR = 1 << OCF2; /*Clear the time compare bit if set*/

	TIMSK = (1 << OCIE1A) | (1 << OCIE2); /*interrupt on Output Compare A and Timer2 Output Compare*/

	sei(); /*Enable interrupts*/
}
/**
 * @}
 * */

