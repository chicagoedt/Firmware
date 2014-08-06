/**
 * @defgroup phobosfirmware_component_sysmon System Monitor
 * @ingroup phobosfirmware_components
 *
 * @brief The System Monitor checks on various robot systems and reports problems.
 *
 * The System Monitor monitors the charge level of the batteries and the state
 * of the circuit breakers. It also passes this information to the Control
 * Panel via the I2C interface, where it can be viewed by the user and the
 * computer software.
 *
 * This code is designed to run on an Atmel ATmega8 microcontroller running at
 * 16 MHz. Please refer to the System Monitor requirements
 * and design documents when examining or modifying this code.
 *
 * Pin assignments:
 * - PB1: OC1A, status LED for 12V battery low - output, high = battery low, flashing = battery critically low
 * - PB2: OC1B, status LED for 24V battery low - output, high = battery low, flashing = battery critically low
 * - PB3: Status LED for 12V battery good - output, high = battery good
 * - PB4: Status LED for 24V battery good - output, high = battery good
 * - PC0: ADC0, measures 12V battery system voltage - input
 * - PC1: ADC1, measures 24V battery system voltage - input
 * - PC2: Checks left circut breaker - input, high = breaker popped, low = breaker closed
 * - PC3: Checks right circut breaker - input, high = breaker popped, low = breaker closed
 * - PC4: SDA
 * - PC5: SCL
 * - PD0: Status LEDs for left circuit breaker - output, high = breaker popped, low = breaker closed
 * - PD1: Status LEDs for right circuit breaker - output, high = breaker popped, low = breaker closed
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 *
 * @version 1.0
 *
 * @{
 */

/*Include definition files*/
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <stdint.h>
#include <avr/sleep.h>


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
 * @brief Constant to set 8 bit Timer2 prescaler.
 *
 * See page 118 of the ATmega8 data sheet for valid values.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define PRESCALER_2 7

/**
 * @brief Number of timer ticks to wait before starting another program iteration.
 *
 * This constant is equal to (operating frequency) / (main program loop
 * frequency * prescaler).
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define DELAY_TCKS 125

/**
 * @brief Value to set PWM frequency
 *
 * This value determines the TOP value used for 16 bit Phase and Frequency
 * Correct PWM based on clock frequency. See page 94 of the ATmega8 data sheet
 * for valid values.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define PWM_TOP 31250

/**
 * @brief Value to set PWM duty cycle
 *
 * This value determines the amount of time the PWM is high. This value is 
 * equal to PWM_TOP * (duty cycle).
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define DUTY_CYCLE 15625

/**
 * @brief Value where the 12V battery system is deemed to be low.
 *
 * This constant is used to determine if the 12V battery system is low on
 * charge but not critically low. The batteries should be changed soon.This
 * value is equal to (low 12V threshold) / (64 mV).
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define LOW_THRESHOLD_12V 191

/**
 * @brief Value where the 12V battery system is deemed to be critically low.
 *
 * This constant is used to determine if the 12V battery system is critically
 * low on charge such that robot operation is unsafe. The robot should be
 * stopped immediately and the batteries need to be changed. This
 * value is equal to (critical 12V threshold) / (64 mV).
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define CRIT_THRESHOLD_12V 184

/**
 * @brief Value where the 24V battery system is deemed to be low.
 *
 * This constant is used to determine if the 12V battery system is low on
 * charge but not critically low. The batteries should be changed soon.This
 * value is equal to (low 24V threshold) / (128 mV).
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define LOW_THRESHOLD_24V 191

/**
 * @brief Value where the 24V battery system is deemed to be critically low.
 *
 * This constant is used to determine if the 24V battery system is critically
 * low on charge such that robot operation is unsafe. The robot should be
 * stopped immediately and the batteries need to be changed. This
 * value is equal to (critical 24V threshold) / (128 mV).
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define CRIT_THRESHOLD_24V 184

/**
 * @brief I2C address of this module.
 *
 */
#define I2C_ADDRESS 0x1A

/**
 * @brief Bit flag to indicate left circuit breaker status.
 *
 * This flag is to be used with the #statusI2C variable. If the bit
 * corresponding to this flag is set in that variable, it means the left
 * circuit breaker popper, otherwise the breaker is fine.
 */
#define FLG_LEFT_BRKR_POP _BV(0)

/**
 * @brief Bit flag to indicate right circuit breaker status.
 *
 * This flag is to be used with the #statusI2C variable. If the bit
 * corresponding to this flag is set in that variable, it means the right
 * circuit breaker popper, otherwise the breaker is fine.
 */
#define FLG_RIGHT_BRKR_POP _BV(1)

/**
 * @brief Bit flag to indicate 12V battery system is low.
 *
 * This flag is to be used with the #statusI2C variable. If the bit
 * corresponding to this flag is set in that variable, it means the 12V battery
 * system is running low on charge, otherwise the battery system is fine.
 */
#define FLG_12V_BATT_LOW _BV(4)

/**
 * @brief Bit flag to indicate 12V battery system is critically low.
 *
 * This flag is to be used with the #statusI2C variable. If the bit
 * corresponding to this flag is set in that variable, it means the 12V battery
 * system is running critically low on charge.
 */
#define FLG_12V_BATT_CRIT _BV(5)

/**
 * @brief Bit flag to indicate 24V battery system is low.
 *
 * This flag is to be used with the #statusI2C variable. If the bit
 * corresponding to this flag is set in that variable, it means the 24V battery
 * system is running low on charge, otherwise the battery system is fine.
 */
#define FLG_24V_BATT_LOW _BV(6)

/**
 * @brief Bit flag to indicate 24V battery system is critically low.
 *
 * This flag is to be used with the #statusI2C variable. If the bit
 * corresponding to this flag is set in that variable, it means the 24V battery
 * system is running critically low on charge.
 */
#define FLG_24V_BATT_CRIT _BV(7)

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
 * @brief Stores the 24V system battery voltage.
 *
 * The analog to digital converter samples the voltage of the 24V system and
 * stores its value here. This value is then sent to the Control Panel via the
 * I2C interface and also analyzed to determine if the battery levels are low.
 */
static volatile char reading_24V;

/**
 * @brief Stores the 12V system battery voltage.
 *
 * The analog to digital converter samples the voltage of the 12V system and
 * stores its value here. This value is then sent to the Control Panel via the
 * I2C interface and also analyzed to determine if the battery levels are low.
 */
static volatile char reading_12V;

/**
 * @brief Stores the information that is to be sent to the CP via I2C.
 *
 * This variable contains information about the state of the
 * circuit breakers and batteries. This, along with the battery voltages,
 * should be sent to the CP when requested through I2C.
 *
 * - Bit 0 is 1 if there is a fault on the left ciruit breaker or 0 otherwise.
 * - Bit 1 is 1 if there is a fault on the right ciruit breaker or 0 otherwise.
 * - Bit 4 is 1 if the 12V battery system is low or 0 otherwise.
 * - Bit 5 is 1 if the 12V battery system is critically low or 0 otherwise.
 * - Bit 6 is 1 if the 24V battery system is low or 0 otherwise.
 * - Bit 7 is 1 if the 24V battery system is critically low or 0 otherwise.
 */
static volatile char statusI2C;

/**
 * @brief This function initializes the microcontroller.
 *
 * The 16 bit timer, the ADC, the I2C interface, the outputs pins, and the
 * watchdog timer of the microcontroller are initialized. This should be the
 * first function called in the program.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void init(void);

/**
 * @brief Interrupt handler for Timer2 output compare event.
 *
 * Whenever Timer2 encounters a successful timer compare, this interrupt will
 * be called and wake the uC from sleep mode. This interrupt handler polls the
 * circuit breaker status and measures the battery voltages.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
ISR(TIMER2_COMP_vect)
{
	wdt_reset(); /*feed the watchdog*/

	/*Checking left circuit breaker*/
	if (PINC & (1 << PINC2))
	{
		BSET(FLG_LEFT_BRKR_POP, statusI2C);
		BSET(1 << PORTD0, PORTD);
	}
	else
	{
		BCLR(FLG_LEFT_BRKR_POP, statusI2C);
		BCLR(1 << PORTD0, PORTD);
	}

	/*Checking right circuit breaker*/
	if (PINC & (1 << PINC3))
	{
		BSET(FLG_RIGHT_BRKR_POP, statusI2C);
		BSET(1 << PORTD1, PORTD);
	}
	else
	{
		BCLR(FLG_RIGHT_BRKR_POP, statusI2C);
		BCLR(1 << PORTD1, PORTD);
	}

	/*Measure 12V system*/
	BCLR(1 << MUX0, ADMUX);
	BSET(1 << ADSC, ADCSRA);
	while (!(ADCSRA & (1 << ADIF)))
	{
	}
	reading_12V = ADCH;
	BSET((1 << ADIF), ADCSRA);

	if (reading_12V < CRIT_THRESHOLD_12V) /*If 12V system is critically low*/
	{
		BCLR((1 << PB3), PORTB);
		BCLR((1 << COM1A0), TCCR1A);
		BSET((1 << COM1A1), TCCR1A);

		BSET(FLG_12V_BATT_CRIT | FLG_12V_BATT_LOW, statusI2C);
	}
	else if(reading_12V < LOW_THRESHOLD_12V) /*If 12V system is low*/
	{
		BCLR((1 << PB3), PORTB);
		BSET((1 << PB1), PORTB);
		BCLR((1 << COM1A1) | (1 << COM1A0), TCCR1A);

		BSET(FLG_12V_BATT_LOW, statusI2C);
		BCLR(FLG_12V_BATT_CRIT, statusI2C);
	}
	else /*If 12V system is fine*/
	{
		BSET((1 << PB3), PORTB);
		BCLR((1 << PB1), PORTB);
		BCLR((1 << COM1A1) | (1 << COM1A0), TCCR1A);

		BSET(FLG_12V_BATT_CRIT | FLG_12V_BATT_LOW, statusI2C);
	}

	/*Measure 24V system*/
	BSET(1 << MUX0, ADMUX);
	BSET(1 << ADSC, ADCSRA);
	while (!(ADCSRA & (1 << ADIF)))
	{
	}
	reading_24V = ADCH;
	BSET((1 << ADIF), ADCSRA);

	if (reading_24V < CRIT_THRESHOLD_24V) /*If 24V system is critically low*/
	{
		BCLR((1 << PB4), PORTB);
		BCLR((1 << COM1B0), TCCR1A);
		BSET((1 << COM1B1), TCCR1A);

		BSET(FLG_24V_BATT_CRIT | FLG_24V_BATT_LOW, statusI2C);
	}
	else if (reading_24V < LOW_THRESHOLD_24V) /*If 24V system is low*/
	{
		BCLR((1 << PB4), PORTB);
		BSET((1 << PB2), PORTB);
		BCLR((1 << COM1B1) | (1 << COM1B0), TCCR1A);

		BSET(FLG_24V_BATT_LOW, statusI2C);
		BCLR(FLG_24V_BATT_CRIT, statusI2C);
	}
	else /*If 24V system is fine*/
	{
		BSET((1 << PB4), PORTB);
		BCLR((1 << PB2), PORTB);
		BCLR((1 << COM1B1) | (1 << COM1B0), TCCR1A);

		BSET(FLG_24V_BATT_CRIT | FLG_24V_BATT_LOW, statusI2C);
	}

}

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
	int result = 0;
	reading_24V = 0;
	reading_12V = 0;
	statusI2C = 0;
	init();

	wdt_reset(); /*feed the watchdog*/

	while (TRUE)
	{
		sleep_cpu(); /*enter sleep mode*/
	}

	return result;
}

/*See function prototype for documentation.*/
void init(void)
{
	/*Initialize watchdog*/
	wdt_enable(WDTO_15MS); /*Time is 15 ms*/

	/*Initialize ports*/
	DDRB = (1 << DDB4) | (1 << DDB3) | (1 << DDB2)| (1 << DDB1);
	DDRD = (1 << DDD0) | (1 << DDD1);

	/*Setup sleep mode*/
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();

	/*Setup I2C*/

	/*Setup ADC*/
	ADMUX = (1 << REFS0) | (1 << ADLAR);
	ADCSRA = (1 << ADEN) | (7 << ADPS0);

	/*Initialize timers*/
	TCNT1 = 0;
	TCCR1A = 0;
	TCCR1B = (2 << WGM12) | (PRESCALER_1 << CS10);
	ICR1 = PWM_TOP;
	OCR1A = DUTY_CYCLE;
	OCR1B = DUTY_CYCLE;
	TIMSK = (1 << OCIE2); /*interrupt on Output Compare 2*/
	TCCR2 = (PRESCALER_1 << CS20) | (1 << WGM21);
	TCNT2 = 0;
	OCR2 = DELAY_TCKS - 1; /*This value is subtracted by one since CTC mode will go for (OCR2 + 1) cycles before starting over*/
	TIFR = 1 << OCF2; /*Clear the time compare bit if set*/

	sei(); /*Enable interrupts*/
}

/**
 * @}
 * */
