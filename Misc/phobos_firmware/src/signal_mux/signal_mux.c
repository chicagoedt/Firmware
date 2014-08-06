/**
 * @defgroup phobosfirmware_component_signal_mux Signal Multiplexer
 * @ingroup phobosfirmware_components
 *
 * @brief The Signal Multiplexer controls which motor control signals are sent
 * to the motorcontrollers.
 *
 * This system chooses whether the robot should stay stationary, move under RC
 * control, or move under autonomous control based on user inputs. These inputs
 * may come either from the Control Panel or buttons on the Signal Multiplexer.
 *
 * This code is designed to run on an Atmel ATmega8 microcontroller running at
 * 16 MHz. Please refer to the Signal Multiplexer requirements and design
 * documents when examining or modifying this code.
 *
 * Pin assignments:
 * - PB0: PWM from receiver gear channel - input
 * - PB2: Safety PWM - output
 * - PB3: MOSI - output
 * - PB4: MISO - input
 * - PB5: SCK - output
 * - PC0: MUX Channel Select A - output
 * - PC1: MUX Channel Select B - output
 * - PC2: RCK on SIPO - output
 * - PC4: SDA
 * - PC5: SCL
 * - PD0: RCK on PISO - output
 *
 * 74HC595 SIP0 pin assignments:
 * - Q_A: Status LEDs for RC signal status, high = signal good, low = no signal
 * - Q_B: Status LED for safety mode selected, high = light on
 * - Q_C: Status LED for RC mode selected, high = light on
 * - Q_D: Status LED for autonomous selected, high = light on
 * - Q_E: Status LED for user select mode selected, high = light on
 * - Q_F: Status LED for robot in safety mode, high = light on
 * - Q_G: Status LED for robot under RC control, high = light on
 * - Q_H: Status LED for robot under autonomous control, high = light on
 *
 * 74HC165 PISO pin assignments:
 * - Q_A: Switch to select safety mode, low = selected
 * - Q_B: Switch to select RC mode, low = selected
 * - Q_C: Switch to select autonomous mode, low = selected
 * - Q_D: Switch to select user select mode, low = selected
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 *
 * @version 2.0
 *
 * @{
 */

/*Include definition files*/
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <stdint.h>

/**
 * @brief I2C address of this module.
 *
 */
#define I2C_ADDRESS 0x19

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
 * program loop frequency * Timer2 prescaler * NUM_8BIT_CYCLES).
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
 #define CLK_TCKS_2 125

/**
 * @brief Number of program loop iterations before timing out.
 *
 * This constant is used to determine how often a PWM signal check timeout
 * occurs. This method is needed because the timeout period is independent of
 * the PMW and Timer2 does not have the range for such frequencies. The 16 bit timer
 * is being used for PWM detection and thus is not available for this purpose.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define NUM_8BIT_CYCLES 25

/**
 * @brief Constant to set 16 bit Timer1 prescaler.
 *
 * See page 100 of the ATmega8 data sheet for valid values.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define PRESCALER_1 2

/**
 * @brief Constant to set 8 bit Timer2 prescaler.
 *
 * See page 118 of the ATmega8 data sheet for valid values.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define PRESCALER_2 5

/**
 * @brief Minimum number of clock cycles for low PWM signal.
 *
 * This constant represent the lowest number of timer cycles that a PWM pulse
 * width can be and still register as a low signal.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define LOW_SIG_THRESHOLD_L 1600

/**
 * @brief Maximum number of clock cycles for low PWM signal.
 *
 * This constant represent the largest number of timer cycles that a PWM pulse
 * width can be and still register as a low signal.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define LOW_SIG_THRESHOLD_U 2800

/**
 * @brief Minimum number of clock cycles for high PWM signal.
 *
 * This constant represent the lowest number of timer cycles that a PWM pulse
 * width can be and still register as a high signal.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define HIGH_SIG_THRESHOLD_L 3200

/**
 * @brief Maximum number of clock cycles for high PWM signal.
 *
 * This constant represent the largest number of timer cycles that a PWM pulse
 * width can be and still register as a high signal.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define HIGH_SIG_THRESHOLD_U 4400

/**
 * @brief Constant used to determine neutral PWM signal high time.
 *
 * This is the value that should be added to OCR1B to create the high part of
 * the PWM signal that puts the motor controllers in a neutral state.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define TIME_HIGH 3000

/**
 * @brief Constant used to determine neutral PWM signal low time.
 *
 * This is the value that should be added to OCR1B to create the low part of
 * the PWM signal that puts the motor controllers in a neutral state.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define TIME_LOW 37000

/**
 * @brief Bit flag for position of the safety mode switch on the PISO.
 *
 * This flag is to be used with the #switchesPISO variable. If the bit
 * corresponding to this flag is cleared in that variable, it means the user
 * has pressed the button for safety mode.
 */
#define FLG_SAFETY_SWITCH _BV(0)

/**
 * @brief Bit flag for position of the RC mode switch on the PISO.
 *
 * This flag is to be used with the #switchesPISO variable. If the bit
 * corresponding to this flag is cleared in that variable, it means the user
 * has pressed the button for RC mode.
 */
#define FLG_RC_SWITCH _BV(1)

/**
 * @brief Bit flag for position of the autonomous mode switch on the PISO.
 *
 * This flag is to be used with the #switchesPISO variable. If the bit
 * corresponding to this flag is cleared in that variable, it means the user
 * has pressed the button for autonomous mode.
 */
#define FLG_AUTONOMOUS_SWITCH _BV(2)

/**
 * @brief Bit flag for position of the user select mode switch on the PISO.
 *
 * This flag is to be used with the #switchesPISO variable. If the bit
 * corresponding to this flag is cleared in that variable, it means the user
 * has pressed the button for user select mode.
 */
#define FLG_REMOTE_SEL_SWITCH _BV(3)

/**
 * @brief Bit flag for position of the RC connection LEDs on the SIPO.
 *
 * This flag is to be used with the #lightsSIPO variable. If the bit
 * corresponding to this flag is set in that variable, it means there is a
 * good RC connection or a bad connection is cleared.
 */
#define FLG_LOST_CONN_LED _BV(0)

/**
 * @brief Bit flag for position of the safety mode LED on the SIPO.
 *
 * This flag is to be used with the #lightsSIPO variable. If the bit
 * corresponding to this flag is set in that variable, it means the user
 * selected safety mode.
 */
#define FLG_WANT_SAFETY_LED _BV(1)

/**
 * @brief Bit flag for position of the RC mode LED on the SIPO.
 *
 * This flag is to be used with the #lightsSIPO variable. If the bit
 * corresponding to this flag is set in that variable, it means the user
 * selected RC mode.
 */
#define FLG_WANT_RC_LED _BV(2)

/**
 * @brief Bit flag for position of the autonomous mode LED on the SIPO.
 *
 * This flag is to be used with the #lightsSIPO variable. If the bit
 * corresponding to this flag is set in that variable, it means the user
 * selected autonomous mode.
 */
#define FLG_WANT_AUTONOMOUS_LED _BV(3)

/**
 * @brief Bit flag for position of the user select mode LED on the SIPO.
 *
 * This flag is to be used with the #lightsSIPO variable. If the bit
 * corresponding to this flag is set in that variable, it means the user
 * selected user select mode.
 */
#define FLG_WANT_REMOTE_SEL_LED _BV(4)

/**
 * @brief Bit flag for position of the robot under safety control LED on the SIPO.
 *
 * This flag is to be used with the #lightsSIPO variable. If the bit
 * corresponding to this flag is set in that variable, it means the robot
 * in under the control of safety mode.
 */
#define FLG_USING_SAFETY_LED _BV(5)

/**
 * @brief Bit flag for position of the robot under RC control LED on the SIPO.
 *
 * This flag is to be used with the #lightsSIPO variable. If the bit
 * corresponding to this flag is set in that variable, it means the robot
 * in under RC control.
 */
#define FLG_USING_RC_LED _BV(6)

/**
 * @brief Bit flag for position of the robot under autonomous control LED on the SIPO.
 *
 * This flag is to be used with the #lightsSIPO variable. If the bit
 * corresponding to this flag is set in that variable, it means the robot
 * in under autonomous control.
 */
#define FLG_USING_AUTONOMOUS_LED _BV(7)


/**
 * @brief Boolean logic data type.
 *
 * This type should only take on one of two values: TRUE or FALSE.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
typedef unsigned char BOOL_T;

/**
 * @brief List of values that a input PWM signal can represent.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
typedef enum
{
	/** The PWM signal has a duty cycle between 8% and 11%.*/
	SIGNAL_HIGH,
	/** The PWM signal has a duty cycle between 4% and 7%.*/
	SIGNAL_LOW,
	/** The PWM signal does not belong to any other category.*/
	SIGNAL_UNDEFINED
} SIGNAL_VALUE;

/**
 * @brief A list of program states.
 *
 * The program can either claim to have not have a signal, to have a signal,
 * or be waiting to see if a signal is present.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
typedef enum
{
	/** A valid PWM signal is not present at the Input Capture pin.*/
	NO_SIGNAL,
	/** A valid PWM signal is present at the Input Capture pin.*/
	YES_SIGNAL, 
	/** A valid PWM signal may be present at the Input Capture pin and is currently being tested for validity.*/
	POTENTIAL_SIGNAL
} SIGNAL_STATE;

/**
 * @brief A list of signal source selections.
 *
 * The program user can either direct the robot to operate autonomously, via
 * remote control, in safety mode, or have its mode selected with the RC
 * controller.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
typedef enum {
	/** The robot should be controlled autonomously.*/
	SRC_AUTO,
	/** The robot should be controled via remote control.*/
	SRC_RC,
	/** The signal source should be chosen via remote control.*/
	SRC_SELECT,
	/** The robot should be in safety mode and not move.*/
	SRC_SAFETY,
	/** The user has not made a mode selection.*/
	SRC_NONE
} SIGNAL_SRC;

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
 * @brief Value of the present PWM signal.
 *
 * This variable holds the value of the current PWM signal if one is present.
 * If no valid PWM signal is present, the contents of this variable is
 * meaningless.
 *
 * This variable is volatile because it is modified by interrupts.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static volatile SIGNAL_VALUE pwmValue;

/**
 * @brief Status of the PWM signal.
 *
 * This variable contains the state of the PWM signal by stating if a valid
 * signal is present or not, or if the signal is currently being validated.
 *
 * This variable is volatile because it is modified by interrupts.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static volatile SIGNAL_STATE signalState;

/**
 * @brief Variable that counts the number of loop iterations in the program.
 *
 * This variable is used to count how many program iterations have passed in
 * order to precisely determine the timeout interval. When this variable
 * equals #NUM_8BIT_CYCLES, a timeout occurs.
 *
 * This variable is volatile because it is modified by interrupts.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static volatile unsigned char loopIterations;

/**
 * @brief The time of the rising edge of the PWM.
 *
 * This variable is set at the rising edge of the PWM pulse. The difference
 * between this variable and #fallingEdge is the pulse width.
 *
 * This variable is volatile because it is modified by interrupts.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static volatile uint16_t risingEdge;

/**
 * @brief The time of the falling edge of the PWM.
 *
 * This variable is set at the falling edge of the PWM pulse. The difference
 * between this variable and #risingEdge is the pulse width.
 *
 * This variable is volatile because it is modified by interrupts.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static volatile uint16_t fallingEdge;

/**
 * @brief The current system mode of operation.
 *
 * This variable holds the style of signal selection to be used.
 *
 * This variable is volatile because it is modified by interrupts. Only the following are valid modes:
 * - SRC_AUTO,
 * - SRC_RC,
 * - SRC_SELECT,
 * - SRC_SAFETY,
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static volatile SIGNAL_SRC mode;

/**
 * @brief Stores the state of the indicator LEDs.
 *
 * This variable contains the bit pattern that is to be sent to the SIPO. It
 * contains information on which lights should be on or off.
 *
 * Data format:
 * - Bit 0: Status LEDs for RC signal status, high = signal good, low = no signal
 * - Bit 1: Status LED for safety mode selected, high = light on
 * - Bit 2: Status LED for RC mode selected, high = light on
 * - Bit 3: Status LED for autonomous selected, high = light on
 * - Bit 4: Status LED for user select mode selected, high = light on
 * - Bit 5: Status LED for robot in safety mode, high = light on
 * - Bit 6: Status LED for robot under RC control, high = light on
 * - Bit 7: Status LED for robot under autonomous control, high = light on
 *
 * This variable is volatile because it is modified by interrupts.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static volatile unsigned char lightsSIPO;

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
 * @brief Stores the state of the input switches from the CP.
 *
 * Data format:
 * - Bit 1:0: Safety, RC, Autonomous, USR select
 *
 * This variable is volatile because it is modified by interrupts.
 *
 * @author Marios Fanourakis aristogenes@gmail.com
 */
static volatile unsigned char switchesCP;

/**
 * @brief Stores the information that is to be sent to the CP via I2C.
 * 
 * This variable should be sent when the I2C bus requests data. The format of
 * this variable is:
 * - Bit 0: Set if motors are powered or cleared if power is cut.
 * - Bits 2:1: Represents user selection status
 *   - 00: Safety mode
 *   - 01: RC control
 *   - 10: Autonomous control
 *   - 11: User Select mode 
 * - Bits 5:4: Represents current robot movement signal source
 *   - 00: Safety mode
 *   - 01: RC control
 *   - 10: Autonomous control 
 *
 * This variable is volatile because it is modified by interrupts.
*/
static volatile unsigned char statusI2C;

/**
 * @brief Sets the select lines on the multiplexer.
 *
 * @param src The signal that is to be sent to the motor controllers. Only the following are valid sources:
 * - SRC_AUTO,
 * - SRC_RC,
 * - SRC_SAFETY,
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void setMUX(SIGNAL_SRC src);

/**
 * @brief Sets the system mode based on user input.
 *
 * @param new_mode The mode that the system should be set to. Only the following are valid modes:
 * - SRC_AUTO
 * - SRC_RC
 * - SRC_SAFETY
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void setDesiredMode(SIGNAL_SRC new_mode);

/**
 * @brief Resets the timer for the signal timeout.
 *
 * This function resets the timeout to allow a new attempt to read a PWM
 * signal without having a timeout occur in the middle of a PWM pulse.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void resetTimer();

/**
 * @brief This function initializes the microcontroller.
 *
 * The 16 bit timer, the 8 bit timer, the outputs pins, interrupts, and the
 * watchdog timer of the microcontroller are initialized. This should be the
 * first function called in the program.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void init(void);

/**
 * @brief This function determines the signal value based on the pulse width,
 *
 * Whenever a PWM signal is detected, its pulse width is measured. The pulse
 * width determines whether the signal is low or high. If the pulse width does
 * not cleanly fall into the high or low categories, it is classified as
 * undefined.
 *
 * @param width The pulse width in Timer1 ticks.
 *
 * @return The PWM signal value based on the pulse width.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static SIGNAL_VALUE getValue(uint16_t width);

/**
 * @brief Interrupt handler for Input Capture event.
 *
 * This interrupt handler checks whether the PWM signal is valid or not by
 * measuring the signal's pulse width.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
ISR(TIMER1_CAPT_vect)
{
	if (TCCR1B & (1 << ICES1)) /*if looking for rising edge*/
	{
		risingEdge = ICR1; /*get time of rising edge*/
		BCLR(1 << ICES1, TCCR1B); /*set to interrupt on falling edge*/
		TIFR = 1 << ICF1;		
		resetTimer();
		if (signalState != YES_SIGNAL)
		{
			signalState = POTENTIAL_SIGNAL;
		}
	}
	else /*if looking for falling edge*/
	{
		fallingEdge = ICR1; /*get time of falling edge*/
		BSET(1 << ICES1, TCCR1B); /*set to interrupt on rising edge*/
		TIFR = 1 << ICF1;
		signalState = YES_SIGNAL; /*signal was detected*/
		pwmValue = getValue(fallingEdge - risingEdge); /*determine nature of signal*/
		resetTimer();
	}
}

/**
 * @brief Interrupt handler for Timer1 Compare B event.
 *
 * This handler generates a 50 Hz PWM signal with duty cycle of 7.5%. This is
 * to be used as the safety signal for the system since the motor will not move
 * when this signal is applied.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
ISR(TIMER1_COMPB_vect)
{
	if (TCCR1A & (1 << COM1B0)) /*if we just went high*/
	{
		OCR1B = OCR1B + TIME_HIGH; /*set time of next interrupt*/
		BCLR(1 << COM1B0, TCCR1A); /*go low on next interrupt*/

	}
	else /*if we just went low*/
	{
		OCR1B = OCR1B + TIME_LOW; /*set time of next interrupt*/
		BSET(1 << COM1B0, TCCR1A); /*go high on next interrupt*/

	}
}

/**
 * @brief Interrupt handler for Timer2 Compare event.
 *
 * This handler is responsible for the main program loop. Whenever the timer2
 * compare event occurs, the next iteration of the program is run.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
ISR(TIMER2_COMP_vect)
{
	wdt_reset(); /*feed the watchdog*/

	switchesCP = 0x0F & (PINC >> 4);

	/*Fire clock on PISO*/
	BCLR(1 << PD0, PORTD);
	BSET(1 << PD0, PORTD);

	/*This block writes to the SIPO from SPI*/
	if ((SPSR & (1 << SPIF)) == 0)
	{
		SPDR = lightsSIPO;
	}

	/*This block reads in the value of PISO from SPI*/
	while ((SPSR & (1 << SPIF)) == 0)
	{
	}
	switchesPISO = SPDR;

	/*Fire clock on SIPO*/
	BSET(1 << PC2, PORTC);
	BCLR(1 << PC2, PORTC);

	if (switchesPISO == 0x0F)
	{
		if (switchesCP == 0x00)
		{
			switchesPISO = 0x0E;
		}
		else if (switchesCP == 0x01)
		{
			switchesPISO = 0x0D;
		}
		else if (switchesCP == 0x02)
		{
			switchesPISO = 0x0B;
		}
		else if (switchesCP == 0x03)
		{
			switchesPISO = 0x07;
		}
	}

	/*This block analyzes the switches on the PISO*/
	if (!(switchesPISO & FLG_SAFETY_SWITCH))
	{
		setDesiredMode(SRC_SAFETY);
	}
	else if (!(switchesPISO & FLG_RC_SWITCH))
	{
		setDesiredMode(SRC_RC);
	}
	else if (!(switchesPISO & FLG_AUTONOMOUS_SWITCH))
	{
		setDesiredMode(SRC_AUTO);
	}
	else if (!(switchesPISO & FLG_REMOTE_SEL_SWITCH))
	{
		setDesiredMode(SRC_SELECT);
	}

	/*This block sets the connection status LED*/
	if (signalState == YES_SIGNAL)
	{
		BSET(FLG_LOST_CONN_LED, lightsSIPO); /*turn on signal LED*/
		BSET((1 << 0), statusI2C);
	}
	else
	{
		BCLR(FLG_LOST_CONN_LED, lightsSIPO); /*turn off signal LED*/
		BCLR((1 << 0), statusI2C);
	}

	switch (mode)
	{
		case SRC_AUTO: /*put robot under autonomous control*/
			setMUX(SRC_AUTO);
			BCLR(FLG_USING_RC_LED | FLG_USING_SAFETY_LED, lightsSIPO);
			BSET(FLG_USING_AUTONOMOUS_LED, lightsSIPO);
			break;
		case SRC_RC:
			if (signalState == YES_SIGNAL) /*put robot under RC control*/
			{
				setMUX(SRC_RC);
				BCLR(FLG_USING_AUTONOMOUS_LED | FLG_USING_SAFETY_LED, lightsSIPO);
				BSET(FLG_USING_RC_LED, lightsSIPO);
			}
			else /*put robot under safety control*/
			{
				setMUX(SRC_SAFETY);
				BCLR(FLG_USING_AUTONOMOUS_LED | FLG_USING_RC_LED, lightsSIPO);
				BSET(FLG_USING_SAFETY_LED, lightsSIPO);
			}
			break;
		case SRC_SELECT:
			if (signalState == YES_SIGNAL)
			{
				switch (pwmValue)
				{
					case SIGNAL_HIGH: /*put robot under RC control*/
						setMUX(SRC_RC);
						BCLR(FLG_USING_AUTONOMOUS_LED | FLG_USING_SAFETY_LED, lightsSIPO);
						BSET(FLG_USING_RC_LED, lightsSIPO);
						break;
					case SIGNAL_LOW: /*put robot under autonomous control*/
						setMUX(SRC_AUTO);
						BCLR(FLG_USING_RC_LED | FLG_USING_SAFETY_LED, lightsSIPO);
						BSET(FLG_USING_AUTONOMOUS_LED, lightsSIPO);
						break;
					default: /*put robot under safety control*/
						setMUX(SRC_SAFETY);
						BCLR(FLG_USING_AUTONOMOUS_LED | FLG_USING_RC_LED, lightsSIPO);
						BSET(FLG_USING_SAFETY_LED, lightsSIPO);
						break;
				}
			}
			else /*put robot under safety control*/
			{
				setMUX(SRC_SAFETY);
				BCLR(FLG_USING_AUTONOMOUS_LED | FLG_USING_RC_LED, lightsSIPO);
				BSET(FLG_USING_SAFETY_LED, lightsSIPO);
			}
			break;
		default: /*put robot under safety control*/
			setMUX(SRC_SAFETY);
			BCLR(FLG_USING_AUTONOMOUS_LED | FLG_USING_RC_LED, lightsSIPO);
			BSET(FLG_USING_SAFETY_LED, lightsSIPO);
			break;
	}

	++loopIterations;

	/*if a time out occurs due to no valid RC PWM signal*/
	if (loopIterations >= NUM_8BIT_CYCLES)
	{
		resetTimer(); /*reset the timer to try again to find a signal*/
		signalState = NO_SIGNAL; /*No signal present if timed out.*/
		BSET(1 << ICES1, TCCR1B); /*look for rising edge*/
		TIFR = 1 << ICF1;
	}
}

ISR(TWI_vect)
{
}

/**
 * @brief This function is the program execution starting point.
 * 
 * This function initializes the uC and sets up the program loop.
 *
 * @return Returns zero upon program termination.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com 
 */
int main()
{
	int result;
	result = 0;
	init();
	while (TRUE)
	{
		sleep_cpu(); /*enter sleep mode*/
	}
	return result;
}

/*See function prototype for documentation.*/
void setDesiredMode(SIGNAL_SRC new_mode)
{
	switch (new_mode)
	{
		case SRC_SAFETY:
			BSET(FLG_WANT_SAFETY_LED, lightsSIPO);
			BCLR(FLG_WANT_RC_LED | FLG_WANT_AUTONOMOUS_LED | FLG_WANT_REMOTE_SEL_LED, lightsSIPO);
			mode = SRC_SAFETY;
			BCLR((1 << 2) | (1 << 1), statusI2C);
			break;
		case SRC_RC:
			BSET(FLG_WANT_RC_LED, lightsSIPO);
			BCLR(FLG_WANT_SAFETY_LED | FLG_WANT_AUTONOMOUS_LED | FLG_WANT_REMOTE_SEL_LED, lightsSIPO);
			mode = SRC_RC;
			BCLR((1 << 2), statusI2C);
			BSET((1 << 1), statusI2C);
			break;
		case SRC_AUTO:
			BSET(FLG_WANT_AUTONOMOUS_LED, lightsSIPO);
			BCLR(FLG_WANT_RC_LED | FLG_WANT_SAFETY_LED | FLG_WANT_REMOTE_SEL_LED, lightsSIPO);
			mode = SRC_AUTO;
			BCLR((1 << 1), statusI2C);
			BSET((1 << 2), statusI2C);
			break;
		case SRC_SELECT:
			BSET(FLG_WANT_REMOTE_SEL_LED, lightsSIPO);
			BCLR(FLG_WANT_RC_LED | FLG_WANT_AUTONOMOUS_LED | FLG_WANT_SAFETY_LED, lightsSIPO);
			mode = SRC_SELECT;
			BSET((1 << 2) | (1 << 1), statusI2C);
			break;
		default:
			break;
	}
}

/*See function prototype for documentation.*/
void resetTimer(void)
{
	loopIterations = 0;
}

/*See function prototype for documentation.*/
void init(void)
{
	wdt_enable(WDTO_15MS); /*Initialize watchdog, timeout is 15 ms*/

	pwmValue = SIGNAL_UNDEFINED;
	signalState = NO_SIGNAL;
	loopIterations = 0;
	statusI2C = 0;
	setDesiredMode(SRC_SAFETY);

	/*Initialize ports*/
	DDRB = (1 << DDB5) | (1 << DDB3) | (1 << DDB2);
	DDRC = (0 << DDC5) | (0 << DDC4) | (1 << DDC2) | (1 << DDC1) | (1 << DDC0);
	DDRD = (1 << DDD0);

	BSET(1 << PD0, PORTD); /*PISO loads data when RCK low so we have to normally keep this pin high*/

	/*Initialize timers 1 and 2*/
	TCCR1A = (2 << COM1B0);
	TCCR1B = (1 << ICNC1) | (1 << ICES1) | (PRESCALER_1 << CS10);
	OCR1B = TIME_HIGH + TCNT1;
	TIMSK = (1 << TICIE1) | (1 << OCIE1B) | (1 << OCIE2); /*Enable input capture interrupts*/

	TCCR2 = (PRESCALER_2 << CS20) | (1 << WGM21);
	OCR2 = CLK_TCKS_2 - 1; /*This value is subtracted by one since CTC mode will go for (OCR2 + 1) cycles before starting over*/
	TIFR = 1 << OCF2; /*Clear the time compare bit if set*/

	/*Setup SPI*/
	SPCR = (1 << SPE) | (1 << MSTR);

	/*Setup I2C*/

	/*Setup sleep mode*/
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();

	sei(); /* set global interrupt enable */
}

/*See function prototype for documentation.*/
static SIGNAL_VALUE getValue(uint16_t width)
{
	SIGNAL_VALUE result = SIGNAL_UNDEFINED;
	if (width < LOW_SIG_THRESHOLD_U && width > LOW_SIG_THRESHOLD_L)
	{
		result = SIGNAL_LOW;
	}
	else if (width < HIGH_SIG_THRESHOLD_U && width > HIGH_SIG_THRESHOLD_L)
	{
		result = SIGNAL_HIGH;
	}
	return result;
}

/*See function prototype for documentation.*/
static void setMUX(SIGNAL_SRC src)
{
	switch (src)
	{
		case SRC_RC:
			BSET((1 << PC1) | (1 << PC0), PORTC);
			BCLR((1 << 5), statusI2C);
			BSET((1 << 4), statusI2C);
			break;
		case SRC_AUTO:
			BCLR((1 << PC0), PORTC);
			BSET((1 << PC1), PORTC);
			BCLR((1 << 4), statusI2C);
			BSET((1 << 5), statusI2C);
			break;
		default:
			BCLR((1 << PC1) | (1 << PC0), PORTC);
			BCLR((1 << 4) | (1 << 5), statusI2C);
			break;
	}
}

/**
 * @}
 * */
