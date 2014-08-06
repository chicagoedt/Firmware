/**
 * @defgroup phobosfirmware_component_drivetrain Embedded Drivetrain Control System
 * @ingroup phobosfirmware_components
 *
 * @brief The Embedded Drivetrain Control System controls the motors and
 * reports odometry from wheel encoders. 
 *
 * This module, for velocity measurement, counts the number of encoder ticks
 * for a period of time and then sends this tick count to the computers when
 * asked for. For position data, the module counts the amount of ticks obtained
 * since the position data was last queried and this is sent to the computer
 * when requested. In addition, based on commands from the computer, this
 * module generates a PWM to control a motor.
 *
 * This code is designed to run on an Atmel ATmega8 microcontroller running at
 * 16 MHz. Please refer to the Embedded Drivetrain Control System requirements
 * and design documents when examining or modifying this code.
 *
 * Pin assignments:
 * - PB0: ICP1, encoder channel A - input
 * - PB1: Encoder channel B - input
 * - PB2: OC1B, PWM for motorcontroller - output
 * - PC0: Switch for selecting side - input, high = left, low = right
 * - PC1: Switch for selecting encode direction as positive - input, high = leading, low = lagging
 * - PC2: Switch for selecting motor signal inversion - input, high = invert, low = normal
 * - PC4: SDA
 * - PC5: SCL
 * - PD0: RxD - input
 * - PD1: TxD - output
 * - PD2: Status LEDs for left or right side - output, high = left, low = right
 * - PD3: Status LEDs for lagging or leading encoder signals as positive - output, high = leading, low = lagging
 * - PD4: Status LEDs for normal or inverted motor control - output, high = invert, low = normal
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
#define PRESCALER_1 2

/**
 * @brief Constant to set 8 bit Timer2 prescaler.
 *
 * See page 118 of the ATmega8 data sheet for valid values.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define PRESCALER_2 6

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
 * @brief Value of the maximum number of allowed velocity forward ticks.
 *
 * If the number of ticks counted is greater than this value, this value will
 * be sent to the computer to indicate an overflow condition in the forward
 * direction.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define MAX_FWD_VEL_TCKS 8191

/**
 * @brief Value of the maximum number of allowed velocity reverse ticks.
 *
 * If the number of ticks counted is less than this value, this value will
 * be sent to the computer to indicate an overflow condition in the reverse
 * direction.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define MAX_REV_VEL_TCKS -8192

/**
 * @brief Value of the maximum number of allowed position forward ticks.
 *
 * If the number of ticks counted is greater than this value, this value will
 * be sent to the computer to indicate an overflow condition in the forward
 * direction.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define MAX_FWD_POS_TCKS 134217727

/**
 * @brief Value of the maximum number of allowed position reverse ticks.
 *
 * If the number of ticks counted is less than this value, this value will
 * be sent to the computer to indicate an overflow condition in the reverse
 * direction.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define MAX_REV_POS_TCKS -134217728


/**
 * @brief I2C address of this module if it is attached to the left wheel.
 *
 */
#define LEFT_I2C_ADDR 0x16

/**
 * @brief I2C address of this module if it is attached to the right wheel.
 *
 */
#define RIGHT_I2C_ADDR 0x17

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

/**
 * @brief Command to query the velocity tick count.
 *
 * When this command is received through the USART interface, the tick count of
 * the last complete period will be returned as a 14 bit signed integer.
 */
#define CMD_QUERY_VEL_TICKS 0xC5

/**
 * @brief Command to query the position tick count.
 *
 * When this command is received through the USART interface, the position tick count
 * will be returned as a 28 bit signed integer and then zeroed.
 */
#define CMD_QUERY_POS_TICKS 0xC3

/**
 * @brief Command to query the motor power setting.
 *
 * When this command is received through the USART interface, the motor power
 * setting will be returned as an 8 bit signed integer.
 */
#define CMD_QUERY_POWER 0xC7

/**
 * @brief Command to set the motor power setting.
 *
 * When this command is received through the USART interface, the motor power
 * will be changed to the new value sent by the computer.
 */
#define CMD_SET_POWER 0xD3


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
	/** The system is reporting the velocity tick count to the computer.*/
	COMM_MODE_QUERY_VEL_TICKS,
	/** The system is reporting the position tick count to the computer.*/
	COMM_MODE_QUERY_POS_TICKS,
	/** The system is reporting the motor power level to the computer.*/
	COMM_MODE_QUERY_POWER,
	/** The system is setting the motor power level to the computer's command*/
	COMM_MODE_SET_POWER,
	/** The system has received an invalid command, data or has timed out.*/
	COMM_MODE_ERROR
} COMM_MODE;

/**
 * @brief List of system states while setting the motor power level.
 *
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
typedef enum
{
	/** The system just received the command to set motor power level.*/
	STATE_SET_POWER_1,
	/** The system just received the amount of power to set the motors to.*/
	STATE_SET_POWER_2,
} STATE_SET_POWER;

/**
 * @brief List of system states while reporting the velocity tick count.
 *
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
typedef enum
{
	/** The system just transmitted the status byte to the computer.*/
	STATE_QRY_VEL_TICKS_1,
	/** The system just transmitted the first data byte to the computer.*/
	STATE_QRY_VEL_TICKS_2,
} STATE_QRY_VEL_TICKS;

/**
 * @brief List of system states while reporting the position tick count.
 *
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
typedef enum
{
	/** The system just transmitted the status byte to the computer.*/
	STATE_QRY_POS_TICKS_1,
	/** The system just transmitted the first data byte to the computer.*/
	STATE_QRY_POS_TICKS_2,
	/** The system just transmitted the second byte to the computer.*/
	STATE_QRY_POS_TICKS_3,
	/** The system just transmitted the third data byte to the computer.*/
	STATE_QRY_POS_TICKS_4,
} STATE_QRY_POS_TICKS;

/**
 * @brief List of system states while reporting the motor power level.
 *
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
typedef enum
{
	/** The system just transmitted the status byte to the computer.*/
	STATE_QRY_POWER_1,
	/** The system just transmitted the first data byte to the computer.*/
	STATE_QRY_POWER_2,
} STATE_QRY_POWER;

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
 * @brief This variable tells where the PWM output is normal or inverted.
 *
 * Inverted operation means that if the computer is asking for power to be
 * given in a certain direction with a certain magnitude, the power actually
 * given will have the same magnitude but opposite direction.
 */
static BOOL_T isPowerInverted;

/**
 * @brief This variable maintains the current internal communication state.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile COMM_MODE commMode;

/**
 * @brief This variable holds the current state while setting the power level.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile STATE_SET_POWER stateSetPower;

/**
 * @brief This variable holds the current state while reporting the velocity tick count.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile STATE_QRY_VEL_TICKS stateQryVelTicks;

/**
 * @brief This variable holds the current state while reporting the position tick count.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile STATE_QRY_VEL_TICKS stateQryPosTicks;

/**
 * @brief This variable holds the current state while reporting power level.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile STATE_QRY_POWER stateQryPower;

/**
 * @brief Number of encoder ticks for velocity measurement in the current counting period.
 *
 * The function that counts wheel encoder ticks updates this variable when
 * a tick is detected. This variable is incremented if the wheel is moving
 * forward and decremented if the wheel is moving backwards. At regular
 * intervals, this variable is saved into #prevPeriodTicks for sending to
 * the computer. It is then zeroed to start measuring again.
 *
 * This variable is volatile because it is modified by interrupts.
 *
 */
static volatile int16_t currentPeriodVelTicks;

/**
 * @brief The velocity tick count of the previous velocity period.
 *
 * This variable is updated at the end of every period with #currentPeriodTicks
 *  and is sent to the computers whenever the tick count is queried.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile int16_t prevPeriodVelTicks;

/**
 * @brief Copy of the previous period velocity tick count for RS-232 data transmission purposes.
 *
 * This variable is updated to #prevPeriodVelTicks whenever the computer wants
 * to query the tick count. This is done because #prevVelPeriodTicks might be
 * changed in the time between the two data byte transmissions, leading to
 * data corruption.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile int16_t velTicksRS232;

/**
 * @brief Number of encoder ticks for position measurement in the current counting period.
 *
 * The function that counts wheel encoder ticks updates this variable when
 * a tick is detected. This variable is incremented if the wheel is moving
 * forward and decremented if the wheel is moving backwards. When queried for
 * the position tick count, this is sent to the PC.
 * The count is then zeroed to start a new counting period.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile int32_t posTicks;

/**
 * @brief Copy of the previous period position tick count for RS-232 data transmission purposes.
 *
 * This variable is updated to #posTicks whenever the computer wants
 * to query the tick count. This is done because #posTics is reset to zero
 * following the data request and a data copy is needed to send to the PC.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile int32_t posTicksRS232;

/**
 * @brief This variable increases the maximum period of Timer2.
 *
 * Whenever Timer2 compare occurs, this variable gets incremented or reset to
 * zero so that longer periods of time can be controlled by Timer 2.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile unsigned char counter8bit;

/**
 * @brief Temporary variable to receive first byte of new motor power setting.
 *
 * This variable is combined with bitwise operations with the second received
 * byte to yield the actual new motor power setting.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile unsigned char tempPwrSetting;

/**
 * @brief This variable contains the current motor power setting.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile unsigned char powerSetting;

/**
 * @brief This variable contains the system's I2C address.
 *
 * This value is dependent on which side of the robot this system is
 * responsible for.
 */
static unsigned char address;

/**
 * @brief Array of constants to set PWM duty cycles.
 *
 * The entries of this data table determine the number of clock ticks that the
 * PWM signal is to be held high. The duty cycle is equal to (entry in table) /
 * PWM_TOP * 100%. The data is designed to be indexed by a single byte of data
 * in two's compliment format. The lowest duty cycle corresponds to index 128
 * which is -128 in two's compliment format. The highest duty cycle corresponds
 * to index 127, which is +127 in two's compliment.
 */
static const uint16_t dtyCycles[] PROGMEM = {
1500, 1504, 1508, 1512, 1516, 1520, 1524, 1528,
1532, 1536, 1540, 1544, 1548, 1552, 1556, 1560,
1564, 1568, 1572, 1576, 1580, 1584, 1588, 1592,
1596, 1600, 1604, 1608, 1612, 1616, 1620, 1624,
1628, 1632, 1636, 1640, 1644, 1648, 1652, 1656,
1660, 1664, 1668, 1672, 1676, 1680, 1684, 1688,
1692, 1696, 1700, 1704, 1708, 1712, 1716, 1720,
1724, 1728, 1732, 1736, 1740, 1744, 1748, 1752,
1756, 1760, 1764, 1768, 1772, 1776, 1780, 1784,
1788, 1792, 1796, 1800, 1804, 1808, 1812, 1816,
1820, 1824, 1828, 1832, 1836, 1840, 1844, 1848,
1852, 1856, 1860, 1864, 1868, 1872, 1876, 1880,
1884, 1888, 1892, 1896, 1900, 1904, 1908, 1912,
1916, 1920, 1924, 1928, 1932, 1936, 1940, 1944,
1948, 1952, 1956, 1960, 1964, 1968, 1972, 1976,
1980, 1984, 1988, 1992, 1996, 2000, 2000, 2000,
1000, 1000, 1000, 1000, 1004, 1008, 1012, 1016,
1020, 1024, 1028, 1032, 1036, 1040, 1044, 1048,
1052, 1056, 1060, 1064, 1068, 1072, 1076, 1080,
1084, 1088, 1092, 1096, 1100, 1104, 1108, 1112,
1116, 1120, 1124, 1128, 1132, 1136, 1140, 1144,
1148, 1152, 1156, 1160, 1164, 1168, 1172, 1176,
1180, 1184, 1188, 1192, 1196, 1200, 1204, 1208,
1212, 1216, 1220, 1224, 1228, 1232, 1236, 1240,
1244, 1248, 1252, 1256, 1260, 1264, 1268, 1272,
1276, 1280, 1284, 1288, 1292, 1296, 1300, 1304,
1308, 1312, 1316, 1320, 1324, 1328, 1332, 1336,
1340, 1344, 1348, 1352, 1356, 1360, 1364, 1368,
1372, 1376, 1380, 1384, 1388, 1392, 1396, 1400,
1404, 1408, 1412, 1416, 1420, 1424, 1428, 1432,
1436, 1440, 1444, 1448, 1452, 1456, 1460, 1464,
1468, 1472, 1476, 1480, 1484, 1488, 1492, 1496
};

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
 * @brief This function reports the wheel encoder tick count to the Phobos
 * controls system,
 *
 * A tick count in the form of a two's compliment 14 bit signed number packaged
 * in 2 bytes is sent to the Phobos controls system using the R232 serial
 * format. The tick count will be split into two parts with the upper 7 data
 * bits in one group and the lower 7 data bits in the second group. The first
 * byte sent will have the 7 higher data bits in bits 0 through 6 and have bit
 * 7 set to one. The second byte sent will have the 7 lower data bits in bits
 * 0 through 6 and have bit 7 set to zero. Positive values of the count
 * indicate forward motion and negative values indicate reverse motion. This
 * function detects overflow and returns either #MAX_FWD_TCKS if the overflow
 * occurred when the wheel was moving forward or #MAX_REV_TCKS if the overflow
 * occurred when the wheel was moving backwards.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void recordTicks();

/**
 * @brief This function sets the motor power level.
 *
 * The output PWM of OCR1B tells the motorcontroller how much power to send to
 * the motor. This function modifies the PWM to change the motor power,
 *
 * @param power The power level to supply to the motor. Positive values
 * indicate that the motor should spin forwards and negative values indicate
 * that the motor should spin in reverse. The magnitude of the paramter
 * represents the amount of power to provide the motor.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void updatePWM(signed char power);

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
static BOOL_T isDataByte(unsigned char b);

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
 * @brief Starts a timeout.
 *
 * When a timeout is started, the system has the timeout period to perform
 * further action or the system will return an error to the computer.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void startTimeout();

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
 * @brief Interrupt handler for Timer2 Compare event.
 *
 * Whenever this event happens, the handler determines whether the full tick
 * counting period has elapsed, and if so records the tick count.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
ISR(TIMER2_COMP_vect)
{
	wdt_reset(); /*feed the watchdog*/
	++counter8bit;
	if (counter8bit >= NUM_8BIT_CYCLES) /*check if the counting period has elapsed*/
	{
		counter8bit = 0; /*reset the counter*/
		recordTicks(); /*record the count so it can be sent to the computer*/
	}
}

/**
 * @brief Interrupt handler for Input Capture event.
 *
 * Whenever an Input Capture event happens, this handler will either
 * increment or decrement the tick count based on the state of the quadrature
 * encoder values. This handler takes into account whether leading or lagging
 * quadrature input is considered forward because the correct edge to detect
 * (rising or falling) is selected during initialization..
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
ISR(TIMER1_CAPT_vect)
{
	if (PINB & (1 << PINB1)) /*If wheel is going forward*/
	{
		++currentPeriodVelTicks;
		++posTicks;
	}
	else/*If wheel is going backwards*/
	{
		--currentPeriodVelTicks;
		--posTicks;
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
					case CMD_QUERY_VEL_TICKS: /*if we are responding to a request for velocity tick count*/
						transmitStatusSuccess(2); /*tell the PC that whatever it wanted to do succeeded and 2 more data bytes are coming*/
						commMode = COMM_MODE_QUERY_VEL_TICKS;
						stateQryVelTicks = STATE_QRY_VEL_TICKS_1;
						velTicksRS232 = prevPeriodVelTicks; /*update the tick count for the  RS232 communication*/
						break;
					case CMD_QUERY_POS_TICKS: /*if we are responding to a request for position tick count*/
						transmitStatusSuccess(4); /*tell the PC that whatever it wanted to do succeeded and 4 more data bytes are coming*/
						commMode = COMM_MODE_QUERY_POS_TICKS;
						stateQryPosTicks = STATE_QRY_POS_TICKS_1;
						if (posTicks > MAX_FWD_POS_TCKS) /*check if we overflowed counting forward*/
						{
							posTicksRS232 = MAX_FWD_POS_TCKS; /*update the tick count for the RS232 communication with forward overflow value */
						}
						else if (posTicks < MAX_REV_POS_TCKS) /*check if we overflowed counting backwards*/
						{
							posTicksRS232 = MAX_REV_POS_TCKS; /*update the tick count for the RS232 communication with reverse overflow value */
						}
						else
						{
							posTicksRS232 = posTicks; /*update the tick count for the RS232 communication*/
						}
						posTicks = 0; /*start the new position count period*/
						break;
					case CMD_QUERY_POWER: /*if we are responding to a request for power setting*/
						transmitStatusSuccess(2); /*tell the PC that whatever it wanted to do succeeded and 2 more data bytes are coming*/
						commMode = COMM_MODE_QUERY_POWER;
						stateQryPower = STATE_QRY_POWER_1;
						break;
					case CMD_SET_POWER:/*if we are responding to a change in power setting*/
						commMode = COMM_MODE_SET_POWER;
						stateSetPower = STATE_SET_POWER_1;
						startTimeout();
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
		case COMM_MODE_SET_POWER:
			if (isDataByte(received))
			{
				switch (stateSetPower)
				{
					case STATE_SET_POWER_1:
						postponeTimeout(); /*give some time to get the next data byte*/
						tempPwrSetting = received; /*grab the lower 7 bits of the new power setting*/
						stateSetPower = STATE_SET_POWER_2; /*go to the next stage of setting the power*/
						break;
					case STATE_SET_POWER_2:
						transmitStatusSuccess(0); /*tell the PC that whatever it wanted to do succeeded*/
						tempPwrSetting |= (received << 7); /*combine the 2 received data bytes to make the new setting*/
						powerSetting = tempPwrSetting; /*save the new power setting*/
						updatePWM(powerSetting); /*use the new power setting*/
						commMode = COMM_MODE_IDLE; /*reset the state machine*/
						break;
					default: /*shouldn't ever get here*/
						transmitStatusFail(0); /*tell the PC that whatever it wanted to do failed*/
						commMode = COMM_MODE_IDLE; /*reset the state machine*/
						break;
				}
			}
			else /*shouldn't get a command byte*/
			{
				transmitStatusFail(0); /*tell the PC that whatever it wanted to do failed*/
				commMode = COMM_MODE_IDLE; /*reset the state machine*/
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
		case COMM_MODE_QUERY_VEL_TICKS: /*if we are responding to a request for velocity tick count*/
			switch (stateQryVelTicks)
			{
				case STATE_QRY_VEL_TICKS_1:
					transmitByte((velTicksRS232 >> 7) & 0x7F); /*send the upper part of the tick count, bits 13:7*/
					stateQryVelTicks = STATE_QRY_VEL_TICKS_2; /*go to next stage of sending ticks*/
					break;
				case STATE_QRY_VEL_TICKS_2:
					transmitByte(velTicksRS232 & 0x7F); /*send the lower part of the tick count, bits 6:0*/
					commMode = COMM_MODE_IDLE; /*reset the state machine*/
					break;
				default: /*shouldn't ever get here*/
					commMode = COMM_MODE_IDLE; /*reset the state machine*/
					break;
			}
			break;
		case COMM_MODE_QUERY_POS_TICKS: /*if we are responding to a request for position tick count*/
			switch (stateQryPosTicks)
			{
				case STATE_QRY_POS_TICKS_1:
					transmitByte((posTicksRS232 >> 21) & 0x7F); /*send the upper part of the tick count, bits 27:21*/
					stateQryPosTicks = STATE_QRY_POS_TICKS_2; /*go to next stage of sending ticks*/
					break;
				case STATE_QRY_POS_TICKS_2:
					transmitByte((posTicksRS232 >> 14) & 0x7F); /*send part of the tick count, bits 20:14*/
					stateQryPosTicks = STATE_QRY_POS_TICKS_3; /*go to next stage of sending ticks*/
					break;
				case STATE_QRY_POS_TICKS_3:
					transmitByte((posTicksRS232 >> 7) & 0x7F); /*send part of the tick count, bits 13:7*/
					stateQryPosTicks = STATE_QRY_POS_TICKS_4; /*go to next stage of sending ticks*/
					break;
				case STATE_QRY_POS_TICKS_4:
					transmitByte(posTicksRS232 & 0x7F); /*send the lower part of the tick count, bits 6:0*/
					commMode = COMM_MODE_IDLE; /*reset the state machine*/
					break;
				default: /*shouldn't ever get here*/
					commMode = COMM_MODE_IDLE; /*reset the state machine*/
					break;
			}
			break;
		case COMM_MODE_QUERY_POWER: /*if we are responding to a request for power setting*/
			switch (stateQryPower)
			{
				case STATE_QRY_POWER_1:
					transmitByte((powerSetting >> 7) & 0x1);/*send the higher part of the power setting, bit 7*/
					stateQryPower = STATE_QRY_POWER_2; /*go to the next stage of reporting power setting*/
					break;
				case STATE_QRY_POWER_2:
					transmitByte(powerSetting & 0x7F); /*send the lower part of the power setting, bits 6:0*/
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
	counter8bit = 0;
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
	wdt_enable(WDTO_15MS); /*Initialize watchdog, timeout is 15 ms*/

	/*Initialize ports*/
	DDRD = (1 << DDD2) | (1 << DDD3) | (1 << DDD4);
	DDRB = (1 << DDB2);

	/*initialize variables*/
	currentPeriodVelTicks = 0; 
	prevPeriodVelTicks = 0;
	velTicksRS232 = 0;
	posTicks = 0;
	posTicksRS232 = 0;
	powerSetting = 0;
	commMode = COMM_MODE_IDLE;

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

	/*determine if the motor power should be inverted*/
	if (PINC & (1 << PINC2))
	{
		isPowerInverted = TRUE;
		BSET(1 << PORTD4, PORTD);
	}
	else
	{
		isPowerInverted = FALSE;
		BCLR(1 << PORTD4, PORTD);
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
	TCCR1A = (2 << COM1B0) | (1 << WGM10);
	TCCR1B = (2 << WGM12) | (PRESCALER_1 << CS10);
	OCR1A = PWM_TOP;
	OCR1B = dtyCycles[0];
	TIMSK = (1 << TICIE1) | (1 << OCIE2);

	TCCR2 = (PRESCALER_2 << CS20) | (1 << WGM21);
	OCR2 = CLK_TCKS_2 - 1; /*This value is subtracted by one since CTC mode will go for (OCR2 + 1) cycles before starting over*/
	TCNT2 = 0;
	TIFR = 1 << OCF2; /*Clear the time compare bit if set*/

	/*determine if we want the leading or lagging tick count*/
	if (PINC & (1 << PINC1)) /*if quadrature input leading is positive*/
	{
		BSET(1 << ICES1, TCCR1B);
		BSET(1 << PORTD3, PORTD);
	}
	else /*if quadrature input lagging is positive*/
	{
		BCLR(1 << ICES1, TCCR1B);
		BCLR(1 << PORTD3, PORTD);
	}


	sei(); /* set global interrupt enable */
}

/*See function prototype for documentation.*/
static void recordTicks()
{
	if (currentPeriodVelTicks > MAX_FWD_VEL_TCKS) /*check if we overflowed counting forward*/
	{
		prevPeriodVelTicks = MAX_FWD_VEL_TCKS;
	}
	else if (currentPeriodVelTicks < MAX_REV_VEL_TCKS) /*check if we overflowed counting backwards*/
	{
		prevPeriodVelTicks = MAX_REV_VEL_TCKS;
	}
	else
	{
		prevPeriodVelTicks = currentPeriodVelTicks;
	}
	currentPeriodVelTicks = 0; /*reset tick count for next reporting of ticks*/
}

/*See function prototype for documentation.*/
static void updatePWM(signed char power)
{
	if (isPowerInverted)
	{
		OCR1B = pgm_read_word(&dtyCycles[(unsigned char) (-power)]); /*change duty cycle on 16 bit timer*/
	}
	else
	{
		OCR1B = pgm_read_word(&dtyCycles[(unsigned char) power]); /*change duty cycle on 16 bit timer*/
	}
}

/*See function prototype for documentation.*/
static BOOL_T isDataByte(unsigned char b)
{
	BOOL_T result = FALSE;
	if ((b & 0x80) == 0) /*check that bit 7 is 0*/
	{
		result = TRUE;
	}
	return result;
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

