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
#define CLK_TCKS_2 25

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
#define PRESCALER_0 4

/**
 * @brief Constant to set 16 bit Timer1 prescaler.
 *
 * See page 110 of the ATmega32 data sheet for valid values.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define PRESCALER_1 2

/**
 * @brief Constant to set 8 bit Timer2 prescaler.
 *
 * See page 125 of the ATmega32 data sheet for valid values.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
#define PRESCALER_2 4



/*=============================================================================
 *============================USART DEFINES====================================
 */


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
 * @brief Command to query the System Status.
 *
 * When this command is received through the USART interface, the tick count of
 * the last complete period will be returned as two unsigned char bytes.
 */
#define CMD_QUERY_SYSTEM_STATUS 0xC7

/*==========================END USART DEFINES==================================
 *=============================================================================
 */

/*=============================================================================
 *===========================I2C(TWI) MASTER DEFINES===========================
 */

/**
 * @brief I2C clock prescaler
 *
 */
#define PRESCALER_I2C 0

/**
 * @bried I2C bit rate
 *
 */
#define BITRATE_I2C 8

/**
 * @brief START status code (Prescaler = 0)
 *
 */
#define START 0x08

 /**
 * @brief Repeated START status code (Prescaler = 0)
 *
 */
#define STARTR 0x10

 /**
 * @brief SLA+W ACK received status code (Prescaler = 0)
 *
 */
#define MT_SLA_ACK 0x18

 /**
 * @brief SLA+W NOT ACK received status code (Prescaler = 0)
 *
 */
#define MT_SLA_NACK 0x20

 /**
 * @brief DATA ACK received status code (Prescaler = 0)
 *
 */
#define MT_DATA_ACK 0x28

 /**
 * @brief DATA NOT ACK received status code (Prescaler = 0)
 *
 */
#define MT_DATA_NACK 0x30

 /**
 * @brief Arbitration lost in SLA+W or data bytes status code (Prescaler = 0)
 *
 */
#define ARB_LOST 0x38

 /**
 * @brief SLA+R ACK received status code (Prescaler = 0)
 *
 */
#define MR_SLA_ACK 0x40

 /**
 * @brief SLA+R NOT ACK received status code (Prescaler = 0)
 *
 */
#define MR_SLA_NACK 0x48

 /**
 * @brief DATA ACK returned status code (Prescaler = 0)
 *
 */
#define MR_DATA_ACK 0x50

 /**
 * @brief DATA NOT ACK returned status code (Prescaler = 0)
 *
 */
#define MR_DATA_NACK 0x58

/**
 * @brief I2C address of the left EDCS.
 *
 */
#define EDCS_LEFT_ADDR 0x16

/**
 * @brief I2C address of the right EDCS.
 *
 */
#define EDCS_RIGHT_ADDR 0x17

/**
 * @brief I2C address of the E-STOP.
 *
 */
#define ESTOP_ADDR 0x18

/**
 * @brief I2C address of the Signal Multiplexer.
 *
 */
#define SIG_MUX_ADDR 0x19

/**
 * @brief I2C address of the System Monitor.
 *
 */
#define SYS_MON_ADDR 0x1A

/**
 * @brief I2C address of this module.
 *
 */
#define CONTROL_PANEL_ADDR 0x1B

/**
 * @brief I2C address of the front Sonar.
 *
 */
#define SONAR_FRONT_ADDR 0x20

/**
 * @brief I2C address of the back Sonar.
 *
 */
#define SONAR_BACK_ADDR 0x21



/*==========================END I2C(TWI) MASTER DEFINES========================
 *=============================================================================
 */

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

/*=============================================================================
 *===========================USART TYPEDEF=====================================
 */

/**
 * @brief List of states that the system can be in.
 *
 * Each of these states refers to a type of communication with the computer
 * that needs to be processed.
 *
 * @author Marios Fanourakis aristogenes@gmail.com
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
	COMM_MODE_QUERY_SYSTEM_STATUS,
	/** The system has received an invalid command, data or has timed out.*/
	COMM_MODE_ERROR
} COMM_MODE;

/**
 * @brief List of system states while reporting system status.
 *
 *
 * @author Marios Fanourakis aristogenes@gmail.com
 */
typedef enum
{
	/** The system just transmitted the status byte to the computer.*/
	STATE_QRY_SYSTEM_STATUS_1,
	/** The system just transmitted the first data byte to the computer.*/
	STATE_QRY_SYSTEM_STATUS_2,
} STATE_QRY_SYSTEM_STATUS;

/*==========================END USART TYPEDEF==================================
 *=============================================================================
 */

/*=============================================================================
 *===========================USART VARIABLES===================================
 */

/**
 * @brief This variable maintains the current internal communication state.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile COMM_MODE commMode;

/**
 * @brief This variable holds the current state while reporting the velocity tick count.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile STATE_QRY_SYSTEM_STATUS stateQrySystemStatus;

/**
 * @brief This variable holds the first byte of the system status.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile unsigned char systemStatus1;

/**
 * @brief This variable holds the second byte of the system status.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile unsigned char systemStatus2;

/*==========================END USART VARIABLES================================
 *=============================================================================
 */

/*=============================================================================
 *===========================I2C(TWI) MASTER VARIABLES=========================
 */

/**
 * @brief This variable contains the system's I2C address.
 *
 */
static unsigned char address;

/**
 * @brief This variable represents the status of the I2C START condition
 *
 * The possible values of this variable along with its corresponding meaning:
 * - 0: succesful START
 * - 1: START failure
 *
 * This variable is volatile because it is modified by interrupts.
 */
 static volatile unsigned char I2Cstart_status;

 /**
 * @brief This variable represents the status of the I2Cwrite function
 *
 * The possible values of this variable along with its corresponding meaning:
 * - 0: succesful write
 * - 1: START failure
 * - 2: MT_SLA_ACK error
 * - 3: MT_DATA_ACK error
 *
 * This variable is volatile because it is modified by interrupts.
 */
 static volatile unsigned char I2Cwrite_status;

 /**
 * @brief This variable represents the status of the I2Cread function
 *
 * The possible values of this variable along with its corresponding meaning:
 * - 0: succesful read
 * - 1: START failure
 * - 2: MR_SLA_ACK error
 * - 3: MR_DATA_ACK or MR_DATA_NACK error
 * - 4: in multiple byte read sequence
 * - 5: invalid mode number
 *
 * This variable is volatile because it is modified by interrupts.
 */
 static volatile unsigned char I2Cread_status;

/**
 * @brief This variable contains the current mode of the signal multiplexer
 *
 * The possible values of this variable along with its corresponding mode:
 * - Bit 0 : Set if valid signal
 * - Bit 2:1 : Current selection; 00 safety, 01 RC, 10 auto, 11 usr select
 * - Bit 3 : Reserved
 * - Bit 5:4 : Current mode; 00 safety, 01 RC, 10 Auto
 * - Bit 6 : Reserved
 * - Bit 7 : Reserved
 * It also indicates the error code in case of an I2C communication error.
 *
 * This variable is volatile because it is modified by interrupts.
 */
 static volatile unsigned char sigMuxI2CreadByte;

/**
 * @brief This variable contains the selected mode of the signal multiplexer
 *
 * The possible values of this variable along with its corresponding mode:
 * - Bit 0 : Reserved
 * - Bit 2:1 : Current selection; 00 safety, 01 RC, 10 auto, 11 usr select
 * - Bit 3:7 : Reserved
 *
 * This variable is volatile because it is modified by interrupts.
 */
 static volatile unsigned char sigMuxI2CwriteByte;

/**
 * @bried This variable signifies if there has been an I2C error for this system
 *
 * This variable is volatile because it is modified by interrups.
 */
static volatile BOOL_T sigMuxI2Cerror;

/**
 * @brief This variable determines if there needs to be a change in the signal multiplexer
 *
 * This variable is volatile because it is modified by interrups.
 */
static volatile BOOL_T sigMuxChange;

/**
 * @brief This variable holds the first byte of the edcs left read (power level)
 *
 * This variable contains the power level of the edcs left. It also indicates the error
 * code in case of an I2C communication error.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile signed char edcs_leftI2CreadByte1;

/**
 * @brief This variable holds the second byte of the edcs left read (lower byte for speed)
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile signed char edcs_leftI2CreadByte2;

/**
 * @brief This variable holds the third byte of the edcs left read (upper byte for speed)
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile signed char edcs_leftI2CreadByte3;

/**
 * @bried This variable signifies if there has been an I2C error for this system
 *
 * This variable is volatile because it is modified by interrups.
 */
static volatile BOOL_T edcs_leftI2Cerror;

/**
 * @brief This variable holds the first byte of the edcs right read (power level)
 *
 * This variable contains the power level of the edcs right. It also indicates the error
 * code in case of an I2C communication error.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile signed char edcs_rightI2CreadByte1;

/**
 * @brief This variable holds the second byte of the edcs right read (lower byte for speed)
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile signed char edcs_rightI2CreadByte2;

/**
 * @brief This variable holds the third byte of the edcs right read (upper byte for speed)
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile signed char edcs_rightI2CreadByte3;

/**
 * @bried This variable signifies if there has been an I2C error for this system
 *
 * This variable is volatile because it is modified by interrups.
 */
static volatile BOOL_T edcs_rightI2Cerror;

/**
 * @brief This variable holds the E-STOP read byte
 *
 * This variable contains status of the E-STOP. It also indicates the error
 * code in case of an I2C communication error.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile signed char estopI2CreadByte;

/**
 * @bried This variable signifies if there has been an I2C error for this system
 *
 * This variable is volatile because it is modified by interrups.
 */
static volatile BOOL_T estopI2Cerror;

/**
 * @bried This is holds the read byte in the I2CmasterRead function
 *
 * This variable is volatile because it is modified by interrups.
 */
static volatile unsigned char I2CreadByte;



/**
 * @brief This variable holds the first read byte for system monitor (12V read)
 *
 * This variable contains the 12V battery level information. It also indicates the error
 * code in case of an I2C communication error.
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile signed char sysMonI2CreadByte1;

/**
 * @brief This variable holds the second read byte for system monitor (24V read)
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile signed char sysMonI2CreadByte2;

/**
 * @brief This variable holds the third read byte for system monitor (system status)
 *
 * - Bit 0: Set if left side CB is popped
 * - Bit 1: Set if right side CB is popped
 * - Bit 2: Reserved
 * - Bit 3: Reserved
 * - Bit 4: Set if 12V is low
 * - Bit 5: Set if 12V is critically low
 * - Bit 6: Set if 24V is low
 * - Bit 7: Set if 24V is critically low
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile signed char sysMonI2CreadByte3;

/**
 * @bried This variable signifies if there has been an I2C error for this system
 *
 * This variable is volatile because it is modified by interrups.
 */
static volatile BOOL_T sysMonI2Cerror;

/** 
 * @brief This variable contains the first received byte from I2C(TWI)
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile unsigned char I2CByte1;

/** 
 * @brief This variable contains the second received byte from I2C(TWI)
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile unsigned char I2CByte2;

/** 
 * @brief This variable contains the third received byte from I2C(TWI)
 *
 * This variable is volatile because it is modified by interrupts.
 */
static volatile unsigned char I2CByte3;

/*==========================END I2C(TWI) MASTER VARIABLES======================
 *=============================================================================
 */

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

/*=============================================================================
 *===========================USART FUNCTIONS===================================
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
static void startTimeoutUSART();

/**
 * @brief Postpones the time until a timeout.
 *
 * When a timeout is postponed, more time is given to the system to finish
 * its task. The new amount of time is the timeout period.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void postponeTimeoutUSART();

/**
 * @brief Disables the timeout.
 *
 * When a timeout is disabled, the system no longer has to take further action
 * under a time limit. No error will be sent to the computer.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
static void stopTimeoutUSART();

/*==========================END USART FUNCTIONS================================
 *=============================================================================
 */

/*=============================================================================
 *===========================I2C(TWI) MASTER FUNCTIONS=========================
 */

/**
 * @brief initiates I2C communication
 *
 * This function sends the start condition through the I2C bus
 *
 * @return Returns 0 upon success, 1 upon failure
 *
 * @author Marios Fanourakis aristogenes@gmail.com
 */
 static unsigned char I2Cstart();

/** 
 * @brief terminates I2C communication
 *
 * This function sends the stop condition through the I2C bus
 *
 * @author Marios Fanourakis aristogenes@gmail.com
 */
 static void I2Cstop();

/**
 * @brief sends data through the I2C for master
 * 
 * This function sends one byte to any slave device through the I2C(TWI)
 * I2Cwrite_status will be changed to indicate the status of the write funciton
 * - 0: succesful write
 * - 1: START failure
 * - 2: MT_SLA_ACK error
 * - 3: MT_DATA_ACK error
 *
 * @param addr the slave address to write data
 * @param data the data to write
 *
 * @author Marios Fanourakis aristogenes@gmail.com
 */
static void I2CmasterWrite(unsigned char addr, unsigned char data);

/**
 * @brief requests data through the I2C for master
 * 
 * This function requests data from any slave device through the I2C(TWI).
 * I2Cread_status will be changed to indicate the status of the read function
 * - 0: succesful read
 * - 1: START failure
 * - 2: MR_SLA_ACK error
 * - 3: MR_DATA_ACK or MR_DATA_NACK error
 * - 4: in multiple byte read sequence
 * - 5: invalid mode number
 *
 * @param addr the slave address to read data from
 * @param mode the mode of the read operation:
 * - 0: read a single byte
 * - 1: initiate multiple byte read sequence, and read first byte
 * - 2: while in the multiple byte read sequence read another byte
 * - 3: read last byte and end multiple byte read sequence
 *
 * @return Returns read byte
 *
 * @author Marios Fanourakis aristogenes@gmail.com
 */
static unsigned char I2CmasterRead(unsigned char addr, unsigned char NumBytes);

/**
 * @brief I2C error handler
 *
 * This functions handles I2C communication errors
 * 
 * @author Marios Fanourakis aristogenes@gmail.com
 */
 static void I2Cerror();


/*==========================END I2C(TWI) MASTER FUNCTIONS======================
 *=============================================================================
 */

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
 * @brief Interrupt handler for Timer2 Overflow event.
 *
 * Whenever this event happens, a failure status byte will be sent to the
 * computer to indicate a timeout condition. The system will then be reset to
 * the idle state.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
ISR(TIMER2_OVF_vect)
{
	BCLR(1 << TOIE2, TIMSK); /*Clear the interrupt enable for Timer2 overflow to disable timeouts.*/
	commMode = COMM_MODE_IDLE; /*Reset system to idle mode*/
	transmitStatusFail(0); /*Send a failure notice to the computers*/
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
					case CMD_QUERY_SYSTEM_STATUS: /*if we are responding to a request for system status info*/
						transmitStatusSuccess(2); /*tell the PC that whatever it wanted to do succeeded and 2 more data bytes are coming*/
						commMode = COMM_MODE_QUERY_SYSTEM_STATUS;
						stateQrySystemStatus = STATE_QRY_SYSTEM_STATUS_1;
						//velTicksRS232 = prevPeriodVelTicks; /*update the tick count for the  RS232 communication*/
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

		case COMM_MODE_QUERY_SYSTEM_STATUS: /*if we are responding to a request for system status*/
			switch (stateQrySystemStatus)
			{
				case STATE_QRY_SYSTEM_STATUS_1:
					transmitByte(systemStatus1);/*send the first byte of the system*/
					stateQrySystemStatus = STATE_QRY_SYSTEM_STATUS_2; /*go to the next stage of reporting system status*/
					break;
				case STATE_QRY_SYSTEM_STATUS_2:
					transmitByte(systemStatus2); /*send the second byte of the system status*/
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
 * @brief Interrupt handler for Timer1 Compare event.
 *
 * This handler is responsible for the main program loop. Whenever the 1
 * compare event occurs, the next iteration of the program is run.
 *
 * @author Marios Fanourakis aristogenes@gmail.com
 */
ISR(TIMER2_COMP_vect)
{
	wdt_reset(); /*feed the watchdog*/

	/*Fire clock on PISO*/
	BCLR(1 << PD2, PORTD);
	BSET(1 << PD2, PORTD);

	/*This block reads in the value of PISO from SPI*/
	while ((SPSR & (1 << SPIF)) == 0)
	{
	}
	switchesPISO = SPDR;
	sigMuxSwitches = switchesPISO & 0x0F;

	/*This block analyzes the sig mux switches on the PISO*/
	if ((sigMuxSwitches != sigMuxSwitchesb))
	{
		if (sigMuxSwitches == ~0x01)
		{
			sigMuxI2CwriteByte = 0x00;
			sigMuxChange = TRUE;
		}
		else if (sigMuxSwitches == ~0x02)
		{
			sigMuxI2CwriteByte = 0x02;
			sigMuxChange = TRUE;
		}
		else if (sigMuxSwitches == ~0x04)
		{
			sigMuxI2CwriteByte = 0x04;
			sigMuxChange = TRUE;
		}
		else if (sigMuxSwitches == ~0x08)
		{
			sigMuxI2CwriteByte = 0x06;
			sigMuxChange = TRUE;
		}
		else
		{
			sigMuxChange = FALSE;
		}
	}
	else
	{
		sigMuxChange = FALSE;
	}

	/*Send commands to SigMux through I2C(TWI) if needed*/
	if (sigMuxChange)
	{
		sigMuxSwitchesb = sigMuxSwitches; /*Update if valid button press*/

		I2CmasterWrite(SIG_MUX_ADDR, sigMuxI2CwriteByte); /*Update SigMux*/

		if (I2Cwrite_status == 0)
		{
			sigMuxI2Cerror = FALSE;
		}
		else
		{
			sigMuxI2Cerror = TRUE;
		}
	}

	/*Request data from high priority slave devices through I2C(TWI): E-Stop, Signal Multiplexer, System Monitor*/
	/*E-STOP request*/
	estopI2CreadByte = I2CmasterRead(ESTOP_ADDR, 0);
	if (I2Cread_status == 0)
	{
		estopI2Cerror = FALSE;
	}
	else
	{
		estopI2CreadByte = 0xE0 + I2Cread_status;
		estopI2Cerror = TRUE;
	}

	/*Signal Multiplexer request*/
	sigMuxI2CreadByte =  I2CmasterRead(SIG_MUX_ADDR, 0);
	if (I2Cread_status == 0)
	{
		sigMuxI2Cerror = FALSE;
	}
	else
	{
		sigMuxI2CreadByte = 0xE0 + I2Cread_status;
		sigMuxI2Cerror = TRUE;
	}

	/*System Monitor request*/
	sysMonI2CreadByte1 = I2CmasterRead(SYS_MON_ADDR, 1);
	sysMonI2CreadByte2 = I2CmasterRead(SYS_MON_ADDR, 2);
	sysMonI2CreadByte3 = I2CmasterRead(SYS_MON_ADDR, 3);
	if(I2Cread_status == 0)
	{
		sysMonI2Cerror = FALSE;
	}
	else
	{
		sysMonI2CreadByte1 = 0xE0 + I2Cread_status;
		sysMonI2Cerror = TRUE;
	}

	++loopIterations;

	if(loopIterations >= NUM_CYCLES)
	{
		resetTimer();
		/*Request data from low priority slave devices through I2C(TWI): EDCS, Sonar*/
		/*edcs left request*/
		edcs_leftI2CreadByte1 = I2CmasterRead(EDCS_LEFT_ADDR, 1);
		edcs_leftI2CreadByte2 = I2CmasterRead(EDCS_LEFT_ADDR, 2);
		edcs_leftI2CreadByte3 = I2CmasterRead(EDCS_LEFT_ADDR, 3);
		if (I2Cread_status == 0)
		{
			edcs_leftI2Cerror = FALSE;
		}
		else
		{
			edcs_leftI2CreadByte1 = 0xE0 + I2Cread_status;
			edcs_leftI2Cerror = TRUE;
		}

		/*edcs right request*/
		edcs_rightI2CreadByte1 = I2CmasterRead(EDCS_RIGHT_ADDR, 1);
		edcs_rightI2CreadByte1 = I2CmasterRead(EDCS_RIGHT_ADDR, 2);
		edcs_rightI2CreadByte1 = I2CmasterRead(EDCS_RIGHT_ADDR, 3);
		if (I2Cread_status == 0)
		{
			edcs_rightI2Cerror = FALSE;
		}
		else
		{
			edcs_rightI2CreadByte1 = 0xE0 + I2Cread_status;
			edcs_rightI2Cerror = TRUE;
		}

		/*Sonar back request*/

		/*Sonar front request*/


		/*Update LCD*/
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
	wdt_enable(WDTO_15MS); /*Initialize watchdog, timeout is 15 ms*/

	/*Initialize ports*/
	DDRA = 0xFF;
	DDRB = (1 << DDB5) | (1 << DDB7);
	DDRC = (1 << DDC2) | (1 << DDC3) | (1 << DDC4) | (1 << DDC5);
	DDRD = (1 << DDD1) | (1 << DDD2);

	/*Set variables to default values*/
	loopIterations = 0;
	sigMuxSwitchesb = 0;

	/*Setup SPI*/
	SPCR = (1 << SPE) | (1 << MSTR);

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

	/*Setup the Control Panel Address */
	address = CONTROL_PANEL_ADDR;

	/*Initialize I2C(TWI)*/
	TWBR = BITRATE_I2C;
	TWSR = PRESCALER_I2C << TWPS0;
	TWAR = CONTROL_PANEL_ADDR << 1;
	TWCR = (1 << TWEA) | (1 << TWEN);

	/*Configure LCD*/

	/*Setup sleep mode*/
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();

	/*Configure Timers*/
	TCCR0 = (PRESCALER_0 << CS00);

	TCCR2 = (PRESCALER_2 << CS20) | (1 << WGM21);
	OCR2 = CLK_TCKS_2 - 1; /*This value is subtracted by one since CTC mode will go for (OCR2 + 1) cycles before starting over*/
	TCNT2 = 0;
	TIFR = 1 << OCF2; /*Clear the time compare bit if set*/


	sei(); /* set global interrupt enable */
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
	stopTimeoutUSART();
	transmitByte(0x80 | numDataBytes);
}

/*See function prototype for documentation.*/
static void transmitStatusSuccess(unsigned char numDataBytes)
{
	stopTimeoutUSART();
	transmitByte(0xC0 | numDataBytes);
}

/*See function prototype for documentation.*/
static void startTimeoutUSART()
{
	TCNT0 = 0; /*reset the counter so full timeout period can be used*/
	if ((1 << TOV0) & TIFR) /*checks for Timer0 overflow falg*/
	{
		BSET(1 << TOV0, TIFR); /*clear Timer0 overlow flag*/
	}
	BSET(1 << TOIE0, TIMSK); /*enable Timer0 interrupt*/
}

/*See function prototype for documentation.*/
static void postponeTimeoutUSART()
{
	TCNT0 = 0; /*reset the counter so its interrupt is postponed*/
}

/*See function prototype for documentation.*/
static void stopTimeoutUSART()
{
	BCLR(1 << TOIE0, TIMSK); /*turns off the Timer0 interrupt*/
}

/*See function prototype for documentation.*/
static unsigned char I2Cstart()
{
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (0 << TWSTO);

	while (!(TWCR & (1<<TWINT)))
	{
	}

	if ((TWSR & 0xF8) != START)
	{
		I2Cerror();
		return 0x01;
	}

	return 0x00;
}

/*See function prototype for documentation.*/
static void I2Cstop()
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 <<TWSTO);
}

/*See function prototype for documentation.*/
static void I2CmasterWrite(unsigned char addr, unsigned char data)
{
	/*Start I2C*/
	I2Cstart_status = I2Cstart();
	if(I2Cstart_status!=0)
	{
		I2Cwrite_status = I2Cstart_status;
		return;
	}
	
	/*Send Slave address*/
	TWDR = (addr << 1);
	TWCR = (1 << TWINT) | (1 << TWEN);

	while (!(TWCR & (1 << TWINT)))
	{
	}

	if ((TWSR & 0xF8) != MT_SLA_ACK)
	{
		I2Cerror();
		I2Cwrite_status = 0x02;
		return;
	}

	/*Send Data*/
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);

	while (!(TWCR & (1 << TWINT)))
	{
	}

	if ((TWSR & 0xF8) != MT_DATA_ACK)
	{
		I2Cerror();
		I2Cwrite_status = 0x03;
		return;
	}

	/*Stop I2C*/
	I2Cstop();

	I2Cwrite_status = 0;
	return;
}

/*See function prototype for documentation.*/
static unsigned char I2CmasterRead(unsigned char addr, unsigned char mode)
{
	/*Read only one byte*/
	if (mode == 0)
	{
		/*Start I2C*/
		I2Cstart_status = I2Cstart();
		if(I2Cstart_status!=0)
		{
			I2Cread_status = I2Cstart_status;
			return 0;
		}

		/*Send Address*/
		TWDR = (addr << 1) + 1;
		TWCR = (1 << TWINT) | (1 << TWEN);
		while (!(TWCR & (1 << TWINT)))
		{
		}

		if ((TWSR & 0xF8) != MR_SLA_ACK)
		{
			I2Cerror();
			I2Cread_status = 0x02;
			return 0;
		}
		
		/*Send not acknowledge*/
		TWCR = (1 << TWINT) | (0 << TWEA) | (1 << TWEN);
		while (!(TWCR & (1 << TWINT)))
		{
		}

		/*Read byte, stop I2C, and return the read byte*/
		I2CreadByte = TWDR;
		I2Cstop();
		return I2CreadByte;
	}

	/*Initiate multiple byte read sequence and read firt byte*/
	else if (mode == 1)
	{
		/*Start I2C*/
		I2Cstart_status = I2Cstart();
		if(I2Cstart_status!=0)
		{
			I2Cread_status = I2Cstart_status;
			return 0;
		}
	
		/*Send address*/
		TWDR = (addr << 1) + 1;
		TWCR = (1 << TWINT) | (1 << TWEN);
		while (!(TWCR & (1 << TWINT)))
		{
		}

		if ((TWSR & 0xF8) != MR_SLA_ACK)
		{
			I2Cerror();
			I2Cread_status = 0x02;
			return 0;
		}

		/*Send Acknowledge*/
		TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
		while (!(TWCR & (1 << TWINT)))
		{
		}

		I2Cread_status = 4;		

		/*Return read byte*/
		I2CreadByte = TWDR;
		return I2CreadByte;

	}
	/*While in multiple byte read sequence read a byte*/
	else if (mode == 2)
	{
		/*Send acknowledge*/
		TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
		while (!(TWCR & (1 << TWINT)))
		{
		}

		I2Cread_status = 4;

		/*return read byte*/
		I2CreadByte = TWDR;
		return I2CreadByte;

	}
	/*Read last byte and termintate multiple byte read sequence*/
	else if (mode == 3)
	{
		/*Send not acknowledge*/
		TWCR = (1 << TWINT) | (0 << TWEA) | (1 << TWEN);
		while (!(TWCR & (1 << TWINT)))
		{
		}

		I2Cread_status = 0;

		/*stop I2C, return read byte*/
		I2CreadByte = TWDR;
		I2Cstop();
		return I2CreadByte;

	}
	/*Invalid input*/
	else
	{
		I2Cerror();
		I2Cread_status = 5;
		return 0;
	}
	
}

/*See function prototype for documentation.*/
static void I2Cerror()
{
	I2Cstop();
}


/**
 * @}
 * */

