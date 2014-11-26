/**
 * @defgroup phobosfirmware_component_control_panel_gpu Control Panel GPU
 * @ingroup phobosfirmware_component_control_panel
 *
 * @brief Control Panel controls the motors and
 * reports odometry from wheel encoders. 
 *
 * This module count the number of encoder ticks for a period of time and then
 * send the tick count to the computers at the end of the time period. Two bars
 * of LEDs will illuminate, one for forward motion and one for reverse motion,
 * giving an indication of the magnitude and direction of the wheel speed. In
 * addtion, the module will output a PWM signal to the motorcontroller. Two
 * bars of LEDs will illuminate, one for forward power delivered and one for
 * reverse power delivered, giving an indication of the magnitude and direction
 * of the power delivered to the motor.
 *
 * This code is designed to run on an Atmel ATmega8 microcontroller running at
 * 16 MHz. Please refer to the Control Panel requirements
 * and design documents while examining or modifying this code.
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
#include <avr/sleep.h>
#include <stdint.h>
#include <avr/pgmspace.h> 

/**
 * @brief Operating Frequency of uC in Hertz.
 */
#define F_CPU 8000000UL
#include <util/delay.h>

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


static const unsigned char letters[] PROGMEM =  {
	/*space*/
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,

	/*0*/
	0x00,0x00,0x00,0x00,
	0x3C,0x66,0x6E,0x6E,
	0x76,0x76,0x66,0x3C,
	0x00,0x00,0x00,0x00,

	/*1*/
	0x00,0x00,0x00,0x00,
	0x18,0x38,0x78,0x18,
	0x18,0x18,0x18,0x7E,
	0x00,0x00,0x00,0x00,

	/*2*/
	0x00,0x00,0x00,0x00,
	0x3C,0x66,0x06,0x1C,
	0x30,0x60,0x60,0x7E,
	0x00,0x00,0x00,0x00,

	/*3*/
	0x00,0x00,0x00,0x00,
	0x3C,0x66,0x06,0x1C,
	0x1C,0x06,0x66,0x3C,
	0x00,0x00,0x00,0x00,

	/*4*/
	0x00,0x00,0x00,0x00,
	0x0C,0x1C,0x3C,0x6C,
	0x6C,0x7E,0x0C,0x0C,
	0x00,0x00,0x00,0x00,

	/*5*/
	0x00,0x00,0x00,0x00,
	0x7E,0x60,0x60,0x7C,
	0x06,0x06,0x66,0x3C,
	0x00,0x00,0x00,0x00,

	/*6*/
	0x00,0x00,0x00,0x00,
	0x1C,0x30,0x60,0x7C,
	0x66,0x66,0x66,0x3C,
	0x00,0x00,0x00,0x00,

	/*7*/
	0x00,0x00,0x00,0x00,
	0x7E,0x06,0x0C,0x18,
	0x30,0x30,0x30,0x30,
	0x00,0x00,0x00,0x00,

	/*8*/
	0x00,0x00,0x00,0x00,
	0x3C,0x66,0x66,0x3C,
	0x3C,0x66,0x66,0x3C,
	0x00,0x00,0x00,0x00,

	/*9*/
	0x00,0x00,0x00,0x00,
	0x3C,0x66,0x66,0x7E,
	0x06,0x06,0x0C,0x38,
	0x00,0x00,0x00,0x00,

	/*:*/
	0x00,0x00,0x00,0x00,
	0x00,0x18,0x18,0x00,
	0x00,0x18,0x18,0x00,
	0x00,0x00,0x00,0x00,

	/*A*/
	0x00,0x00,0x00,0x00,
	0x3C,0x66,0x66,0x66,
	0x7E,0x66,0x66,0x66,
	0x00,0x00,0x00,0x00,

	/*B*/
	0x00,0x00,0x00,0x00,
	0x7C,0x66,0x66,0x7C,
	0x66,0x66,0x66,0x7C,
	0x00,0x00,0x00,0x00,

	/*C*/
	0x00,0x00,0x00,0x00,
	0x3C,0x66,0x60,0x60,
	0x60,0x60,0x66,0x3C,
	0x00,0x00,0x00,0x00,

	/*D*/
	0x00,0x00,0x00,0x00,
	0x78,0x4C,0x46,0x46,
	0x46,0x46,0x4C,0x78,
	0x00,0x00,0x00,0x00,

	/*E*/
	0x00,0x00,0x00,0x00,
	0x7E,0x60,0x60,0x60,
	0x7C,0x60,0x60,0x7E,
	0x00,0x00,0x00,0x00,

	/*F*/
	0x00,0x00,0x00,0x00,
	0x7E,0x60,0x60,0x60,
	0x7C,0x60,0x60,0x60,
	0x00,0x00,0x00,0x00,

	/*G*/
	0x00,0x00,0x00,0x00,
	0x3C,0x66,0x60,0x60,
	0x7E,0x66,0x66,0x3E,
	0x00,0x00,0x00,0x00,

	/*H*/
	0x00,0x00,0x00,0x00,
	0x66,0x66,0x66,0x66,
	0x7E,0x66,0x66,0x66,
	0x00,0x00,0x00,0x00,

	/*I*/
	0x00,0x00,0x00,0x00,
	0x3C,0x18,0x18,0x18,
	0x18,0x18,0x18,0x3C,
	0x00,0x00,0x00,0x00,

	/*J*/
	0x00,0x00,0x00,0x00,
	0x1E,0x0C,0x0C,0x0C,
	0x0C,0x6C,0x6C,0x38,
	0x00,0x00,0x00,0x00,

	/*K*/
	0x00,0x00,0x00,0x00,
	0x66,0x6C,0x78,0x78,
	0x78,0x78,0x6C,0x66,
	0x00,0x00,0x00,0x00,

	/*L*/
	0x00,0x00,0x00,0x00,
	0x60,0x60,0x60,0x60,
	0x60,0x60,0x60,0x7E,
	0x00,0x00,0x00,0x00,

	/*M*/
	0x00,0x00,0x00,0x00,
	0x66, 0xFF, 0xFF, 0xDB,
	0xDB, 0xC3, 0xC3, 0xC3,
	0x00,0x00,0x00,0x00,

	/*N*/
	0x00,0x00,0x00,0x00,
	0x66,0x76,0x76,0x76,
	0x6E,0x6E,0x6E,0x66,
	0x00,0x00,0x00,0x00,

	/*O*/
	0x00,0x00,0x00,0x00,
	0x3C,0x66,0x66,0x66,
	0x66,0x66,0x66,0x3C,
	0x00,0x00,0x00,0x00,

	/*P*/
	0x00,0x00,0x00,0x00,
	0x7C,0x66,0x66,0x66,
	0x7C,0x60,0x60,0x60,
	0x00,0x00,0x00,0x00,

	/*Q*/
	0x00,0x00,0x00,0x00,
	0x3C,0x42,0x42,0x42,
	0x42,0x7E,0x3C,0x06,
	0x00,0x00,0x00,0x00,

	/*R*/
	0x00,0x00,0x00,0x00,
	0x7C,0x66,0x66,0x66,
	0x7C,0x78,0x6C,0x66,
	0x00,0x00,0x00,0x00,

	/*S*/
	0x00,0x00,0x00,0x00,
	0x3C,0x66,0x60,0x3C,
	0x3C,0x06,0x66,0x3C,
	0x00,0x00,0x00,0x00,

	/*T*/
	0x00,0x00,0x00,0x00,
	0x7E,0x18,0x18,0x18,
	0x18,0x18,0x18,0x18,
	0x00,0x00,0x00,0x00,

	/*U*/
	0x00,0x00,0x00,0x00,
	0x66,0x66,0x66,0x66,
	0x66,0x66,0x66,0x3C,
	0x00,0x00,0x00,0x00,

	/*V*/
	0x00,0x00,0x00,0x00,
	0x66,0x66,0x66,0x66,
	0x66,0x66,0x3C,0x18,
	0x00,0x00,0x00,0x00,

	/*W*/
	0x00,0x00,0x00,0x00,
	0xC3, 0xC3, 0xC3, 0xDB,
	0xDB, 0xFF, 0xFF, 0x66,
	0x00,0x00,0x00,0x00,

	/*X*/
	0x00,0x00,0x00,0x00,
	0x66,0x66,0x3C,0x18,
	0x18,0x3C,0x66,0x66,
	0x00,0x00,0x00,0x00,

	/*Y*/
	0x00,0x00,0x00,0x00,
	0x66,0x66,0x66,0x66,
	0x3C,0x18,0x18,0x18,
	0x00,0x00,0x00,0x00,

	/*Z*/
	0x00,0x00,0x00,0x00,
	0x7E,0x02,0x06,0x1E,
	0x78,0x60,0x40,0x7E,
	0x00,0x00,0x00,0x00,

	/*.*/
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x0,
	0x00,0x00,0x18,0x18,
	0x00,0x00,0x00,0x00,

	//Progress Bar Characters; notes:index 39 starts below, goes from 'a' to 'z'
	/*Progress Bar Left Most + 0, Progress Bar Right Mid + 0*/ //a
	0x00,0x00,0x00,0x00,
	0xFF,0x80,0x80,0x80,
	0x80,0x80,0x80,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Right Most + 0, Progress Bar Left Mid + 0*/ //b
	0x00,0x00,0x00,0x00,
	0xFF,0x01,0x01,0x01,
	0x01,0x01,0x01,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Complete*/ //c
	0x00,0x00,0x00,0x00,
	0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Clear*/ //d
	0x00,0x00,0x00,0x00,
	0xFF,0x00,0x00,0x00,
	0x00,0x00,0x00,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Left Most + 1, Progress bar Right Most + 1*/ //e
	0x00,0x00,0x00,0x00,
	0xFF,0x81,0x81,0x81,
	0x81,0x81,0x81,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Left Most + 2*/ //f
	0x00,0x00,0x00,0x00,
	0xFF,0x83,0x83,0x83,
	0x83,0x83,0x83,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Left Most + 3*/ //g
	0x00,0x00,0x00,0x00,
	0xFF,0x87,0x87,0x87,
	0x87,0x87,0x87,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Left Most + 4*/ //h
	0x00,0x00,0x00,0x00,
	0xFF,0x8F,0x8F,0x8F,
	0x8F,0x8F,0x8F,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Left Most + 5*/ //i
	0x00,0x00,0x00,0x00,
	0xFF,0x9F,0x9F,0x9F,
	0x9F,0x9F,0x9F,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Left Most + 6*/ //j
	0x00,0x00,0x00,0x00,
	0xFF,0xBF,0xBF,0xBF,
	0xBF,0xBF,0xBF,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Right Most + 2*/ //k
	0x00,0x00,0x00,0x00,
	0xFF,0xC1,0xC1,0xC1,
	0xC1,0xC1,0xC1,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Right Most + 3*/ //l
	0x00,0x00,0x00,0x00,
	0xFF,0xE1,0xE1,0xE1,
	0xE1,0xE1,0xE1,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Right Most + 4*/ //m
	0x00,0x00,0x00,0x00,
	0xFF,0xF1,0xF1,0xF1,
	0xF1,0xF1,0xF1,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Right Most + 5*/ //n
	0x00,0x00,0x00,0x00,
	0xFF,0xF9,0xF9,0xF9,
	0xF9,0xF9,0xF9,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Right Most + 6*/ //o
	0x00,0x00,0x00,0x00,
	0xFF,0xFD,0xFD,0xFD,
	0xFD,0xFD,0xFD,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Left Mid + 1*/ //p
	0x00,0x00,0x00,0x00,
	0xFF,0x03,0x03,0x03,
	0x03,0x03,0x03,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Left Mid + 2*/ //q
	0x00,0x00,0x00,0x00,
	0xFF,0x07,0x07,0x07,
	0x07,0x07,0x07,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Left Mid + 3*/ //r
	0x00,0x00,0x00,0x00,
	0xFF,0x0F,0x0F,0x0F,
	0x0F,0x0F,0x0F,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Left Mid + 4*/ //s
	0x00,0x00,0x00,0x00,
	0xFF,0x1F,0x1F,0x1F,
	0x1F,0x1F,0x1F,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Left Mid + 5*/ //t
	0x00,0x00,0x00,0x00,
	0xFF,0x3F,0x3F,0x3F,
	0x3F,0x3F,0x3F,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Left Mid + 6*/ //u
	0x00,0x00,0x00,0x00,
	0xFF,0x7F,0x7F,0x7F,
	0x7F,0x7F,0x7F,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Right Mid + 1*/ //v
	0x00,0x00,0x00,0x00,
	0xFF,0xC0,0xC0,0xC0,
	0xC0,0xC0,0xC0,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Right Mid + 2*/ //w
	0x00,0x00,0x00,0x00,
	0xFF,0xE0,0xE0,0xE0,
	0xE0,0xE0,0xE0,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Right Mid + 3*/ //x
	0x00,0x00,0x00,0x00,
	0xFF,0xF0,0xF0,0xF0,
	0xF0,0xF0,0xF0,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Right Mid + 4*/ //y
	0x00,0x00,0x00,0x00,
	0xFF,0xF8,0xF8,0xF8,
	0xF8,0xF8,0xF8,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Right Mid + 5*/ //z
	0x00,0x00,0x00,0x00,
	0xFF,0xFC,0xFC,0xFC,
	0xFC,0xFC,0xFC,0xFF,
	0x00,0x00,0x00,0x00,

	/*Progress Bar Right Mid + 6*/ //notes:26 progress bar chars, this is the 64th index //{
	0x00,0x00,0x00,0x00,
	0xFF,0xFE,0xFE,0xFE,
	0xFE,0xFE,0xFE,0xFF,
	0x00,0x00,0x00,0x00
};

//order which strings will display
//pixels will be displayed vertically and move to the right by one
static const unsigned char stringToDisplayOrder[] = {0,16,32,48,1,17,33,49,2,18,34,50,3,19,35,51,
								  4,20,36,52,5,21,37,53,6,22,38,54,7,23,39,55,
								  8,24,40,56,9,25,41,57,10,26,42,58,11,27,43,59,
								  12,28,44,60,13,29,45,61,14,30,46,62,15,31,47,63};

static volatile unsigned char currentLetter;

static volatile char buffer[64];

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

//takes ascii char and returns the index of our data
static uint16_t getIndex(unsigned char c, uint16_t line);

static void writeStr(volatile char* str);
static void setY(unsigned char y);
static void setX(unsigned char x);
static void enableLCD(void);
static void writeByte(unsigned char b);
static void clearCS();
static void select2();
static void select1();

ISR(SPI_STC_vect)
{
	unsigned char data = SPDR;
	BSET(1 << PC5, PORTC);
	if (currentLetter < 64)
	{
		buffer[currentLetter] = data;
		currentLetter++;
		if (currentLetter == 64)
		{
			BCLR(1 << PC5, PORTC);
			writeStr("AKOT");
		}
		
	}
}

/**
 * @brief Interrupt handler for Input Capture event.
 *
 * @author Anup Kotadia anup.kotadia@gmail.com
 */
ISR(TIMER1_CAPT_vect)
{
	currentLetter = 0;
}

int main()
{
	int result = 0;
	currentLetter = 0;
	_delay_ms(20);
	init();
	//writeStr("MARIOS IS GAY                                                                                       ");
	while (TRUE)
	{
		sleep_cpu(); /*enter sleep mode*/
	}
	return result;
}

/*See function prototype for documentation.*/
void init(void)
{
	//wdt_enable(WDTO_15MS); /*Initialize watchdog, timeout is 15 ms*/

	/*Initialize ports*/
	DDRD = (1 << DDD0) |  (1 << DDD1) | (1 << DDD2) | (1 << DDD3) | (1 << DDD4) | (1 << DDD5) | (1 << DDD6) | (1 << DDD7);
	DDRC = (1 << DDC0) | (1 << DDC1) | (1 << DDC2) | (1 << DDC3) | (1 << DDC5);

	BSET(1 << PC1, PORTC);

	/*Setup SPI*/
	SPCR = (1 << SPE) | (1 << SPIE);

	/*Setup sleep mode*/
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();

	/*Setup timer*/
	TCCR1A = 0;
	TCCR1B = 1 << CS10;
	TIMSK = 1 << TICIE1;

	/*turn on LCD*/
	//initialize left side controller
	BCLR(1<<PC0, PORTC);
	_delay_us(2);


	BSET(1<<PC2, PORTC);
	_delay_us(2);
	BCLR(1<<PC3, PORTC);
	_delay_us(2);

	PORTD=0xc0;
	BSET(1<<PC1, PORTC);
	_delay_us(2);
	BCLR(1<<PC1, PORTC);
	_delay_us(2);
	BCLR(1<<PC2, PORTC);
	_delay_us(2);
	BCLR(1<<PC3, PORTC);
	_delay_us(2);
	
	BSET(1<<PC2, PORTC);
	_delay_us(2);
	BCLR(1<<PC3, PORTC);
	_delay_us(2);

	PORTD=0x40;
	BSET(1<<PC1, PORTC);
	_delay_us(2);
	BCLR(1<<PC1, PORTC);
	_delay_us(2);
	BCLR(1<<PC2, PORTC);
	_delay_us(2);
	BCLR(1<<PC3, PORTC);
	_delay_us(2);
	
	BSET(1<<PC2, PORTC);
	_delay_us(2);
	BCLR(1<<PC3, PORTC);
	_delay_us(2);

	PORTD=0xb8;
	BSET(1<<PC1, PORTC);
	_delay_us(2);
	BCLR(1<<PC1, PORTC);
	_delay_us(2);
	BCLR(1<<PC2, PORTC);
	_delay_us(2);
	BCLR(1<<PC3, PORTC);
	_delay_us(2);
	
	BSET(1<<PC2, PORTC);
	_delay_us(2);
	BCLR(1<<PC3, PORTC);
	_delay_us(2);

	PORTD=0x3f;
	BSET(1<<PC1, PORTC);
	_delay_us(2);
	BCLR(1<<PC1, PORTC);
	_delay_us(2);
	BCLR(1<<PC2, PORTC);
	_delay_us(2);
	BCLR(1<<PC3, PORTC);
	_delay_us(2);
	

	//initialize right side controller
	BCLR(1<<PC0, PORTC);
	_delay_us(2);


	BCLR(1<<PC2, PORTC);
	_delay_us(2);
	BSET(1<<PC3, PORTC);
	_delay_us(2);

	PORTD=0xc0;
	BSET(1<<PC1, PORTC);
	_delay_us(2);
	BCLR(1<<PC1, PORTC);
	_delay_us(2);
	BCLR(1<<PC2, PORTC);
	_delay_us(2);
	BCLR(1<<PC3, PORTC);
	_delay_us(2);
	
	BCLR(1<<PC2, PORTC);
	_delay_us(2);
	BSET(1<<PC3, PORTC);
	_delay_us(2);

	PORTD=0x40;
	BSET(1<<PC1, PORTC);
	_delay_us(2);
	BCLR(1<<PC1, PORTC);
	_delay_us(2);
	BCLR(1<<PC2, PORTC);
	_delay_us(2);
	BCLR(1<<PC3, PORTC);
	_delay_us(2);
	
	BCLR(1<<PC2, PORTC);
	_delay_us(2);
	BSET(1<<PC3, PORTC);
	_delay_us(2);

	PORTD=0xb8;
	BSET(1<<PC1, PORTC);
	_delay_us(2);
	BCLR(1<<PC1, PORTC);
	_delay_us(2);
	BCLR(1<<PC2, PORTC);
	_delay_us(2);
	BCLR(1<<PC3, PORTC);
	_delay_us(2);
	
	BCLR(1<<PC2, PORTC);
	_delay_us(2);
	BSET(1<<PC3, PORTC);
	_delay_us(2);

	PORTD=0x3f;
	BSET(1<<PC1, PORTC);
	_delay_us(2);
	BCLR(1<<PC1, PORTC);
	_delay_us(2);
	BCLR(1<<PC2, PORTC);
	_delay_us(2);
	BCLR(1<<PC3, PORTC);
	_delay_us(2);

	sei(); /* set global interrupt enable */
}

	//takes ascii char and returns the index of our data
static uint16_t getIndex(unsigned char c, uint16_t line)
{
	uint16_t result = 0;
	if (c >= '0' && c <= ':')
	{
		result = c - 47;
	}
	else if (c >= 'A' && c <= 'Z')
	{
		result = c - 53;
	}
	else if (c == '.')
	{
		result = 38;
	}

	else if(c >= 'a' && c <= '{')
	{
		result = c - 58;
	}

	return (result << 4) + line; //shift left by four bits (multiply by 16)
}

static void writeStr(volatile char* str)
{
	for (int i = 0; i < 16; ++i)
	{
		if (i < 8)
		{
			select1();
		}
		else
		{
			select2();
		}
		setX(i);
		clearCS();

		if (i < 8)
		{
			select1();
		}
		else
		{
			select2();
		}
		setY(0);

		for (int j = 0; j < 4; j++)
		{
			char c = str[stringToDisplayOrder[(i << 2) + j]];
			for (int k = 0; k < 16; k++)
			{
				if (i < 8)
				{
					select1();
				}
				else
				{
					select2();
				}
				writeByte(pgm_read_byte(&letters[getIndex(c, k)]));
			}
		}
	}
}

static void writeByte(unsigned char b)
{
	PORTD = b;
	BSET(1 << PC0, PORTC);
	enableLCD();
	clearCS();
}

static void setY(unsigned char y)
{
	PORTD = (1 << PD6) | (y & 0x3F);
	BCLR(1 << PC0, PORTC);
	enableLCD();
}

static void setX(unsigned char x)
{
	PORTD = (1 << PD7) | (1 << PD5) | (1 << PD4) | (1 << PD3) | (x & 0x07);
	BCLR(1 << PC0, PORTC);
	_delay_us(2);
	enableLCD();
}

static void enableLCD(void)
{
	_delay_us(2);
	BSET(1<<PC1, PORTC);
	_delay_us(2);
	BCLR(1<<PC1, PORTC);
	_delay_us(2);
}

static void clearCS()
{
	BCLR(1<<PC2, PORTC);
	_delay_us(2);
	BCLR(1<<PC3, PORTC);
	_delay_us(2);	
}

static void select1()
{
	BSET(1 << PC2, PORTC);
	BCLR(1 << PC3, PORTC);
	_delay_us(2);
}

static void select2()
{
	BSET(1 << PC3, PORTC);
	BCLR(1 << PC2, PORTC);
	_delay_us(2);
}