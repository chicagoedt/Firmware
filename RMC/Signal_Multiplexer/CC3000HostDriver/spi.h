/*
             BUTTLOAD - Butterfly ISP Programmer

              Copyright (C) Dean Camera, 2007.
              
             dean [at] fourwalledcubicle [dot] com
                  www.fourwalledcubicle.com

           Released under the GPL Licence, Version 2.

	Modified by John Sabino on 10/01/2013 for Chicago EDT.

*/
#ifndef SPI_H
#define SPI_H

#ifndef F_CPU
	#define F_CPU 4000000UL
#endif

	// INCLUDES:
	#include <avr/io.h>
	#include <string.h>
	#include <util/delay.h>


	//Definitions:

	#define	CC3000_MINIMAL_TX_SIZE      (130 + 1)  
	#define	CC3000_MAXIMAL_TX_SIZE      (1519 + 1)

//	#define CC3000_RX_BUFFER_SIZE   (CC3000_MINIMAL_TX_SIZE)
//	#define CC3000_TX_BUFFER_SIZE   (CC3000_MAXIMAL_TX_SIZE)
/*	
	//*****************************************************************************
	//                  ERROR CODES
	//*****************************************************************************
	#define ESUCCESS        0
	#define EFAIL          -1
	#define EERROR          EFAIL

	//=============================================================================
*/
	#define READ                            (3)
	#define WRITE                           (1)
	#define HI(value)                       (((value) & 0xFF00) >> 8)
	#define LO(value)                       ((value) & 0x00FF)
	#define HEADERS_SIZE_EVNT               (SPI_HEADER_SIZE + 5)
	#define SPI_HEADER_SIZE                 (5)

	//=============================================================================

	#define eSPI_STATE_POWERUP              (0)
	#define eSPI_STATE_INITIALIZED          (1)
	#define eSPI_STATE_IDLE                 (2)
	#define eSPI_STATE_WRITE_IRQ            (3)
	#define eSPI_STATE_WRITE_FIRST_PORTION  (4)
	#define eSPI_STATE_WRITE_EOT            (5)
	#define eSPI_STATE_READ_IRQ             (6)
	#define eSPI_STATE_READ_FIRST_PORTION   (7)
	#define eSPI_STATE_READ_EOT             (8)

	// The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
	// for the purpose of detection of the overrun. The location of the memory where the magic number
	// resides shall never be written. In case it is written - the overrun occured and either recevie function
	// or send function will stuck forever.
	#define CC3000_BUFFER_MAGIC_NUMBER (0xDE)			
	//=============================================================================

	// MACROS:
	#define SPI_SPIOFF()	     MACROS{ PRR |= (1 << PRSPI); }MACROE
	/*CC3000_ASSERT_CS sets the chip select line high, and then doubles the SCK frequency to improve synchronization.*/
	#define CC3000_ASSERT_CS 		{PORTB &= ~(1 << PORTB0); SPSR |= (1 << SPI2X);}
	#define CC3000_DEASSERT_CS 		{PORTB |= (1 << PORTB0);}
	// ======================================================================================	

	
	typedef void (*gcSpiHandleRx)(void *p);
	typedef void (*gcSpiHandleTx)(void);

	extern unsigned char wlan_tx_buffer[];
/*
	typedef struct
	{
		  gcSpiHandleRx  SPIRxHandler;

		  unsigned short usTxPacketLength;
		  unsigned short usRxPacketLength;
		  unsigned long  ulSpiState;
		  unsigned char *pTxPacket;
		  unsigned char *pRxPacket;

	} tSpiInformation;
*/
	

	// PROTOTYPES:
	extern void SPI_SPIInit(void);
	extern long SPI_SPITransmit(const long Data);
	extern void SpiOpen(gcSpiHandleRx pfRxHandler);
	extern void SpiRead(void);
	extern long SpiWrite(unsigned char *pUserBuffer, unsigned short usLength);
	extern void SpiClose(void);
	extern void SpiResumeSpi(void);
	extern long SpiFirstWrite(unsigned char *ucBuf, unsigned short usLength);
	extern void SpiWriteDataSynchronous(unsigned char *data, unsigned short size);
	extern void SpiReadDataSynchronous(unsigned char *data, unsigned short size);	
	extern void SpiReadHeader(void);
	extern inline init_spi(void);
	extern void SPI_IRQ(void);
	extern void SpiCleanGPIOISR(void);	
	extern long TXBufferIsEmpty(void);
	extern long RXBufferIsEmpty(void);
	
	//Additional
	extern void SpiWriteAsync(const unsigned char *data, unsigned short size);
	extern void SpiPauseSpi(void);
	extern void SSIContReadOperation(void);
	extern void cc3k_int_poll(void);

	
#endif
