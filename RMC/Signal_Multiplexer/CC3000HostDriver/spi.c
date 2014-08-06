/*
	Library: 		spi.c
	
	Author:			John Sabino
	Organization:		Chicago Engineering Design Team
	Date Created:		09/16/2013
	Last Date Modified:	04/02/2014
	
Revisions:
        
        0.1 (09/16/2013) - File created.
        1.0 (10/08/2013) - Fully operational code completed.
        1.1 (04/02/2014) - Added revisions list. Removed Dean Camera copyright, none of his code was used. 	
*/

#include "./hci.h"
#include "./netapp.h"
#include "./evnt_handler.h"
#include "./cc3000_common.h"
#include "./spi.h"

typedef struct
{
	  gcSpiHandleRx  SPIRxHandler;

	  unsigned short usTxPacketLength;
	  unsigned short usRxPacketLength;
	  unsigned long  ulSpiState;
	  unsigned char *pTxPacket;
	  unsigned char *pRxPacket;

} tSpiInformation;

static volatile tSpiInformation sSpiInformation;

//Global Variables:
char spi_buffer[CC3000_RX_BUFFER_SIZE];
unsigned char wlan_tx_buffer[CC3000_TX_BUFFER_SIZE];

static volatile uint8_t ccspi_is_in_irq   = 0;
static volatile uint8_t ccspi_int_enabled = 0;


//#######################################################################
//#######################################################################
//#######################################################################
//#######################################################################

/*
 NAME:      | SPI_SPIInit
 PURPOSE:   | Initializes the SPI subsystem ready for data transfers
 ARGUMENTS: | None
 RETURNS:   | None
*/
void SPI_SPIInit(void)
{
/*
	PRR &= ~(1 << PRSPI);              // Enable the SPI system by clearing the power save register SPI disable bit

	SPCR = ((1 << SPE) | (1 << MSTR) | (1 << CPHA) | (1 << CPOL) | (1 << SPR1)); // Master, Sample falling edge (setup rising), Fcpu/64 speed (7.3MHz/16 ~= 114KHz)
*/
}


long SPI_SPITransmit(const long Data)
{
	SPDR = Data;                       // Loading a byte into the data register, data is shifted out automatically
//PORTB |= (1 << PB4);
//PORTD |= (1 << PD7) | (1 << PD6);
	while (!(SPSR & (1 << SPIF)));     // Wait until transmission completed
//PORTB &= ~(1 << PB4);
//PORTD &= ~((1 << PD7) | (1 << PD6));
//_delay_ms(1000);
//PORTB |= (1 << PB4);
//_delay_ms(1000);
//PORTB &= ~(1 << PB4);
	return SPDR;
}
//#########################################################################################################
//#########################################################################################################
/*!

 */
/**************************************************************************/
void SpiPauseSpi(void)
{
  //DEBUGPRINT_F("\tCC3000: SpiPauseSpi\n\r");

  ccspi_int_enabled = 0;
  tSLInformation.WlanInterruptDisable();
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void SpiTriggerRxProcessing(void)
{

  /* Trigger Rx processing */
  SpiPauseSpi();
  CC3000_DEASSERT_CS;

  /* The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
   * for the purpose of detection of the overrun. If the magic number is overriten - buffer overrun
   * occurred - and we will stuck here forever! */
  if (sSpiInformation.pRxPacket[CC3000_RX_BUFFER_SIZE - 1] != CC3000_BUFFER_MAGIC_NUMBER)
  {
    /* You've got problems if you're here! */
    while (1);
  }

  sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
  sSpiInformation.SPIRxHandler(sSpiInformation.pRxPacket + SPI_HEADER_SIZE);
}

void SpiOpen(gcSpiHandleRx pfRxHandler)
{
	sSpiInformation.ulSpiState = eSPI_STATE_POWERUP;
	
	memset(spi_buffer, 0, sizeof(spi_buffer)); 
	memset(wlan_tx_buffer, 0, sizeof(wlan_tx_buffer));

	sSpiInformation.SPIRxHandler      = pfRxHandler;
  	sSpiInformation.usTxPacketLength  = 0;
  	sSpiInformation.pTxPacket         = NULL;
  	sSpiInformation.pRxPacket         = (unsigned char *)spi_buffer;
  	sSpiInformation.usRxPacketLength  = 0;

	spi_buffer[CC3000_RX_BUFFER_SIZE - 1]     = CC3000_BUFFER_MAGIC_NUMBER;
	wlan_tx_buffer[CC3000_TX_BUFFER_SIZE - 1] = CC3000_BUFFER_MAGIC_NUMBER;

	/* Enable interrupt on the GPIO pin of WLAN IRQ */
	tSLInformation.WlanInterruptEnable();

}//End SpiOpen

long SpiFirstWrite(unsigned char *ucBuf, unsigned short usLength)
{ 
  /* Workaround for the first transaction */
  CC3000_ASSERT_CS;

  /* delay (stay low) for ~50us */
  _delay_us(50);
  //_delay_us(25);

  /* SPI writes first 4 bytes of data */
  SpiWriteDataSynchronous(ucBuf, 4);

  _delay_us(50);
  //_delay_us(25);

  SpiWriteDataSynchronous(ucBuf + 4, usLength - 4);

  /* From this point on - operate in a regular manner */
  sSpiInformation.ulSpiState = eSPI_STATE_IDLE;

  CC3000_DEASSERT_CS;

  return(0);
}//End SpiFirstWrite


//*********************************************************************************************************
void SpiRead()
{
}//End SpiRead
//*********************************************************************************************************

///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
///&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
///CHECK THIS CODE:
///&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
long SpiWrite(unsigned char *pUserBuffer, unsigned short usLength)
{
  unsigned char ucPad = 0;
  
  /* Figure out the total length of the packet in order to figure out if there is padding or not */
  if(!(usLength & 0x0001))		{ucPad++;}

  pUserBuffer[0] = WRITE;
  pUserBuffer[1] = HI(usLength + ucPad);
  pUserBuffer[2] = LO(usLength + ucPad);
  pUserBuffer[3] = 0;
  pUserBuffer[4] = 0;

  usLength += (SPI_HEADER_SIZE + ucPad);

//PORTB |= (1 << PB4);
//PORTD |= (1 << PD6) | (1 << PD7);

  /* The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
   * for the purpose of overrun detection. If the magic number is overwritten - buffer overrun
   * occurred - and we will be stuck here forever! */
  if (wlan_tx_buffer[CC3000_TX_BUFFER_SIZE - 1] != CC3000_BUFFER_MAGIC_NUMBER)		{while (1);}

  //PORTB &= ~(1 << PB4);
  //PORTD &= ~((1 << PD6) | (1 << PD7));



  if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP){while (sSpiInformation.ulSpiState != eSPI_STATE_INITIALIZED);}

 //PORTB &= ~(1 << PB4);
 //PORTD &= ~((1 << PD6) | (1 << PD7));


  if (sSpiInformation.ulSpiState == eSPI_STATE_INITIALIZED)
  {
//PORTB &= ~(1 << PB4);
	/* This is time for first TX/RX transactions over SPI: the IRQ is down - so need to send read buffer size command */
    SpiFirstWrite(pUserBuffer, usLength);
  }
  else
  {
//PORTD &= ~(1 << PD6);
    /* We need to prevent here race that can occur in case two back to back packets are sent to the
     * device, so the state will move to IDLE and once again to not IDLE due to IRQ */
//POSSIBLE SOURCE OF ERROR:
	//EIMSK &= ~(1 << INT0);	//No, John, No.
	tSLInformation.WlanInterruptDisable();

    while (sSpiInformation.ulSpiState != eSPI_STATE_IDLE);

//PORTD &= ~(1 << PD7);

    sSpiInformation.ulSpiState = eSPI_STATE_WRITE_IRQ;
    sSpiInformation.pTxPacket = pUserBuffer;
    sSpiInformation.usTxPacketLength = usLength;

    /* Assert the CS line and wait till SSI IRQ line is active and then initialize write operation */
    CC3000_ASSERT_CS;

    /* Re-enable IRQ - if it was not disabled - this is not a problem... */
   // EIMSK |= (1 << INT0);
    tSLInformation.WlanInterruptEnable();

    /* Check for a missing interrupt between the CS assertion and enabling back the interrupts */
//if ((PIND & (1 << PIND0)) == 0)
if (bit_is_clear(PIND, PIND0))
    {
      SpiWriteDataSynchronous(sSpiInformation.pTxPacket, sSpiInformation.usTxPacketLength);

      sSpiInformation.ulSpiState = eSPI_STATE_IDLE;

      CC3000_DEASSERT_CS;
    }
  }

  //PORTB &= ~(1 << PB4);
  //PORTD &= ~((1 << PD6) | (1 << PD7));

/////////ERROR HERE!!!!!
//======================================
  /* Due to the fact that we are currently implementing a blocking situation
   * here we will wait till end of transaction */
  while (eSPI_STATE_IDLE != sSpiInformation.ulSpiState);
//======================================

	//PORTB |= (1 << PB4);
	//PORTD |= (1 << PD7) | (1 << PD6);

  return(0);
}//End SpiWrite
//*********************************************************************************************************
void SpiClose()
{
	if (sSpiInformation.pRxPacket)	{sSpiInformation.pRxPacket = 0;}

	/*  Disable Interrupt in GPIOA module... */
	tSLInformation.WlanInterruptDisable();
}//End SpiClose
//*********************************************************************************************************
void SpiResumeSpi()
{
  	ccspi_int_enabled = 1;
	tSLInformation.WlanInterruptEnable();
}//End SpiResumeSpi

void SpiWriteDataSynchronous(unsigned char *data, unsigned short size)
{
  	unsigned char dummy;

  	unsigned short loc;

 	for (loc = 0; loc < size; loc ++) 		{dummy = SPI_SPITransmit(data[loc]);}
}//End SpiWriteDataSynchronous

//*********************************************************************************************************

void SpiReadDataSynchronous(unsigned char *data, unsigned short size)
{
  	unsigned short i = 0;
  
	for (i = 0; i < size; i ++)		{data[i] = SPI_SPITransmit(0x03);}
}//End SpiReadDataSynchronous

//**********************************************************************************************************

void SpiReadHeader(void)		
{
	SpiReadDataSynchronous(sSpiInformation.pRxPacket, 10);
}//End SpiReadHeader

inline init_spi(void)
{
	/* Set POWER_EN pin to output and disable the CC3000 by default */
  	DDRB |= (1 << DDB5);
  	PORTB &= ~(1 << PORTB5);
  	_delay_ms(500);

  	/* Set CS pin to output (don't de-assert yet) */
  	DDRB |= (1 << DDB0);
	CC3000_DEASSERT_CS;

  	/* Set interrupt/gpio pin to input */
	MCUCR &= ~(1 << PUD);	// w/ weak pullup.  
	DDRD  &= ~(1 << DDD0);
	PORTD |= (1 << PORTD0); //Set pullup resistor.

 	//Set which pins will be outputs:
        DDRB |= (1 << DDB1) | (1 << DDB2);


///////////////////////  SpiConfigStoreOld(); // prime ccspi_old* values for DEASSERT

  	/* Initialise SPI (Mode 1) */
 	//Enable SPI & Set the communication with MSB & Set the Sigmux as master & sample on the trailing edge:
        SPCR = (1 << SPE) | (0 << DORD) | (1 << MSTR) | (1 << CPHA);
        //**********************************************************************************************
	
//SPSR = (1 << SPI2X);

////////////////////////SpiConfigStoreMy(); // prime ccspi_my* values for ASSERT

  	// Newly-initialized SPI is in the same state that ASSERT_CS will set it
  	// to.  Invoke DEASSERT (which also restores SPI registers) so the next
  	// ASSERT call won't clobber the ccspi_old* values -- we need those!
  	CC3000_DEASSERT_CS;

  /* ToDo: Configure IRQ interrupt! */
  //return(ESUCCESS);
}//End init_spi

long SpiReadDataCont(void)
{
  long data_to_recv;
  unsigned char *evnt_buff, type;

  /* Determine what type of packet we have */
  evnt_buff =  sSpiInformation.pRxPacket;
  data_to_recv = 0;
  STREAM_TO_UINT8((uint8_t *)(evnt_buff + SPI_HEADER_SIZE), HCI_PACKET_TYPE_OFFSET, type);

  switch(type)
  {
    case HCI_TYPE_DATA:
      {
        /* We need to read the rest of data.. */
        STREAM_TO_UINT16((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_DATA_LENGTH_OFFSET, data_to_recv);
        if (!((HEADERS_SIZE_EVNT + data_to_recv) & 1))
        {
          data_to_recv++;
        }

        if (data_to_recv)
        {
          SpiReadDataSynchronous(evnt_buff + 10, data_to_recv);
        }
        break;
      }
    case HCI_TYPE_EVNT:
      {
        /* Calculate the rest length of the data */
        STREAM_TO_UINT8((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_EVENT_LENGTH_OFFSET, data_to_recv);
        data_to_recv -= 1;

        /* Add padding byte if needed */
        if ((HEADERS_SIZE_EVNT + data_to_recv) & 1)
        {
          data_to_recv++;
        }

        if (data_to_recv)
        {
          SpiReadDataSynchronous(evnt_buff + 10, data_to_recv);
        }

        sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT;
        break;
      }
  }

  return (0);
}//END SpiReadDataCont

void SSIContReadOperation(void)
{  
  /* The header was read - continue with  the payload read */
  if (!SpiReadDataCont())
  {
    /* All the data was read - finalize handling by switching to teh task
     *  and calling from task Event Handler */
    SpiTriggerRxProcessing();
  }
}//End SSIContReadOperation

void SPI_IRQ(void)
{
  ccspi_is_in_irq = 1;
   
  if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
  {
    /* IRQ line was low ... perform a callback on the HCI Layer */
    sSpiInformation.ulSpiState = eSPI_STATE_INITIALIZED;
  }
  else if (sSpiInformation.ulSpiState == eSPI_STATE_IDLE)
  {
    //DEBUGPRINT_F("IDLE\n\r");
    sSpiInformation.ulSpiState = eSPI_STATE_READ_IRQ;    
    /* IRQ line goes down - start reception */

    CC3000_ASSERT_CS;

    // Wait for TX/RX Compete which will come as DMA interrupt
    SpiReadHeader();
    sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT;
    SSIContReadOperation();
  }
  else if (sSpiInformation.ulSpiState == eSPI_STATE_WRITE_IRQ)
  {
    SpiWriteDataSynchronous(sSpiInformation.pTxPacket, sSpiInformation.usTxPacketLength);
    sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
    CC3000_DEASSERT_CS;
  }
 
 ccspi_is_in_irq = 0;
  return;

}//End SPI_IRQ

//*****************************************************************************
//
//!  cc3k_int_poll
//!
//!  \brief               checks if the interrupt pin is low
//!                       just in case the hardware missed a falling edge
//!                       function is in ccspi.cpp
//
//*****************************************************************************
/*
void cc3k_int_poll()
{
  if (digitalRead(g_irqPin) == LOW && ccspi_is_in_irq == 0 && ccspi_int_enabled != 0) {
    SPI_IRQ();
  }
}
*/
