// SX1272 version
// Differs from SX1276 in AGCautoOn and RxCrcPayloadOn location
#define SX1272

#include "lora.h"

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LR_OCP				   0X0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_PKT_SNR_VALUE        0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_RX_TIMEOUT_LSB		   0x1F
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PaDac				      0x4d

// modes
#ifdef SX1272
 #define MODE_LONG_RANGE_MODE     0x80
#else
 #define MODE_LONG_RANGE_MODE     0x80
#endif
#define MODE_HF						0x08
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06
#define MAX_PKT_LENGTH           255

// GPIO pins
#define RST		24
#define RX_TX	23
#define LED		26
#define MOSI 	10
#define MISO   9
#define SCCK   11
#define CEO		8

// Settings
#define BAND 868.E6
#define spreadingFactor 7
#define SignalBandwidth 41.7E3
#define preambleLength 12
#define codingRateDenominator 7
#define TxPower  14
#define RxTimeoutTics  0xFF

#define SPI_BUF 8
uint8_t tx_buffer[SPI_BUF];
uint8_t rx_buffer[SPI_BUF];
static spi_config_t spi_config;
static long _frequency = 0;
static int _once = 0 ;
static int _implicitHeaderMode = 0;
static int _packetindex = 0 ;

int spifd ;   // spi device

uint8_t singleTransfer(uint8_t address, uint8_t value)
{

  tx_buffer[0] = address ;
  tx_buffer[1] = value ;
  rx_buffer[0] = 0 ;
  rx_buffer[1] = 0 ;
  spi_xfer(spifd,tx_buffer,2,rx_buffer,2);

  return rx_buffer[1] ;
}

uint8_t readRegister(uint8_t address)
{
  return singleTransfer(address & 0x7f, 0x00);
}

void writeRegister(uint8_t address, uint8_t value)
{
  singleTransfer(address | 0x80, value);
  usleep(1000) ;
}


// LoRa functions

void setTxPower(int8_t power, int8_t outputPin)
{
	  uint8_t paConfig = 0;
	  uint8_t paDac = 0;

	  paConfig = readRegister( REG_PA_CONFIG );
	  paDac = readRegister( REG_PaDac );

	  paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | outputPin;
	  paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

	  if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
	  {
	    if( power > 17 )
	    {
	      paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
	    }
	    else
	    {
	      paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
	    }
	    if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
	    {
	      if( power < 5 )
	      {
	        power = 5;
	      }
	      if( power > 20 )
	      {
	        power = 20;
	      }
	      paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
	    }
	    else
	    {
	      if( power < 2 )
	      {
	        power = 2;
	      }
	      if( power > 17 )
	      {
	        power = 17;
	      }
	      paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
	    }
	  }
	  else
	  {
	    if( power < -1 )
	    {
	      power = -1;
	    }
	    if( power > 14 )
	    {
	      power = 14;
	    }
	    paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
	  }

	  writeRegister( REG_PA_CONFIG, paConfig );
	  writeRegister( REG_PaDac, paDac );
//     writeRegister(REG_LR_OCP,0x2f);    //120 mA limit only for +20
}

void setSyncWord(int sw)
{
  writeRegister(REG_SYNC_WORD, sw);
}

void enableCrc()
{
#ifdef SX1272
  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x02);
#else
// SX1276
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
#endif
}

void disableCrc()
{
#ifdef SX1272
  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfd);
#else
// SX1276
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
#endif
}

void setTxPowerMax(int level)
{
	if (level < 5)		{
		level = 5;
	}
	else if(level > 20)	{
		level = 20;
	}
	writeRegister(REG_LR_OCP,0x3f);
	writeRegister(REG_PaDac,0x87);//Open PA_BOOST
	writeRegister(REG_PA_CONFIG, RF_PACONFIG_PASELECT_PABOOST | (level - 5));
}

void setFrequency(long frequency)
{ _frequency = frequency ;
  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
  writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

int packetRssi()
{
	int8_t snr=0;
   int8_t SnrValue = readRegister( 0x19 );
   int16_t rssi = readRegister(REG_PKT_RSSI_VALUE);

	if( SnrValue & 0x80 ) // The SNR sign bit is 1
	{
		// Invert and divide by 4
		snr = ( ( ~SnrValue + 1 ) & 0xFF ) >> 2;
		snr = -snr;
	}
	else
	{
		// Divide by 4
		snr = ( SnrValue & 0xFF ) >> 2;
	}
    if(snr<0)
    {
    	rssi = rssi - (_frequency < 525E6 ? 164 : 157) + ( rssi >> 4 ) + snr;
    }
    else
    {
    	rssi = rssi - (_frequency < 525E6 ? 164 : 157) + ( rssi >> 4 );
    }

  return ( rssi );
}

int packetSnr()
{
  return (((int8_t)readRegister(REG_PKT_SNR_VALUE) + 2) >> 2);
}

void setSpreadingFactor(int sf)
{
  if (sf < 6) {
  	sf = 6; 
	}
  else if (sf > 12) {
  	sf = 12; 
  	}
  if (sf == 6) {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
  }
  writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

void dumpRegisters()
{
  for (int i = 0; i < 128/16; i++)  
  { printf("Reg%02x", i << 4) ;
    for(int j = 0; j < 16; j++)
      printf(" %02x ", readRegister((i << 4) + j));
    printf("\n") ;
  }
}


void idlemode()
{
  writeRegister(REG_OP_MODE, MODE_HF | MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void sleepmode()
{
  writeRegister(REG_OP_MODE, MODE_HF | MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void setSignalBandwidth(long sbw)
{
  int bw;
#ifdef SX1272
  if (sbw <= 125E3) { bw = 0; }
  else if (sbw <= 250E3) { bw = 1; }
  else if (sbw <= 500.8E3) { bw = 2; }
  else { bw = 0 ; }
#else
  if (sbw <= 7.8E3) { bw = 0; }
  else if (sbw <= 10.4E3) { bw = 1; }
  else if (sbw <= 15.6E3) { bw = 2; }
  else if (sbw <= 20.8E3) { bw = 3; }
  else if (sbw <= 31.25E3) { bw = 4; }
  else if (sbw <= 41.7E3) { bw = 5; }
  else if (sbw <= 62.5E3) { bw = 6; }
  else if (sbw <= 125E3) { bw = 7; }
  else if (sbw <= 250E3) { bw = 8; }
  else /*if (sbw <= 250E3)*/ { bw = 9; }
#endif

#ifdef SX1272
  writeRegister(REG_MODEM_CONFIG_1,(readRegister(REG_MODEM_CONFIG_1) & 0x3f) | (bw << 6));
#else
  writeRegister(REG_MODEM_CONFIG_1,(readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
#endif
}

void setCodingRate4(int denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }
  int cr = denominator - 4;
#ifdef SX1272
  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xc7) | (cr << 3));
#else
  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
#endif
}

void setPreambleLength(long length)
{
  writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void setRxTimeout(int tics)
{
  writeRegister(REG_RX_TIMEOUT_LSB, (uint8_t)(tics));
}
void explicitHeaderMode()
{
  _implicitHeaderMode = 0;
#ifdef SX1272
  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfb);
#else
  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
#endif
}

void implicitHeaderMode()
{
  _implicitHeaderMode = 1;
#ifdef SX1272
  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0x04);
#else
  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
#endif
}

int RxSeqDetect(int mask)
{
  int packetLength = 0;
  int irqFlags ;

 // clear IRQ's once
  if (_once == 0)
   { _once = 1 ;
     explicitHeaderMode();
     writeRegister(REG_IRQ_FLAGS, 0xff);
     if (readRegister(REG_OP_MODE) != (MODE_HF | MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS))
     {
      // not currently in RX mode
      // reset FIFO address
      writeRegister(REG_FIFO_ADDR_PTR, 0);
      // put in single RX mode
      writeRegister(REG_OP_MODE, MODE_HF | MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
     }
   }
  if (((irqFlags = readRegister(REG_IRQ_FLAGS))& mask) == mask)
   {
    idlemode();
    _once = 0 ;
   }
  return irqFlags;
}

int beginPacket()
{
  // put in standby mode
  idlemode();
  explicitHeaderMode();
  // reset FIFO address and paload length
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  writeRegister(REG_PAYLOAD_LENGTH, 0);
  return 1;
}

int endPacket()
{
// Set RF switch to TX
#ifdef SX1272
  digitalWrite(RX_TX, LOW);
#endif
  // put in TX mode
  writeRegister(REG_OP_MODE, MODE_HF | MODE_LONG_RANGE_MODE | MODE_TX);
  // wait for TX done
  while((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)
     delay(5);
  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
// Set RF switch to RX
#ifdef SX1272
  digitalWrite(RX_TX, HIGH);
#endif
  return 1;
}

int writebuf(const uint8_t *buffer, int size)
{
  int currentLength = readRegister(REG_PAYLOAD_LENGTH);
  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }
  // write data
  for (size_t i = 0; i < size; i++) {
    writeRegister(REG_FIFO, buffer[i]);
  }
  // update length
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);
  return size;
}

int writebyte(uint8_t byte)
{
  return writebuf(&byte, sizeof(byte));
}

int available()
{
  return (readRegister(REG_RX_NB_BYTES) - _packetindex);
}

int readbyte()
{
  if (!available()) {
  	return -1; 
	}
  _packetindex++;
  return readRegister(REG_FIFO);
}

int peek()
{
  if (!available()) {
  	return -1; 
	}
  // store current FIFO address
  int currentAddress = readRegister(REG_FIFO_ADDR_PTR);
  // read
  uint8_t b = readRegister(REG_FIFO);
  // restore FIFO address
  writeRegister(REG_FIFO_ADDR_PTR, currentAddress);
  return b;
}

// Receive a packet without blocking. Explicit mode only
int RxPak_nb()
{
  int packetLength = 0;
  int irqFlags ;

 // clear IRQ's once
  if (_once == 0)
   { _once = 1 ;
     explicitHeaderMode();
     writeRegister(REG_IRQ_FLAGS, 0xff);
      // reset FIFO address
     writeRegister(REG_FIFO_ADDR_PTR, 0);
      // put in continuous RX mode
     writeRegister(REG_OP_MODE, MODE_HF | MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
   }
  if (((irqFlags = readRegister(REG_IRQ_FLAGS)) &
      (IRQ_RX_DONE_MASK | IRQ_VALID_HEADER_MASK)) ==
      (IRQ_RX_DONE_MASK | IRQ_VALID_HEADER_MASK))
   {
 // received a packet
    _packetindex = 0;
    packetLength = readRegister(REG_RX_NB_BYTES);
    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));
    idlemode();
    _once = 0 ;
   }
  return packetLength;
}

void cancelRx_nb()
{
  if (_once != 0)
   {
     _once = 0 ;
     idlemode() ;
   }
}

// Reset the SX1272
void resetSX()
{
  digitalWrite(RST, HIGH);
  delay(5) ;
  digitalWrite(RST, LOW);
#ifdef SX1272
  digitalWrite(RX_TX, HIGH);   //receive
#endif
  delay(5) ; ;
}

// For the SX1752 set PABOOST is complusory as RFI not connected
// unlike the HelTec SX1726
void init_lora(long frequency,bool PABOOST)
{
  resetSX() ;
// put SX1752 in sleep mode
  sleepmode() ;
  usleep(5000) ;
  // Disable DIO as enabled by reset
  writeRegister(REG_DIO_MAPPING_1, 0xff);
  // set frequency
  setFrequency(frequency);
  // set base addresses
  writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
  writeRegister(REG_FIFO_RX_BASE_ADDR, 0);
  // set LNA boost
  writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);
  // set auto AGC
#ifdef SX1272
   writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
#else
// SX1276
    writeRegister(REG_MODEM_CONFIG_3, 0x04);
#endif
  // set output power to 17 dBm
  if(PABOOST == true)
	  setTxPower(14, RF_PACONFIG_PASELECT_PABOOST);
  else
	  setTxPower(17, RF_PACONFIG_PASELECT_RFO);
  setSyncWord(0x34);
  disableCrc();
  idlemode();
  delay(10) ;
}

