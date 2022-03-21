#define SX1272

#include "lora.h"

// PI GPIO pins
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

//SPI
#define SPI_BUF 8
extern uint8_t tx_buffer[SPI_BUF];
extern uint8_t rx_buffer[SPI_BUF];
spi_config_t spi_config;
extern int spifd ;   // Spi device

//LoRa
static long _frequency = 0;
static int _once = 0 ;
static int _implicitHeaderMode = 0;

// GPIO setup
void init_gpio()
{
  wiringPiSetupGpio() ;
  pinMode(LED, OUTPUT) ;
  pinMode(RST, OUTPUT) ;
  pinMode(RX_TX, OUTPUT) ;

}
// Spi setup
void exit_spi(void)
{  spi_close(spifd); }

void init_spi()
{
  spi_config.mode=0;
  spi_config.speed=1000000;
  spi_config.delay=0;
  spi_config.bits_per_word=8;
  spifd=spi_open("/dev/spidev0.0",spi_config);
  if (spifd == -1)
   { printf("Dev open error\n") ;
     exit(1);
   }
  memset(tx_buffer,0,SPI_BUF);
  memset(rx_buffer,0,SPI_BUF);
  atexit(exit_spi) ;
}


void main(void)
{
    int ad = 0 ;
    static int irqflags ;
    static int packetLength ;
    int rssi, snr ;
    long tcnt = 0 ;
    uint8_t sbuf[32] ;
    init_gpio() ;
    init_spi() ;

    printf("Lora Sender ctrl c to exit\n") ;
    init_lora(BAND, true) ;
    setSpreadingFactor(spreadingFactor);
    setSignalBandwidth(SignalBandwidth);
    setCodingRate4(codingRateDenominator);
    setPreambleLength(preambleLength);
    setRxTimeout(RxTimeoutTics) ;
    setTxPower(TxPower, RF_PACONFIG_PASELECT_PABOOST) ;
    usleep(5000) ;
    dumpRegisters() ;
    while (1)
     {
       sprintf(sbuf,"Hi :%d", tcnt) ;
       beginPacket() ;
       writebuf(sbuf, strlen(sbuf)) ;
       endPacket() ;
       printf("Tx: %ld\n", tcnt) ;
       delay(1000) ;
     }
}
