#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPi.h>
#include "spi.h"

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40
#define IRQ_CD_DET_MASK            0x1
#define IRQ_CD_DONE_MASK           0x4
#define IRQ_VALID_HEADER_MASK      0x10

// RegPaConfig
#define RF_PACONFIG_PASELECT_MASK                   0x7F
#define RF_PACONFIG_PASELECT_PABOOST                0x80
#define RF_PACONFIG_PASELECT_RFO                    0x00 // Default
#define RF_PACONFIG_MAX_POWER_MASK                  0x8F
#define RF_PACONFIG_OUTPUTPOWER_MASK                0xF0

// RegPaDac
#define RF_PADAC_20DBM_MASK                         0xF8
#define RF_PADAC_20DBM_ON                           0x07
#define RF_PADAC_20DBM_OFF                          0x04  // Default
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

// GPIO pins
#define RST		24
#define RX_TX	23
#define LED		26
#define MOSI 	10
#define MISO   9
#define SCCK   11
#define CEO		8

// Functions
  void receive(int size);
  void idlemode();
  void sleepmode();
  void setTxPower(int8_t power, int8_t outputPin);
  void setTxPowerMax(int level);
  void setFrequency(long frequency);
  void setSpreadingFactor(int sf);
  void setSignalBandwidth(long sbw);
  void setCodingRate4(int denominator);
  void setRxTimeout(int tics);
  void setPreambleLength(long length);
  void setSyncWord(int sw);
  void enableCrc();
  void disableCrc();
  long getSignalBandwidth();
  long packetFrequencyError();
  int  packetRssi();
  int  packetSnr();
  void explicitHeaderMode();
  int  RxSeqDetect(int mask);
  int  RxPak_nb();
  void cancelRx_nb();
  int available();
  int writebuf(const uint8_t *buffer, int size);
  int writebyte(uint8_t byte);
  int readbyte();
  int peek();
  int beginPacket();
  int endPacket();
  void resetSX();
  void dumpRegisters();
  void init_lora(long frequency,bool PABOOST);


