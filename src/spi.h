

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40
#define IRQ_CD_DET_MASK            0x1
#define IRQ_CD_DONE_MASK           0x4
#define IRQ_VALID_HEADER_MASK      0x10

/*!
 * RegPaConfig
 */
#define RF_PACONFIG_PASELECT_MASK                   0x7F
#define RF_PACONFIG_PASELECT_PABOOST                0x80
#define RF_PACONFIG_PASELECT_RFO                    0x00 // Default

#define RF_PACONFIG_MAX_POWER_MASK                  0x8F

#define RF_PACONFIG_OUTPUTPOWER_MASK                0xF0

/*!
 * RegPaDac
 */
#define RF_PADAC_20DBM_MASK                         0xF8
#define RF_PADAC_20DBM_ON                           0x07
#define RF_PADAC_20DBM_OFF                          0x04  // Default

typedef struct {
    uint8_t mode;
    uint8_t bits_per_word;
    uint32_t speed;
    uint16_t delay;
} spi_config_t;

int spi_open(char *device, spi_config_t config);
int spi_close(int fd);
int spi_xfer(int fd, uint8_t *tx_buffer, uint8_t tx_len,
             uint8_t *rx_buffer, uint8_t rx_len);
int spi_read(int fd, uint8_t *rx_buffer, uint8_t rx_len);
int spi_write(int fd, uint8_t *tx_buffer, uint8_t tx_len);

