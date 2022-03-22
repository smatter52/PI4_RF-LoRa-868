
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

