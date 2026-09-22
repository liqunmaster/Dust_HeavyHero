#include "bsp_spi.h"

#include <errno.h>
#include <string.h>

static int set_chip_select(bsp_spi_t *spi, bool selected)
{
    return gpio_pin_set_raw(spi->cs_port, spi->cs_pin, selected ? 0 : 1);
}

int bsp_spi_init(bsp_spi_t *spi, const struct device *device,
                 const struct device *cs_port, gpio_pin_t cs_pin,
                 uint32_t frequency, spi_operation_t operation)
{
    if (spi == NULL || device == NULL || cs_port == NULL || frequency == 0U ||
        !device_is_ready(device) || !device_is_ready(cs_port)) {
        return -EINVAL;
    }
    int ret = gpio_pin_configure(cs_port, cs_pin,
                                 GPIO_OUTPUT_HIGH | GPIO_PULL_UP);
    if (ret != 0) return ret;
    spi->device = device;
    spi->cs_port = cs_port;
    spi->cs_pin = cs_pin;
    memset(&spi->config, 0, sizeof(spi->config));
    spi->config.frequency = frequency;
    spi->config.operation = SPI_OP_MODE_CONTROLLER | SPI_WORD_SET(8) |
                            operation;
    spi->config.peripheral = 0U;
    k_mutex_init(&spi->mutex);
    spi->initialized = true;
    return 0;
}

int bsp_spi_transceive(bsp_spi_t *spi, const uint8_t *tx, uint8_t *rx,
                       size_t length)
{
    if (spi == NULL || !spi->initialized || tx == NULL || rx == NULL ||
        length == 0U) return -EINVAL;
    const struct spi_buf tx_buffer = {.buf = (void *)tx, .len = length};
    const struct spi_buf rx_buffer = {.buf = rx, .len = length};
    const struct spi_buf_set tx_set = {.buffers = &tx_buffer, .count = 1U};
    const struct spi_buf_set rx_set = {.buffers = &rx_buffer, .count = 1U};
    k_mutex_lock(&spi->mutex, K_FOREVER);
    int ret = set_chip_select(spi, true);
    if (ret == 0) {
        ret = spi_transceive(spi->device, &spi->config, &tx_set, &rx_set);
    }
    (void)set_chip_select(spi, false);
    k_mutex_unlock(&spi->mutex);
    return ret;
}

int bsp_spi_write(bsp_spi_t *spi, const uint8_t *data, size_t length)
{
    if (spi == NULL || !spi->initialized || data == NULL || length == 0U) {
        return -EINVAL;
    }
    const struct spi_buf buffer = {.buf = (void *)data, .len = length};
    const struct spi_buf_set set = {.buffers = &buffer, .count = 1U};
    k_mutex_lock(&spi->mutex, K_FOREVER);
    int ret = set_chip_select(spi, true);
    if (ret == 0) ret = spi_write(spi->device, &spi->config, &set);
    (void)set_chip_select(spi, false);
    k_mutex_unlock(&spi->mutex);
    return ret;
}
