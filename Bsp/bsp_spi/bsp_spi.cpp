#include "bsp_spi.hpp"

namespace {

    int bsp_spi_check_ready(const bsp_spi *spi, const uint8_t *data, size_t length)
    {
        if (spi == nullptr || !spi->initialized || spi->device == nullptr || data == nullptr || length == 0U) {
            return -EINVAL;
        }
        return 0;
    }

    int bsp_spi_transfer_locked(bsp_spi *spi, const uint8_t *tx_data, uint8_t *rx_data, size_t length)
    {
        if (length > BSP_SPI_BUFFER_SIZE) return -EMSGSIZE;

    const size_t queued = ring_buffer_write(&spi->tx_buffer, tx_data, length);
    if (queued != length) {
        ring_buffer_clear(&spi->tx_buffer);
        return -ENOBUFS;
    }

    uint8_t tx[BSP_SPI_BUFFER_SIZE]{};
    uint8_t rx[BSP_SPI_BUFFER_SIZE]{};
    (void)ring_buffer_read(&spi->tx_buffer, tx, length);

    struct spi_buf tx_buf{tx, length};
    struct spi_buf_set tx_set{&tx_buf, 1U};
    struct spi_buf rx_buf{rx, length};
    struct spi_buf_set rx_set{&rx_buf, 1U};
    const int ret = spi_transceive(spi->device, &spi->config, &tx_set, rx_data != nullptr ? &rx_set : nullptr);
    if (ret != 0) return ret;

    if (rx_data != nullptr) {
        if (ring_buffer_write(&spi->rx_buffer, rx, length) != length) {
            ring_buffer_clear(&spi->rx_buffer);
            return -ENOBUFS;
        }
        (void)ring_buffer_read(&spi->rx_buffer, rx_data, length);
    }
    return 0;
}

}

int bsp_spi_init(bsp_spi *spi, const struct device *device, uint16_t peripheral, uint32_t frequency, uint16_t operation)
{
    if (spi == nullptr || device == nullptr || frequency == 0U || !device_is_ready(device)) {
        return -EINVAL;
    }
    if (spi->initialized) {
        return spi->device == device ? 0 : -EBUSY;
    }

    spi->device = device;
    memset(&spi->config, 0, sizeof(spi->config));
    spi->config.frequency = frequency;
    spi->config.operation = operation | SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
    spi->config.peripheral = peripheral;
    k_mutex_init(&spi->lock);
    ring_buffer_init(&spi->tx_buffer, spi->tx_storage, sizeof(spi->tx_storage));
    ring_buffer_init(&spi->rx_buffer, spi->rx_storage, sizeof(spi->rx_storage));
    spi->initialized = true;
    return 0;
}

int bsp_spi_write(bsp_spi *spi, const uint8_t *data, size_t length)
{
    const int valid = bsp_spi_check_ready(spi, data, length);
    if (valid != 0) return valid;
    k_mutex_lock(&spi->lock, K_FOREVER);
    const int ret = bsp_spi_transfer_locked(spi, data, nullptr, length);
    k_mutex_unlock(&spi->lock);
    return ret;
}

int bsp_spi_read(bsp_spi *spi, uint8_t *data, size_t length)
{
    const int valid = bsp_spi_check_ready(spi, data, length);
    if (valid != 0) return valid;
    uint8_t dummy[BSP_SPI_BUFFER_SIZE]{};
    k_mutex_lock(&spi->lock, K_FOREVER);
    const int ret = bsp_spi_transfer_locked(spi, dummy, data, length);
    k_mutex_unlock(&spi->lock);
    return ret;
}

int bsp_spi_transceive(bsp_spi *spi, const uint8_t *tx_data, uint8_t *rx_data, size_t length)
{
    const int valid = bsp_spi_check_ready(spi, tx_data, length);
    if (valid != 0 || rx_data == nullptr) return valid != 0 ? valid : -EINVAL;
    k_mutex_lock(&spi->lock, K_FOREVER);
    const int ret = bsp_spi_transfer_locked(spi, tx_data, rx_data, length);
    k_mutex_unlock(&spi->lock);
    return ret;
}

void bsp_spi_clear_buffers(bsp_spi *spi)
{
    if (spi == nullptr || !spi->initialized) return;
    k_mutex_lock(&spi->lock, K_FOREVER);
    ring_buffer_clear(&spi->tx_buffer);
    ring_buffer_clear(&spi->rx_buffer);
    k_mutex_unlock(&spi->lock);
}
