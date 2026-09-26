#pragma once

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

#include "ring_buffer.hpp"

#define BSP_SPI_BUFFER_SIZE 64U

struct bsp_spi {
    const struct device *device;
    struct spi_config config;
    struct k_mutex lock;
    ring_buffer_t tx_buffer;
    ring_buffer_t rx_buffer;
    uint8_t tx_storage[BSP_SPI_BUFFER_SIZE] __aligned(4);
    uint8_t rx_storage[BSP_SPI_BUFFER_SIZE] __aligned(4);
    bool initialized;
};

int bsp_spi_init(bsp_spi *spi, const struct device *device, uint16_t peripheral, uint32_t frequency, uint16_t operation);

int bsp_spi_write(bsp_spi *spi, const uint8_t *data, size_t length);

int bsp_spi_read(bsp_spi *spi, uint8_t *data, size_t length);

int bsp_spi_transceive(bsp_spi *spi, const uint8_t *tx_data, uint8_t *rx_data, size_t length);

void bsp_spi_clear_buffers(bsp_spi *spi);
