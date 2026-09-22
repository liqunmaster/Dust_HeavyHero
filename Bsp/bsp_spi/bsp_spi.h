#pragma once

#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const struct device *device;
    const struct device *cs_port;
    gpio_pin_t cs_pin;
    struct spi_config config;
    struct k_mutex mutex;
    bool initialized;
} bsp_spi_t;

int bsp_spi_init(bsp_spi_t *spi, const struct device *device,
                 const struct device *cs_port, gpio_pin_t cs_pin,
                 uint32_t frequency, spi_operation_t operation);
int bsp_spi_transceive(bsp_spi_t *spi, const uint8_t *tx, uint8_t *rx,
                       size_t length);
int bsp_spi_write(bsp_spi_t *spi, const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif
