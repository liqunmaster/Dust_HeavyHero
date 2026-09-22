#pragma once

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "double_buffer.hpp"
#include "imu.hpp"

struct icm42688phxy_config {
    const struct device *spi_device;
    const struct device *gpio_device;
    gpio_pin_t chip_select_pin;
    gpio_pin_t data_ready_pin;
    uint32_t spi_frequency;
};

class icm42688phxy : public ImuDevice {
public:
    int init(const icm42688phxy_config &config);

private:
    static void worker_entry(void *instance, void *unused1, void *unused2);
    static void data_ready_callback(void *user_data);
    int sample_once();
    int read_registers(uint8_t address, uint8_t *data, size_t length);
    int write_register(uint8_t address, uint8_t value);

    bsp_spi_t spi_{};
    struct bsp_gpio_irq data_ready_gpio_{};
    struct k_sem data_ready_sem_{};
    struct k_thread worker_{};
    K_KERNEL_STACK_MEMBER(worker_stack_, 2048);
    uint8_t raw_storage_[2][14]{};
    double_buffer_t raw_buffer_{};
};
