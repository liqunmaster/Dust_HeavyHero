#pragma once

#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stddef.h>
#include <stdarg.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include "ring_buffer.hpp"

int bsp_uart_init(const struct device *device = nullptr);

int bsp_uart_receive(void *data, size_t length);

int bsp_uart_transmit(const void *data, size_t length);
