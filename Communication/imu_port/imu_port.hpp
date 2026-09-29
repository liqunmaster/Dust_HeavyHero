#pragma once

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

#include "bsp_gpio.h"
#include "bsp_spi.hpp"
#include "bsp_uart.hpp"
#include "icm42688p_hxy.hpp"
#include "icm42688p_hxy_reg.h"

using imu_sample = icm42688phxy_sample;

int imu_port_init();

int imu_port_get_sample(imu_sample &sample);

uint32_t imu_port_feedback_count();
