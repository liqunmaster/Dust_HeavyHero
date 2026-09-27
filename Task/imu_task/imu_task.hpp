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
#include "icm42688p_hxy.hpp"
#include "icm42688p_hxy_reg.h"

using imu_sample = icm42688phxy_sample;

int imu_init();

int imu_task_get_sample(imu_sample &sample);

uint32_t imu_task_feedback_count();
