#pragma once

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include "bsp_uart.hpp"


#ifdef CONFIG_REMOTE_DEVICE_DT7
#include "dt7.hpp"
using remote_sample = dt7_sample;

#elif CONFIG_REMOTE_DEVICE_VT02
#error "VT02 decoder is not implemented yet"

#elif CONFIG_REMOTE_DEVICE_VT03
#include "vt03.hpp"
using remote_sample = vt03_sample;
#endif

int remote_init();
int remote_task_get_sample(remote_sample &sample);

uint32_t remote_task_feedback_count();
