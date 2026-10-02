#pragma once

#include <errno.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "bsp_fdcan.hpp"
#include "c610.hpp"
#include "c620.hpp"
#include "dm_motor.hpp"
#include "fdcan_channel.hpp"

class c610;
class c620;

int fdcan_port_init();

int fdcan_port_bind(c610 &motor);
int fdcan_port_bind(c620 &motor);
int fdcan_port_bind(dm_motor &motor);

int fdcan_port_submit(fdcan_device device, const fdcan_frame &frame);
int fdcan_port_submit(const c610 &motor);
int fdcan_port_submit(const c620 &motor);
int fdcan_port_submit(const dm_motor &motor);

int fdcan_port_send_once(fdcan_device device, const fdcan_frame &frame);
int fdcan_port_request_mode(dm_motor &motor, DmControlMode mode);

uint32_t fdcan_port_received_count(const c610 &motor);
uint32_t fdcan_port_received_count(const c620 &motor);
uint32_t fdcan_port_received_count(const dm_motor &motor);
uint32_t fdcan_port_rx_dropped_count();
uint32_t fdcan_port_tx_error_count();
uint32_t fdcan_port_control_dropped_count();
