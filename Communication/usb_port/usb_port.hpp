#pragma once

#include <stddef.h>
#include <stdint.h>

int usb_port_init();
int usb_port_receive(void *data, size_t length);
int usb_port_transmit(const void *data, size_t length);
bool usb_port_connected();

uint32_t usb_port_received_count();
uint32_t usb_port_dropped_count();
