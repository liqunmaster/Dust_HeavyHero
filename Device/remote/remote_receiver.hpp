#pragma once
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "bsp_uart.h"
#include "ring_buffer.hpp"
#include "remote.hpp"

class RemoteReceiver : public RemoteDevice {
public:
    int init(const RemoteProtocol &protocol);
    int start();
    bsp_uart_t *uart() { return &uart_; }
private:
    static void rx_bytes(const uint8_t *, size_t, void *);
    static void worker_entry(void *, void *, void *);
    void process_byte(uint8_t);
    RemoteProtocol protocol_{};
    bsp_uart_t uart_{};
    uint8_t frame_[64]{};
    size_t frame_length_{};
    ring_buffer_t byte_ring_{};
    uint8_t byte_ring_storage_[256]{};
    struct k_sem data_ready_{};
    atomic_t rx_overrun_{};
    struct k_thread worker_{};
    K_KERNEL_STACK_MEMBER(worker_stack_, 1536);
    bool initialized_{};
    bool started_{};
};
