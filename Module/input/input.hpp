#pragma once

#include <zephyr/kernel.h>

#include "double_buffer.hpp"
#include "remote.hpp"
#include "remote_receiver.hpp"

class InputModule {
public:
    int init();
    bsp_uart_t *remote_uart() { return receiver_.uart(); }

private:
    static void sample_callback(const RemoteSample *sample, void *user_data);
    static void worker_entry(void *instance, void *unused1, void *unused2);
    void process_sample(const RemoteSample &sample);

    RemoteSample sample_storage_[2]{};
    double_buffer_t sample_buffer_{};
    struct k_sem sample_ready_{};
    struct k_thread worker_{};
    K_KERNEL_STACK_MEMBER(worker_stack_, 1536);
    RemoteReceiver receiver_{};
};
