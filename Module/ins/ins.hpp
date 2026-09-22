#pragma once

#include <zephyr/kernel.h>

#include "attitude_estimator.hpp"
#include "double_buffer.hpp"
#include "imu.hpp"

class InsModule {
public:
    int init(ImuDevice &imu);

private:
    static void sample_callback(const ImuSample *sample, void *user_data);
    static void worker_entry(void *instance, void *unused1, void *unused2);
    void process_sample(const ImuSample &sample);

    ImuSample sample_storage_[2]{};
    double_buffer_t sample_buffer_{};
    struct k_sem sample_ready_{};
    struct k_thread worker_{};
    K_KERNEL_STACK_MEMBER(worker_stack_, 2048);
    AttitudeEstimator estimator_{};
    uint32_t last_timestamp_ms_{};
};
