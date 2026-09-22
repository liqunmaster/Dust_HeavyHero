#pragma once

#include <stdint.h>

#include <zephyr/kernel.h>

#include "bsp_fdcan.h"
#include "c620.hpp"

class AppChassis {
public:
    int init(const struct device *can_device);

private:
    static void control_timer_expiry(struct k_timer *timer);
    static void control_thread_entry(void *instance, void *unused1,
                                     void *unused2);
    void control_step();

    fdcan_t can_bus_{};
    DjiMotorBus motor_bus_{};
    C620Motor motors_[4]{};
    struct k_timer control_timer_{};
    struct k_sem control_tick_sem_{};
    struct k_thread control_thread_{};
    K_KERNEL_STACK_MEMBER(control_stack_, 2048);
};
