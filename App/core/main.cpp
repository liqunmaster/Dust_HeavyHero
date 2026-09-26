#include <zephyr/kernel.h>

#include "app_shoot.hpp"

extern "C" {
volatile int g_can_init_result;
volatile int g_can_send_result;
}

int main(void)
{
    g_can_init_result = app_shoot_init();
    if (g_can_init_result != 0) {
        while (1) {
            k_sleep(K_MSEC(100));
        }
    }

    while (1) {
        g_can_send_result = app_shoot_run();
        k_sleep(K_MSEC(100));
    }
}
