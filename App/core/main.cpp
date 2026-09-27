#include <zephyr/kernel.h>

#include "remote_task.hpp"
#include "imu_task.hpp"

int main(void)
{
    const int remote_status = remote_init();
    if (remote_status != 0) {
        return remote_status;
    }

    const int imu_status = imu_init();
    if (imu_status != 0) {
        return imu_status;
    }

    while (1) {
        k_sleep(K_MSEC(100));
    }
}
