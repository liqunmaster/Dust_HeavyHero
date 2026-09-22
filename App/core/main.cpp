#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <new>
#include <stdio.h>

#include "app_chassic.hpp"
#include "bsp_uart.h"
#include "icm42688p_hxy.hpp"
#include "input.hpp"
#include "ins.hpp"

alignas(AppChassis) static uint8_t chassis_storage[sizeof(AppChassis)];
alignas(InputModule) static uint8_t input_storage[sizeof(InputModule)];
alignas(icm42688phxy) static uint8_t imu_storage[sizeof(icm42688phxy)];
alignas(InsModule) static uint8_t ins_storage[sizeof(InsModule)];

int main(void)
{
    AppChassis *chassis = new (chassis_storage) AppChassis{};
    InputModule *input = new (input_storage) InputModule{};
    icm42688phxy *imu = new (imu_storage) icm42688phxy{};
    InsModule *ins = new (ins_storage) InsModule{};
    const struct device *can0 = DEVICE_DT_GET(DT_ALIAS(can0));
    int ret = chassis->init(can0);
    if (ret != 0) {
        printf("chassis init failed: %d\n", ret);
        return ret;
    }

    ret = input->init();
    if (ret != 0) printf("input init failed: %d\n", ret);
    if (ret == 0) bsp_uart_set_stdio(input->remote_uart());
    ret = ins->init(*imu);
    if (ret != 0) printf("INS init failed: %d\n", ret);
    const icm42688phxy_config imu_config = {
        .spi_device = DEVICE_DT_GET(DT_NODELABEL(spi2)),
        .gpio_device = DEVICE_DT_GET(DT_NODELABEL(gpiob)),
        .chip_select_pin = 10U,
        .data_ready_pin = 15U,
        .spi_frequency = 2000000U,
    };
    ret = imu->init(imu_config);
    if (ret != 0) printf("ICM42688 init failed: %d\n", ret);

    return 0;
}
