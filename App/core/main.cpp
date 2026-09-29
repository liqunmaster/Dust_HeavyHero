#include <zephyr/kernel.h>
#include "bsp_uart.hpp"
#include "remote_port.hpp"
#include "imu_port.hpp"
#include "fdcan_port.hpp"
#include "ws2812b.hpp"

int main(void)
{
    (void)bsp_uart_init();
    (void)fdcan_port_init();
    (void)remote_port_init();
    (void)imu_port_init();
    
    constexpr ws2812b_color colors[] = {
        {32U, 0U, 0U},
        {0U, 32U, 0U},
        {0U, 0U, 32U},
    };
    size_t color_index = 0U;
    uint8_t half_second_ticks = 0U;

    while (1) {
        (void)ws2812b_set_color(colors[color_index]);
        color_index = (color_index + 1U) % 3U;
        if (++half_second_ticks == 2U) {
            half_second_ticks = 0U;
        }
        k_sleep(K_MSEC(1000));
    }
}
