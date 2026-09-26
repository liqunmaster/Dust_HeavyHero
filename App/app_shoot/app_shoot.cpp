#include "app_shoot.hpp"

#include <stdint.h>

#include "c620.hpp"

extern "C" {
volatile int g_can_bus_state = -1;
volatile uint8_t g_can_tx_error_count;
volatile uint8_t g_can_rx_error_count;
volatile uint32_t g_can_tx_queued;
volatile uint32_t g_can_tx_completed;
volatile uint32_t g_can_tx_dropped;
volatile uint32_t g_can_tx_errors;
volatile uint32_t g_can_rx_received;
volatile uint32_t g_can_tx_pending;
volatile uint32_t g_can_run_count;
}

namespace {
constexpr fdcan_device kShootBus = FDCAN_DEVICE_CAN1;

c620 shoot_motors[4];

constexpr C620_ID shoot_ids[4] = {
    C620_ID_0x201,
    C620_ID_0x202,
    C620_ID_0x203,
    C620_ID_0x204,
};
}

int app_shoot_init(void)
{
    for (uint32_t motor = 0U; motor < 4U; ++motor) {
        const int ret = shoot_motors[motor].init(kShootBus, shoot_ids[motor]);
        if (ret != 0) {
            return ret;
        }
    }

    for (uint32_t motor = 0U; motor < 4U; ++motor) {
        const int ret = shoot_motors[motor].set_current(10.0F);
        if (ret != 0) {
            return ret;
        }
    }
    return 0;
}

int app_shoot_run(void)
{
    ++g_can_run_count;
    fdcan_bus_state state{};
    fdcan_error_count errors{};
    const int state_result = bsp_fdcan_get_state(kShootBus, &state, &errors);
    if (state_result != 0) {
        return state_result;
    }
    g_can_bus_state = state;
    g_can_tx_error_count = errors.transmit;
    g_can_rx_error_count = errors.receive;

    fdcan_statistics statistics{};
    bsp_fdcan_get_statistics(kShootBus, &statistics);
    g_can_tx_queued = statistics.tx_queued;
    g_can_tx_completed = statistics.tx_completed;
    g_can_tx_dropped = statistics.tx_dropped;
    g_can_tx_errors = statistics.tx_errors;
    g_can_rx_received = statistics.rx_received;
    g_can_tx_pending = bsp_fdcan_tx_pending(kShootBus);

    if (state == FDCAN_BUS_OFF) {
        return bsp_fdcan_recover(kShootBus, FDCAN_NO_WAIT);
    }

    int first_error = 0;

    for (uint32_t motor = 0U; motor < 4U; ++motor) {
        const int ret = shoot_motors[motor].set_current(10.0F);
        if (ret != 0 && first_error == 0) {
            first_error = ret;
        }
    }

    const int ret = shoot_motors[0].transmit();
    if (ret != 0 && first_error == 0) {
        first_error = ret;
    }

    return first_error;
}
