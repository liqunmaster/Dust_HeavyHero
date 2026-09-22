#include "dt7.hpp"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

static constexpr uint16_t kMin = 364U;
static constexpr uint16_t kMax = 1684U;

static uint16_t channel(const uint8_t *d, uint8_t index)
{
    switch (index) {
    case 0: return (d[0] | (uint16_t(d[1]) << 8)) & 0x07ffU;
    case 1: return ((d[1] >> 3) | (uint16_t(d[2]) << 5)) & 0x07ffU;
    case 2: return ((d[2] >> 6) | (uint16_t(d[3]) << 2) |
                    (uint16_t(d[4]) << 10)) & 0x07ffU;
    case 3: return ((d[4] >> 1) | (uint16_t(d[5]) << 7)) & 0x07ffU;
    default: return 1024U;
    }
}

static bool validate(const uint8_t *frame, size_t length)
{
    if (frame == nullptr || length != 18U) return false;
    for (uint8_t i = 0; i < 4; ++i) {
        const uint16_t value = channel(frame, i);
        if (value < kMin || value > kMax) return false;
    }
    const uint8_t packed = frame[5] >> 4;
    const uint8_t left = (packed >> 2) & 3U;
    const uint8_t right = packed & 3U;
    return left >= 1U && left <= 3U && right >= 1U && right <= 3U;
}

static bool decode(const uint8_t *frame, size_t length, RemoteSample &sample)
{
    if (!validate(frame, length)) return false;
    for (uint8_t i = 0; i < 4; ++i) sample.channel[i] = channel(frame, i);
    const uint8_t packed = frame[5] >> 4;
    sample.switch_left = (packed >> 2) & 3U;
    sample.switch_right = packed & 3U;
    sample.type = RemoteType::Dt7;
    sample.timestamp_ms = k_uptime_get_32();
    return true;
}

const RemoteProtocol &dt7_protocol()
{
    static const RemoteProtocol protocol = {
        .type = RemoteType::Dt7,
        .uart_device = DEVICE_DT_GET(DT_NODELABEL(uart3)),
        .uart_config = {.baudrate = 100000U,
                        .parity = UART_CFG_PARITY_EVEN,
                        .stop_bits = UART_CFG_STOP_BITS_1,
                        .data_bits = UART_CFG_DATA_BITS_8,
                        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE},
        .frame_size = 18U, .validate = validate, .decode = decode};
    return protocol;
}
