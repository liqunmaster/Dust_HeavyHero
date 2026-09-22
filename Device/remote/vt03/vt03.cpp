#include "vt03.hpp"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include "remote_crc.hpp"

struct Vt03Frame {
    uint8_t sof0;
    uint8_t sof1;
    uint64_t channel0 : 11;
    uint64_t channel1 : 11;
    uint64_t channel2 : 11;
    uint64_t channel3 : 11;
    uint64_t cns : 2;
    uint64_t pause : 1;
    uint64_t fn1 : 1;
    uint64_t fn2 : 1;
    uint64_t wheel : 11;
    uint64_t trigger : 1;
    uint64_t reserved0 : 3;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left : 2;
    uint8_t mouse_right : 2;
    uint8_t mouse_middle : 2;
    uint8_t reserved1 : 2;
    uint16_t keyboard;
    uint16_t crc16;
} __packed;
static_assert(sizeof(Vt03Frame) == 21U);

static bool validate(const uint8_t *data, size_t length)
{
    return data != nullptr && length == sizeof(Vt03Frame) &&
           data[0] == 0xa9U && data[1] == 0x53U &&
           remote_crc16_valid(data, length);
}
static bool decode(const uint8_t *data, size_t length, RemoteSample &sample)
{
    if (!validate(data, length)) return false;
    const auto *raw = reinterpret_cast<const Vt03Frame *>(data);
    sample.channel[0] = raw->channel0;
    sample.channel[1] = raw->channel1;
    sample.channel[2] = raw->channel2;
    sample.channel[3] = raw->channel3;
    sample.switch_left = uint8_t(raw->cns) + 1U;
    sample.switch_right = raw->pause ? 2U : 1U;
    sample.mouse_x = raw->mouse_x;
    sample.mouse_y = raw->mouse_y;
    sample.mouse_z = raw->mouse_z;
    sample.mouse_left = raw->mouse_left;
    sample.mouse_right = raw->mouse_right;
    sample.keyboard = raw->keyboard;
    sample.wheel = static_cast<int16_t>(raw->wheel) - 1024;
    sample.cns = raw->cns;
    sample.pause = raw->pause;
    sample.fn1 = raw->fn1;
    sample.fn2 = raw->fn2;
    sample.trigger = raw->trigger;
    sample.type = RemoteType::Vt03;
    sample.timestamp_ms = k_uptime_get_32();
    return true;
}
const RemoteProtocol &vt03_protocol()
{
    static const RemoteProtocol protocol = {
        .type = RemoteType::Vt03,
        .uart_device = DEVICE_DT_GET(DT_NODELABEL(uart2)),
        .uart_config = {.baudrate = 921600U, .parity = UART_CFG_PARITY_NONE,
                        .stop_bits = UART_CFG_STOP_BITS_1,
                        .data_bits = UART_CFG_DATA_BITS_8,
                        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE},
        .frame_size = sizeof(Vt03Frame), .validate = validate, .decode = decode};
    return protocol;
}
