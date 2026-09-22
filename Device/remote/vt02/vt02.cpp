#include "vt02.hpp"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include "remote_crc.hpp"

static constexpr size_t kFrameSize = 21U;
static int16_t read_i16(const uint8_t *data)
{
    return static_cast<int16_t>(uint16_t(data[0]) | (uint16_t(data[1]) << 8));
}
static bool validate(const uint8_t *data, size_t length)
{
    return data != nullptr && length == kFrameSize && data[0] == 0xa5U &&
           remote_crc16_valid(data, length);
}
static bool decode(const uint8_t *data, size_t length, RemoteSample &sample)
{
    if (!validate(data, length)) return false;
    for (uint16_t &value : sample.channel) value = 1024U;
    sample.switch_left = 1U;
    sample.switch_right = 1U;
    sample.mouse_x = read_i16(data + 7);
    sample.mouse_y = read_i16(data + 9);
    sample.mouse_z = read_i16(data + 11);
    sample.mouse_left = data[13];
    sample.mouse_right = data[14];
    sample.keyboard = uint16_t(data[15]) | (uint16_t(data[16]) << 8);
    sample.wheel = read_i16(data + 17);
    sample.type = RemoteType::Vt02;
    sample.timestamp_ms = k_uptime_get_32();
    return true;
}
const RemoteProtocol &vt02_protocol()
{
    static const RemoteProtocol protocol = {
        .type = RemoteType::Vt02,
        .uart_device = DEVICE_DT_GET(DT_NODELABEL(uart2)),
        .uart_config = {.baudrate = 921600U, .parity = UART_CFG_PARITY_NONE,
                        .stop_bits = UART_CFG_STOP_BITS_1,
                        .data_bits = UART_CFG_DATA_BITS_8,
                        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE},
        .frame_size = kFrameSize, .validate = validate, .decode = decode};
    return protocol;
}
