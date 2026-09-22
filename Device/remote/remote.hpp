#pragma once

#include <stdint.h>
#include <stddef.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

enum class RemoteType : uint8_t { Dt7, Vt02, Vt03 };

struct RemoteSample {
    uint16_t channel[4];
    uint8_t switch_left;
    uint8_t switch_right;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint16_t keyboard;
    int16_t wheel;
    uint8_t mouse_left;
    uint8_t mouse_right;
    uint8_t cns;
    uint8_t pause;
    uint8_t fn1;
    uint8_t fn2;
    uint8_t trigger;
    RemoteType type;
    uint32_t timestamp_ms;
};

struct RemoteProtocol {
    RemoteType type;
    const struct device *uart_device;
    struct uart_config uart_config;
    size_t frame_size;
    bool (*validate)(const uint8_t *, size_t);
    bool (*decode)(const uint8_t *, size_t, RemoteSample &);
};

class RemoteDevice {
public:
    using SampleCallback = void (*)(const RemoteSample *sample,
                                    void *user_data);

    void set_sample_callback(SampleCallback callback, void *user_data)
    {
        callback_ = callback;
        callback_user_data_ = user_data;
    }

protected:
    void publish_sample(const RemoteSample &sample) const
    {
        if (callback_ != nullptr) callback_(&sample, callback_user_data_);
    }

private:
    SampleCallback callback_{};
    void *callback_user_data_{};
};
