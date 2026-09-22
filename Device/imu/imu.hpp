#pragma once

#include <stdint.h>

struct ImuSample {
    float accel_m_s2[3];
    float gyro_rad_s[3];
    float temperature_c;
    uint32_t timestamp_ms;
};

class ImuDevice {
public:
    using SampleCallback = void (*)(const ImuSample *sample, void *user_data);

    void set_sample_callback(SampleCallback callback, void *user_data)
    {
        callback_ = callback;
        callback_user_data_ = user_data;
    }

protected:
    void publish_sample(const ImuSample &sample) const
    {
        if (callback_ != nullptr) callback_(&sample, callback_user_data_);
    }

private:
    SampleCallback callback_{};
    void *callback_user_data_{};
};
