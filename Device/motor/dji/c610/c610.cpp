#include "c610.hpp"

int C610Motor::init(uint8_t id, const C610MotorConfig &config)
{
    const DjiMotorModel model = {
        .gear_ratio = config.gear_ratio,
        .current_max_amp = 10.0F,
        .current_max_raw = 10000,
        .temperature_valid = false,
    };
    return configure(id, model, config.control);
}
