#include "c620.hpp"

int C620Motor::init(uint8_t id, const C620MotorConfig &config)
{
    const DjiMotorModel model = {
        .gear_ratio = config.gear_ratio,
        .current_max_amp = 20.0F,
        .current_max_raw = 16384,
        .temperature_valid = true,
    };
    return configure(id, model, config.control);
}
