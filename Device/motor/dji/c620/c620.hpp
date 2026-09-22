#pragma once

#include "dji_motor.hpp"

struct C620MotorConfig {
    float gear_ratio{3591.0F / 187.0F};
    DjiMotorControlConfig control{};
};

class C620Motor : public DjiMotor {
public:
    int init(uint8_t id, const C620MotorConfig &config);
};
