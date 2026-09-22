#pragma once

#include "dji_motor.hpp"

struct C610MotorConfig {
    float gear_ratio{36.0F};
    DjiMotorControlConfig control{};
};

class C610Motor : public DjiMotor {
public:
    int init(uint8_t id, const C610MotorConfig &config);
};
