#pragma once

#include <stdint.h>

class SineTable {
public:
    void initialize();
    float sine(float angle_rad) const;
    float cosine(float angle_rad) const;

private:
    float values_[256]{};
};
