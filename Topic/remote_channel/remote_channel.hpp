#pragma once

#include <stdint.h>

struct RemoteTopicData {
    float forward;
    float lateral;
    float rotation;
    uint8_t switch_left;
    uint8_t switch_right;
    bool valid;
    uint32_t timestamp_ms;
};
