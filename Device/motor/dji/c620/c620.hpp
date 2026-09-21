#pragma once

enum DJI_C620_STATUS{
    DJI_C620_STATUS_DISABLE = 0,
    DJI_C620_STATUS_ENABLE = 1,
};

enum DJI_C620_MODE{
    DJI_C620_MODE_CURRENT = 0,
    DJI_C620_MODE_OMEGA = 1,
    DJI_C620_MODE_ANGLE = 2,
};

enum DJI_C620_ID{
    DJI_C620_ID_0x201 = 1,
    DJI_C620_ID_0x202 = 2,
    DJI_C620_ID_0x203 = 3,
    DJI_C620_ID_0x204 = 4,
    DJI_C620_ID_0x205 = 5,
    DJI_C620_ID_0x206 = 6,
    DJI_C620_ID_0x207 = 7,
    DJI_C620_ID_0x208 = 8,
};

struct DJI_C620_RX_DATA{
    float now_angle;
    float now_omega;
    float now_current;
    float now_temp;
    uint8_t error_code;
};