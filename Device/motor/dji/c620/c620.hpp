#pragma once

#include <errno.h>
#include <stdint.h>

#include "bsp_fdcan.hpp"
#include "math.h"

// C620 电调 ID
enum C620_ID {
    C620_ID_0x201 = 1,          // 反馈 ID 0x201  
    C620_ID_0x202 = 2,          // 反馈 ID 0x202
    C620_ID_0x203 = 3,          // 反馈 ID 0x203
    C620_ID_0x204 = 4,          // 反馈 ID 0x204
    C620_ID_0x205 = 5,          // 反馈 ID 0x205
    C620_ID_0x206 = 6,          // 反馈 ID 0x206
    C620_ID_0x207 = 7,          // 反馈 ID 0x207
    C620_ID_0x208 = 8,          // 反馈 ID 0x208
};

class c620 {
public:

    int init(fdcan_device device, C620_ID id, float gear_ratio = 1.0F);

    int set_current(float current);

    int transmit();

    int process_feedback(const fdcan_frame &frame);

    float get_current() const;

    int16_t get_omega() const;

    float get_omega_rad_s() const;

    int64_t get_total_encoder() const;

    float get_angle() const;

    float get_angle_degree() const;

    uint16_t get_encoder() const;

    uint8_t get_temperature() const;

    uint8_t get_error() const;

private:
    static constexpr uint16_t encoder_resolution_ = 8192U;     // 每圈编码器计数
    static constexpr int16_t  current_raw_limit_  = 16384;     // 电流指令满量程
    static constexpr float    current_limit_      = 20.0F;     // 电流物理量满量程

    static bool     valid_device(fdcan_device device);
    static bool     valid_id(C620_ID id);
    static uint8_t  group_index(C620_ID id);
    static uint8_t  slot_index(C620_ID id);
    static int16_t  read_i16(const uint8_t *data);
    static uint16_t read_u16(const uint8_t *data);
    static int      init_bus(fdcan_device device);
    static int      transmit_group(fdcan_device device, uint8_t group);

    void unpack_feedback(const uint8_t *data); 

    fdcan_device   device_              = FDCAN_DEVICE_COUNT;  // 使用的 CAN 控制器
    C620_ID        id_                  = C620_ID_0x201;       // 电调 ID
    float          gear_ratio_          = 1.0F;                // 减速比
    bool           initialized_         = false;               // 初始化完成标志
    float          target_current_amp_  = 0.0F;                // 已下发电流 A
    uint16_t       encoder_             = 0U;                  // 单圈编码器值 0 - 8191
    int64_t        total_encoder_       = 0;                   // 累计编码器值
    int16_t        omega_rpm_           = 0;                   // 电机转子转速 rpm
    int16_t        current_raw_         = 0;                   // 反馈电流原始值
    float          current_amp_         = 0.0F;                // 反馈电流 A
    float          omega_rad_s_         = 0.0F;                // 输出轴角速度 rad/s
    float          angle_rad_           = 0.0F;                // 输出轴累计角度 rad
    float          angle_degree_        = 0.0F;                // 输出轴累计角度 degree
    uint8_t        temperature_c_       = 0U;                  // 电机温度 摄氏度
    uint8_t        error_               = 0U;                  // 电机错误码
    bool           encoder_initialized_ = false;               // 是否已收到首个编码器值

    static int16_t current_commands_[FDCAN_DEVICE_COUNT][2][4];
};
