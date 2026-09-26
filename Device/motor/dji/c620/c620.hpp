#pragma once

#include <errno.h>
#include <limits.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "bsp_fdcan.hpp"
#include "dji_motor.hpp"
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

struct C620RxData {
    uint16_t encoder;           // 转子机械角度原始值
    int16_t  omega;             // 转子转速 单位 rpm
    uint16_t current;           // 实际电流的 16 位补码位模式
    uint8_t  temperature;       // 电机温度 单位摄氏度
    uint8_t  error;             // 电调错误码
} __attribute__((packed));

struct C620Data {
    float    now_angle;         // 输出轴累计角度 单位 rad
    float    now_omega;         // 输出轴角速度 单位 rad/s
    float    now_current;       // 实际电流 单位 A
    float    now_temperature;   // 电机温度 单位摄氏度
    uint32_t pre_encoder;       // 上一帧转子编码器值
    int32_t  total_encoder;     // 转子累计编码器计数
    int32_t  total_round;       // 转子编码器累计跨圈次数
};

class c620 {
public:

    int init(fdcan_device device, C620_ID id, float gear_ratio = 1.0F);

    int set_current(float current);

    int transmit();

    int process_feedback(const fdcan_frame &frame);

    bool is_online(uint32_t timeout_ms = 100U) const;

    uint32_t get_feedback_count() const;

    C620RxData get_rx_data() const;

    C620Data get_data() const;

private:
    static constexpr uint16_t encoder_resolution_ = 8192U;     // 每圈编码器计数
    static constexpr int16_t  current_raw_limit_  = 16384;     // 电流指令满量程
    static constexpr float    current_limit_      = 20.0F;     // 电流物理量满量程

    static C620RxData decode_feedback(const uint8_t *data);

    void unpack_feedback(const uint8_t *data); 

    fdcan_device   device_              = FDCAN_DEVICE_COUNT;  // 使用的 CAN 控制器
    C620_ID        id_                  = C620_ID_0x201;       // 电调 ID
    float          gear_ratio_          = 1.0F;                // 减速比
    bool           initialized_         = false;               // 初始化完成标志
    C620RxData     rx_data_{};                                 // 最近一次有效反馈
    C620Data       data_{};                                    // 换算后的电机数据
    int64_t        total_encoder_       = 0;                   // 完整累计编码器值
    bool           encoder_initialized_ = false;               // 是否已收到首个编码器值
    mutable struct k_spinlock feedback_lock_{};                // 反馈数据访问锁
    atomic_t       feedback_count_{};                          // 累计有效反馈帧数
    atomic_t       last_feedback_ms_{};                        // 最近一次有效反馈时间
    atomic_t       has_feedback_{};                            // 是否收到过有效反馈

};
