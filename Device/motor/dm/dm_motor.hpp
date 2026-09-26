#pragma once

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "bsp_fdcan.hpp"

enum DmControlMode {
    MOTOR_DM_CONTROL_METHOD_NORMAL_MIT         = 1,
    MOTOR_DM_CONTROL_METHOD_NORMAL_ANGLE_OMEGA = 2,
    MOTOR_DM_CONTROL_METHOD_NORMAL_OMEGA       = 3,
    MOTOR_DM_CONTROL_METHOD_NORMAL_EMIT        = 4,
};

struct DmMitLimits {
    float position;          // PMAX 位置映射半量程 单位 rad
    float velocity;          // VMAX 速度映射半量程 单位 rad/s
    float torque;            // TMAX 力矩映射半量程 单位 Nm
};

struct DmRxData {
    uint8_t motor_id;        // 反馈数据中的电机 ID 低 4 位
    uint8_t status;          // 反馈数据中的状态或故障码高 4 位
    uint16_t position;       // 16 位位置映射原始值
    uint16_t velocity;       // 12 位速度映射原始值
    uint16_t torque;         // 12 位力矩映射原始值
    uint8_t mos_temperature; // 驱动 MOS 温度 单位℃
    uint8_t rotor_temperature; // 电机线圈温度 单位℃
};

struct DmData {
    float now_angle;             // 当前位置 单位 rad
    float now_omega;             // 当前角速度 单位 rad/s
    float now_torque;            // 当前力矩 单位 Nm
    float now_mos_temperature;   // 驱动 MOS 温度 单位℃
    float now_rotor_temperature; // 电机线圈温度 单位℃
    uint8_t status;              // 当前状态或故障码
};

class dm_motor {
public:

    int init(fdcan_device device, uint8_t motor_id, DmControlMode mode, DmMitLimits limits, fdcan_protocol protocol, uint16_t master_id = 0U);

    int set_mit(float position, float velocity, float kp, float kd, float torque);

    int set_position_velocity(float position, float max_velocity);

    int set_velocity(float velocity);

    int set_position_torque(float position, float max_velocity, float current_ratio);
    
    int transmit();

    int enable();

    int disable();

    int clear_error();

    int save_zero();

    int switch_mode(DmControlMode mode);

    int process_feedback(const fdcan_frame &frame);

    bool is_online(uint32_t timeout_ms = 100U) const;

    uint32_t get_feedback_count() const;

    DmRxData get_rx_data() const;

    DmData get_data() const;

    DmControlMode get_mode() const;

private:
    static bool valid_mode(DmControlMode mode);
    
    static uint16_t control_id(uint8_t motor_id, DmControlMode mode);

    static uint16_t encode(float value, float minimum, float maximum, uint16_t maximum_raw);

    static float decode(uint16_t raw, float limit, uint16_t maximum_raw);

    static void write_f32(uint8_t *dst, float value);

    static void write_u16(uint8_t *dst, uint16_t value);

    int send_command(uint8_t command);

    int send_frame(uint16_t id, const uint8_t *data, uint8_t length);

    fdcan_device device_ = FDCAN_DEVICE_COUNT;

    uint8_t motor_id_ = 0U;

    uint16_t master_id_ = 0U;
    
    DmControlMode mode_ = DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_MIT;

    fdcan_protocol protocol_ = FDCAN_PROTOCOL_CLASSIC;

    DmControlMode requested_mode_ = DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_MIT;

    bool mode_change_pending_ = false;

    bool initialized_ = false;

    DmMitLimits limits_{};

    uint8_t tx_data_[8]{};

    uint8_t tx_length_ = 0U;

    DmRxData rx_data_{};

    DmData data_{};

    mutable struct k_spinlock lock_{};

    atomic_t feedback_count_{};

    atomic_t last_feedback_ms_{};

    atomic_t has_feedback_{};
};
