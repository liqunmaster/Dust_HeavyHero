#pragma once

#include <stdint.h>

#include <zephyr/kernel.h>

#include "bsp_fdcan.h"
#include "pid.hpp"
#include "ring_buffer.hpp"

enum class DjiMotorControlMode : uint8_t {
    Current,
    Speed,
    Angle,
};

struct DjiMotorCanFeedback {
    uint8_t encoder_be[2];
    uint8_t speed_rpm_be[2];
    uint8_t current_be[2];
    uint8_t temperature;
    uint8_t error;
} __packed;

static_assert(sizeof(DjiMotorCanFeedback) == 8U);

struct DjiMotorFeedback {
    uint16_t rotor_encoder;
    int16_t rotor_speed_rpm;
    int16_t current_raw;
    float output_angle_rad;
    float output_speed_rad_s;
    float current_amp;
    float temperature_c;
    uint8_t error;
    uint32_t timestamp_ms;
    uint32_t receive_count;
};

struct DjiMotorModel {
    float gear_ratio;
    float current_max_amp;
    int16_t current_max_raw;
    bool temperature_valid;
};

struct DjiMotorControlConfig {
    PIDConfig angle_pid;
    PIDConfig speed_pid;
    uint32_t online_timeout_ms{100U};
};

class DjiMotor {
public:
    int configure(uint8_t id, const DjiMotorModel &model,
                  const DjiMotorControlConfig &control);
    void set_control_mode(DjiMotorControlMode mode);
    void set_target_current(float current_amp);
    void set_target_speed(float speed_rad_s);
    void set_target_angle(float angle_rad);
    void set_speed_feedforward(float speed_rad_s);
    void set_current_feedforward(float current_amp);
    int16_t update_control(float dt);
    void stop();

    uint8_t id() const;
    DjiMotorControlMode control_mode() const;
    bool feedback(DjiMotorFeedback *feedback) const;
    bool online(uint32_t now_ms) const;
    int16_t current_command_raw() const;

private:
    friend class DjiMotorBus;
    void accept_feedback(const DjiMotorCanFeedback &frame, uint32_t now_ms);

    uint8_t id_{};
    DjiMotorModel model_{};
    DjiMotorControlConfig control_{};
    DjiMotorControlMode mode_{DjiMotorControlMode::Current};
    PIDController angle_pid_{};
    PIDController speed_pid_{};
    DjiMotorFeedback feedback_{};
    int32_t rotor_turns_{};
    uint16_t previous_encoder_{};
    bool encoder_valid_{};
    float target_current_amp_{};
    float target_speed_rad_s_{};
    float target_angle_rad_{};
    float speed_feedforward_{};
    float current_feedforward_{};
    int16_t current_command_raw_{};
    mutable struct k_spinlock lock_{};
};

class DjiMotorBus {
public:
    int init(fdcan_t *bus);
    void deinit();
    int attach(DjiMotor *motor);
    void process_rx();
    int queue_group(uint8_t group_index, uint32_t now_ms);

private:
    struct TxItem {
        fdcan_frame_t frame;
        uint32_t queued_ms;
    };

    static void rx_callback(const fdcan_frame_t *frame, void *user_data);
    static void tx_worker(void *instance, void *unused1, void *unused2);
    int build_group_frame(uint8_t group_index, fdcan_frame_t *frame) const;

    static constexpr size_t kRxQueueDepth = 16U;
    static constexpr size_t kTxQueueDepth = 8U;
    fdcan_t *bus_{};
    int filter_id_{-1};
    DjiMotor *motors_[8]{};
    uint8_t rx_storage_[sizeof(fdcan_frame_t) * kRxQueueDepth]{};
    ring_buffer_t rx_buffer_{};
    struct k_msgq tx_queue_{};
    char tx_queue_storage_[sizeof(TxItem) * kTxQueueDepth] __aligned(4){};
    struct k_thread tx_thread_{};
    K_KERNEL_STACK_MEMBER(tx_stack_, 1536);
    bool tx_thread_started_{};
};
