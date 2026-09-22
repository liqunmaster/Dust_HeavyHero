#include "dji_motor.hpp"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>

static constexpr uint32_t kFeedbackIdBase = 0x200U;
static constexpr uint32_t kFirstCommandId = 0x200U;
static constexpr uint32_t kSecondCommandId = 0x1FFU;
static constexpr int32_t kEncoderCounts = 8192;
static constexpr int32_t kEncoderHalfRange = kEncoderCounts / 2;
static constexpr float kTwoPi = 6.28318530717958647692F;
static constexpr float kRpmToRadiansPerSecond = kTwoPi / 60.0F;

static float clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

int DjiMotor::configure(uint8_t id, const DjiMotorModel &model,
                        const DjiMotorControlConfig &control)
{
    if (id < 1U || id > 8U || !isfinite(model.gear_ratio) ||
        model.gear_ratio <= 0.0F || !isfinite(model.current_max_amp) ||
        model.current_max_amp <= 0.0F || model.current_max_raw <= 0) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    id_ = id;
    model_ = model;
    control_ = control;
    feedback_ = {};
    rotor_turns_ = 0;
    previous_encoder_ = 0U;
    encoder_valid_ = false;
    target_current_amp_ = 0.0F;
    target_speed_rad_s_ = 0.0F;
    target_angle_rad_ = 0.0F;
    speed_feedforward_ = 0.0F;
    current_feedforward_ = 0.0F;
    current_command_raw_ = 0;
    mode_ = DjiMotorControlMode::Current;
    k_spin_unlock(&lock_, key);
    angle_pid_.configure(control.angle_pid);
    speed_pid_.configure(control.speed_pid);
    return 0;
}

void DjiMotor::set_control_mode(DjiMotorControlMode mode)
{
    if (mode_ == mode) return;
    mode_ = mode;
    angle_pid_.reset();
    speed_pid_.reset();
}

void DjiMotor::set_target_current(float value) { target_current_amp_ = value; }
void DjiMotor::set_target_speed(float value) { target_speed_rad_s_ = value; }
void DjiMotor::set_target_angle(float value) { target_angle_rad_ = value; }
void DjiMotor::set_speed_feedforward(float value) { speed_feedforward_ = value; }
void DjiMotor::set_current_feedforward(float value) { current_feedforward_ = value; }

int16_t DjiMotor::update_control(float dt)
{
    DjiMotorFeedback state{};
    if (!feedback(&state) || !isfinite(dt) || dt <= 0.0F) {
        stop();
        return 0;
    }

    float current_amp = target_current_amp_;
    if (mode_ == DjiMotorControlMode::Angle) {
        target_speed_rad_s_ = angle_pid_.update(
            target_angle_rad_, state.output_angle_rad, dt);
    }
    if (mode_ == DjiMotorControlMode::Speed ||
        mode_ == DjiMotorControlMode::Angle) {
        current_amp = speed_pid_.update(
            target_speed_rad_s_ + speed_feedforward_,
            state.output_speed_rad_s, dt);
    }
    current_amp = clamp(current_amp + current_feedforward_,
                        model_.current_max_amp);
    current_feedforward_ = 0.0F;
    speed_feedforward_ = 0.0F;
    const float raw = current_amp *
        (static_cast<float>(model_.current_max_raw) / model_.current_max_amp);
    current_command_raw_ = static_cast<int16_t>(
        clamp(raw, static_cast<float>(model_.current_max_raw)));
    return current_command_raw_;
}

void DjiMotor::stop()
{
    target_current_amp_ = 0.0F;
    target_speed_rad_s_ = 0.0F;
    speed_feedforward_ = 0.0F;
    current_feedforward_ = 0.0F;
    current_command_raw_ = 0;
    angle_pid_.reset();
    speed_pid_.reset();
}

uint8_t DjiMotor::id() const { return id_; }
DjiMotorControlMode DjiMotor::control_mode() const { return mode_; }

bool DjiMotor::feedback(DjiMotorFeedback *feedback) const
{
    if (feedback == nullptr) return false;
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    *feedback = feedback_;
    k_spin_unlock(&lock_, key);
    return feedback->receive_count != 0U;
}

bool DjiMotor::online(uint32_t now_ms) const
{
    DjiMotorFeedback state{};
    return feedback(&state) &&
        static_cast<uint32_t>(now_ms - state.timestamp_ms) <=
            control_.online_timeout_ms;
}

int16_t DjiMotor::current_command_raw() const { return current_command_raw_; }

void DjiMotor::accept_feedback(const DjiMotorCanFeedback &frame,
                               uint32_t now_ms)
{
    const uint16_t encoder = sys_get_be16(frame.encoder_be);
    const int16_t speed_rpm = static_cast<int16_t>(
        sys_get_be16(frame.speed_rpm_be));
    const int16_t current_raw = static_cast<int16_t>(
        sys_get_be16(frame.current_be));

    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (encoder_valid_) {
        const int32_t delta = static_cast<int32_t>(encoder) - previous_encoder_;
        if (delta > kEncoderHalfRange) --rotor_turns_;
        if (delta < -kEncoderHalfRange) ++rotor_turns_;
    } else {
        encoder_valid_ = true;
    }
    previous_encoder_ = encoder;
    const int32_t total_encoder = rotor_turns_ * kEncoderCounts + encoder;
    feedback_.rotor_encoder = encoder;
    feedback_.rotor_speed_rpm = speed_rpm;
    feedback_.current_raw = current_raw;
    feedback_.output_angle_rad = static_cast<float>(total_encoder) * kTwoPi /
        (static_cast<float>(kEncoderCounts) * model_.gear_ratio);
    feedback_.output_speed_rad_s = static_cast<float>(speed_rpm) *
        kRpmToRadiansPerSecond / model_.gear_ratio;
    feedback_.current_amp = static_cast<float>(current_raw) *
        model_.current_max_amp / static_cast<float>(model_.current_max_raw);
    feedback_.temperature_c = model_.temperature_valid ?
        static_cast<float>(frame.temperature) : 0.0F;
    feedback_.error = frame.error;
    feedback_.timestamp_ms = now_ms;
    ++feedback_.receive_count;
    k_spin_unlock(&lock_, key);
}

int DjiMotorBus::init(fdcan_t *bus)
{
    if (!bsp_fdcan_is_ready(bus)) return -ENODEV;
    if (filter_id_ >= 0) return bus_ == bus ? 0 : -EALREADY;
    bus_ = bus;
    memset(motors_, 0, sizeof(motors_));
    ring_buffer_init(&rx_buffer_, rx_storage_, sizeof(rx_storage_));
    k_msgq_init(&tx_queue_, tx_queue_storage_, sizeof(TxItem), kTxQueueDepth);
    filter_id_ = bsp_fdcan_add_rx_filter(bus_, kFeedbackIdBase, 0x7F0U,
                                         0U, rx_callback, this);
    if (filter_id_ < 0) {
        const int ret = filter_id_;
        bus_ = nullptr;
        return ret;
    }
    k_tid_t tid = k_thread_create(
        &tx_thread_, tx_stack_, K_KERNEL_STACK_SIZEOF(tx_stack_), tx_worker,
        this, nullptr, nullptr, K_PRIO_PREEMPT(3), 0, K_NO_WAIT);
    if (tid == nullptr) {
        (void)bsp_fdcan_remove_rx_filter(bus_, filter_id_);
        filter_id_ = -1;
        bus_ = nullptr;
        return -ENOMEM;
    }
    tx_thread_started_ = true;
    (void)k_thread_name_set(tid, "dji_can_tx");
    return 0;
}

void DjiMotorBus::deinit()
{
    if (tx_thread_started_) {
        k_thread_abort(&tx_thread_);
        tx_thread_started_ = false;
    }
    if (bus_ != nullptr && filter_id_ >= 0) {
        (void)bsp_fdcan_remove_rx_filter(bus_, filter_id_);
    }
    filter_id_ = -1;
    bus_ = nullptr;
    memset(motors_, 0, sizeof(motors_));
}

void DjiMotorBus::tx_worker(void *instance, void *unused1, void *unused2)
{
    ARG_UNUSED(unused1);
    ARG_UNUSED(unused2);
    auto *motor_bus = static_cast<DjiMotorBus *>(instance);
    TxItem item{};
    while (true) {
        if (k_msgq_get(&motor_bus->tx_queue_, &item, K_FOREVER) != 0) continue;
        if (static_cast<uint32_t>(k_uptime_get_32() - item.queued_ms) > 10U) {
            continue;
        }
        (void)bsp_fdcan_send(motor_bus->bus_, &item.frame, K_MSEC(2));
    }
}

int DjiMotorBus::attach(DjiMotor *motor)
{
    if (motor == nullptr || motor->id() < 1U || motor->id() > 8U) {
        return -EINVAL;
    }
    DjiMotor *&slot = motors_[motor->id() - 1U];
    if (slot != nullptr && slot != motor) return -EALREADY;
    slot = motor;
    return 0;
}

void DjiMotorBus::rx_callback(const fdcan_frame_t *frame, void *user_data)
{
    if (frame == nullptr || user_data == nullptr || frame->len != 8U ||
        (frame->flags & (CAN_FRAME_IDE | CAN_FRAME_RTR | CAN_FRAME_FDF)) != 0U ||
        frame->id < 0x201U || frame->id > 0x208U) return;
    auto *motor_bus = static_cast<DjiMotorBus *>(user_data);
    if (ring_buffer_free(&motor_bus->rx_buffer_) >= sizeof(*frame)) {
        (void)ring_buffer_write(&motor_bus->rx_buffer_, frame, sizeof(*frame));
    }
}

void DjiMotorBus::process_rx()
{
    fdcan_frame_t frame{};
    while (ring_buffer_read(&rx_buffer_, &frame, sizeof(frame)) == sizeof(frame)) {
        const uint8_t id = static_cast<uint8_t>(frame.id - kFeedbackIdBase);
        DjiMotor *motor = motors_[id - 1U];
        if (motor == nullptr) continue;
        DjiMotorCanFeedback wire{};
        memcpy(&wire, frame.data, sizeof(wire));
        motor->accept_feedback(wire, k_uptime_get_32());
    }
}

int DjiMotorBus::build_group_frame(uint8_t group_index,
                                   fdcan_frame_t *frame) const
{
    if (frame == nullptr || group_index > 1U) return -EINVAL;
    *frame = {};
    frame->id = group_index == 0U ? kFirstCommandId : kSecondCommandId;
    frame->len = 8U;
    const uint8_t offset = group_index * 4U;
    for (uint8_t slot = 0U; slot < 4U; ++slot) {
        const DjiMotor *motor = motors_[offset + slot];
        const int16_t command = motor == nullptr ? 0 :
            motor->current_command_raw();
        sys_put_be16(static_cast<uint16_t>(command), &frame->data[slot * 2U]);
    }
    return 0;
}

int DjiMotorBus::queue_group(uint8_t group_index, uint32_t now_ms)
{
    if (bus_ == nullptr) return -ENODEV;
    TxItem item{};
    int ret = build_group_frame(group_index, &item.frame);
    if (ret != 0) return ret;
    item.queued_ms = now_ms;
    ret = k_msgq_put(&tx_queue_, &item, K_NO_WAIT);
    if (ret == 0) return 0;
    TxItem discarded{};
    (void)k_msgq_get(&tx_queue_, &discarded, K_NO_WAIT);
    return k_msgq_put(&tx_queue_, &item, K_NO_WAIT);
}
