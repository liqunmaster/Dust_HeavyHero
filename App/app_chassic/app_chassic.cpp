#include "app_chassic.hpp"

#include <errno.h>
#include <math.h>

#include "imu_chan.hpp"
#include "remote_chan.hpp"

static constexpr uint32_t kControlPeriodMs = 1U;
static constexpr uint32_t kRemoteTimeoutMs = 100U;
static constexpr uint32_t kImuTimeoutMs = 50U;
static constexpr float kMaxWheelRpm = 1200.0F;
static constexpr float kGearRatio = 3591.0F / 187.0F;
static constexpr float kRpmToRadiansPerSecond =
    6.28318530717958647692F / 60.0F;

static C620MotorConfig chassis_motor_config()
{
    C620MotorConfig config{};
    config.gear_ratio = kGearRatio;
    config.control.online_timeout_ms = 100U;
    config.control.angle_pid = {
        .kp = 8.0F,
        .ki = 0.0F,
        .kd = 0.1F,
        .output_limit = 20.0F,
        .derivative_filter_tau = 0.002F,
        .derivative_mode = PIDDerivativeMode::Measurement,
    };
    config.control.speed_pid = {
        .kp = 0.6F,
        .ki = 4.0F,
        .kd = 0.0F,
        .integral_output_limit = 3.0F,
        .output_limit = 4.0F,
        .integral_separation_threshold = 15.0F,
        .variable_integral_start = 2.0F,
        .variable_integral_end = 10.0F,
        .derivative_filter_tau = 0.002F,
        .derivative_mode = PIDDerivativeMode::Measurement,
    };
    return config;
}

int AppChassis::init(const struct device *can_device)
{
    k_sem_init(&control_tick_sem_, 0, 1);
    k_timer_init(&control_timer_, control_timer_expiry, nullptr);
    k_timer_user_data_set(&control_timer_, this);

    const fdcan_config_t can_config = {
        .bitrate = 1000000U,
        .data_bitrate = 0U,
        .mode = CAN_MODE_NORMAL,
    };
    int ret = bsp_fdcan_init(&can_bus_, can_device, &can_config);
    if (ret != 0) return ret;

    ret = motor_bus_.init(&can_bus_);
    if (ret != 0) {
        (void)bsp_fdcan_deinit(&can_bus_);
        return ret;
    }

    const C620MotorConfig motor_config = chassis_motor_config();
    for (uint8_t index = 0U; index < 4U; ++index) {
        ret = motors_[index].init(index + 1U, motor_config);
        if (ret == 0) ret = motor_bus_.attach(&motors_[index]);
        if (ret != 0) {
            motor_bus_.deinit();
            (void)bsp_fdcan_deinit(&can_bus_);
            return ret;
        }
        motors_[index].set_control_mode(DjiMotorControlMode::Speed);
    }

    k_timer_start(&control_timer_, K_MSEC(kControlPeriodMs),
                  K_MSEC(kControlPeriodMs));
    k_tid_t tid = k_thread_create(
        &control_thread_, control_stack_, K_KERNEL_STACK_SIZEOF(control_stack_),
        control_thread_entry, this, nullptr, nullptr, K_PRIO_PREEMPT(5), 0,
        K_NO_WAIT);
    if (tid == nullptr) {
        k_timer_stop(&control_timer_);
        return -ENOMEM;
    }
    (void)k_thread_name_set(tid, "chassis");
    return 0;
}

void AppChassis::control_thread_entry(void *instance, void *unused1,
                                      void *unused2)
{
    ARG_UNUSED(unused1);
    ARG_UNUSED(unused2);
    auto *app = static_cast<AppChassis *>(instance);
    while (true) {
        (void)k_sem_take(&app->control_tick_sem_, K_FOREVER);
        app->control_step();
    }
}

void AppChassis::control_timer_expiry(struct k_timer *timer)
{
    auto *app = static_cast<AppChassis *>(k_timer_user_data_get(timer));
    if (app != nullptr) k_sem_give(&app->control_tick_sem_);
}

void AppChassis::control_step()
{
    motor_bus_.process_rx();
    RemoteTopicData remote{};
    ImuTopicData imu{};
    const bool remote_read = remote_topic_read(remote) == 0;
    const bool imu_read = imu_topic_read(imu) == 0;
    const uint32_t now = k_uptime_get_32();
    const bool remote_online = remote_read && remote.valid &&
        static_cast<uint32_t>(now - remote.timestamp_ms) <= kRemoteTimeoutMs;
    const bool imu_online = imu_read && imu.valid &&
        static_cast<uint32_t>(now - imu.timestamp_ms) <= kImuTimeoutMs;

    bool all_motors_online = true;
    for (uint8_t i = 0U; i < 4U; ++i) {
        all_motors_online = all_motors_online && motors_[i].online(now);
    }
    const bool control_enabled = remote_online && all_motors_online;

    float target_rpm[4]{};
    if (control_enabled) {
        const float yaw = imu_online ? imu.yaw_rad : 0.0F;
        const float cos_yaw = cosf(yaw);
        const float sin_yaw = sinf(yaw);
        const float vx = cos_yaw * remote.forward - sin_yaw * remote.lateral;
        const float vy = sin_yaw * remote.forward + cos_yaw * remote.lateral;
        const float omega = remote.rotation;
        target_rpm[0] = (vx + vy + omega) * kMaxWheelRpm;
        target_rpm[1] = (-vx + vy + omega) * kMaxWheelRpm;
        target_rpm[2] = (-vx - vy + omega) * kMaxWheelRpm;
        target_rpm[3] = (vx - vy + omega) * kMaxWheelRpm;
        float max_abs = 0.0F;
        for (float rpm : target_rpm) {
            const float magnitude = fabsf(rpm);
            if (magnitude > max_abs) max_abs = magnitude;
        }
        if (max_abs > kMaxWheelRpm) {
            const float scale = kMaxWheelRpm / max_abs;
            for (float &rpm : target_rpm) rpm *= scale;
        }
    }

    for (uint8_t i = 0U; i < 4U; ++i) {
        if (!control_enabled) {
            motors_[i].stop();
        } else {
            const float target_output_speed = target_rpm[i] *
                kRpmToRadiansPerSecond / kGearRatio;
            motors_[i].set_target_speed(target_output_speed);
            (void)motors_[i].update_control(0.001F);
        }
    }
    (void)motor_bus_.queue_group(0U, now);
}
