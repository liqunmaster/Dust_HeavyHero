#include "c610.hpp"

/**
 * @brief 解码 C610 的 8 字节反馈报文
 *
 * @param data 反馈报文数据区
 * @return 解码后的原始反馈数据
 */
C610RxData c610::decode_feedback(const uint8_t *data)
{
    return {
        dji_motor::read_u16(&data[0]),
        dji_motor::signed_value(dji_motor::read_u16(&data[2])),
        dji_motor::read_u16(&data[4]),
        data[6],
        data[7],
    };
}

/**
 * @brief 初始化 C610 电调及 CAN 控制器
 *
 * @param device CAN 控制器编号
 * @param id C610 ID
 * @param gear_ratio 电机的减速比
 * @return 0 表示成功 负值表示错误
 */
int c610::init(fdcan_device device, C610_ID id, float gear_ratio)
{
    if (!dji_motor::valid_device(device) || !dji_motor::valid_id(id) || !__builtin_isfinite(gear_ratio) || gear_ratio <= 0.0F) {
        return -EINVAL;
    }

    const int result = dji_motor::init_bus(device);
    if (result != 0) {
        return result;
    }

    device_     = device;
    id_         = id;
    gear_ratio_ = gear_ratio;
    const k_spinlock_key_t key = k_spin_lock(&feedback_lock_);
    rx_data_             = {};
    data_                = {};
    total_encoder_       = 0;
    encoder_initialized_ = false;
    k_spin_unlock(&feedback_lock_, key);
    atomic_set(&feedback_count_, 0);
    atomic_set(&last_feedback_ms_, 0);
    atomic_set(&has_feedback_, 0);
    initialized_ = true;
    return 0;
}

/**
 * @brief 更新 C610 的目标电流
 *
 * @param current 目标电流，单位 A
 * @return 0 表示成功 负值表示错误
 */
int c610::set_current(float current)
{
    if (!initialized_) {
        return -ENODEV;
    }
    if (!__builtin_isfinite(current)) {
        return -EINVAL;
    }

    const int16_t command = math::current_to_raw(current, current_limit_, current_raw_limit_);
    return dji_motor::set(device_, static_cast<uint8_t>(id_), command);
}

/**
 * @brief 发送当前电调所在组的电流控制报文
 *
 * @return 0 表示成功 负值表示错误
 */
int c610::transmit()
{
    if (!initialized_) {
        return -ENODEV;
    }
    return dji_motor::transmit(device_, static_cast<uint8_t>(id_));
}

/**
 * @brief 更新编码器累计值和物理量反馈
 *
 * @param data 已校验的 8 字节反馈数据
 */
void c610::unpack_feedback(const uint8_t *data)
{
    const C610RxData next = decode_feedback(data);
    const k_spinlock_key_t key = k_spin_lock(&feedback_lock_);

    if (!encoder_initialized_) {
        total_encoder_ = next.encoder;
        encoder_initialized_ = true;
    } else {
        int32_t delta = static_cast<int32_t>(next.encoder) - static_cast<int32_t>(data_.pre_encoder);
        
        if (delta > encoder_resolution_ / 2) {
            delta -= encoder_resolution_;
            --data_.total_round;
        } else if (delta < -encoder_resolution_ / 2) {
            delta += encoder_resolution_;
            ++data_.total_round;
        }
        total_encoder_ += delta;
    }

    rx_data_            = next;
    data_.pre_encoder   = next.encoder;
    data_.total_encoder = total_encoder_ > INT32_MAX ? INT32_MAX : total_encoder_ < INT32_MIN ? INT32_MIN: static_cast<int32_t>(total_encoder_);

    data_.now_current   = math::raw_to_current(dji_motor::signed_value(next.current), current_raw_limit_, current_limit_);
    data_.now_omega     = math::rpm_to_radian_per_second(next.omega, gear_ratio_);
    data_.now_angle     = math::encoder_to_radian(total_encoder_, encoder_resolution_, gear_ratio_);
    k_spin_unlock(&feedback_lock_, key);
}

/**
 * @brief 校验并解析当前 C610 的反馈帧
 *
 * @param frame CAN 接收帧
 * @return 0 表示已处理 负值表示未匹配或报文错误
 */
int c610::process_feedback(const fdcan_frame &frame)
{
    if (!initialized_) {
        return -ENODEV;
    }
    const int result = dji_motor::validate_feedback(frame, static_cast<uint8_t>(id_));
    if (result != 0) {
        return result;
    }

    unpack_feedback(frame.data);
    atomic_set(&last_feedback_ms_, static_cast<atomic_val_t>(k_uptime_get_32()));
    atomic_inc(&feedback_count_);
    atomic_set(&has_feedback_, 1);
    return 0;
}

/**
 * @brief 判断电调是否在超时时间内反馈过数据
 *
 * @param timeout_ms 反馈超时时间
 * @return true 在线 false 尚未收到反馈或已超时
 */
bool c610::is_online(uint32_t timeout_ms) const
{
    if (atomic_get(&has_feedback_) == 0) {
        return false;
    }
    const uint32_t last = static_cast<uint32_t>(atomic_get(&last_feedback_ms_));
    return static_cast<uint32_t>(k_uptime_get_32() - last) <= timeout_ms;
}

/**
 * @brief 获取累计有效反馈帧数
 *
 * @return 累计有效反馈帧数
 */
uint32_t c610::get_feedback_count() const
{
    return static_cast<uint32_t>(atomic_get(&feedback_count_));
}

/**
 * @brief 获取最近一帧原始反馈
 *
 * @return 受锁保护的原始反馈快照
 */
C610RxData c610::get_rx_data() const
{
    const k_spinlock_key_t key = k_spin_lock(&feedback_lock_);
    const C610RxData snapshot = rx_data_;
    k_spin_unlock(&feedback_lock_, key);
    return snapshot;
}

/**
 * @brief 获取换算后的电机状态
 *
 * @return 受锁保护的电机状态快照
 */
C610Data c610::get_data() const
{
    const k_spinlock_key_t key = k_spin_lock(&feedback_lock_);
    const C610Data snapshot = data_;
    k_spin_unlock(&feedback_lock_, key);
    return snapshot;
}
