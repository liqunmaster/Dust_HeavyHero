#include "dm_motor.hpp"

namespace {
    constexpr uint16_t kRegisterCommandId = 0x7FFU;
    constexpr uint8_t  kModeRegister      = 0x0AU;
    constexpr uint16_t kTwelveBitMax      = 0x0FFFU;
    constexpr uint16_t kSixteenBitMax     = 0xFFFFU;

    /**
     * @brief 检查控制参数是否为有限浮点数
     *
     * @param value 待检查的浮点值
     * @return true 数值有限；false 为 NaN 或无穷大
     */
    bool finite(float value)
    {
        return __builtin_isfinite(value);
    }
} 

/**
 * @brief 检查控制模式是否属于手册支持的四种模式
 *
 * @param mode 待检查的控制模式
 * @return true 模式有效
 * @return false 模式无效
 */
bool dm_motor::valid_mode(DmControlMode mode)
{
    return mode >= DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_MIT && mode <= DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_EMIT;
}

/**
 * @brief 计算当前控制模式的 CAN 发送 ID
 *
 * @param motor_id 电机接收 ID
 * @param mode 控制模式
 * @return 叠加模式偏移后的标准帧 ID
 */
uint16_t dm_motor::control_id(uint8_t motor_id, DmControlMode mode)
{
    return static_cast<uint16_t>(motor_id) + (static_cast<uint16_t>(mode) - 1U) * 0x100U;
}

/**
 * @brief 将物理量线性映射为报文中的无符号整数
 *
 * @param value 待编码的物理量
 * @param minimum 映射区间下限
 * @param maximum 映射区间上限
 * @param maximum_raw 原始整数的最大值
 * @return 编码后的原始整数
 */
uint16_t dm_motor::encode(float value, float minimum, float maximum, uint16_t maximum_raw)
{
    return static_cast<uint16_t>((value - minimum) * maximum_raw / (maximum - minimum));
}

/**
 * @brief 将反馈原始值映射回对称物理量区间
 *
 * @param raw 反馈报文中的无符号原始值
 * @param limit 对称区间的正向半量程
 * @param maximum_raw 原始整数的最大值
 * @return 区间 [-limit, limit] 内的物理量
 */
float dm_motor::decode(uint16_t raw, float limit, uint16_t maximum_raw)
{
    return static_cast<float>(raw) * (2.0F * limit) / maximum_raw - limit;
}

/**
 * @brief 按低字节在前的顺序写入 32 位浮点数
 *
 * @param dst 至少 4 字节的报文数据区
 * @param value 待写入的浮点值
 */
void dm_motor::write_f32(uint8_t *dst, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    for (uint8_t i = 0U; i < 4U; ++i) {
        dst[i] = static_cast<uint8_t>(bits >> (8U * i));
    }
}

/**
 * @brief 按低字节在前的顺序写入 16 位整数
 *
 * @param dst 至少 2 字节的报文数据区
 * @param value 待写入的无符号整数
 */
void dm_motor::write_u16(uint8_t *dst, uint16_t value)
{
    dst[0] = static_cast<uint8_t>(value);
    dst[1] = static_cast<uint8_t>(value >> 8U);
}

/**
 * @brief 初始化电机参数和所选 CAN 控制器
 *
 * @param device CAN 控制器编号
 * @param motor_id 电机接收 ID
 * @param mode 电机当前配置的控制模式
 * @param limits 与电机 PMAX、VMAX、TMAX 寄存器一致的量程
 * @param protocol 电机反馈帧协议
 * @param master_id 电机反馈帧
 * @return 0 表示成功 负值表示参数、总线或协议配置错误
 */
int dm_motor::init(fdcan_device device, uint8_t motor_id, DmControlMode mode, DmMitLimits limits, fdcan_protocol protocol, uint16_t master_id)
{
    if (device < FDCAN_DEVICE_CAN0 || device >= FDCAN_DEVICE_COUNT ||
        motor_id == 0U || motor_id > 15U || master_id > 0x7FFU ||
        (protocol != FDCAN_PROTOCOL_CLASSIC && protocol != FDCAN_PROTOCOL_FD) ||
        !valid_mode(mode) || !finite(limits.position) || limits.position <= 0.0F ||
        !finite(limits.velocity) || limits.velocity <= 0.0F ||
        !finite(limits.torque) || limits.torque <= 0.0F) {
        return -EINVAL;
    }

    if (!bsp_fdcan_is_ready(device)) {
        const fdcan_config config = {
            .mode = FDCAN_MODE_NORMAL,
            .retransmission = FDCAN_RETRANSMISSION_DISABLED,
        };
        const int result = bsp_fdcan_init(device, &config);
        if (result != 0 && result != -EALREADY) {
            return result;
        }
    }
    if (protocol == FDCAN_PROTOCOL_FD && bsp_fdcan_get_data_bitrate(device) == 0U) {
        return -ENOTSUP;
    }

    const k_spinlock_key_t key = k_spin_lock(&lock_);
    device_ = device;
    motor_id_ = motor_id;
    master_id_ = master_id;
    mode_ = mode;
    protocol_ = protocol;
    requested_mode_ = mode;
    mode_change_pending_ = false;
    limits_ = limits;
    memset(tx_data_, 0, sizeof(tx_data_));
    tx_length_ = 0U;
    rx_data_ = {};
    data_ = {};
    initialized_ = true;
    k_spin_unlock(&lock_, key);
    atomic_set(&feedback_count_, 0);
    atomic_set(&last_feedback_ms_, 0);
    atomic_set(&has_feedback_, 0);
    return 0;
}

/**
 * @brief 设置 MIT 模式的位置、速度、增益和前馈力矩
 *
 * @param position 目标位置 单位 rad
 * @param velocity 目标速度 单位 rad/s
 * @param kp 位置比例系数 范围 0~500
 * @param kd 速度阻尼系数 范围 0~5
 * @param torque 前馈力矩 单位 Nm
 * @return 0 表示成功 负值表示模式错误或参数越界
 */
int dm_motor::set_mit(float position, float velocity, float kp, float kd, float torque)
{
    if (!finite(position) || !finite(velocity) || !finite(kp) || !finite(kd) || !finite(torque)) {
        return -EINVAL;
    }

    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_ || mode_ != DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_MIT || mode_change_pending_) {
        k_spin_unlock(&lock_, key);
        return !initialized_ ? -ENODEV : -EPERM;
    }
    if (position < -limits_.position || position > limits_.position ||
        velocity < -limits_.velocity || velocity > limits_.velocity ||
        torque < -limits_.torque || torque > limits_.torque ||
        kp < 0.0F || kp > 500.0F || kd < 0.0F || kd > 5.0F) {
        k_spin_unlock(&lock_, key);
        return -ERANGE;
    }

    const uint16_t p   = encode(position, -limits_.position, limits_.position, kSixteenBitMax);
    const uint16_t v   = encode(velocity, -limits_.velocity, limits_.velocity, kTwelveBitMax);
    const uint16_t k_p = encode(kp, 0.0F, 500.0F, kTwelveBitMax);
    const uint16_t k_d = encode(kd, 0.0F, 5.0F, kTwelveBitMax);
    const uint16_t t   = encode(torque, -limits_.torque, limits_.torque, kTwelveBitMax);
    tx_data_[0] = static_cast<uint8_t>(p >> 8U);
    tx_data_[1] = static_cast<uint8_t>(p);
    tx_data_[2] = static_cast<uint8_t>(v >> 4U);
    tx_data_[3] = static_cast<uint8_t>((v << 4U) | (k_p >> 8U));
    tx_data_[4] = static_cast<uint8_t>(k_p);
    tx_data_[5] = static_cast<uint8_t>(k_d >> 4U);
    tx_data_[6] = static_cast<uint8_t>((k_d << 4U) | (t >> 8U));
    tx_data_[7] = static_cast<uint8_t>(t);
    tx_length_  = 8U;
    k_spin_unlock(&lock_, key);
    return 0;
}

/**
 * @brief 设置位置速度模式的目标位置和速度上限
 *
 * @param position 目标位置
 * @param max_velocity 运动过程中的最大绝对速度
 * @return 0 表示成功，负值表示模式错误或参数无效
 */
int dm_motor::set_position_velocity(float position, float max_velocity)
{
    if (!finite(position) || !finite(max_velocity) || max_velocity < 0.0F) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_ || mode_ != DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_ANGLE_OMEGA || mode_change_pending_) {
        k_spin_unlock(&lock_, key);
        return !initialized_ ? -ENODEV : -EPERM;
    }
    write_f32(&tx_data_[0], position);
    write_f32(&tx_data_[4], max_velocity);
    tx_length_ = 8U;
    k_spin_unlock(&lock_, key);
    return 0;
}

/**
 * @brief 设置速度模式的目标速度
 *
 * @param velocity 目标速度
 * @return 0 表示成功 负值表示模式错误或参数无效
 */
int dm_motor::set_velocity(float velocity)
{
    if (!finite(velocity)) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_ || mode_ != DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_OMEGA || mode_change_pending_) {
        k_spin_unlock(&lock_, key);
        return !initialized_ ? -ENODEV : -EPERM;
    }
    write_f32(&tx_data_[0], velocity);
    memset(&tx_data_[4], 0, 4U);
    tx_length_ = 4U;
    k_spin_unlock(&lock_, key);
    return 0;
}

/**
 * @brief 设置力位混控模式的目标位置及电流上限
 *
 * @param position 目标位置
 * @param max_velocity 速度上限
 * @param current_ratio 最大相电流的比例
 * @return 0 表示成功 负值表示模式错误或参数无效
 */
int dm_motor::set_position_torque(float position, float max_velocity, float current_ratio)
{
    if (!finite(position) || !finite(max_velocity) || !finite(current_ratio) ||
        max_velocity < 0.0F || max_velocity > 100.0F ||
        current_ratio < 0.0F || current_ratio > 1.0F) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_ || mode_ != DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_EMIT || mode_change_pending_) {
        k_spin_unlock(&lock_, key);
        return !initialized_ ? -ENODEV : -EPERM;
    }
    write_f32(&tx_data_[0], position);
    write_u16(&tx_data_[4], static_cast<uint16_t>(max_velocity * 100.0F));
    write_u16(&tx_data_[6], static_cast<uint16_t>(current_ratio * 10000.0F));
    tx_length_ = 8U;
    k_spin_unlock(&lock_, key);
    return 0;
}

/**
 * @brief 发送经典 CAN 标准控制帧
 *
 * @param id 控制帧 CAN ID
 * @param data 控制帧数据区
 * @param length 有效数据长度
 * @return 0 表示发送请求成功 负值表示 CAN 发送错误
 */
int dm_motor::send_frame(uint16_t id, const uint8_t *data, uint8_t length)
{
    fdcan_frame frame{};
    frame.id = id;
    frame.id_type = FDCAN_ID_STANDARD;
    frame.protocol = FDCAN_PROTOCOL_CLASSIC;
    frame.bitrate_switch = FDCAN_BRS_DISABLED;
    frame.length = length;
    memcpy(frame.data, data, length);
    return bsp_fdcan_transmit(device_, &frame, FDCAN_NO_WAIT);
}

/**
 * @brief 发送最近一次设置的控制目标
 *
 * @return 0 表示发送请求成功 未设置目标、切换模式中或发送失败时返回负值
 */
int dm_motor::transmit()
{
    uint8_t payload[8];
    uint8_t length;
    uint16_t id;
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_ || mode_change_pending_ || tx_length_ == 0U) {
        const int result = !initialized_ ? -ENODEV : mode_change_pending_ ? -EBUSY : -ENODATA;
        k_spin_unlock(&lock_, key);
        return result;
    }
    id = control_id(motor_id_, mode_);
    length = tx_length_;
    memcpy(payload, tx_data_, length);
    k_spin_unlock(&lock_, key);
    return send_frame(id, payload, length);
}

/**
 * @brief 发送以七字节 0xFF 开头的电机特殊命令
 *
 * @param command 命令末字节 用于使能、失能、清错或保存零点
 * @return 0 表示发送请求成功 负值表示未初始化或 CAN 发送错误
 */
int dm_motor::send_command(uint8_t command)
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_) {
        k_spin_unlock(&lock_, key);
        return -ENODEV;
    }
    const uint16_t id = control_id(motor_id_, mode_);
    k_spin_unlock(&lock_, key);
    uint8_t payload[8];
    memset(payload, 0xFF, sizeof(payload));
    payload[7] = command;
    return send_frame(id, payload, sizeof(payload));
}

/**
 * @brief 发送电机使能命令 0xFC
 *
 * @return 0 表示发送请求成功 负值表示错误码
 */
int dm_motor::enable() { 
    return send_command(0xFCU); 
}

/**
 * @brief 发送电机失能命令 0xFD
 *
 * @return 0 表示发送请求成功，负值表示错误码
 */
int dm_motor::disable() {
    return send_command(0xFDU);
}

/**
 * @brief 发送清除电机错误命令 0xFB
 *
 * @return 0 表示发送请求成功，负值表示错误码
 */
int dm_motor::clear_error() { 
    return send_command(0xFBU); 
}

/**
 * @brief 将电机当前输出轴位置保存为零点
 *
 * @return 0 表示发送请求成功，负值表示错误码
 */
int dm_motor::save_zero() {
    return send_command(0xFEU);
}

/**
 * @brief 请求写入控制模式寄存器 0x0A
 *
 * @param mode 目标控制模式 应答丢失时可用同一模式重发
 * @return 0 表示写入帧已提交 负值表示参数错误、切换冲突或发送失败
 */
int dm_motor::switch_mode(DmControlMode mode)
{
    if (!valid_mode(mode)) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_ || (mode_change_pending_ && requested_mode_ != mode)) {
        const int result = !initialized_ ? -ENODEV : -EBUSY;
        k_spin_unlock(&lock_, key);
        return result;
    }
    if (mode == mode_ && !mode_change_pending_) {
        k_spin_unlock(&lock_, key);
        return 0;
    }
    const bool was_pending = mode_change_pending_;
    requested_mode_ = mode;
    mode_change_pending_ = true;
    const uint8_t motor_id = motor_id_;
    k_spin_unlock(&lock_, key);

    const uint8_t payload[8] = {motor_id, 0U, 0x55U, kModeRegister, static_cast<uint8_t>(mode), 0U, 0U, 0U};
    const int result = send_frame(kRegisterCommandId, payload, sizeof(payload));
    if (result != 0 && !was_pending) {
        const k_spinlock_key_t reset_key = k_spin_lock(&lock_);
        mode_change_pending_ = false;
        k_spin_unlock(&lock_, reset_key);
    }
    return result;
}

/**
 * @brief 解析电机状态反馈或模式寄存器写入应答
 *
 * @param frame CAN 接收帧 ID 与 master_id 一致
 * @return 0 表示已处理 -ENODEV 表示未初始化 -ENOMSG 表示未匹配 -EBADMSG 表示帧格式错误
 */
int dm_motor::process_feedback(const fdcan_frame &frame)
{
    if (!initialized_) {
        return -ENODEV;
    }
    if (frame.id != master_id_) {
        return -ENOMSG;
    }
    if (frame.id_type != FDCAN_ID_STANDARD || frame.protocol != protocol_ || frame.length != 8U) {
        return -EBADMSG;
    }

    if (frame.data[0] == motor_id_ && frame.data[1] == 0U &&
        frame.data[2] == 0x55U && frame.data[3] == kModeRegister) {
        const k_spinlock_key_t key = k_spin_lock(&lock_);
        if (mode_change_pending_ && frame.data[4] == static_cast<uint8_t>(requested_mode_) &&
            frame.data[5] == 0U && frame.data[6] == 0U && frame.data[7] == 0U) {
            mode_ = requested_mode_;
            mode_change_pending_ = false;
            tx_length_ = 0U;
            k_spin_unlock(&lock_, key);
            return 0;
        }
        k_spin_unlock(&lock_, key);
        return -ENOMSG;
    }

    const uint8_t id = frame.data[0] & 0x0FU;
    if (id != motor_id_) {
        return -ENOMSG;
    }
    const DmRxData raw = {
        id,
        static_cast<uint8_t>(frame.data[0] >> 4U),
        static_cast<uint16_t>((static_cast<uint16_t>(frame.data[1]) << 8U) | frame.data[2]),
        static_cast<uint16_t>((static_cast<uint16_t>(frame.data[3]) << 4U) | (frame.data[4] >> 4U)),
        static_cast<uint16_t>(((static_cast<uint16_t>(frame.data[4]) & 0x0FU) << 8U) | frame.data[5]),
        frame.data[6],
        frame.data[7],
    };
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    rx_data_ = raw;
    data_ = {
        decode(raw.position, limits_.position, kSixteenBitMax),
        decode(raw.velocity, limits_.velocity, kTwelveBitMax),
        decode(raw.torque, limits_.torque, kTwelveBitMax),
        static_cast<float>(raw.mos_temperature),
        static_cast<float>(raw.rotor_temperature),
        raw.status,
    };
    k_spin_unlock(&lock_, key);
    atomic_set(&last_feedback_ms_, static_cast<atomic_val_t>(k_uptime_get_32()));
    atomic_inc(&feedback_count_);
    atomic_set(&has_feedback_, 1);
    return 0;
}

/**
 * @brief 判断最近是否收到有效的电机状态反馈
 *
 * @param timeout_ms 反馈超时时间
 * @return true 在超时时间内收到过反馈
 * @return false 尚未收到反馈或已超时
 */
bool dm_motor::is_online(uint32_t timeout_ms) const
{
    if (atomic_get(&has_feedback_) == 0) {
        return false;
    }
    return static_cast<uint32_t>(k_uptime_get_32() - static_cast<uint32_t>(atomic_get(&last_feedback_ms_))) <= timeout_ms;
}

/**
 * @brief 获取累计有效电机状态反馈帧数
 *
 * @return 已解析的状态反馈帧数，不包含模式切换应答
 */
uint32_t dm_motor::get_feedback_count() const
{
    return static_cast<uint32_t>(atomic_get(&feedback_count_));
}

/**
 * @brief 获取最近一次原始状态反馈
 *
 * @return 受锁保护的原始反馈快照
 */
DmRxData dm_motor::get_rx_data() const
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const DmRxData snapshot = rx_data_;
    k_spin_unlock(&lock_, key);
    return snapshot;
}

/**
 * @brief 获取最近一次换算后的电机状态
 *
 * @return 受锁保护的物理量状态快照
 */
DmData dm_motor::get_data() const
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const DmData snapshot = data_;
    k_spin_unlock(&lock_, key);
    return snapshot;
}

/**
 * @brief 获取最近一次已确认的控制模式
 *
 * @return 初始化指定或已收到写入应答的控制模式
 */
DmControlMode dm_motor::get_mode() const
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const DmControlMode snapshot = mode_;
    k_spin_unlock(&lock_, key);
    return snapshot;
}
