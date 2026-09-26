#include "c620.hpp"

// | CAN 控制器 | 命令帧组 | 电机槽位 |
int16_t c620::current_commands_[FDCAN_DEVICE_COUNT][2][4]{};

/**
 * @brief 检查 CAN 控制器编号
 * 
 * @param device CAN 控制器编号
 * @return true 有效
 * @return false 无效
 */
bool c620::valid_device(fdcan_device device)
{
    return device >= FDCAN_DEVICE_CAN0 && device < FDCAN_DEVICE_COUNT;
}

/**
 * @brief 检查 C620 ID
 * 
 * @param id C620 ID
 * @return true 有效
 * @return false 无效
 */
bool c620::valid_id(C620_ID id)
{
    const uint8_t value = static_cast<uint8_t>(id);
    return value >= 1U && value <= 8U;
}

/**
 * @brief 获取电调所属的命令帧组
 * 
 * @param id C620 ID
 * @return uint8_t 0x200/0x1FF
 */
uint8_t c620::group_index(C620_ID id)
{
    return (static_cast<uint8_t>(id) - 1U) / 4U;
}

/**
 * @brief 获取电流槽位
 * 
 * @param id C620 ID
 * @return uint8_t 组内槽位编号
 */
uint8_t c620::slot_index(C620_ID id)
{
    return (static_cast<uint8_t>(id) - 1U) % 4U;
}

/**
 * @brief 读取有符号 16 位整数
 * 
 * @param data C620 反馈帧数据
 * @return int16_t 解包后的有符号整数
 */
int16_t c620::read_i16(const uint8_t *data)
{
    return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8U) | data[1]);
}

/**
 * @brief 读取无符号 16 位整数
 * 
 * @param data C620 反馈帧数据
 * @return uint16_t 解包后的有符号整数
 */
uint16_t c620::read_u16(const uint8_t *data)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8U) | data[1]);
}

/**
 * @brief 初始化 C620 使用的 CAN 总线
 * 
 * @param device CAN 控制器编号
 * @return int 0 表示成功   负值表示失败
 */
int c620::init_bus(fdcan_device device)
{
    if (bsp_fdcan_is_ready(device)) {
        return 0;
    }

    const fdcan_config config = {
        .mode           = FDCAN_MODE_NORMAL,
        .retransmission = FDCAN_RETRANSMISSION_DISABLED,
    };
    const int result = bsp_fdcan_init(device, &config);
    return result    == -EALREADY ? 0 : result;
}

/**
 * @brief C620 初始化
 * 
 * @param device CAN 控制器编号
 * @param id C620 ID
 * @param gear_ratio 减速比
 * @return int 0 表示成功   负值表示失败
 */
int c620::init(fdcan_device device, C620_ID id, float gear_ratio)
{
    if (!valid_device(device) || !valid_id(id) || !__builtin_isfinite(gear_ratio) || gear_ratio <= 0.0F) {
        return -EINVAL;
    }

    const int result = init_bus(device);
    if (result != 0) {
        return result;
    }

    device_      = device;
    id_          = id;
    gear_ratio_  = gear_ratio;
    initialized_ = true;
    return 0;
}

/**
 * @brief 按组发送 C620 电流指令
 * 
 * @param device CAN 控制器编号
 * @param group 命令帧组
 * @return int 0 表示成功   负值表示失败
 */
int c620::transmit_group(fdcan_device device, uint8_t group)
{
    fdcan_frame frame{};
    frame.id = group     == 0U ? 0x200U : 0x1FFU;
    frame.id_type        = FDCAN_ID_STANDARD;
    frame.protocol       = FDCAN_PROTOCOL_CLASSIC;
    frame.bitrate_switch = FDCAN_BRS_DISABLED;
    frame.length         = 8U;

    for (uint8_t slot = 0U; slot < 4U; ++slot) {
        const uint16_t command     = static_cast<uint16_t>(current_commands_[device][group][slot]);
        frame.data[slot * 2U]      = static_cast<uint8_t>(command >> 8U);
        frame.data[slot * 2U + 1U] = static_cast<uint8_t>(command);
    }
    return bsp_fdcan_transmit(device, &frame, FDCAN_NO_WAIT);
}

/**
 * @brief 设置 C620 电流
 * 
 * @param current C620 电流
 * @return int 0 表示成功   负值表示失败
 */
int c620::set_current(float current)
{
    if (!initialized_) {
        return -ENODEV;
    }
    if (!__builtin_isfinite(current)) {
        return -EINVAL;
    }

    const int16_t command = math::current_to_raw(current, current_limit_, current_raw_limit_);
    current_commands_[device_][group_index(id_)][slot_index(id_)] = command;
    target_current_amp_   = math::raw_to_current(command, current_raw_limit_, current_limit_);
    return 0;
}

/**
 * @brief 发送 C620 电流
 * 
 * @return int 0 表示成功   负值表示失败
 */
int c620::transmit()
{
    if (!initialized_) {
        return -ENODEV;
    }
    return transmit_group(device_, group_index(id_));
}

/**
 * @brief 解析校验过的 C620 反馈帧
 * 
 * @param data C620 反馈帧数据
 */
void c620::unpack_feedback(const uint8_t *data)
{
    const uint16_t new_encoder = read_u16(&data[0]);

    if (!encoder_initialized_) {
        total_encoder_         = new_encoder;
        encoder_initialized_   = true;
    } else {
        int32_t delta = static_cast<int32_t>(new_encoder) - encoder_;

        // 将跨越 8191/0 边界的差值还原为最短方向增量。
        if (delta > encoder_resolution_ / 2) {
            delta -= encoder_resolution_;
        } else if (delta < -encoder_resolution_ / 2) {
            delta += encoder_resolution_;
        }
        total_encoder_ += delta;
    }

    encoder_       = new_encoder;
    omega_rpm_     = read_i16(&data[2]);
    current_raw_   = read_i16(&data[4]);
    temperature_c_ = data[6];
    error_         = data[7];

    current_amp_  = math::raw_to_current(current_raw_, current_raw_limit_, current_limit_);
    omega_rad_s_  = math::rpm_to_radian_per_second(omega_rpm_, gear_ratio_);
    angle_rad_    = math::encoder_to_radian(total_encoder_, encoder_resolution_, gear_ratio_);
    angle_degree_ = math::encoder_to_degree(total_encoder_, encoder_resolution_, gear_ratio_);
}

 /**
  * @brief 校验并处理当前电机对应的 C620 反馈帧
  * 
  * @param frame C620 反馈帧
  * @return int 0 表示成功   负值表示失败
  */
int c620::process_feedback(const fdcan_frame &frame)
{
    if (!initialized_) {
        return -ENODEV;
    }
    if (frame.id != 0x200U + static_cast<uint8_t>(id_)) {
        return -ENOMSG;
    }
    if (frame.id_type != FDCAN_ID_STANDARD || frame.protocol != FDCAN_PROTOCOL_CLASSIC || frame.length != 8U) {
        return -EBADMSG;
    }

    unpack_feedback(frame.data);
    return 0;
}

/**
 * @brief 获取 C620 反馈电流 A
 * 
 * @return float 反馈电流 A
 */
float c620::get_current() const { return current_amp_; }

/**
 * @brief 获取 C620 反馈转速 rpm
 * 
 * @return int16_t 反馈转速 rpm
 */
int16_t c620::get_omega() const { return omega_rpm_; }

/**
 * @brief 获取 C620 反馈转速 rad/s
 * 
 * @return float 反馈转速 rad/s
 */
float c620::get_omega_rad_s() const { return omega_rad_s_; }

/**
 * @brief 获取 C620 编码器计数
 * 
 * @return int64_t 编码器计数
 */
int64_t c620::get_total_encoder() const { return total_encoder_; }

/**
 * @brief 获取 C620 输出轴角度 rad
 * 
 * @return float 输出轴角度 rad
 */
float c620::get_angle() const { return angle_rad_; }

/**
 * @brief 获取 C620 输出轴角度 degree
 * 
 * @return float 输出轴角度 degree
 */
float c620::get_angle_degree() const { return angle_degree_; }

/**
 * @brief 获取 C620 单圈转子编码器值
 * 
 * @return uint16_t 编码器值
 */
uint16_t c620::get_encoder() const { return encoder_; }

/**
 * @brief 获取 C620 电机温度 ℃
 * 
 * @return uint8_t 电机温度 ℃
 */
uint8_t c620::get_temperature() const { return temperature_c_; }

/**
 * @brief 获取 C620 错误码
 * 
 * @return uint8_t 错误码
 */
uint8_t c620::get_error() const { return error_; }
