#include "dji_motor.hpp"

#include <errno.h>

namespace {
    int16_t commands[FDCAN_DEVICE_COUNT][2][4]{};
    struct k_spinlock commands_lock{};
}

/**
 * @brief 检查 CAN 控制器编号是否有效
 *
 * @param device CAN 控制器编号
 * @return true 编号在支持范围内
 * @return false 编号无效
 */
bool dji_motor::valid_device(fdcan_device device)
{
    return device >= FDCAN_DEVICE_CAN0 && device < FDCAN_DEVICE_COUNT;
}

/**
 * @brief 检查 dji ID 是否有效
 *
 * @param motor_id 电调 ID
 * @return true ID 有效 false ID 无效
 */
bool dji_motor::valid_id(int motor_id)
{
    return motor_id >= 1 && motor_id <= 8;
}

/**
 * @brief 初始化尚未就绪的 CAN 控制器
 *
 * @param device CAN 控制器编号
 * @return 0 表示成功 负值表示失败
 */
int dji_motor::init_bus(fdcan_device device)
{
    if (!valid_device(device)) {
        return -EINVAL;
    }
    if (bsp_fdcan_is_ready(device)) {
        return 0;
    }

    const fdcan_config config = {
        .mode = FDCAN_MODE_NORMAL,
        .retransmission = FDCAN_RETRANSMISSION_DISABLED,
    };
    const int result = bsp_fdcan_init(device, &config);
    return result == -EALREADY ? 0 : result;
}

/**
 * @brief 按大端字节顺序读取反馈报文中的 16 位字段
 *
 * @param data 指向至少 2 字节的反馈数据
 * @return 解码后的无符号 16 位值
 */
uint16_t dji_motor::read_u16(const uint8_t *data)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8U) | data[1]);
}

/**
 * @brief 将 16 位补码位模式转换为有符号值
 *
 * @param raw 报文中的 16 位原始值
 * @return 转换后的有符号 16 位值
 */
int16_t dji_motor::signed_value(uint16_t raw)
{
    return static_cast<int16_t>(static_cast<int32_t>(raw) - ((raw & 0x8000U) != 0U ? 0x10000 : 0));
}

/**
 * @brief 校验指定电调的反馈帧 ID、帧类型和长度
 *
 * @param frame 收到的 CAN 帧
 * @param motor_id 电调 ID，范围 1~8
 * @return 0 表示有效 -ENOMSG 表示 ID 不匹配 -EBADMSG 表示帧格式错误
 */
int dji_motor::validate_feedback(const fdcan_frame &frame, uint8_t motor_id)
{
    if (frame.id != 0x200U + motor_id) {
        return -ENOMSG;
    }
    if (frame.id_type != FDCAN_ID_STANDARD || frame.protocol != FDCAN_PROTOCOL_CLASSIC || frame.length != 8U) {
        return -EBADMSG;
    }
    return 0;
}

/**
 * @brief 更新电调所在分组的电流指令
 *
 * @param device CAN 控制器编号
 * @param motor_id 电调 ID
 * @param raw 电流指令原始值
 * @return 0 表示成功 负值表示错误
 */
int dji_motor::set(fdcan_device device, uint8_t motor_id, int16_t raw)
{
    if (!valid_device(device) || !valid_id(motor_id)) {
        return -EINVAL;
    }

    const uint8_t group = (motor_id - 1U) / 4U;
    const uint8_t slot = (motor_id - 1U) % 4U;
    const k_spinlock_key_t key = k_spin_lock(&commands_lock);
    commands[device][group][slot] = raw;
    k_spin_unlock(&commands_lock, key);
    return 0;
}

/**
 * @brief 发送电调所在分组的四路电流指令
 *
 * @param device CAN 控制器编号
 * @param motor_id 电调 ID，决定 0x200/0x1FF
 * @return 0 表示成功 负值表示错误
 */
int dji_motor::transmit(fdcan_device device, uint8_t motor_id)
{
    if (!valid_device(device) || !valid_id(motor_id)) {
        return -EINVAL;
    }

    const uint8_t group = (motor_id - 1U) / 4U;
    int16_t snapshot[4];
    const k_spinlock_key_t key = k_spin_lock(&commands_lock);
    for (uint8_t slot = 0U; slot < 4U; ++slot) {
        snapshot[slot] = commands[device][group][slot];
    }
    k_spin_unlock(&commands_lock, key);

    fdcan_frame frame{};
    frame.id = group == 0U ? 0x200U : 0x1FFU;
    frame.id_type = FDCAN_ID_STANDARD;
    frame.protocol = FDCAN_PROTOCOL_CLASSIC;
    frame.bitrate_switch = FDCAN_BRS_DISABLED;
    frame.length = 8U;

    for (uint8_t slot = 0U; slot < 4U; ++slot) {
        const uint16_t value = static_cast<uint16_t>(snapshot[slot]);
        frame.data[slot * 2U] = static_cast<uint8_t>(value >> 8U);
        frame.data[slot * 2U + 1U] = static_cast<uint8_t>(value);
    }
    return bsp_fdcan_transmit(device, &frame, FDCAN_NO_WAIT);
}
