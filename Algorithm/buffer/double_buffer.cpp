#include "double_buffer.hpp"

/**
 * @brief 初始化双缓冲并将第一个缓冲区设为当前可读缓冲区
 * 
 * @param buffer 双缓冲控制结构
 * @param first 第一个缓冲区地址
 * @param second 第二个缓冲区地址
 * @param size 每个缓冲区的字节数
 */
void double_buffer_init(double_buffer_t *buffer, void *first, void *second, size_t size)
{
    if (buffer == nullptr) {
        return;
    }

    buffer->storage[0] = static_cast<uint8_t *>(first);
    buffer->storage[1] = static_cast<uint8_t *>(second);
    buffer->size = size;
    buffer->write_index = 1U;
    __atomic_store_n(&buffer->published, 0U, __ATOMIC_RELAXED);
    __atomic_store_n(&buffer->generation, 0U, __ATOMIC_RELAXED);
}

/**
 * @brief 获取当前可写缓冲区的地址
 * 
 * @param buffer 双缓冲控制结构
 * @return void* 当前可写缓冲区地址
 */
void *double_buffer_write_array(double_buffer_t *buffer)
{
    if (buffer == nullptr) {
        return nullptr;
    }
    return buffer->storage[buffer->write_index];
}

/**
 * @brief 发布已写完的数据并切换下一次写入使用的缓冲区
 * 
 * @param buffer 双缓冲控制结构
 */
void double_buffer_publish(double_buffer_t *buffer)
{
    if (buffer == nullptr) {
        return;
    }

    __atomic_store_n(&buffer->published, buffer->write_index, __ATOMIC_RELAXED);
    __atomic_add_fetch(&buffer->generation, 1U, __ATOMIC_RELEASE);
    buffer->write_index ^= 1U;
}

/**
 * @brief 获取最近一次已发布缓冲区的只读地址
 * 
 * @param buffer 双缓冲控制结构
 * @return const void* 最近发布的缓冲区地址
 */
const void *double_buffer_read_array(const double_buffer_t *buffer)
{
    if (buffer == nullptr) {
        return nullptr;
    }

    (void)__atomic_load_n(&buffer->generation, __ATOMIC_ACQUIRE);
    const uint8_t index = __atomic_load_n(&buffer->published, __ATOMIC_RELAXED);
    return buffer->storage[index];
}

/**
 * @brief 将最近发布的完整数据复制到目标缓冲区
 * 
 * @param buffer 双缓冲控制结构
 * @param destination 容量至少为buffer->size字节的目标缓冲区
 * @return true 复制成功
 * @return false 参数无效或缓冲区大小为0
 */
bool double_buffer_snapshot(const double_buffer_t *buffer, void *destination)
{
    if (buffer == nullptr || destination == nullptr || buffer->size == 0U) {
        return false;
    }

    for (;;) {
        const uint32_t before =
            __atomic_load_n(&buffer->generation, __ATOMIC_ACQUIRE);
        const uint8_t index =
            __atomic_load_n(&buffer->published, __ATOMIC_RELAXED);
        memcpy(destination, buffer->storage[index], buffer->size);
        const uint32_t after =
            __atomic_load_n(&buffer->generation, __ATOMIC_ACQUIRE);
        if (before == after) {
            return true;
        }
    }
}

/**
 * @brief 使用指定数据重置两个缓冲区并恢复初始读写状态
 * 
 * @param buffer 双缓冲控制结构
 * @param value 长度为buffer->size字节的初始数据
 */
void double_buffer_reset(double_buffer_t *buffer, const void *value)
{
    if (buffer == nullptr || value == nullptr || buffer->size == 0U) {
        return;
    }

    memcpy(buffer->storage[0], value, buffer->size);
    memcpy(buffer->storage[1], value, buffer->size);
    buffer->write_index = 1U;
    __atomic_store_n(&buffer->published, 0U, __ATOMIC_RELAXED);
    __atomic_add_fetch(&buffer->generation, 1U, __ATOMIC_RELEASE);
}
