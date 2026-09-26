#include "ring_buffer.hpp"

/**
 * @brief 使用调用方提供的存储区初始化环形缓冲
 * 
 * @param buffer 环形缓冲控制结构
 * @param storage 用于保存数据的字节数组
 * @param capacity storage的容量
 */
void ring_buffer_init(ring_buffer_t *buffer, void *storage, size_t capacity)
{
    if (buffer == nullptr) {
        return;
    }

    buffer->storage = static_cast<uint8_t *>(storage);
    buffer->capacity = capacity;
    __atomic_store_n(&buffer->head, 0U, __ATOMIC_RELAXED);
    __atomic_store_n(&buffer->tail, 0U, __ATOMIC_RELAXED);
}

/**
 * @brief 获取环形缓冲中当前已保存的字节数
 * 
 * @param buffer 环形缓冲控制结构
 * @return size_t 当前已保存的字节数
 */
size_t ring_buffer_size(const ring_buffer_t *buffer)
{
    if (buffer == nullptr || buffer->capacity == 0U) {
        return 0U;
    }

    const size_t head = __atomic_load_n(&buffer->head, __ATOMIC_ACQUIRE);
    const size_t tail = __atomic_load_n(&buffer->tail, __ATOMIC_ACQUIRE);
    return head - tail;
}

/**
 * @brief 获取环形缓冲当前剩余的可写字节数
 * 
 * @param buffer 环形缓冲控制结构
 * @return size_t 剩余可写字节数
 */
size_t ring_buffer_free(const ring_buffer_t *buffer)
{
    if (buffer == nullptr) {
        return 0U;
    }

    const size_t used = ring_buffer_size(buffer);
    return used < buffer->capacity ? buffer->capacity - used : 0U;
}

/**
 * @brief 按FIFO顺序向环形缓冲写入字节数据
 * 
 * @param buffer 环形缓冲控制结构
 * @param data 待写入数据的地址
 * @param length 请求写入的字节数
 * @return size_t 实际写入的字节数
 */
size_t ring_buffer_write(ring_buffer_t *buffer, const void *data, size_t length)
{
    if (buffer == nullptr || buffer->storage == nullptr || data == nullptr ||
        buffer->capacity == 0U || length == 0U) {
        return 0U;
    }

    const uint8_t *source = static_cast<const uint8_t *>(data);
    const size_t head = __atomic_load_n(&buffer->head, __ATOMIC_RELAXED);
    const size_t tail = __atomic_load_n(&buffer->tail, __ATOMIC_ACQUIRE);
    const size_t free = buffer->capacity - (head - tail);
    const size_t count = length < free ? length : free;

    for (size_t i = 0U; i < count; ++i) {
        buffer->storage[(head + i) % buffer->capacity] = source[i];
    }
    __atomic_store_n(&buffer->head, head + count, __ATOMIC_RELEASE);
    return count;
}

/**
 * @brief 按FIFO顺序从环形缓冲读取并移除字节数据
 * 
 * @param buffer 环形缓冲控制结构
 * @param data 接收数据的目标地址
 * @param length 请求读取的字节数
 * @return size_t 实际读取的字节数
 */
size_t ring_buffer_read(ring_buffer_t *buffer, void *data, size_t length)
{
    if (buffer == nullptr || buffer->storage == nullptr || data == nullptr ||
        buffer->capacity == 0U || length == 0U) {
        return 0U;
    }

    uint8_t *destination = static_cast<uint8_t *>(data);
    const size_t tail = __atomic_load_n(&buffer->tail, __ATOMIC_RELAXED);
    const size_t head = __atomic_load_n(&buffer->head, __ATOMIC_ACQUIRE);
    const size_t used = head - tail;
    const size_t count = length < used ? length : used;

    for (size_t i = 0U; i < count; ++i) {
        destination[i] = buffer->storage[(tail + i) % buffer->capacity];
    }
    __atomic_store_n(&buffer->tail, tail + count, __ATOMIC_RELEASE);
    return count;
}

/**
 * @brief 判断环形缓冲中是否没有待读取数据
 * 
 * @param buffer 环形缓冲控制结构
 * @return true 缓冲区为空或buffer无效
 * @return false 缓冲区中存在数据
 */
bool ring_buffer_empty(const ring_buffer_t *buffer)
{
    return ring_buffer_size(buffer) == 0U;
}

/**
 * @brief 判断环形缓冲是否已经没有可写空间
 * 
 * @param buffer 环形缓冲控制结构
 * @return true 缓冲区已满
 * @return false 缓冲区未满或buffer无效
 */
bool ring_buffer_full(const ring_buffer_t *buffer)
{
    return buffer != nullptr && buffer->capacity != 0U &&
           ring_buffer_size(buffer) >= buffer->capacity;
}

/**
 * @brief 丢弃环形缓冲中所有尚未读取的数据
 * 
 * @param buffer 环形缓冲控制结构
 */
void ring_buffer_clear(ring_buffer_t *buffer)
{
    if (buffer == nullptr) {
        return;
    }

    const size_t head = __atomic_load_n(&buffer->head, __ATOMIC_ACQUIRE);
    __atomic_store_n(&buffer->tail, head, __ATOMIC_RELEASE);
}
