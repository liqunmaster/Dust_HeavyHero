# 双缓冲与环形缓冲

本目录提供可以直接操作数组的 C++ 实现，不使用类、模板或命名空间：

- `double_buffer.hpp/.cpp`：保存最新的一组完整数据。
- `ring_buffer.hpp/.cpp`：按顺序保存连续到达的字节流。

两种缓冲都不使用动态内存，数组由调用方提供。

## 双缓冲

双缓冲适合 IMU、遥控器、ADC 或 DMA 每次产生一整组数据，并且使用方只关心
最新完整结果的场景。

```cpp
#include "Algorithm/buffer/double_buffer.hpp"

static uint8_t buffer_a[64];
static uint8_t buffer_b[64];
static double_buffer_t imu_buffer;

void init()
{
    double_buffer_init(&imu_buffer, buffer_a, buffer_b, sizeof(buffer_a));
}

void receive_one_frame(const uint8_t *data)
{
    memcpy(double_buffer_write_array(&imu_buffer), data, sizeof(buffer_a));
    double_buffer_publish(&imu_buffer);
}

void process_latest_frame()
{
    uint8_t data[64];
    if (double_buffer_snapshot(&imu_buffer, data)) {
        // 使用 data
    }
}
```

写入顺序必须是：取得写数组、写完整帧、调用 `double_buffer_publish()`。读取端
可以通过 `double_buffer_read_array()` 直接取得当前数组；跨线程或中断读取时，
优先通过 `double_buffer_snapshot()` 复制稳定快照。

双缓冲不保存历史帧。新的完整帧发布后，旧帧最终会被覆盖。

## 环形缓冲

环形缓冲适合 UART、CAN、SPI 等连续数据流。每个字节保持先进先出顺序，生产端
和消费端可以短时间使用不同速率。

```cpp
#include "Algorithm/buffer/ring_buffer.hpp"

static uint8_t storage[256];
static ring_buffer_t uart_rx;

void init()
{
    ring_buffer_init(&uart_rx, storage, sizeof(storage));
}

void uart_rx_callback(const uint8_t *data, size_t length)
{
    const size_t written = ring_buffer_write(&uart_rx, data, length);
    if (written != length) {
        // 缓冲区已满，记录丢包
    }
}

void process_uart()
{
    uint8_t data[32];
    const size_t length = ring_buffer_read(&uart_rx, data, sizeof(data));
    if (length > 0U) {
        // 处理 data[0..length-1]
    }
}
```

`ring_buffer_write()` 和 `ring_buffer_read()` 返回实际处理的字节数。空间不足时，
写入函数不会阻塞，也不会覆盖尚未读取的数据。

## 如何选择

```text
只关心最新的一整组数据        -> 双缓冲
每个字节或消息都要按顺序处理  -> 环形缓冲
DMA 完成后整体交付一帧         -> 双缓冲
UART/SPI 连续接收字节流        -> 环形缓冲
```

当前并发模型是一个生产者和一个消费者，例如一个外设 ISR 写入、一个线程读取。
多个中断或多个线程同时写入同一个缓冲时，需要在外层增加临界区或互斥锁。
