#include "bsp_uart.hpp"

extern "C" void __stdout_hook_install(int (*hook)(int));

namespace {
    // UART 控制器运行上下文
    struct uart_context {
        const struct device *device;
        ring_buffer_t rx_buffer;
        ring_buffer_t tx_buffer;
        uint8_t rx_storage[512];
        uint8_t tx_storage[128];
        uint8_t rx_dma[2][128] __aligned(4);
        uint8_t tx_dma[128] __aligned(4);
        bool rx_owned[2];
        bool rx_active;
        bool ready;
        atomic_t rx_overflow;
        atomic_t tx_busy;
        size_t tx_count;
        int tx_error;
        struct k_mutex tx_lock;
        struct k_sem tx_done;
    };

    static uart_context context;

    /**
     * @brief 
     * 
     * @param dev 
     * @param event 
     */
    static void uart_callback(const struct device *dev, struct uart_event *event, void *user_data)
    {
        auto *ctx = static_cast<uart_context *>(user_data);
        if (ctx == nullptr || dev != ctx->device) return;
        const unsigned int key = irq_lock();
        switch (event->type) {
        case UART_RX_RDY:
            if (ring_buffer_write(&ctx->rx_buffer, event->data.rx.buf + event->data.rx.offset, event->data.rx.len) != event->data.rx.len) {
                atomic_set(&ctx->rx_overflow, 1);
            }
        break;
        case UART_RX_BUF_REQUEST:
            for (size_t i = 0; i < 2; ++i) {
                if (!ctx->rx_owned[i]) {
                    ctx->rx_owned[i] = true;
                    if (uart_rx_buf_rsp(dev, ctx->rx_dma[i], sizeof(ctx->rx_dma[i])) != 0) {
                        ctx->rx_owned[i] = false;
                    }
                    break;
                }
            }
            break;
        case UART_RX_BUF_RELEASED:
            for (size_t i = 0; i < 2; ++i) {
                if (event->data.rx_buf.buf == ctx->rx_dma[i]) ctx->rx_owned[i] = false;
            }
            break;
        case UART_RX_DISABLED:
            ctx->rx_active = false;
            break;
        case UART_TX_DONE:
        case UART_TX_ABORTED:
            ctx->tx_count = event->data.tx.len;
            ctx->tx_error = event->type == UART_TX_DONE ? 0 : -EIO;
            atomic_clear(&ctx->tx_busy);
            k_sem_give(&ctx->tx_done);
            break;
        default:
            break;
        }
        irq_unlock(key);
    }

    /**
     * @brief 
     * 
     * @return int 
     */
    static int start_rx(uart_context *ctx)
    {
        ctx->rx_owned[0] = true;
        ctx->rx_owned[1] = false;
        ctx->rx_active = true;
        const int ret = uart_rx_enable(ctx->device, ctx->rx_dma[0], sizeof(ctx->rx_dma[0]), 1000);
        if (ret != 0) ctx->rx_active = false;
        return ret;
    }

    /**
     * @brief 
     * 
     * @param character 
     * @return int 
     */
    static int stdout_character(int character)
    {
        const uint8_t byte = static_cast<uint8_t>(character);
        return bsp_uart_transmit(&byte, 1) == 1 ? byte : EOF;
    }

} 

/**
 * @brief 
 * 
 * @param device 
 * @return int 
 */
int bsp_uart_init(const struct device *device)
{
    if (k_is_in_isr()) return -EWOULDBLOCK;
    if (device == nullptr) device = DEVICE_DT_GET(DT_NODELABEL(uart3));
    uart_context *ctx = &context;
    if (ctx->ready) return ctx->device == device ? 0 : -EBUSY;
    if (!device_is_ready(device)) return -ENODEV;
    ctx->device = device;
    k_mutex_init(&ctx->tx_lock);
    k_sem_init(&ctx->tx_done, 0, 1);
    ring_buffer_init(&ctx->rx_buffer, ctx->rx_storage, sizeof(ctx->rx_storage));
    ring_buffer_init(&ctx->tx_buffer, ctx->tx_storage, sizeof(ctx->tx_storage));
    int ret = uart_callback_set(device, uart_callback, ctx);
    if (ret != 0) return ret;
    ret = start_rx(ctx);
    if (ret != 0) {
        (void)uart_callback_set(device, nullptr, nullptr);
        return ret;
    }
    ctx->ready = true;
    __stdout_hook_install(stdout_character);
    return 0;
}

/**
 * @brief 
 * 
 * @param data 
 * @param length 
 * @return int 
 */
int bsp_uart_receive(void *data, size_t length)
{
    if (k_is_in_isr()) return -EWOULDBLOCK;
    uart_context *ctx = &context;
    if (!ctx->ready) return -ENODEV;
    if (data == nullptr && length != 0) return -EINVAL;
    if (length > INT_MAX) return -EMSGSIZE;
    if (length == 0) return 0;
    const unsigned int key = irq_lock();
    const int ret = ctx->rx_active ? 0 : start_rx(ctx);
    const bool overflow = atomic_set(&ctx->rx_overflow, 0) != 0;
    const int count = overflow ? -ENOBUFS :
        static_cast<int>(ring_buffer_read(&ctx->rx_buffer, data, length));
    irq_unlock(key);
    return count != 0 ? count : ret;
}

/**
 * @brief 
 * 
 * @param data 
 * @param length 
 * @return int 
 */
int bsp_uart_transmit(const void *data, size_t length)
{
    if (k_is_in_isr()) return -EWOULDBLOCK;
    uart_context *ctx = &context;
    if (!ctx->ready) return -ENODEV;
    if (data == nullptr && length != 0) return -EINVAL;
    if (length > INT_MAX) return -EMSGSIZE;
    if (length == 0) return 0;
    k_mutex_lock(&ctx->tx_lock, K_FOREVER);
    const auto *bytes = static_cast<const uint8_t *>(data);
    size_t sent = 0;
    int ret = atomic_get(&ctx->tx_busy) ? -EBUSY : 0;
    while (ret == 0 && sent < length) {
        const int64_t deadline = k_uptime_get() + 1000;
        while (uart_irq_tx_complete(ctx->device) == 0 && k_uptime_get() < deadline) {
            k_usleep(50);
        }
        if (uart_irq_tx_complete(ctx->device) <= 0) {
            ret = -ETIMEDOUT;
            break;
        }
        const size_t chunk = ring_buffer_write(&ctx->tx_buffer, bytes + sent, length - sent);
        ring_buffer_read(&ctx->tx_buffer, ctx->tx_dma, chunk);
        k_sem_reset(&ctx->tx_done);
        ctx->tx_count = 0;
        ctx->tx_error = 0;
        atomic_set(&ctx->tx_busy, 1);
        ret = uart_tx(ctx->device, ctx->tx_dma, chunk, SYS_FOREVER_US);
        if (ret != 0) {
            atomic_clear(&ctx->tx_busy);
            break;
        }
        if (k_sem_take(&ctx->tx_done, K_MSEC(1000)) != 0) {
            const unsigned int key = irq_lock();
            if (atomic_get(&ctx->tx_busy)) {
                (void)uart_tx_abort(ctx->device);
                ret = -ETIMEDOUT;
            }
            irq_unlock(key);
        }
        sent += ctx->tx_count;
        if (ret == 0) ret = ctx->tx_error;
        if (ret == 0 && ctx->tx_count != chunk) ret = -EIO;
    }
    k_mutex_unlock(&ctx->tx_lock);
    return sent != 0 ? static_cast<int>(sent) : ret;
}

/**
 * @brief printf 函数
 * 
 */
extern "C" int printf(const char *format, ...)
{
    if (k_is_in_isr()) {
        errno = EWOULDBLOCK;
        return EOF;
    }
    char message[256];
    va_list args;
    va_start(args, format);
    const int length = vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    if (length < 0) return length;
    if (static_cast<size_t>(length) >= sizeof(message)) {
        errno = EMSGSIZE;
        return EOF;
    }
    const int ret = bsp_uart_transmit(message, length);
    if (ret != length) {
        errno = ret < 0 ? -ret : EIO;
        return EOF;
    }
    return ret;
}
