#include "bsp_uart.h"
#include <errno.h>
#include <string.h>

static bsp_uart_t *stdio_uart;

static void async_callback(const struct device *device, struct uart_event *event, void *user_data)
{
    bsp_uart_t *uart = user_data;
    if (uart == NULL || device != uart->device) return;
    if (event->type == UART_TX_DONE || event->type == UART_TX_ABORTED) {
        if (atomic_cas(&uart->tx_busy, 1, 0)) {
            k_sem_give(&uart->tx_available);
        }
    } else if (event->type == UART_RX_RDY) {
        if (uart->rx_callback != NULL && event->data.rx.len > 0U)
            uart->rx_callback(event->data.rx.buf + event->data.rx.offset, event->data.rx.len, uart->rx_user_data);
    } else if (event->type == UART_RX_BUF_REQUEST) {
        uint8_t *next = event->data.rx_buf.buf == uart->rx_buffers[0] ? uart->rx_buffers[1] : uart->rx_buffers[0];
        (void)uart_rx_buf_rsp(uart->device, next, BSP_UART_RX_BUFFER_SIZE);
    } else if (event->type == UART_RX_DISABLED && uart->rx_callback != NULL) {
        (void)uart_rx_enable(uart->device, uart->rx_buffers[0],
                             BSP_UART_RX_BUFFER_SIZE, SYS_FOREVER_MS);
    }
}

int bsp_uart_init(bsp_uart_t *uart, const struct device *device, const struct uart_config *config)
{
    if (uart == NULL || device == NULL || config == NULL || !device_is_ready(device)) return -EINVAL;
    if (uart->initialized) return uart->device == device ? 0 : -EBUSY;
    uart->device = device;
    int ret = uart_configure(device, config);
    if (ret != 0) return ret;
    k_sem_init(&uart->tx_available, 1, 1);
    atomic_clear(&uart->tx_busy);
    ret = uart_callback_set(device, async_callback, uart);
    if (ret != 0) return ret;
    uart->initialized = true;
    return 0;
}

int bsp_uart_rx_attach(bsp_uart_t *uart, bsp_uart_rx_callback_t callback,
                       void *user_data)
{
    if (uart == NULL || !uart->initialized || callback == NULL) return -EINVAL;
    uart->rx_callback = callback;
    uart->rx_user_data = user_data;
    return uart_rx_enable(uart->device, uart->rx_buffers[0],
                          BSP_UART_RX_BUFFER_SIZE, SYS_FOREVER_MS);
}

static int write_with_timeout(bsp_uart_t *uart, const uint8_t *data,
                              size_t length, k_timeout_t timeout)
{
    if (uart == NULL || !uart->initialized || data == NULL || length == 0U)
        return -EINVAL;
    if (length > BSP_UART_TX_ITEM_SIZE) return -EMSGSIZE;
    if (k_sem_take(&uart->tx_available, timeout) != 0) return -EBUSY;
    atomic_set(&uart->tx_busy, 1);
    memcpy(uart->tx_buffer, data, length);
    const int ret = uart_tx(uart->device, uart->tx_buffer, length, 50);
    if (ret != 0 && atomic_cas(&uart->tx_busy, 1, 0)) {
        k_sem_give(&uart->tx_available);
    }
    return ret;
}

int bsp_uart_write(bsp_uart_t *uart, const uint8_t *data, size_t length)
{
    return write_with_timeout(uart, data, length, K_NO_WAIT);
}

void bsp_uart_set_stdio(bsp_uart_t *uart) { stdio_uart = uart; }
int bsp_uart_stdio_write(const uint8_t *data, size_t length)
{
    return write_with_timeout(stdio_uart, data, length, K_MSEC(20));
}
