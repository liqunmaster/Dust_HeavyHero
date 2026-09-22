#include "bsp_uart.h"
#include <errno.h>
#include <string.h>

static bsp_uart_t *stdio_uart;

static void async_callback(const struct device *device, struct uart_event *event, void *user_data)
{
    bsp_uart_t *uart = user_data;
    if (uart == NULL || device != uart->device) return;
    if (event->type == UART_TX_DONE || event->type == UART_TX_ABORTED) {
        atomic_clear(&uart->tx_busy);
    } else if (event->type == UART_RX_RDY) {
        if (uart->rx_callback != NULL && event->data.rx.len > 0U)
            uart->rx_callback(event->data.rx.buf + event->data.rx.offset,
                              event->data.rx.len, uart->rx_user_data);
    } else if (event->type == UART_RX_BUF_REQUEST) {
        uint8_t *next = event->data.rx_buf.buf == uart->rx_buffers[0]
                            ? uart->rx_buffers[1] : uart->rx_buffers[0];
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

int bsp_uart_write(bsp_uart_t *uart, const uint8_t *data, size_t length)
{
    if (uart == NULL || !uart->initialized || data == NULL || length == 0U)
        return -EINVAL;
    if (length > BSP_UART_TX_ITEM_SIZE) return -EMSGSIZE;
    if (!atomic_cas(&uart->tx_busy, 0, 1)) return -EBUSY;
    memcpy(uart->tx_buffer, data, length);
    const int ret = uart_tx(uart->device, uart->tx_buffer, length, 50);
    if (ret != 0) atomic_clear(&uart->tx_busy);
    return ret;
}

void bsp_uart_set_stdio(bsp_uart_t *uart) { stdio_uart = uart; }
int bsp_uart_stdio_write(const uint8_t *data, size_t length)
{ return bsp_uart_write(stdio_uart, data, length); }
