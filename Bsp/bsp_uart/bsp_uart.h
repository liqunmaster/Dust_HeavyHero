#pragma once
#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#ifdef __cplusplus
extern "C" {
#endif
#define BSP_UART_TX_ITEM_SIZE 128U
#define BSP_UART_RX_BUFFER_SIZE 64U
typedef void (*bsp_uart_rx_callback_t)(const uint8_t *, size_t, void *);
typedef struct {
    const struct device *device;
    bsp_uart_rx_callback_t rx_callback;
    void *rx_user_data;
    uint8_t rx_buffers[2][BSP_UART_RX_BUFFER_SIZE] __aligned(4);
    uint8_t tx_buffer[BSP_UART_TX_ITEM_SIZE] __aligned(4);
    atomic_t tx_busy;
    bool initialized;
} bsp_uart_t;
int bsp_uart_init(bsp_uart_t *, const struct device *, const struct uart_config *);
int bsp_uart_rx_attach(bsp_uart_t *, bsp_uart_rx_callback_t, void *);
int bsp_uart_write(bsp_uart_t *, const uint8_t *, size_t);
void bsp_uart_set_stdio(bsp_uart_t *);
int bsp_uart_stdio_write(const uint8_t *, size_t);
#ifdef __cplusplus
}
#endif
