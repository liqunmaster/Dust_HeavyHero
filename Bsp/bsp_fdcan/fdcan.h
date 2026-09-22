#ifndef FDCAN_H_
#define FDCAN_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_FDCAN_MAX_RX_FILTERS 8U

typedef struct {
    uint32_t id;
    uint8_t flags;
    uint8_t len;
    uint8_t data[CAN_MAX_DLEN];
} fdcan_frame_t;

typedef void (*fdcan_rx_callback_t)(const fdcan_frame_t *frame, void *user_data);

typedef struct {
    uint32_t bitrate;
    uint32_t data_bitrate;
    can_mode_t mode;
} fdcan_config_t;

struct fdcan;

typedef struct {
    struct fdcan *owner;
    fdcan_rx_callback_t callback;
    void *user_data;
    int filter_id;
    bool used;
} fdcan_rx_slot_t;

typedef struct fdcan {
    const struct device *device;
    fdcan_config_t config;
    fdcan_rx_slot_t rx_slots[BSP_FDCAN_MAX_RX_FILTERS];
    bool started;
} fdcan_t;

#ifdef __cplusplus
}
#endif

#endif
