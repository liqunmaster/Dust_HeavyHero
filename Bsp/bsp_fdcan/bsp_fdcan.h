#ifndef BSP_FDCAN_H_
#define BSP_FDCAN_H_

#include <zephyr/kernel.h>

#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

int bsp_fdcan_init(fdcan_t *bus, const struct device *device,
                   const fdcan_config_t *config);
int bsp_fdcan_start(fdcan_t *bus);
int bsp_fdcan_stop(fdcan_t *bus);
int bsp_fdcan_deinit(fdcan_t *bus);

int bsp_fdcan_send(fdcan_t *bus, const fdcan_frame_t *frame,
                   k_timeout_t timeout);

int bsp_fdcan_add_rx_filter(fdcan_t *bus, uint32_t id, uint32_t mask,
                            uint8_t flags, fdcan_rx_callback_t callback,
                            void *user_data);
int bsp_fdcan_remove_rx_filter(fdcan_t *bus, int filter_id);

int bsp_fdcan_get_state(const fdcan_t *bus, enum can_state *state,
                        struct can_bus_err_cnt *error_count);
int bsp_fdcan_recover(fdcan_t *bus, k_timeout_t timeout);
bool bsp_fdcan_is_ready(const fdcan_t *bus);

#ifdef __cplusplus
}
#endif

#endif
