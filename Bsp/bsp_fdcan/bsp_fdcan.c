#include "bsp_fdcan.h"

#include <errno.h>
#include <string.h>

static void fdcan_zephyr_rx_callback(const struct device *device,
                                     struct can_frame *frame,
                                     void *user_data)
{
    ARG_UNUSED(device);

    fdcan_rx_slot_t *slot = user_data;
    if (slot == NULL || !slot->used || slot->callback == NULL) {
        return;
    }

    fdcan_frame_t received = {
        .id = frame->id,
        .flags = frame->flags,
        .len = can_dlc_to_bytes(frame->dlc),
    };
    memcpy(received.data, frame->data, received.len);
    slot->callback(&received, slot->user_data);
}

bool bsp_fdcan_is_ready(const fdcan_t *bus)
{
    return bus != NULL && bus->device != NULL &&
           device_is_ready(bus->device);
}

int bsp_fdcan_init(fdcan_t *bus, const struct device *device,
                   const fdcan_config_t *config)
{
    if (bus == NULL || device == NULL || config == NULL) {
        return -EINVAL;
    }
    if (!device_is_ready(device)) {
        return -ENODEV;
    }

    memset(bus, 0, sizeof(*bus));
    bus->device = device;
    bus->config = *config;
    for (size_t i = 0U; i < BSP_FDCAN_MAX_RX_FILTERS; ++i) {
        bus->rx_slots[i].filter_id = -1;
    }

    int ret = can_stop(device);
    if (ret != 0 && ret != -EALREADY) {
        return ret;
    }

    ret = can_set_mode(device, config->mode);
    if (ret != 0) {
        return ret;
    }

    if (config->bitrate != 0U) {
        ret = can_set_bitrate(device, config->bitrate);
        if (ret != 0) {
            return ret;
        }
    }

    if ((config->mode & CAN_MODE_FD) != 0U) {
#if defined(CONFIG_CAN_FD_MODE)
        if (config->data_bitrate == 0U) {
            return -EINVAL;
        }
        ret = can_set_bitrate_data(device, config->data_bitrate);
        if (ret != 0) {
            return ret;
        }
#else
        return -ENOTSUP;
#endif
    }

    return bsp_fdcan_start(bus);
}

int bsp_fdcan_start(fdcan_t *bus)
{
    if (!bsp_fdcan_is_ready(bus)) {
        return -ENODEV;
    }
    if (bus->started) {
        return 0;
    }

    int ret = can_start(bus->device);
    if (ret == 0 || ret == -EALREADY) {
        bus->started = true;
        return 0;
    }
    return ret;
}

int bsp_fdcan_stop(fdcan_t *bus)
{
    if (!bsp_fdcan_is_ready(bus)) {
        return -ENODEV;
    }
    if (!bus->started) {
        return 0;
    }

    int ret = can_stop(bus->device);
    if (ret == 0 || ret == -EALREADY) {
        bus->started = false;
        return 0;
    }
    return ret;
}

int bsp_fdcan_deinit(fdcan_t *bus)
{
    if (bus == NULL) {
        return -EINVAL;
    }

    for (size_t i = 0U; i < BSP_FDCAN_MAX_RX_FILTERS; ++i) {
        if (bus->rx_slots[i].used) {
            can_remove_rx_filter(bus->device, bus->rx_slots[i].filter_id);
            memset(&bus->rx_slots[i], 0, sizeof(bus->rx_slots[i]));
            bus->rx_slots[i].filter_id = -1;
        }
    }

    int ret = bsp_fdcan_stop(bus);
    if (ret == 0) {
        bus->device = NULL;
    }
    return ret;
}

int bsp_fdcan_send(fdcan_t *bus, const fdcan_frame_t *frame, k_timeout_t timeout)
{
    if (!bsp_fdcan_is_ready(bus) || frame == NULL) {
        return -EINVAL;
    }
    if (!bus->started) {
        return -ENETDOWN;
    }
    if (((frame->flags & CAN_FRAME_IDE) == 0U &&
         frame->id > CAN_STD_ID_MASK) ||
        ((frame->flags & CAN_FRAME_IDE) != 0U &&
         frame->id > CAN_EXT_ID_MASK)) {
        return -EINVAL;
    }

    const uint8_t max_length =
        (frame->flags & CAN_FRAME_FDF) != 0U ? CAN_MAX_DLEN : 8U;
    if (frame->len > max_length) {
        return -EMSGSIZE;
    }

    struct can_frame zephyr_frame = {
        .id = frame->id,
        .dlc = can_bytes_to_dlc(frame->len),
        .flags = frame->flags,
    };
    memcpy(zephyr_frame.data, frame->data, frame->len);

    return can_send(bus->device, &zephyr_frame, timeout, NULL, NULL);
}

int bsp_fdcan_add_rx_filter(fdcan_t *bus, uint32_t id, uint32_t mask, uint8_t flags, fdcan_rx_callback_t callback, void *user_data)
{
    if (!bsp_fdcan_is_ready(bus) || callback == NULL) {
        return -EINVAL;
    }

    size_t slot_index = BSP_FDCAN_MAX_RX_FILTERS;
    for (size_t i = 0U; i < BSP_FDCAN_MAX_RX_FILTERS; ++i) {
        if (!bus->rx_slots[i].used) {
            slot_index = i;
            break;
        }
    }
    if (slot_index == BSP_FDCAN_MAX_RX_FILTERS) {
        return -ENOSPC;
    }

    fdcan_rx_slot_t *slot = &bus->rx_slots[slot_index];
    slot->owner = bus;
    slot->callback = callback;
    slot->user_data = user_data;
    slot->used = true;

    const struct can_filter filter = {
        .id = id,
        .mask = mask,
        .flags = flags,
    };

    const int filter_id = can_add_rx_filter(bus->device, fdcan_zephyr_rx_callback, slot, &filter);
    if (filter_id < 0) {
        memset(slot, 0, sizeof(*slot));
        slot->filter_id = -1;
        return filter_id;
    }

    slot->filter_id = filter_id;
    return filter_id;
}

int bsp_fdcan_remove_rx_filter(fdcan_t *bus, int filter_id)
{
    if (!bsp_fdcan_is_ready(bus) || filter_id < 0) {
        return -EINVAL;
    }

    for (size_t i = 0U; i < BSP_FDCAN_MAX_RX_FILTERS; ++i) {
        fdcan_rx_slot_t *slot = &bus->rx_slots[i];
        if (slot->used && slot->filter_id == filter_id) {
            can_remove_rx_filter(bus->device, filter_id);
            memset(slot, 0, sizeof(*slot));
            slot->filter_id = -1;
            return 0;
        }
    }
    return -ENOENT;
}

int bsp_fdcan_get_state(const fdcan_t *bus, enum can_state *state, struct can_bus_err_cnt *error_count)
{
    if (!bsp_fdcan_is_ready(bus) || state == NULL || error_count == NULL) {
        return -EINVAL;
    }
    return can_get_state(bus->device, state, error_count);
}

int bsp_fdcan_recover(fdcan_t *bus, k_timeout_t timeout)
{
    if (!bsp_fdcan_is_ready(bus)) {
        return -ENODEV;
    }
    return can_recover(bus->device, timeout);
}
