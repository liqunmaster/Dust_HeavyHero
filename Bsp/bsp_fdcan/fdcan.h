#ifndef FDCAN_H_
#define FDCAN_H_

#include <zephyr/drivers/can.h>
typedef struct FdcanRxBuffer {
	uint32_t id;
	uint8_t flags;
	uint8_t len;
	uint8_t data[CAN_MAX_DLEN];
} fdcan_frame_t;

typedef void (*FdcanCallback)(const fdcan_frame_t *frame, void *user_data);

typedef struct {
	uint32_t bitrate;
	uint32_t data_bitrate;
	bool fd_mode;
	struct can_filter rx_filter;
	FdcanCallback callback_function;
	void *callback_user_data;
} fdcan_config_t;

#endif
