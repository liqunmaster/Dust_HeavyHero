#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>

K_SEM_DEFINE(can2_tx_done, 0, 1);
K_SEM_DEFINE(can3_tx_done, 0, 1);

static const struct device *const buses[] = {
	DEVICE_DT_GET(DT_NODELABEL(can2)),
	DEVICE_DT_GET(DT_NODELABEL(can3)),
};

static struct k_sem *const tx_done_sems[] = {
	&can2_tx_done,
	&can3_tx_done,
};

volatile int can23_init_result[2];
volatile int can23_send_result[2];
volatile int can23_tx_result[2];
volatile unsigned int can23_tx_attempts[2];
volatile unsigned int can23_tx_done[2];
volatile unsigned int can23_bus_off[2];
volatile unsigned int can23_restarts[2];

static void tx_done(const struct device *dev, int error, void *user_data)
{
	unsigned int index = (unsigned int)(uintptr_t)user_data;

	(void)dev;
	can23_tx_result[index] = error;
	if (error == 0) {
		++can23_tx_done[index];
	}
	k_sem_give(tx_done_sems[index]);
}

int main(void)
{
	for (unsigned int i = 0; i < 2; ++i) {
		if (!device_is_ready(buses[i])) {
			can23_init_result[i] = -ENODEV;
			continue;
		}

		can23_init_result[i] = can_set_mode(buses[i], CAN_MODE_ONE_SHOT);
		if (can23_init_result[i] == 0) {
			can23_init_result[i] = can_start(buses[i]);
		}
	}

	while (true) {
		for (unsigned int i = 0; i < 2; ++i) {
			if (can23_init_result[i] != 0) {
				continue;
			}
			enum can_state state;
			if (can_get_state(buses[i], &state, NULL) == 0 && state == CAN_STATE_BUS_OFF) {
				++can23_bus_off[i];
				++can23_restarts[i];
				can_stop(buses[i]);
				can23_init_result[i] = can_start(buses[i]);
				if (can23_init_result[i] != 0) {
					continue;
				}
			}

			const unsigned int sequence = ++can23_tx_attempts[i];
			const struct can_frame frame = {
				.id = 0x520 + i,
				.dlc = 8,
				.data = {0x43, 0x41, 0x4e, 0x32 + i,
					 (uint8_t)sequence, (uint8_t)(sequence >> 8),
					 (uint8_t)(sequence >> 16), (uint8_t)(sequence >> 24)},
			};

			k_sem_reset(tx_done_sems[i]);
			can23_send_result[i] = can_send(buses[i], &frame, K_MSEC(100),
							  tx_done, (void *)(uintptr_t)i);
			if (can23_send_result[i] != 0) {
				continue;
			}
			if (k_sem_take(tx_done_sems[i], K_MSEC(100)) != 0) {
				can23_tx_result[i] = -ETIMEDOUT;
				++can23_restarts[i];
				can_stop(buses[i]);
				can23_init_result[i] = can_start(buses[i]);
				continue;
			}
		}
		k_sleep(K_MSEC(500));
	}
}
