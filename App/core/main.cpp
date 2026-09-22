#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "c620.hpp"

static DJI_C620 motor;

int main(void)
{
	const struct device *can = DEVICE_DT_GET(DT_ALIAS(can0));
	printk("HPM5361 UART3 alive\n");
	int ret = motor.init(can, 2000);
	printk("C620 init result: %d\n", ret);
	const int init_result = ret;
	int64_t next_report = 0;
	enum can_state can_state = CAN_STATE_STOPPED;
	struct can_bus_err_cnt can_errors = {};

	while (1) {
		if (init_result == 0) {
			ret = motor.set_current(2000);
			while (motor.process_feedback(K_NO_WAIT) == 0) {
			}
		}
		if (k_uptime_get() >= next_report) {
			next_report = k_uptime_get() + 1000;
			int state_ret = can_get_state(can, &can_state, &can_errors);
			printk("UART3 heartbeat\n");
		}
		k_sleep(K_MSEC(10));
	}
	return 0;
}
