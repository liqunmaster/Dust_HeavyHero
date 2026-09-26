#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define CAN2_TXD_GPIO DEVICE_DT_GET(DT_NODELABEL(gpioy))
#define CAN2_TXD_PIN 0U

volatile unsigned int can2_txd_probe_cycles;
volatile int can2_txd_probe_error;

int main(void)
{
	const struct device *gpio = CAN2_TXD_GPIO;

	if (!device_is_ready(gpio)) {
		can2_txd_probe_error = -1;
		return -1;
	}

	can2_txd_probe_error = gpio_pin_configure(gpio, CAN2_TXD_PIN, GPIO_OUTPUT_HIGH);
	if (can2_txd_probe_error != 0) {
		return can2_txd_probe_error;
	}

	while (true) {
		gpio_pin_set(gpio, CAN2_TXD_PIN, 0);
		k_sleep(K_MSEC(500));
		gpio_pin_set(gpio, CAN2_TXD_PIN, 1);
		k_sleep(K_MSEC(500));
		++can2_txd_probe_cycles;
	}
}
