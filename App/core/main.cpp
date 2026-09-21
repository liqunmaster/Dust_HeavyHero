#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main);

#define LED0_NODE DT_ALIAS(led0)

int main(void)
{
	const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
	printk("Hello from HPM5361 (Zephyr)!\n");

	int ret = 0;
#if DT_NODE_EXISTS(LED0_NODE)
	const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
	ret = gpio_is_ready_dt(&led);
	if (ret) {
		ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	}
#endif

	(void)uart;
	(void)ret;

	while (1) {
		k_sleep(K_SECONDS(1));
		LOG_INF("tick");
	}
	return 0;
}