/*
 * Copyright (c) 2018 qianfan Zhao
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>

#include <stm32l1xx_hal.h>

#include <zephyr/logging/log.h>
#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(main);

/*
 * Devicetree node identifiers for the buttons and LED this sample
 * supports.
 */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define LED3_NODE DT_ALIAS(led3)
#define LED4_NODE DT_ALIAS(led4)
#define LED5_NODE DT_ALIAS(led5)
#define CAPS_NODE DT_ALIAS(caps0)

#if !DT_NODE_EXISTS(LED0_NODE)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

/*
 * Helper macro for initializing a gpio_dt_spec from the devicetree
 * with fallback values when the nodes are missing.
 */
#define GPIO_SPEC(node_id) GPIO_DT_SPEC_GET_OR(node_id, gpios, {0})

/*
 * Create gpio_dt_spec structures from the devicetree.
 */
static const struct gpio_dt_spec led0 = GPIO_SPEC(LED0_NODE);
static const struct gpio_dt_spec led1 = GPIO_SPEC(LED1_NODE);
static const struct gpio_dt_spec led2 = GPIO_SPEC(LED2_NODE);
static const struct gpio_dt_spec led3 = GPIO_SPEC(LED3_NODE);
static const struct gpio_dt_spec led4 = GPIO_SPEC(LED4_NODE);
static const struct gpio_dt_spec led5 = GPIO_SPEC(LED5_NODE);
static const struct gpio_dt_spec caps = GPIO_SPEC(CAPS_NODE);
const struct device *const kscan_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_keyboard_scan));

static void kb_callback(const struct device *dev, uint32_t row, uint32_t col,
			bool pressed)
{
	ARG_UNUSED(dev);
	if (col == 0 && row == 0) {
		gpio_pin_set_dt(&led0, (int)pressed);
	} else if (col == 1 && row == 0) {
		gpio_pin_set_dt(&led1, (int)pressed);
	} else if (col == 2 && row == 0) {
		gpio_pin_set_dt(&led2, (int)pressed);
	} else if (col == 3 && row == 0) {
		gpio_pin_set_dt(&led3, (int)pressed);
	} else if (col == 4 && row == 0) {
		gpio_pin_set_dt(&led4, (int)pressed);
	} else if (col == 6 && row == 5) {
		gpio_pin_set_dt(&led5, (int)pressed);
	}
}


void main(void)
{
	int ret = 0;
	if (!device_is_ready(led0.port) ||
	    !device_is_ready(led1.port) ||
	    !device_is_ready(led2.port) ||
	    !device_is_ready(led3.port) ||
	    !device_is_ready(led4.port) ||
	    !device_is_ready(led5.port) ||
	    !device_is_ready(caps.port)) {
		LOG_ERR("LED device %s is not ready", led0.port->name);
		return;
	}
	ret |= gpio_pin_configure_dt(&led0, GPIO_OUTPUT);
	ret |= gpio_pin_configure_dt(&led1, GPIO_OUTPUT);
	ret |= gpio_pin_configure_dt(&led2, GPIO_OUTPUT);
	ret |= gpio_pin_configure_dt(&led3, GPIO_OUTPUT);
	ret |= gpio_pin_configure_dt(&led4, GPIO_OUTPUT);
	ret |= gpio_pin_configure_dt(&led5, GPIO_OUTPUT);
	ret |= gpio_pin_configure_dt(&caps, GPIO_OUTPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure the LED pin, error: %d", ret);
		return;
	}
	if (kscan_config(kscan_dev, kb_callback) < 0) {
		LOG_ERR("Failed to configure kscan");
		return;
	}
	LOG_INF("Enabling kscan");
	kscan_enable_callback(kscan_dev);

	LOG_INF("Starting main loop");
	while (true) {
		k_msleep(500);
		gpio_pin_toggle_dt(&caps);
	}
}
