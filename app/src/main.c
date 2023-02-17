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
#include <zephyr/sys/util.h>

#include <stm32l1xx_hal.h>

#include <zephyr/logging/log.h>
#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(main);

/* Defines */
#define LEDS_LEN 6
#define CAPS_NODE DT_ALIAS(caps0)

#define LED_EXISTS(i) !DT_NODE_EXISTS(DT_ALIAS(led##i))

#if FOR_EACH(LED_EXISTS, (||), 0, 1, 2, 3, 4, 5)
#error "Unsupported board: devicetree led alias is not defined"
#endif
#if !DT_NODE_EXISTS(CAPS_NODE)
#error "Unsupported board: devicetree caps0 alias is not defined"
#endif

// Helper macro for initializing a gpio_dt_spec from the devicetree
// with fallback values when the nodes are missing.
#define GPIO_SPEC(node_id) GPIO_DT_SPEC_GET_OR(node_id, gpios, {0})
#define LED_LISTIFY(i, _) GPIO_SPEC(DT_ALIAS(led##i))

/* Local Variables */
// Keyscan releated items
static K_SEM_DEFINE(kscan_sem, 0, 1);
static const struct gpio_dt_spec leds[] = {LISTIFY(LEDS_LEN, LED_LISTIFY, (,))};
static const struct gpio_dt_spec caps = GPIO_SPEC(CAPS_NODE);
static const struct device *const kscan_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_keyboard_scan));

/* Functions */

static void kb_callback(const struct device *dev, uint32_t row, uint32_t col,
			bool pressed) {
	ARG_UNUSED(dev);
	if (col == 0 && row == 0) {
		gpio_pin_set_dt(&leds[0], (int)pressed);
	} else if (col == 1 && row == 0) {
		gpio_pin_set_dt(&leds[1], (int)pressed);
	} else if (col == 2 && row == 0) {
		gpio_pin_set_dt(&leds[2], (int)pressed);
	} else if (col == 3 && row == 0) {
		gpio_pin_set_dt(&leds[3], (int)pressed);
	} else if (col == 4 && row == 0) {
		gpio_pin_set_dt(&leds[4], (int)pressed);
	} else if (col == 6 && row == 5) {
		gpio_pin_set_dt(&leds[5], (int)pressed);
	}
}

static int configure_leds(const struct gpio_dt_spec * const gpio) {
	int ret = 0;
	if (!device_is_ready(gpio->port)) {
		LOG_ERR("GPIO device not ready");
		return -1;
	}
	ret = gpio_pin_configure_dt(gpio, GPIO_OUTPUT);
	if (ret < 0) {
		LOG_ERR("Can't configure GPIO");
	}
	return ret;
}

void main(void) {
	int i;

	// Configure LEDs
	LOG_INF(">> Configuring LEDs");
	if (configure_leds(&caps) < 0) return;
	for (i = 0; i < LEDS_LEN; i++) {
		if (configure_leds(&leds[i]) < 0) return;
	}

	// Configure KSCAN
	LOG_INF(">> Configuring kscan");
	if (kscan_config(kscan_dev, kb_callback) < 0) {
		LOG_ERR("Failed to configure kscan");
		return;
	}

	kscan_enable_callback(kscan_dev);


	LOG_INF(">> Starting main loop");
	while (true) {
		k_msleep(500);
		gpio_pin_toggle_dt(&caps);
	}
}
