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
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

#include <stm32l1xx_hal.h>

#include "drivers/kscan_gpio.h"
#include "drivers/kscan_uart.h"

#include <zephyr/logging/log.h>
#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(main);

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	     "Console device is not ACM CDC UART device");

/*******************************************************************/
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

// Macro for converting keycodes to a bitmask
#define REPORT_IDX(c) ((c / 8) + 1)
#define REPORT_MASK(c) (0x1 << (c % 8))

/*******************************************************************/
/* Local Variables */

// Keyscan releated items
static K_SEM_DEFINE(kscan_sem, 0, 1);
static const struct gpio_dt_spec leds[] = {LISTIFY(LEDS_LEN, LED_LISTIFY, (,))};
static const struct gpio_dt_spec caps = GPIO_SPEC(CAPS_NODE);
static const struct device *const kscan_gpio_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_kscan_gpio));
static const struct device *const kscan_uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_kscan_uart));

// USB related items, default to 1 so someone can grab it
static K_SEM_DEFINE(usb_sem, 1, 1);
// Custom keyboard report descriptor to allow for a n-key rollover
static const uint8_t hid_kbd_report_desc[] = {
	HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),			\
	HID_USAGE(HID_USAGE_GEN_DESKTOP_KEYBOARD),		\
	HID_COLLECTION(HID_COLLECTION_APPLICATION),		\
								\
		/* Setup a bitmap for modifier keys */		\
		HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP_KEYPAD),	\
		HID_USAGE_MIN8(0xE0), /* = LeftControl */ 	\
		HID_USAGE_MAX8(0xE7), /* = Right GUI */		\
		HID_LOGICAL_MIN8(0),				\
		HID_LOGICAL_MAX8(1),				\
		HID_REPORT_SIZE(1),				\
		HID_REPORT_COUNT(8),				\
		HID_INPUT(0x02),				\
								\
		/* Setup a bitmap for normal keys */		\
		HID_USAGE_MIN8(0x00),				\
		HID_USAGE_MAX8(0x67),				\
		HID_REPORT_COUNT(0x68),				\
		HID_INPUT(0x02),				\
								\
		/* Setup LED indicators */			\
		HID_USAGE_PAGE(HID_USAGE_GEN_LEDS),		\
		HID_USAGE_MIN8(1), /* = Num Lock */		\
		HID_USAGE_MAX8(5), /* = Kana */			\
		HID_LOGICAL_MIN8(0),				\
		HID_LOGICAL_MAX8(1),				\
		HID_REPORT_SIZE(1),				\
		HID_REPORT_COUNT(5),				\
		HID_OUTPUT(0x02),				\
								\
		/* Pad to a byte */				\
		HID_REPORT_SIZE(3),				\
		HID_REPORT_COUNT(1),				\
		HID_OUTPUT(0x03),				\
	HID_END_COLLECTION,					\
};

typedef struct {
	uint8_t code;	// enum hid_kbd_code
	uint8_t mod;	// enum hid_kbd_modifier
} key_t;

// Table for keymasks
static key_t left_keymask[KSCAN_ROW_LEN][KSCAN_COL_LEN] = {
/*0*/	{{HID_KEY_ESC, 0}, {HID_KEY_F1, 0},	  {HID_KEY_F2, 0}, {HID_KEY_F3, 0}, {HID_KEY_F4, 0}, {HID_KEY_F5, 0}, {HID_KEY_5, 0}},
/*1*/	{{0, 0},	   {HID_KEY_GRAVE, 0},	  {HID_KEY_1, 0},  {HID_KEY_2, 0},  {HID_KEY_3, 0},  {HID_KEY_4, 0},  {HID_KEY_T, 0}},
/*2*/	{{0, 0},	   {HID_KEY_TAB, 0},	  {HID_KEY_Q, 0},  {HID_KEY_W, 0},  {HID_KEY_E, 0},  {HID_KEY_R, 0},  {HID_KEY_G, 0}},
/*3*/	{{0, 0},	   {HID_KEY_CAPSLOCK, 0}, {HID_KEY_A, 0},  {HID_KEY_S, 0},  {HID_KEY_D, 0},  {HID_KEY_F, 0},  {HID_KEY_B, 0}},

/*4*/	{{0, 0}, {0, HID_KBD_MODIFIER_LEFT_SHIFT}, {HID_KEY_Z, 0},			{HID_KEY_X, 0},  {HID_KEY_C, 0},		 {HID_KEY_V, 0},     {HID_KEY_UP, 0}},
/*5*/	{{0, 0}, {0, HID_KBD_MODIFIER_LEFT_CTRL},  {0, HID_KBD_MODIFIER_LEFT_UI},	{0, 0},/*mod*/	 {0, HID_KBD_MODIFIER_LEFT_ALT}, {HID_KEY_SPACE, 0}, {HID_KEY_ENTER, 0}},
};

// The rows are column and columns are rows
static key_t coproc_keymask[] = {
/*0*/	{HID_KEY_F6, 0}, {HID_KEY_6, 0}, {HID_KEY_Y, 0}, {HID_KEY_H, 0}, 		{HID_KEY_N, 0}, 	{0, 0},
/*1*/	{HID_KEY_F7, 0}, {HID_KEY_7, 0}, {HID_KEY_U, 0}, {HID_KEY_J, 0}, 		{HID_KEY_M, 0}, 	{HID_KEY_SPACE, 0},
/*2*/	{HID_KEY_F8, 0}, {HID_KEY_8, 0}, {HID_KEY_I, 0}, {HID_KEY_K, 0}, 		{HID_KEY_COMMA, 0},	{0, HID_KBD_MODIFIER_RIGHT_ALT},
/*3*/	{HID_KEY_F9, 0}, {HID_KEY_9, 0}, {HID_KEY_O, 0}, {HID_KEY_L, 0}, 		{HID_KEY_DOT, 0}, 	{0, HID_KBD_MODIFIER_RIGHT_UI},
/*4*/	{HID_KEY_F10, 0}, {HID_KEY_0, 0}, {HID_KEY_P, 0}, {HID_KEY_SEMICOLON, 0},	{HID_KEY_SLASH, 0}, 	{0, 0},

/*5*/	{HID_KEY_F11, 0}, {HID_KEY_MINUS, 0}, {HID_KEY_LEFTBRACE, 0}, {HID_KEY_APOSTROPHE, 0}, {0, HID_KBD_MODIFIER_RIGHT_SHIFT}, {0, HID_KBD_MODIFIER_RIGHT_CTRL},

/*6*/	{HID_KEY_F12, 0},	{HID_KEY_EQUAL, 0},	{HID_KEY_RIGHTBRACE, 0},{HID_KEY_ENTER, 0}, {0, 0},		 {HID_KEY_LEFT, 0},
/*7*/	{HID_KEY_SYSRQ, 0},	{HID_KEY_BACKSPACE, 0}, {HID_KEY_BACKSLASH, 0}, {0, 0},	    	    {HID_KEY_UP, 0},	 {HID_KEY_DOWN, 0},
/*8*/	{HID_KEY_NUMLOCK, 0},	{HID_KEY_INSERT, 0},	{HID_KEY_DELETE, 0},	{0, 0},	    	    {0, 0},		 {HID_KEY_RIGHT, 0},
/*9*/	{HID_KEY_PAUSE, 0},	{HID_KEY_HOME, 0},	{HID_KEY_END, 0},	{0, 0},	    	    {HID_KEY_KPPLUS, 0}, {HID_KEY_KPENTER, 0},
/*10*/	{HID_KEY_KPMINUS, 0},	{HID_KEY_PAGEUP, 0},	{HID_KEY_PAGEDOWN, 0},	{0, 0},	    	    {0, 0},		 {0, 0},
};

/* USB report array */
static uint8_t report[14] = {0};

K_MUTEX_DEFINE(report_mutex);

// HID ops
static void in_ready_cb(const struct device *dev);
static void out_ready_cb(const struct device *dev);
static const struct hid_ops kbd_ops = {
	.int_in_ready = in_ready_cb,
	.int_out_ready = out_ready_cb,
};

/*******************************************************************/
/* Functions */

static void in_ready_cb(const struct device *dev) {
	ARG_UNUSED(dev);
	k_sem_give(&usb_sem);
}

static void out_ready_cb(const struct device *dev) {
	uint8_t data;
	uint32_t ret;
	if (hid_int_ep_read(dev, &data, sizeof(data), &ret) < 0) {
		LOG_ERR(">> Failed to read out ep");
	}
	if (ret > 0) {
		LOG_INF(">> USB out size = %u, val = %u", ret, data);
		gpio_pin_set_dt(&caps, data & HID_KBD_LED_CAPS_LOCK);
		kscan_uart_set_led(kscan_uart_dev, NUM_LOCK_LED, data & HID_KBD_LED_NUM_LOCK);
	} else {
		LOG_INF("USB out nothing ret");
	}
}

static void status_cb(enum usb_dc_status_code status, const uint8_t *param) {
	LOG_INF(">> Status %d", status);
}

static inline void update_report(key_t kcode, bool pressed) {
	k_mutex_lock(&report_mutex, K_FOREVER);
	if (kcode.code == 0) {
		if (pressed)
			report[0] |= kcode.mod;
		else
			report[0] &= ~kcode.mod;
	} else {
		if (pressed)
			report[REPORT_IDX(kcode.code)] |= REPORT_MASK(kcode.code);
		else
			report[REPORT_IDX(kcode.code)] &= ~REPORT_MASK(kcode.code);
	}
	k_mutex_unlock(&report_mutex);
	k_sem_give(&kscan_sem);
}

static void kb_gpio_callback(const struct device *dev, uint32_t row, uint32_t col,
			     bool pressed) {
	ARG_UNUSED(dev);
	update_report(left_keymask[row][col], pressed);
}

static void kb_uart_callback(const struct device *dev, uint32_t row, uint32_t col,
			     bool pressed) {
	ARG_UNUSED(dev);
	ARG_UNUSED(col);
	update_report(coproc_keymask[row], pressed);
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
	const struct device *hid_dev;

	// Configure LEDs
	LOG_INF(">> Configuring LEDs");
	if (configure_leds(&caps) < 0) return;
	for (i = 0; i < LEDS_LEN; i++) {
		if (configure_leds(&leds[i]) < 0) return;
	}

	// Configure USB
	LOG_INF(">> Configuring USB");
	hid_dev = device_get_binding("HID_0");
	if (!hid_dev) {
		LOG_ERR("Failed to get USB HID Device");
		return;
	}
	usb_hid_register_device(hid_dev, hid_kbd_report_desc, sizeof(hid_kbd_report_desc), &kbd_ops);
	usb_hid_init(hid_dev);

	// Configure KSCAN
	LOG_INF(">> Configuring kscan");
	if (kscan_config(kscan_gpio_dev, kb_gpio_callback) < 0) {
		LOG_ERR("Failed to configure kscan");
		return;
	}
	if (kscan_config(kscan_uart_dev, kb_uart_callback) < 0) {
		LOG_ERR("Failed to configure uart kscan");
	}

	// Enable everything
	if (usb_enable(status_cb) != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}
	k_busy_wait(USEC_PER_SEC);
	LOG_INF("}} %u", sys_clock_hw_cycles_per_sec());
	kscan_enable_callback(kscan_uart_dev);
	kscan_enable_callback(kscan_gpio_dev);

	LOG_INF(">> Starting main loop");
	while (true) {
		k_yield();

		k_sem_take(&kscan_sem, K_FOREVER);
		k_sem_take(&usb_sem, K_FOREVER);
		k_mutex_lock(&report_mutex, K_FOREVER);
		hid_int_ep_write(hid_dev, report, sizeof(report), NULL);
		k_mutex_unlock(&report_mutex);
	}
}
