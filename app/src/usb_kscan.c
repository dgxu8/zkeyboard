#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

#include "drivers/kscan_gpio.h"
#include "drivers/kscan_uart.h"
#include "kb_leds.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb_kscan);

/*********************************************************************/
/* DEFINES */
/*********************************************************************/

// Macro for converting keycodes to a bitmask
#define REPORT_IDX(c) ((c / 8) + 1)
#define REPORT_MASK(c) (0x1 << (c % 8))

#define NUMPAD_LAYERS 2
#define LAYER_CNT 1
#define KSCAN_UART_LEN 64

#define NUMPAD_KEY 0xFF
#define MOD_KEY 0xFE

#define HID_KEY_KP_DOT 0x63

/*********************************************************************/
/* TYPES */
/*********************************************************************/

typedef struct {
	uint8_t code;	// enum hid_kbd_code
	uint8_t mod;	// enum hid_kbd_modifier
} key_t;

/* _row_col */
enum numpads_idx {
	KP_0_0, KP_0_1, KP_0_2,
	KP_1_0, KP_1_1, KP_1_2,
	KP_2_0, KP_2_1, KP_2_2,
	KP_3_0, KP_3_1, KP_3_2,
	KP_4_0, KP_4_1, KP_4_2,
	NUMPAD_LEN,
};

typedef key_t matrix_mask_t[KSCAN_ROW_LEN][KSCAN_COL_LEN];
typedef key_t list_mask_t[KSCAN_UART_LEN];

/*********************************************************************/
/* VARIABLES */
/*********************************************************************/

// Device tree structs
static const struct device *const kscan_gpio_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_kscan_gpio));
static const struct device *const kscan_uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_kscan_uart));

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

/* Keymaps */
static const uint8_t numpad_masks[NUMPAD_LAYERS][NUMPAD_LEN] = {{
	/* Mask when num lock is disabled */
	HID_KEY_INSERT, HID_KEY_HOME, HID_KEY_PAGEUP,
	HID_KEY_DELETE, HID_KEY_END,  HID_KEY_PAGEDOWN,
	0,		0, 	      0,
	0, 		HID_KEY_UP,   0,
	HID_KEY_LEFT, 	HID_KEY_DOWN, HID_KEY_RIGHT,
	},{
	/* Mask when num lock is enabled */
	HID_KEY_INSERT, HID_KEY_KPSLASH, HID_KEY_KPASTERISK,
	HID_KEY_KP_7, HID_KEY_KP_8, HID_KEY_KP_9,
	HID_KEY_KP_4, HID_KEY_KP_5, HID_KEY_KP_6,
	HID_KEY_KP_1, HID_KEY_KP_2, HID_KEY_KP_3,
	HID_KEY_KP_0, HID_KEY_KP_0, HID_KEY_KP_DOT,
}};

// Table for keymasks
//static key_t left_keymask_dflt[KSCAN_ROW_LEN][KSCAN_COL_LEN] = {
static matrix_mask_t left_keymask_dflt = {
/*0*/	{{HID_KEY_ESC, 0}, {HID_KEY_F1, 0},	  {HID_KEY_F2, 0}, {HID_KEY_F3, 0}, {HID_KEY_F4, 0}, {HID_KEY_F5, 0}, {HID_KEY_5, 0}},
/*1*/	{{0, 0},	   {HID_KEY_GRAVE, 0},	  {HID_KEY_1, 0},  {HID_KEY_2, 0},  {HID_KEY_3, 0},  {HID_KEY_4, 0},  {HID_KEY_T, 0}},
/*2*/	{{0, 0},	   {HID_KEY_TAB, 0},	  {HID_KEY_Q, 0},  {HID_KEY_W, 0},  {HID_KEY_E, 0},  {HID_KEY_R, 0},  {HID_KEY_G, 0}},
/*3*/	{{0, 0},	   {HID_KEY_CAPSLOCK, 0}, {HID_KEY_A, 0},  {HID_KEY_S, 0},  {HID_KEY_D, 0},  {HID_KEY_F, 0},  {HID_KEY_B, 0}},

/*4*/	{{0, 0}, {0, HID_KBD_MODIFIER_LEFT_SHIFT}, {HID_KEY_Z, 0},			{HID_KEY_X, 0}, {HID_KEY_C, 0},			{HID_KEY_V, 0},     {HID_KEY_SPACE, 0}},
/*5*/	{{0, 0}, {0, HID_KBD_MODIFIER_LEFT_CTRL},  {0, HID_KBD_MODIFIER_LEFT_UI},	{HID_KEY_NUMLOCK, 0, 0},	{0, HID_KBD_MODIFIER_LEFT_ALT}, {HID_KEY_SPACE, 0}, {HID_KEY_SPACE, 0}},
};

// The rows are column and columns are rows
static list_mask_t uart_keymask_dflt = {
/*0*/	{HID_KEY_F6, 0}, {HID_KEY_6, 0}, {HID_KEY_Y, 0}, {HID_KEY_H, 0},	  {HID_KEY_N, 0},     {HID_KEY_SPACE, 0},
/*1*/	{HID_KEY_F7, 0}, {HID_KEY_7, 0}, {HID_KEY_U, 0}, {HID_KEY_J, 0},	  {HID_KEY_M, 0},     {HID_KEY_SPACE, 0},
/*2*/	{HID_KEY_F8, 0}, {HID_KEY_8, 0}, {HID_KEY_I, 0}, {HID_KEY_K, 0},	  {HID_KEY_COMMA, 0}, {0, HID_KBD_MODIFIER_RIGHT_ALT},
/*3*/	{HID_KEY_F9, 0}, {HID_KEY_9, 0}, {HID_KEY_O, 0}, {HID_KEY_L, 0},	  {HID_KEY_DOT, 0},   {0, HID_KBD_MODIFIER_RIGHT_UI},
/*4*/	{HID_KEY_F10, 0}, {HID_KEY_0, 0}, {HID_KEY_P, 0}, {HID_KEY_SEMICOLON, 0}, {HID_KEY_SLASH, 0}, {HID_KEY_NUMLOCK, 0},

/*5*/	{HID_KEY_F11, 0}, {HID_KEY_MINUS, 0}, {HID_KEY_LEFTBRACE, 0}, {HID_KEY_APOSTROPHE, 0}, {0, HID_KBD_MODIFIER_RIGHT_SHIFT}, {0, HID_KBD_MODIFIER_RIGHT_CTRL},

/*6*/	{HID_KEY_F12, 0},	{HID_KEY_EQUAL, 0},	{HID_KEY_RIGHTBRACE, 0},{HID_KEY_ENTER, 0}, {NUMPAD_KEY, KP_3_0},{NUMPAD_KEY, KP_4_0},
/*7*/	{HID_KEY_NUMLOCK, 0},	{HID_KEY_BACKSPACE, 0}, {HID_KEY_BACKSLASH, 0}, {NUMPAD_KEY, KP_2_0}, {NUMPAD_KEY, KP_3_1}, {NUMPAD_KEY, KP_4_1},
/*8*/	{0, 0},	{NUMPAD_KEY, KP_0_0},	{NUMPAD_KEY, KP_1_0},	{NUMPAD_KEY, KP_2_1}, {NUMPAD_KEY, KP_3_2}, {NUMPAD_KEY, KP_4_2},
/*9*/	{0, 0},	{NUMPAD_KEY, KP_0_1},	{NUMPAD_KEY, KP_1_1},	{NUMPAD_KEY, KP_2_2}, {HID_KEY_KPPLUS, 0}, {HID_KEY_ENTER, 0},
/*10*/	{HID_KEY_KPMINUS, 0},	{NUMPAD_KEY, KP_0_2},	{NUMPAD_KEY, KP_1_2},	{0, 0},
};

static matrix_mask_t *left_keymasks[LAYER_CNT] = {&left_keymask_dflt};
static list_mask_t *coproc_keymasks[LAYER_CNT] = {&uart_keymask_dflt};

/* local variables */
static uint8_t report[14] = {0};
static uint_fast32_t layer = 0;
static uint_fast8_t num_lock = false;
static const struct device *hid_dev;

// Control flow items
K_SEM_DEFINE(usb_sem, 1, 1); // default to 1 so someone cam graab it
K_SEM_DEFINE(kscan_sem, 0, 1);
K_MUTEX_DEFINE(report_mutex);

/*********************************************************************/
/* FUNCTIONS */
/*********************************************************************/

// HID ops
static void in_ready_cb(const struct device *dev);
static void out_ready_cb(const struct device *dev);
static const struct hid_ops kbd_ops = {
	.int_in_ready = in_ready_cb,
	.int_out_ready = out_ready_cb,
};

static void in_ready_cb(const struct device *dev);
static void out_ready_cb(const struct device *dev);
static void status_cb(enum usb_dc_status_code status, const uint8_t *param);
static inline void update_report(key_t kcode, bool pressed);
static void kb_gpio_callback(const struct device *dev, uint32_t row, uint32_t col,
			     bool pressed);
static void kb_uart_callback(const struct device *dev, uint32_t row, uint32_t col,
			     bool pressed);

int usb_kscan_init(void) {
	LOG_INF("Configuring USB");
	hid_dev = device_get_binding("HID_0");
	if (!hid_dev) {
		LOG_ERR("Failed to get USB HID Device");
		return -1;
	}
	usb_hid_register_device(hid_dev, hid_kbd_report_desc, sizeof(hid_kbd_report_desc), &kbd_ops);
	usb_hid_init(hid_dev);

	LOG_INF("Configuring kscan");
	if (kscan_config(kscan_gpio_dev, kb_gpio_callback) < 0) {
		LOG_ERR("Failed to configure kscan");
		return -1;
	}
	if (kscan_config(kscan_uart_dev, kb_uart_callback) < 0) {
		LOG_ERR("Failed to configure uart kscan");
		return -1;
	}

	if (usb_enable(status_cb) != 0) {
		LOG_ERR("Failed to enable USB");
		return -1;
	}
	k_busy_wait(USEC_PER_SEC);
	kscan_enable_callback(kscan_uart_dev);
	kscan_enable_callback(kscan_gpio_dev);
	return 0;
}

void usb_kscan_update(void) {
	hid_int_ep_write(hid_dev, report, sizeof(report), NULL);
}

/*********************************************************************/
/* Private */
/*********************************************************************/

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
		kb_leds_caps_set(!!(data & HID_KBD_LED_CAPS_LOCK));

		num_lock = !!(data & HID_KBD_LED_NUM_LOCK);
		kscan_uart_set_led(kscan_uart_dev, NUM_LOCK_LED, num_lock);
	} else {
		LOG_INF("USB out nothing ret");
	}
}

static void status_cb(enum usb_dc_status_code status, const uint8_t *param) {
	LOG_INF(">> Status %d", status);
}

static inline void update_report(key_t kcode, bool pressed) {
	if (kcode.code == NUMPAD_KEY) {
		kcode.code = numpad_masks[num_lock][kcode.mod];
		kcode.mod = 0;
	}
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
	update_report((*left_keymasks[layer])[row][col], pressed);
}

static void kb_uart_callback(const struct device *dev, uint32_t row, uint32_t col,
			     bool pressed) {
	ARG_UNUSED(dev);
	ARG_UNUSED(col);
	update_report((*coproc_keymasks[layer])[row], pressed);
}
