#define DT_DRV_COMPAT zephyr_gpio_kscan

#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "drivers/kscan_gpio.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(kscan, CONFIG_KSCAN_LOG_LEVEL);

/* Defines */
#define TASK_STACK_SIZE 1024

#define COL_COUNT DT_INST_PROP_LEN(0, col_gpios)
#define ROW_COUNT DT_INST_PROP_LEN(0, row_gpios)
BUILD_ASSERT(COL_COUNT == KSCAN_COL_LEN,
	"col len set in header is not the same as in the driver");
BUILD_ASSERT(ROW_COUNT == KSCAN_ROW_LEN,
	"row len set in header is not the same as in the driver");

#define KSCAN_GET_GPIO(idx, prop) GPIO_DT_SPEC_INST_GET_BY_IDX(0, prop, idx)

/* Types */
struct kscan_gpio_config {
	struct gpio_dt_spec col_gpios[COL_COUNT];
	struct gpio_dt_spec row_gpios[ROW_COUNT];

	const struct pinctrl_dev_config *pcfg;
};

struct kscan_gpio_data {
	bool prev_state[COL_COUNT][ROW_COUNT];
	uint8_t debounce[COL_COUNT][ROW_COUNT];

	kscan_callback_t callback;
	struct k_thread thread;
	atomic_t enable;

	K_KERNEL_STACK_MEMBER(thread_stack, TASK_STACK_SIZE);
};

/**
 * Private functions
 **/
static int init_gpio_list(struct gpio_dt_spec const *const gpios, size_t len, gpio_flags_t flag);
static void polling_task(const struct device *dev, void *dummy2, void *dummy3);

static int init_gpio_list(struct gpio_dt_spec const *const gpios, size_t len, gpio_flags_t flag) {
	for (int i = 0; i < len; i++) {
		if (!device_is_ready(gpios[i].port)) {
			LOG_ERR("GPIOs not ready");
			return -ENODEV;
		}
		gpio_pin_configure_dt(&gpios[i], flag);
	}
	return 0;
}

static void polling_task(const struct device *dev, void *dummy2, void *dummy3) {
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	int ret;
	struct kscan_gpio_data *const data = dev->data;
	struct kscan_gpio_config const *const cfg = dev->config;
	struct gpio_dt_spec const *const cols = cfg->col_gpios;
	struct device const *const rows_port = cfg->row_gpios[0].port;

	gpio_port_value_t val;

	LOG_INF("Starting poll...");
	while (true) {
		k_usleep(50);
		k_yield();
		if (atomic_get(&data->enable) == 0) {
			k_msleep(100);
			continue;
		}
		for (int c = 0; c < COL_COUNT; c++) {
			ret = gpio_pin_set_dt(&cols[c], 1);
			if (unlikely(ret < 0)) {
				LOG_ERR("Failed to set gpio %d", c);
				break;
			}

			//k_busy_wait(1);
			ret = gpio_port_get_raw(rows_port, &val);
			if (unlikely(ret < 0)) {
				LOG_ERR("Failed to read port");
				break;
			}

			val >>= 1;
			for (int r = 0; r < ROW_COUNT; r++) {
				if (data->prev_state[c][r] != (val & 1) && data->debounce[c][r] == 0) {
					data->debounce[c][r] = 10;
					data->prev_state[c][r] = val & 1;
					data->callback(dev, r, c, val & 1);
				} else if (data->debounce[c][r] > 0) {
					data->debounce[c][r]--;
				}
				val >>= 1;
			}

			ret = gpio_pin_set_dt(&cols[c], 0);
			if (unlikely(ret < 0)) {
				LOG_ERR("Failed to set gpio %d", c);
				break;
			}
		}
	}
}

/**
 * Functions required by KSCAN
 **/
static int gpio_kscan_init(const struct device *dev) {
	const struct kscan_gpio_config *const cfg = dev->config;
	struct kscan_gpio_data *const data = dev->data;

	if (pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT) < 0) {
		LOG_ERR("Failed to apply pinctrl");
		return -ENODEV;
	}

	if (init_gpio_list(cfg->col_gpios, COL_COUNT, GPIO_OUTPUT_LOW)) {
		return -ENODEV;
	}
	if (init_gpio_list(cfg->row_gpios, ROW_COUNT, GPIO_INPUT)) {
		return -ENODEV;
	}

	data->enable = ATOMIC_INIT(0);
	memset(data->prev_state, 0, sizeof(data->prev_state));
	memset(data->debounce, 0, sizeof(data->debounce));

	k_thread_create(&data->thread, data->thread_stack, TASK_STACK_SIZE,
		      (k_thread_entry_t)polling_task, (void *)dev, NULL, NULL,
		      K_PRIO_COOP(4), 0, K_NO_WAIT);
	return 0;
}
static int gpio_kscan_configure(const struct device *dev, kscan_callback_t callback) {
	struct kscan_gpio_data *data = dev->data;
	if (!dev) {
		return -EINVAL;
	}
	data->callback = callback;
	return 0;
}
static int gpio_kscan_enable_callback(const struct device *dev) {
	struct kscan_gpio_data *const data = dev->data;
	atomic_set(&data->enable, 1);
	return 0;
}
static int gpio_kscan_disable_callback(const struct device *dev) {
	struct kscan_gpio_data *const data = dev->data;
	atomic_clear(&data->enable);
	return 0;
}


static const struct kscan_driver_api kscan_gpio_driver_api = {
	.config = gpio_kscan_configure,
	.enable_callback = gpio_kscan_enable_callback,
	.disable_callback = gpio_kscan_disable_callback,
};

PINCTRL_DT_INST_DEFINE(0);
static struct kscan_gpio_data kbd_data;
static struct kscan_gpio_config kscan_gpio_cfg_0 = {
	.col_gpios = {LISTIFY(COL_COUNT, KSCAN_GET_GPIO, (,), col_gpios)},
	.row_gpios = {LISTIFY(ROW_COUNT, KSCAN_GET_GPIO, (,), row_gpios)},
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
};

DEVICE_DT_INST_DEFINE(0, &gpio_kscan_init, NULL, &kbd_data,
		      &kscan_gpio_cfg_0, POST_KERNEL,
		      CONFIG_KSCAN_INIT_PRIORITY, &kscan_gpio_driver_api);
