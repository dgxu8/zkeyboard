#define DT_DRV_COMPAT zephyr_uart_kscan

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(uart_kscan, CONFIG_KSCAN_LOG_LEVEL);

#define TASK_STACK_SIZE 1024

struct kscan_uart_config {
	struct gpio_dt_spec boot0;
	struct gpio_dt_spec reset;

	const struct pinctrl_dev_config *pcfg;
};

struct kscan_uart_data {
	uint64_t prev_state;
	atomic_t enable;

	const struct device *dev;
	kscan_callback_t callback;
	struct k_thread thread;
	struct k_sem rx_sem;
	K_KERNEL_STACK_MEMBER(thread_stack, TASK_STACK_SIZE);
};

static void rkey_read_callback(const struct device *dev, struct uart_event *evt, void *user_data) {
	ARG_UNUSED(dev);
	struct kscan_uart_data *const data = (struct kscan_uart_data *const)user_data;

	switch (evt->type) {
	case UART_RX_BUF_RELEASED:
		k_sem_give(&data->rx_sem);
		break;
	default:
		break;
	}
}

void uart_scan(const struct device *dev, void *dummy2, void *dummy3) {
	//const struct kscan_uart_config *const cfg = dev->config;
	struct kscan_uart_data *const data = dev->data;
	uint8_t rx_buf[8] = {0};
	uint64_t raw = 0;

	while (true) {
		k_usleep(50);
		k_yield();
		if (atomic_get(&data->enable) == 0) {
			k_msleep(100);
			continue;
		}
		if (uart_rx_enable(data->dev, rx_buf, sizeof(rx_buf), SYS_FOREVER_MS) < 0) {
			LOG_ERR("rx enable failed");
			continue;
		}
		k_sem_take(&data->rx_sem, K_FOREVER);
		memcpy(&raw, rx_buf, sizeof(rx_buf));
		LOG_INF(">data> 0x%llX", raw);
	}
}

static int uart_kscan_init(const struct device *dev) {
	const struct kscan_uart_config *const cfg = dev->config;
	struct kscan_uart_data *const data = dev->data;

	if (pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT) < 0) {
		LOG_ERR("Failed to apply pinctrl");
		return -ENODEV;
	}
	if (!device_is_ready(cfg->boot0.port)) {
		LOG_ERR("BOOT0 not ready");
		return -ENODEV;
	}
	gpio_pin_configure_dt(&cfg->boot0, GPIO_OUTPUT_HIGH);
	if (!device_is_ready(cfg->reset.port)) {
		LOG_ERR("BOOT0 not ready");
		return -ENODEV;
	}
	gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_LOW);

	/* Init semaphore + atomic */
	k_sem_init(&data->rx_sem, 0, 1);
	atomic_set(&data->enable, 0);
	data->prev_state = 0;

	/* Init UART */
	data->dev = DEVICE_DT_GET(DT_INST_BUS(0));
	if (!device_is_ready(data->dev)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	if (!device_is_ready(data->dev)) {
		LOG_ERR("Failed to initialize uart");
		return -ENODEV;
	}

	if (uart_callback_set(data->dev, rkey_read_callback, (void *)data) < 0) {
		LOG_ERR("Failed to set callback");
		return -ENODEV;
	}

	k_thread_create(&data->thread, data->thread_stack, TASK_STACK_SIZE,
			(k_thread_entry_t)uart_scan, (void *)dev, NULL, NULL,
			K_PRIO_COOP(4), 0, K_NO_WAIT);
	return 0;
}

static int uart_kscan_configure(const struct device *dev, kscan_callback_t callback) {
	struct kscan_uart_data *data = dev->data;
	if (!dev) {
		return -EINVAL;
	}
	data->callback = callback;
	return 0;
}
static int uart_kscan_enable_callback(const struct device *dev) {
	const struct kscan_uart_config *const cfg = dev->config;
	struct kscan_uart_data *const data = dev->data;
	if (unlikely(gpio_pin_set_dt(&cfg->reset, 1) < 0)) {
		LOG_ERR("Failed to start coproc");
		return -ENODEV;
	}
	/* Give some time for the keyboard to startup*/
	k_busy_wait(1000);
	atomic_set(&data->enable, 1);
	return 0;
}
static int uart_kscan_disable_callback(const struct device *dev) {
	const struct kscan_uart_config *const cfg = dev->config;
	struct kscan_uart_data *const data = dev->data;
	if (unlikely(gpio_pin_set_dt(&cfg->reset, 0) < 0)) {
		LOG_ERR("Failed to stop coproc");
		return -ENODEV;
	}
	atomic_clear(&data->enable);
	return 0;
}

static const struct kscan_driver_api kscan_uart_driver_api = {
	.config = uart_kscan_configure,
	.enable_callback = uart_kscan_enable_callback,
	.disable_callback = uart_kscan_disable_callback,
};

PINCTRL_DT_INST_DEFINE(0);
static struct kscan_uart_data kbd_data;
static struct kscan_uart_config kscan_uart_cfg_0 = {
	.boot0 = GPIO_DT_SPEC_INST_GET(0, boot0_gpios),
	.reset = GPIO_DT_SPEC_INST_GET(0, reset_gpios),
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
};
DEVICE_DT_INST_DEFINE(0, &uart_kscan_init, NULL, &kbd_data,
		      &kscan_uart_cfg_0, POST_KERNEL,
		      CONFIG_KSCAN_INIT_PRIORITY, &kscan_uart_driver_api);
