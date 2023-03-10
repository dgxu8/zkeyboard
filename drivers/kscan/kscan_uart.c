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

#define BUFFER_CNT 2
#define PACKET_LEN 8
#define TASK_STACK_SIZE 1024

struct kscan_uart_config {
	struct gpio_dt_spec boot0;
	struct gpio_dt_spec reset;

	const struct pinctrl_dev_config *pcfg;
};

struct kscan_uart_data {
	/* kscan thread specific stuff */
	atomic_t enable;
	kscan_callback_t callback;
	struct k_thread thread;
	K_KERNEL_STACK_MEMBER(thread_stack, TASK_STACK_SIZE);

	/* Previous key state */
	uint64_t prev_state;

	/* UART related variables, used to communicate to coproc */
	const struct device *uart_dev;
	struct k_sem rx_sem;
	struct k_mutex buf_mutex;
	uint8_t *buf_ptr;
	uint8_t buffers[BUFFER_CNT][PACKET_LEN];
	int curr_buff; // Buffer uart is reading to
	int next_buff; // Buffer given to uart to use after curr_buff is done being used

};

static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data);
static void polling_task(const struct device *dev, void *dummy2, void *dummy3);

int kscan_uart_set_led(const struct device *dev, uint8_t led, uint8_t value) {
	struct kscan_uart_data *const data = dev->data;
	uint8_t tx_buff[2];
	static int prev_val = 0;

	if (prev_val == value)
		return 0;

	tx_buff[0] = led;
	tx_buff[1] = value;
	if (unlikely(uart_tx(data->uart_dev, tx_buff, sizeof(tx_buff), SYS_FOREVER_US) < 0)) {
		LOG_ERR("Failed to transmit UART");
		return -1;
	}
	prev_val = value;

	return 0;
}

static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data) {
	ARG_UNUSED(dev);

	int ret, temp;
	struct kscan_uart_data *const data = (struct kscan_uart_data *const)user_data;

	LOG_DBG("[ev] %u", evt->type);

	switch (evt->type) {
	case UART_RX_BUF_REQUEST: // 3
		LOG_INF("[giving] %d", data->next_buff);

		k_mutex_lock(&data->buf_mutex, K_FOREVER);
		ret = uart_rx_buf_rsp(data->uart_dev, data->buffers[data->next_buff], PACKET_LEN);
		k_mutex_unlock(&data->buf_mutex);

		if (unlikely(ret < 0)) {
			LOG_ERR("Receive buff request failed: %d", ret);
		}
		break;
	case UART_RX_BUF_RELEASED: // 4
		LOG_INF("[released] %d. [gave] %d", data->curr_buff, data->next_buff);

		k_mutex_lock(&data->buf_mutex, K_FOREVER);
		data->buf_ptr = data->buffers[data->curr_buff];
		k_mutex_unlock(&data->buf_mutex);
		k_sem_give(&data->rx_sem);

		/*
		 * We gave $next_buff in previous BUF_REQUEST, so now $next_buff
		 * is $curr_buff. This leaves the buffer that was just released
		 * ($curr_buff) avaliable as $next_buff. This effectively means
		 * the buffers have flipped. Do that fliping
		 */
		temp = data->curr_buff;
		data->curr_buff = data->next_buff;
		data->next_buff = temp;
		break;
	case UART_RX_STOPPED: // 6
		LOG_ERR("ERROR: rx stopped %u", evt->data.rx_stop.reason);
		break;
	case UART_RX_RDY: // 2
	default:
		break;
	}
}

static void polling_task(const struct device *dev, void *dummy2, void *dummy3) {
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	uint64_t diff;
	uint64_t raw = 0;
	struct kscan_uart_data *const data = dev->data;

	while (true) {
		k_usleep(10);
		k_yield();
		if (atomic_get(&data->enable) == 0) {
			k_msleep(100);
			continue;
		}

		k_sem_take(&data->rx_sem, K_FOREVER);
		k_mutex_lock(&data->buf_mutex, K_FOREVER);
		memcpy(&raw, data->buf_ptr, PACKET_LEN);
		k_mutex_unlock(&data->buf_mutex);

		diff = raw ^ data->prev_state;
		if (diff != 0) {
			for (int i = 0; i < 64; i++) {
				if ((diff >> i) & 1) {
					data->callback(dev, i, 0, (raw >> i) & 1);
				}
			}
			data->prev_state = raw;
		}
	}
}

static int uart_kscan_init(const struct device *dev) {
	int ret;
	const struct kscan_uart_config *const cfg = dev->config;
	struct kscan_uart_data *const data = dev->data;

	/* Init semaphore + atomic */
	k_mutex_init(&data->buf_mutex);
	k_sem_init(&data->rx_sem, 0, 1);
	atomic_set(&data->enable, 0);

	/* Init normal variables*/
	data->prev_state = 0;
	data->curr_buff = 0;
	data->next_buff = 1;

	/* Configure RESET/BOOT0 pins*/
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

	/* Init UART */
	data->uart_dev = DEVICE_DT_GET(DT_INST_BUS(0));
	if (!device_is_ready(data->uart_dev)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}
	if (!device_is_ready(data->uart_dev)) {
		LOG_ERR("Failed to initialize uart");
		return -ENODEV;
	}
	if (uart_callback_set(data->uart_dev, uart_callback, (void *)data) < 0) {
		LOG_ERR("Failed to set callback");
		return -ENODEV;
	}

	/* Turn on coproc */
	ret = gpio_pin_set_dt(&cfg->reset, 1);
	if (unlikely(ret < 0)) {
		LOG_ERR("Failed to start coproc");
		return -ENODEV;
	}
	/* Give some time for the keyboard to startup*/
	k_busy_wait(500);

	k_thread_create(&data->thread, data->thread_stack, TASK_STACK_SIZE,
			(k_thread_entry_t)polling_task, (void *)dev, NULL, NULL,
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
	int ret;
	struct kscan_uart_data *const data = dev->data;

	ret = uart_rx_enable(data->uart_dev, data->buffers[data->curr_buff], PACKET_LEN, SYS_FOREVER_MS);
	if (unlikely(ret < 0)) {
		LOG_ERR("rx enable failed");
		return -ENODEV;
	}
	atomic_set(&data->enable, 1);
	return 0;
}
static int uart_kscan_disable_callback(const struct device *dev) {
	int ret = 0;
	struct kscan_uart_data *const data = dev->data;

	/* Not tested lol */
	if (unlikely(uart_rx_disable(data->uart_dev) < 0)) {
		LOG_ERR("Failed to disable rx");
		ret = -ENODEV;
	}
	atomic_clear(&data->enable);
	return ret;
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
