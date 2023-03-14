#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>
#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(kb_leds);

/*********************************************************************/
/* DEFINES */
/*********************************************************************/

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

/*********************************************************************/
/* VARIABLES */
/*********************************************************************/

static const struct gpio_dt_spec leds[] = {LISTIFY(LEDS_LEN, LED_LISTIFY, (,))};
static const struct gpio_dt_spec caps = GPIO_SPEC(CAPS_NODE);

/*********************************************************************/
/* FUNCTIONS */
/*********************************************************************/

static int configure_leds(const struct gpio_dt_spec * const gpio);

int kb_leds_init(void) {
	LOG_INF("Configuring LEDs");
	if (configure_leds(&caps) < 0)
		return -1;

	for (int i = 0; i < LEDS_LEN; i++) {
		if (configure_leds(&leds[i]) < 0)
			return -1;
	}
	return 0;
}

void kb_leds_caps_set(int value) {
	gpio_pin_set_dt(&caps, value);
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
