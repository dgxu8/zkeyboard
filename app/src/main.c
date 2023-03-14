#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "usb_kscan.h"
#include "kb_leds.h"

#include <zephyr/logging/log.h>
#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(main);

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	     "Console device is not ACM CDC UART device");

extern struct k_sem kscan_sem;
extern struct k_sem usb_sem;
extern struct k_mutex report_mutex;

void main(void) {
	if (kb_leds_init() < 0)
		return;
	if (usb_kscan_init() < 0)
		return;

	LOG_INF(">> Starting main loop");
	while (true) {
		k_yield();

		k_sem_take(&kscan_sem, K_FOREVER);
		k_sem_take(&usb_sem, K_FOREVER);
		k_mutex_lock(&report_mutex, K_FOREVER);
		usb_kscan_update();
		k_mutex_unlock(&report_mutex);
	}
}
