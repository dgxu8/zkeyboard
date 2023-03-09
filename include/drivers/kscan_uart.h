#ifndef _KSCAN_UART_H
#define _KSCAN_UART_H

#include <zephyr/device.h>

#define NUM_LOCK_LED 1

int kscan_uart_set_led(const struct device *dev, uint8_t led, uint8_t value);

#endif /* _KSCAN_GPIO_H */
