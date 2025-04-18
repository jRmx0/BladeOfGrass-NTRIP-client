#ifndef STATUS_LED_H
#define STATUS_LED_H

#include "esp_err.h"

typedef enum {
    LED_PATTERN_OFF,
    LED_PATTERN_ON,
    LED_PATTERN_SLOW_BLINK,  // 1Hz
    LED_PATTERN_FAST_BLINK,  // 4Hz
} led_pattern_t;

esp_err_t wifi_led_init(int gpio_pin);

void wifi_led_set_pattern(led_pattern_t pattern);

#endif // STATUS_LED_H