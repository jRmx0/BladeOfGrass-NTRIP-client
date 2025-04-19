#ifndef STATUS_LED_H
#define STATUS_LED_H

#include "esp_err.h"

esp_err_t wifi_led_init(int gpio_pin);

#endif // STATUS_LED_H