#ifndef WIFI_MONITOR_H
#define WIFI_MONITOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

typedef enum{
    WIFI_MONITOR_CONNECTING,
    WIFI_MONITOR_CONNECTED,
    WIFI_MONITOR_RECONNECTING,
    WIFI_MONITOR_DISCONNECTED
} wifi_state_t; 

extern TaskHandle_t wifi_monitor_led_handler;
extern TaskHandle_t wifi_monitor_um980_handler;

void wifi_monitor_init(void);

bool connected_to_wifi(void);

#endif // WIFI_MONITOR_H