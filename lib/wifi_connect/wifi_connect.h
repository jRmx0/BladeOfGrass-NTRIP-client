#ifndef WIFI_CONNECT_H
#define WIFI_CONNECT_H

#include "esp_err.h"

// WiFi Status for outside
#define WIFI_STATUS_CONNECTING          BIT0
#define WIFI_STATUS_CONNECTED           BIT1
#define WIFI_STATUS_DISCONNECTED        BIT2
#define WIFI_STATUS_RECONNECTING        BIT3
EventGroupHandle_t wifi_status          = NULL;

void wifi_connect_init(void);
esp_err_t wifi_connect_sta(char *ssid, char *pass, int timeout);
void wifi_connect_ap(const char *ssid, const char *pass);
void wifi_disconnect(void);

#endif