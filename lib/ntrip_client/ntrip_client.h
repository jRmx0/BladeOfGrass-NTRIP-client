#ifndef NTRIP_CLIENT_H
#define NTRIP_CLIENT_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

esp_err_t ntrip_client_init(const char *host, int port, const char *mountpoint, 
                           const char *username, const char *password, 
                           QueueHandle_t gga_queue, QueueHandle_t rtcm_queue);

esp_err_t ntrip_client_start(void);

esp_err_t ntrip_client_stop(void);

bool ntrip_client_is_connected(void);

#endif // NTRIP_CLIENT_H