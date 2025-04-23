#ifndef NTRIP_CLIENT_H
#define NTRIP_CLIENT_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct {
    uint8_t *data;
    size_t length;
} rtcm_data_t;

extern QueueHandle_t rtcm_queue; 

void ntrip_client_init(void);

#endif // NTRIP_CLIENT_H