#ifndef UM980_H
#define UM980_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern QueueHandle_t uart_um980_gga_to_ntrip_caster_queue;

void uart_um980_init(void);

#endif // UM980_H