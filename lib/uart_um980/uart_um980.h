#ifndef UM980_H
#define UM980_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define RX_BUFF_SIZE 256
#define TX_BUFF_SIZE 1024

typedef struct 
{
    uint8_t data[RX_BUFF_SIZE];
    size_t size;
} gga_report_t;

typedef struct 
{
    uint8_t data[TX_BUFF_SIZE];
    size_t size;
} rtcm_corrections_t;

extern QueueHandle_t um980_gga_to_ntrip_caster_queue;
extern QueueHandle_t um980_rtcm_from_ntrip_caster_queue;

void uart_um980_init(void);

#endif // UM980_H