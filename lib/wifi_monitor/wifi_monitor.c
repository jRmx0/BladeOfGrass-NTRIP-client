#include "config_board.h"

#include "wifi_connect.h"
#include "wifi_monitor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#define MONITOR_TAG "WIFI_MONITOR"

static wifi_state_t current_wifi_state = WIFI_MONITOR_DISCONNECTED;
TaskHandle_t wifi_state_handler = NULL;

// Based on wifi state other modules are called or blocked (managing wifi disconections)
static void wifi_monitor_task(void *pvParameters)
{
    while (1)
    {
        if (wifi_status != NULL)
        {
            EventBits_t bits = xEventGroupWaitBits(
                wifi_status,
                WIFI_STATUS_CONNECTING |
                WIFI_STATUS_CONNECTED |
                WIFI_STATUS_DISCONNECTED |
                WIFI_STATUS_RECONNECTING,
                pdTRUE,
                pdFALSE,
                portMAX_DELAY);

            if (bits & (WIFI_STATUS_CONNECTING | WIFI_STATUS_CONNECTED | WIFI_STATUS_DISCONNECTED | WIFI_STATUS_RECONNECTING))
            {
                switch (bits)
                {
                case WIFI_STATUS_CONNECTING:
                    current_wifi_state = WIFI_MONITOR_CONNECTING;
                    break;
                case WIFI_STATUS_CONNECTED:
                    current_wifi_state = WIFI_MONITOR_CONNECTED;
                    break;
                case WIFI_STATUS_DISCONNECTED:
                    current_wifi_state = WIFI_MONITOR_DISCONNECTED;
                    break;
                case WIFI_STATUS_RECONNECTING:
                    current_wifi_state = WIFI_MONITOR_RECONNECTING;
                    break;
                }
                xTaskNotify(wifi_state_handler, current_wifi_state, eSetValueWithOverwrite);
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

void wifi_monitor_init(void)
{
    xTaskCreate(
        wifi_monitor_task,
        "wifi_monitor_task",
        4096,
        NULL,
        5,
        NULL);
}

bool wifi_is_connected(void)
{
    return (current_wifi_state == WIFI_MONITOR_CONNECTED);
}