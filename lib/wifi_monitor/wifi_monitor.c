#include "config_board.h"

#include "wifi_led.h"
#include "wifi_connect.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#define MONITOR_TAG "WIFI_MONITOR"

static TaskHandle_t wifi_monitor_task_handle = NULL;

static void wifi_monitor_task(void *pvParameters)
{
    while (1) {
        if (wifi_status != NULL) {
            EventBits_t bits = xEventGroupWaitBits(
                wifi_status,
                WIFI_STATUS_CONNECTING | 
                WIFI_STATUS_CONNECTED | 
                WIFI_STATUS_DISCONNECTED | 
                WIFI_STATUS_RECONNECTING,
                pdTRUE,         
                pdFALSE,        
                portMAX_DELAY 
            );

            if (bits & WIFI_STATUS_CONNECTING) {
                wifi_led_set_pattern(LED_PATTERN_SLOW_BLINK);
            }
            else if (bits & WIFI_STATUS_CONNECTED) {
                wifi_led_set_pattern(LED_PATTERN_ON);
            }
            else if (bits & WIFI_STATUS_DISCONNECTED) {
                wifi_led_set_pattern(LED_PATTERN_OFF);
            }
            else if (bits & WIFI_STATUS_RECONNECTING) {
                wifi_led_set_pattern(LED_PATTERN_FAST_BLINK);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

void wifi_monitor_init(void)
{
    wifi_led_init(IO_WIFI_LED);
    
    xTaskCreate(
        wifi_monitor_task,      
        "wifi_monitor_task",   
        4096,                   
        NULL,                   
        2,                      
        &wifi_monitor_task_handle 
    );
}