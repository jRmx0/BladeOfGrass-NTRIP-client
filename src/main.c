#include "config_credentials.h"
#include "config_board.h"

#include "wifi_connect.h"
#include "wifi_monitor.h"
#include "wifi_led.h"
#include "uart_um980.h"
#include "ntrip_client.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

//static const char *TAG = "MAIN";

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_connect_init();
    
    wifi_monitor_init();
    wifi_led_init(IO_WIFI_LED);
    uart_um980_init();

    ESP_ERROR_CHECK(wifi_connect_sta(WIFI_SSID, WIFI_PASS, portMAX_DELAY));

    //ntrip_client_init();
}