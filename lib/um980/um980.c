#include "config_board.h"

#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "wifi_monitor.h"

#define TAG "UM980"

#define UART_UM980 UART_NUM_1
#define UART_BAUD_RATE 115200
#define BUF_SIZE 1024
#define MESSAGE_INTERVAL_MS 10000

static void um980_request_gga_task(void *pvParameters);

void um980_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    //ESP_ERROR_CHECK(uart_driver_install(UART_UM980, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_driver_install(UART_UM980, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_UM980, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_UM980, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(
        um980_request_gga_task,
        "um980_request_gga_task",
        4096,
        NULL,
        5,
        &wifi_monitor_um980_handler
    );
}

static void um980_request_gga_task(void *pvParameters)
{
    uint32_t message_count = 0;
    uint32_t notification_value;
    const char *gga_request = "LOG GNGGA ONCE\r\n";
    
    while (1)
    {
        // Wait for initial notification or remain blocked until WiFi connects
        if (message_count == 0) {
            xTaskNotifyWait(0, 0, &notification_value, portMAX_DELAY);
            if (notification_value != WIFI_MONITOR_CONNECTED) {
                // If we get notified but WiFi is not connected, continue waiting
                continue;
            }
        }
    
        message_count++;
        uart_write_bytes(UART_UM980, gga_request, strlen(gga_request));
        
        // Wait for the next interval or a notification
        if (xTaskNotifyWait(0, 0, &notification_value, pdMS_TO_TICKS(MESSAGE_INTERVAL_MS))) {
            if (notification_value != WIFI_MONITOR_CONNECTED) {
                message_count = 0;
            }
        }
    }
}
